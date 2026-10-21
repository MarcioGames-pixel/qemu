/*
 * Qualcomm QSD8x50 / MSM general-purpose (GPT) and debug (DGT) timers
 * (phase 1 model).
 *
 * Copyright (C) 2026 Nexus-One-QEMU contributors
 *
 * This code is licensed under the GNU GPL, version 2 or later.
 *
 * Register layout follows arch/arm/mach-msm/timer.c of the legacy 2.6.29
 * Android kernel, inside the 4 KiB "CSR" block:
 *
 *   GPT: base +0x04   MATCH_VAL +0x00, COUNT_VAL +0x04, ENABLE +0x08,
 *                     CLEAR +0x0C        (32768 Hz)
 *   DGT: base +0x24   same layout        (19.2 MHz / (DIV + 1))
 *   DGT_CLK_CTL       +0x34              bits [1:0] = divider - 1
 *
 * ENABLE bit 0 = counter running, bit 1 = clear counter on match.
 * On a match the output line is dropped and raised again (so a VIC
 * configured for edge sensitivity sees a fresh edge every time) and stays
 * high until the guest rewrites MATCH_VAL, ENABLE or CLEAR (so a VIC
 * configured for level sensitivity also works).
 *
 * QEMU interface:
 *  + sysbus MMIO region 0: registers
 *  + sysbus IRQ 0: GPT match, sysbus IRQ 1: DGT match
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "qemu/host-utils.h"
#include "hw/arm/qsd8x50.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define TMR_GPT_BASE        0x04
#define TMR_DGT_BASE        0x24
#define TMR_CH_SIZE         0x10
#define TMR_DGT_CLK_CTL     0x34

#define CH_MATCH_VAL        0x00
#define CH_COUNT_VAL        0x04
#define CH_ENABLE           0x08
#define CH_CLEAR            0x0C

#define TMR_EN              (1u << 0)
#define TMR_CLR_ON_MATCH    (1u << 1)

#define GPT_HZ              32768u
#define DGT_SRC_HZ          19200000u

typedef struct QSD8x50TimerChannel {
    QEMUTimer *timer;
    qemu_irq irq;
    uint32_t hz;
    uint32_t match;
    uint32_t enable;
    uint32_t base_count;    /* counter value at base_ns */
    int64_t base_ns;
} QSD8x50TimerChannel;

OBJECT_DECLARE_SIMPLE_TYPE(QSD8x50TimerState, QSD8X50_TIMER)

struct QSD8x50TimerState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QSD8x50TimerChannel ch[2];
    uint32_t dgt_clk_ctl;
};

static uint32_t tmr_ch_count(QSD8x50TimerChannel *c)
{
    int64_t elapsed;

    if (!(c->enable & TMR_EN)) {
        return c->base_count;
    }
    elapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - c->base_ns;
    if (elapsed < 0) {
        elapsed = 0;
    }
    return c->base_count +
           (uint32_t)muldiv64(elapsed, c->hz, NANOSECONDS_PER_SECOND);
}

/*
 * Restart counting from 'count' at the current instant and (re)program the
 * QEMU timer for the next match. Does nothing but freeze if disabled.
 */
static void tmr_ch_arm(QSD8x50TimerChannel *c, uint32_t count)
{
    uint64_t delta;
    int64_t ns;

    c->base_count = count;
    c->base_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    timer_del(c->timer);
    if (!(c->enable & TMR_EN)) {
        return;
    }
    delta = (uint32_t)(c->match - count);
    if (delta == 0) {
        delta = 1ULL << 32;
    }
    ns = muldiv64(delta, NANOSECONDS_PER_SECOND, c->hz);
    timer_mod_ns(c->timer, c->base_ns + ns + 1);
}

static void tmr_expire(void *opaque)
{
    QSD8x50TimerChannel *c = opaque;

    qemu_irq_lower(c->irq);
    qemu_irq_raise(c->irq);
    tmr_ch_arm(c, (c->enable & TMR_CLR_ON_MATCH) ? 0 : c->match);
}

static QSD8x50TimerChannel *tmr_lookup(QSD8x50TimerState *s, hwaddr offset,
                                       hwaddr *reg)
{
    if (offset >= TMR_GPT_BASE && offset < TMR_GPT_BASE + TMR_CH_SIZE) {
        *reg = offset - TMR_GPT_BASE;
        return &s->ch[0];
    }
    if (offset >= TMR_DGT_BASE && offset < TMR_DGT_BASE + TMR_CH_SIZE) {
        *reg = offset - TMR_DGT_BASE;
        return &s->ch[1];
    }
    return NULL;
}

static uint64_t tmr_read(void *opaque, hwaddr offset, unsigned size)
{
    QSD8x50TimerState *s = opaque;
    QSD8x50TimerChannel *c;
    hwaddr reg;

    if (offset == TMR_DGT_CLK_CTL) {
        return s->dgt_clk_ctl;
    }
    c = tmr_lookup(s, offset, &reg);
    if (c) {
        switch (reg) {
        case CH_MATCH_VAL:
            return c->match;
        case CH_COUNT_VAL:
            return tmr_ch_count(c);
        case CH_ENABLE:
            return c->enable;
        case CH_CLEAR:
            return 0;
        }
    }
    qemu_log_mask(LOG_UNIMP,
                  "qsd8x50-timer: read of unknown register 0x%03" HWADDR_PRIx
                  "\n", offset);
    return 0;
}

static void tmr_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    QSD8x50TimerState *s = opaque;
    QSD8x50TimerChannel *c;
    uint32_t v = value;
    uint32_t cnt;
    hwaddr reg;

    if (offset == TMR_DGT_CLK_CTL) {
        QSD8x50TimerChannel *dgt = &s->ch[1];

        cnt = tmr_ch_count(dgt);
        s->dgt_clk_ctl = v;
        dgt->hz = DGT_SRC_HZ / ((v & 3) + 1);
        tmr_ch_arm(dgt, cnt);
        return;
    }

    c = tmr_lookup(s, offset, &reg);
    if (!c) {
        qemu_log_mask(LOG_UNIMP,
                      "qsd8x50-timer: write of unknown register 0x%03"
                      HWADDR_PRIx " <- 0x%08x\n", offset, v);
        return;
    }

    switch (reg) {
    case CH_MATCH_VAL:
        cnt = tmr_ch_count(c);
        c->match = v;
        qemu_irq_lower(c->irq);
        tmr_ch_arm(c, cnt);
        break;
    case CH_COUNT_VAL:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "qsd8x50-timer: write to read-only COUNT_VAL\n");
        break;
    case CH_ENABLE:
        cnt = tmr_ch_count(c);          /* uses the OLD enable state */
        c->enable = v & (TMR_EN | TMR_CLR_ON_MATCH);
        qemu_irq_lower(c->irq);
        tmr_ch_arm(c, cnt);
        break;
    case CH_CLEAR:
        qemu_irq_lower(c->irq);
        tmr_ch_arm(c, 0);
        break;
    }
}

static const MemoryRegionOps tmr_ops = {
    .read = tmr_read,
    .write = tmr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void tmr_reset(DeviceState *dev)
{
    QSD8x50TimerState *s = QSD8X50_TIMER(dev);

    s->dgt_clk_ctl = 0;
    for (int i = 0; i < 2; i++) {
        QSD8x50TimerChannel *c = &s->ch[i];

        c->hz = i == 0 ? GPT_HZ : DGT_SRC_HZ;
        c->match = 0;
        c->enable = 0;
        qemu_irq_lower(c->irq);
        tmr_ch_arm(c, 0);
    }
}

static void tmr_init(Object *obj)
{
    QSD8x50TimerState *s = QSD8X50_TIMER(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &tmr_ops, s, TYPE_QSD8X50_TIMER,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->ch[0].irq);
    sysbus_init_irq(sbd, &s->ch[1].irq);
}

static void tmr_realize(DeviceState *dev, Error **errp)
{
    QSD8x50TimerState *s = QSD8X50_TIMER(dev);

    for (int i = 0; i < 2; i++) {
        s->ch[i].timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, tmr_expire,
                                      &s->ch[i]);
    }
}

static const VMStateDescription vmstate_tmr_ch = {
    .name = TYPE_QSD8X50_TIMER "/channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER_PTR(timer, QSD8x50TimerChannel),
        VMSTATE_UINT32(hz, QSD8x50TimerChannel),
        VMSTATE_UINT32(match, QSD8x50TimerChannel),
        VMSTATE_UINT32(enable, QSD8x50TimerChannel),
        VMSTATE_UINT32(base_count, QSD8x50TimerChannel),
        VMSTATE_INT64(base_ns, QSD8x50TimerChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_tmr = {
    .name = TYPE_QSD8X50_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, QSD8x50TimerState, 2, 1, vmstate_tmr_ch,
                             QSD8x50TimerChannel),
        VMSTATE_UINT32(dgt_clk_ctl, QSD8x50TimerState),
        VMSTATE_END_OF_LIST()
    }
};

static void tmr_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = tmr_realize;
    device_class_set_legacy_reset(dc, tmr_reset);
    dc->vmsd = &vmstate_tmr;
}

static const TypeInfo tmr_info = {
    .name          = TYPE_QSD8X50_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(QSD8x50TimerState),
    .instance_init = tmr_init,
    .class_init    = tmr_class_init,
};

static void tmr_register_types(void)
{
    type_register_static(&tmr_info);
}

type_init(tmr_register_types)
