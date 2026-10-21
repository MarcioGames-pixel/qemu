/*
 * Qualcomm QSD8x50 / MSM Vectored Interrupt Controller (phase 1 model).
 *
 * Copyright (C) 2026 Nexus-One-QEMU contributors
 *
 * This code is licensed under the GNU GPL, version 2 or later.
 *
 * Models 64 interrupt lines in two 32-bit banks, with the register layout
 * used by arch/arm/mach-msm/irq.c of the legacy 2.6.29 Android kernel.
 *
 * Simplifications (all deliberate for phase 1):
 *  - POLARITY is stored and read back but NOT applied: every line is treated
 *    as active-high, which matches the models in this series.
 *  - Vector priority (VECTPRIORITYn/VECTADDRn) is not modelled; the vector
 *    registers return the lowest-numbered pending interrupt.
 *  - MASTEREN resets to 1 instead of 0 so that a wrong register offset in this
 *    model cannot silently disable all interrupts. Real hardware resets to 0.
 *
 * QEMU interface:
 *  + sysbus MMIO region 0: registers (4 KiB)
 *  + sysbus IRQ 0: output to the CPU IRQ input
 *  + sysbus IRQ 1: output to the CPU FIQ input
 *  + unnamed GPIO inputs 0..63: interrupt sources
 *
 * Debugging: -global qsd8x50-vic.debug=on and run with -d unimp.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/host-utils.h"
#include "hw/arm/qsd8x50.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "qom/object.h"

#define VIC_NUM_IRQS        64

/* Register offsets; paired registers: bank 0 at X, bank 1 at X + 4 */
#define VIC_INT_SELECT      0x000   /* 0 = IRQ, 1 = FIQ */
#define VIC_INT_EN          0x010
#define VIC_INT_ENCLEAR     0x020
#define VIC_INT_ENSET       0x030
#define VIC_INT_TYPE        0x040   /* 0 = level, 1 = edge */
#define VIC_INT_POLARITY    0x050
#define VIC_NO_PEND_VAL     0x060
#define VIC_INT_MASTEREN    0x064
#define VIC_CONFIG          0x068
#define VIC_PROTECTION      0x06C
#define VIC_IRQ_STATUS      0x080
#define VIC_FIQ_STATUS      0x090
#define VIC_RAW_STATUS      0x0A0
#define VIC_INT_CLEAR       0x0B0
#define VIC_SOFTINT         0x0C0
#define VIC_IRQ_VEC_RD      0x0D0
#define VIC_IRQ_VEC_PEND_RD 0x0D4
#define VIC_IRQ_VEC_WR      0x0D8
#define VIC_IRQ_IN_SERVICE  0x0E0
#define VIC_IRQ_IN_STACK    0x0E4
#define VIC_TEST_BUS_SEL    0x0E8

OBJECT_DECLARE_SIMPLE_TYPE(QSD8x50VICState, QSD8X50_VIC)

struct QSD8x50VICState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq fiq;

    uint32_t select[2];
    uint32_t enable[2];
    uint32_t type[2];
    uint32_t polarity[2];
    uint32_t latched[2];    /* edge latches */
    uint32_t level[2];      /* current input line levels */
    uint32_t softint[2];
    uint32_t masteren;
    uint32_t config;
    uint32_t protection;
    uint32_t no_pend_val;
    bool debug;
};

#define VIC_DBG(s, fmt, ...)                                              \
    do {                                                                  \
        if ((s)->debug) {                                                 \
            qemu_log_mask(LOG_UNIMP, "qsd8x50-vic: " fmt "\n",            \
                          ##__VA_ARGS__);                                 \
        }                                                                 \
    } while (0)

static uint32_t vic_raw(QSD8x50VICState *s, int bank)
{
    return (s->latched[bank] & s->type[bank]) |
           (s->level[bank] & ~s->type[bank]) |
           s->softint[bank];
}

static uint32_t vic_irq_status(QSD8x50VICState *s, int bank)
{
    return vic_raw(s, bank) & s->enable[bank] & ~s->select[bank];
}

static uint32_t vic_fiq_status(QSD8x50VICState *s, int bank)
{
    return vic_raw(s, bank) & s->enable[bank] & s->select[bank];
}

/* Lowest-numbered pending IRQ, or -1 */
static int vic_first_pending(QSD8x50VICState *s)
{
    for (int bank = 0; bank < 2; bank++) {
        uint32_t p = vic_irq_status(s, bank);
        if (p) {
            return bank * 32 + ctz32(p);
        }
    }
    return -1;
}

static void vic_update(QSD8x50VICState *s)
{
    bool irq = false, fiq = false;

    for (int bank = 0; bank < 2; bank++) {
        irq |= vic_irq_status(s, bank) != 0;
        fiq |= vic_fiq_status(s, bank) != 0;
    }
    if (!s->masteren) {
        irq = fiq = false;
    }
    qemu_set_irq(s->irq, irq);
    qemu_set_irq(s->fiq, fiq);
}

static void vic_set_irq(void *opaque, int n, int level)
{
    QSD8x50VICState *s = opaque;
    int bank = n >> 5;
    uint32_t bit = 1u << (n & 31);

    if (level) {
        if (!(s->level[bank] & bit)) {
            s->latched[bank] |= bit;    /* rising edge */
        }
        s->level[bank] |= bit;
    } else {
        s->level[bank] &= ~bit;
    }
    vic_update(s);
}

static uint64_t vic_read(void *opaque, hwaddr offset, unsigned size)
{
    QSD8x50VICState *s = opaque;
    int bank = (offset >> 2) & 1;
    uint32_t r = 0;
    int pend;

    switch (offset) {
    case VIC_INT_SELECT ... VIC_INT_SELECT + 4:
        r = s->select[bank];
        break;
    case VIC_INT_EN ... VIC_INT_EN + 4:
    case VIC_INT_ENCLEAR ... VIC_INT_ENCLEAR + 4:
    case VIC_INT_ENSET ... VIC_INT_ENSET + 4:
        r = s->enable[bank];
        break;
    case VIC_INT_TYPE ... VIC_INT_TYPE + 4:
        r = s->type[bank];
        break;
    case VIC_INT_POLARITY ... VIC_INT_POLARITY + 4:
        r = s->polarity[bank];
        break;
    case VIC_NO_PEND_VAL:
        r = s->no_pend_val;
        break;
    case VIC_INT_MASTEREN:
        r = s->masteren;
        break;
    case VIC_CONFIG:
        r = s->config;
        break;
    case VIC_PROTECTION:
        r = s->protection;
        break;
    case VIC_IRQ_STATUS ... VIC_IRQ_STATUS + 4:
        r = vic_irq_status(s, bank);
        break;
    case VIC_FIQ_STATUS ... VIC_FIQ_STATUS + 4:
        r = vic_fiq_status(s, bank);
        break;
    case VIC_RAW_STATUS ... VIC_RAW_STATUS + 4:
        r = vic_raw(s, bank);
        break;
    case VIC_SOFTINT ... VIC_SOFTINT + 4:
        r = s->softint[bank];
        break;
    case VIC_IRQ_VEC_RD:
    case VIC_IRQ_VEC_PEND_RD:
        pend = vic_first_pending(s);
        r = pend >= 0 ? (uint32_t)pend : s->no_pend_val;
        break;
    case VIC_IRQ_IN_SERVICE:
    case VIC_IRQ_IN_STACK:
    case VIC_TEST_BUS_SEL:
        r = 0;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "qsd8x50-vic: read of unknown register 0x%03" HWADDR_PRIx
                      "\n", offset);
        break;
    }

    VIC_DBG(s, "read  0x%03" HWADDR_PRIx " -> 0x%08x", offset, r);
    return r;
}

static void vic_write(void *opaque, hwaddr offset, uint64_t value,
                      unsigned size)
{
    QSD8x50VICState *s = opaque;
    int bank = (offset >> 2) & 1;
    uint32_t v = value;

    VIC_DBG(s, "write 0x%03" HWADDR_PRIx " <- 0x%08x", offset, v);

    switch (offset) {
    case VIC_INT_SELECT ... VIC_INT_SELECT + 4:
        s->select[bank] = v;
        break;
    case VIC_INT_EN ... VIC_INT_EN + 4:
        s->enable[bank] = v;
        break;
    case VIC_INT_ENCLEAR ... VIC_INT_ENCLEAR + 4:
        s->enable[bank] &= ~v;
        break;
    case VIC_INT_ENSET ... VIC_INT_ENSET + 4:
        s->enable[bank] |= v;
        break;
    case VIC_INT_TYPE ... VIC_INT_TYPE + 4:
        s->type[bank] = v;
        break;
    case VIC_INT_POLARITY ... VIC_INT_POLARITY + 4:
        s->polarity[bank] = v;      /* stored only, see file header */
        break;
    case VIC_NO_PEND_VAL:
        s->no_pend_val = v;
        break;
    case VIC_INT_MASTEREN:
        s->masteren = v & 1;
        break;
    case VIC_CONFIG:
        s->config = v;
        break;
    case VIC_PROTECTION:
        s->protection = v;
        break;
    case VIC_INT_CLEAR ... VIC_INT_CLEAR + 4:
        s->latched[bank] &= ~v;
        s->softint[bank] &= ~v;
        break;
    case VIC_SOFTINT ... VIC_SOFTINT + 4:
        s->softint[bank] |= v;
        break;
    case VIC_IRQ_VEC_WR:
    case VIC_IRQ_IN_SERVICE:
    case VIC_IRQ_IN_STACK:
    case VIC_TEST_BUS_SEL:
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "qsd8x50-vic: write of unknown register 0x%03"
                      HWADDR_PRIx " <- 0x%08x\n", offset, v);
        break;
    }
    vic_update(s);
}

static const MemoryRegionOps vic_ops = {
    .read = vic_read,
    .write = vic_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void vic_reset(DeviceState *dev)
{
    QSD8x50VICState *s = QSD8X50_VIC(dev);

    memset(s->select, 0, sizeof(s->select));
    memset(s->enable, 0, sizeof(s->enable));
    memset(s->type, 0, sizeof(s->type));
    memset(s->polarity, 0, sizeof(s->polarity));
    memset(s->latched, 0, sizeof(s->latched));
    memset(s->level, 0, sizeof(s->level));
    memset(s->softint, 0, sizeof(s->softint));
    s->masteren = 1;                /* deviation, see file header */
    s->config = 0;
    s->protection = 0;
    s->no_pend_val = 0xFFFFFFFF;
    vic_update(s);
}

static void vic_init(Object *obj)
{
    QSD8x50VICState *s = QSD8X50_VIC(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &vic_ops, s, TYPE_QSD8X50_VIC,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
    qdev_init_gpio_in(DEVICE(obj), vic_set_irq, VIC_NUM_IRQS);
}

static const VMStateDescription vmstate_vic = {
    .name = TYPE_QSD8X50_VIC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(select, QSD8x50VICState, 2),
        VMSTATE_UINT32_ARRAY(enable, QSD8x50VICState, 2),
        VMSTATE_UINT32_ARRAY(type, QSD8x50VICState, 2),
        VMSTATE_UINT32_ARRAY(polarity, QSD8x50VICState, 2),
        VMSTATE_UINT32_ARRAY(latched, QSD8x50VICState, 2),
        VMSTATE_UINT32_ARRAY(level, QSD8x50VICState, 2),
        VMSTATE_UINT32_ARRAY(softint, QSD8x50VICState, 2),
        VMSTATE_UINT32(masteren, QSD8x50VICState),
        VMSTATE_UINT32(config, QSD8x50VICState),
        VMSTATE_UINT32(protection, QSD8x50VICState),
        VMSTATE_UINT32(no_pend_val, QSD8x50VICState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property vic_properties[] = {
    DEFINE_PROP_BOOL("debug", QSD8x50VICState, debug, false),
};

static void vic_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_legacy_reset(dc, vic_reset);
    dc->vmsd = &vmstate_vic;
    device_class_set_props(dc, vic_properties);
}

static const TypeInfo vic_info = {
    .name          = TYPE_QSD8X50_VIC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(QSD8x50VICState),
    .instance_init = vic_init,
    .class_init    = vic_class_init,
};

static void vic_register_types(void)
{
    type_register_static(&vic_info);
}

type_init(vic_register_types)
