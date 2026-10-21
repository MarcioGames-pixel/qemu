/*
 * Qualcomm QSD8x50 / MSM UART, non-DM ("UART1/2/3") flavour (phase 1 model).
 *
 * Copyright (C) 2026 Nexus-One-QEMU contributors
 *
 * This code is licensed under the GNU GPL, version 2 or later.
 *
 * Enough of drivers/serial/msm_serial.c (legacy 2.6.29 Android kernel) to
 * get console output and keyboard input: character TX/RX, status, interrupt
 * mask/status and the CR command register.
 *
 * NOT modelled: the DM (data mover) UARTs used for Bluetooth/HS-UART, baud
 * rate generation (MREG/NREG/DREG are stored only), parity/framing errors,
 * break, flow control. The UART TX FIFO is always empty and TX is
 * synchronous.
 *
 * QEMU interface:
 *  + sysbus MMIO region 0: registers
 *  + sysbus IRQ 0: combined interrupt
 *  + chardev property "chardev"
 *
 * Debugging: -global qsd8x50-uart.debug=on and run with -d unimp.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/arm/qsd8x50.h"
#include "hw/core/irq.h"
#include "hw/core/sysbus.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#define UART_FIFO_SIZE      64

/* Register offsets */
#define UART_MR1            0x00
#define UART_MR2            0x04
#define UART_CSR            0x08    /* write */
#define UART_SR             0x08    /* read */
#define UART_TF             0x0C    /* write */
#define UART_RF             0x0C    /* read */
#define UART_CR             0x10    /* write */
#define UART_MISR           0x10    /* read */
#define UART_IMR            0x14    /* write */
#define UART_ISR            0x14    /* read */
#define UART_IPR            0x18
#define UART_TFWR           0x1C
#define UART_RFWR           0x20
#define UART_HCR            0x24
#define UART_MREG           0x28
#define UART_NREG           0x2C
#define UART_DREG           0x30
#define UART_MNDREG         0x34
#define UART_IRDA           0x38
#define UART_REGS_END       0x40    /* generic storage below this offset */

/* Status register */
#define SR_RXRDY            0x01
#define SR_RXFULL           0x02
#define SR_TXRDY            0x04
#define SR_TXEMT            0x08
#define SR_OVERRUN          0x10

/* Interrupt status/mask */
#define ISR_TXLEV           0x01
#define ISR_RXSTALE         0x08
#define ISR_RXLEV           0x10

/* Command register: command field in bits [7:4] */
#define CR_CMD_RESET_RX     0x1
#define CR_CMD_RESET_TX     0x2
#define CR_CMD_RESET_ERR    0x3
#define CR_CMD_RESET_STALE  0x8

OBJECT_DECLARE_SIMPLE_TYPE(QSD8x50UARTState, QSD8X50_UART)

struct QSD8x50UARTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    CharFrontend chr;
    qemu_irq irq;

    uint32_t regs[UART_REGS_END / 4];   /* generic read/write storage */
    uint32_t imr;
    uint8_t rx_fifo[UART_FIFO_SIZE];
    int32_t rx_head;
    int32_t rx_count;
    bool stale;
    bool overrun;
    bool debug;
};

#define UART_DBG(s, fmt, ...)                                             \
    do {                                                                  \
        if ((s)->debug) {                                                 \
            qemu_log_mask(LOG_UNIMP, "qsd8x50-uart: " fmt "\n",           \
                          ##__VA_ARGS__);                                 \
        }                                                                 \
    } while (0)

static uint32_t uart_sr(QSD8x50UARTState *s)
{
    uint32_t sr = SR_TXRDY | SR_TXEMT;

    if (s->rx_count) {
        sr |= SR_RXRDY;
    }
    if (s->rx_count == UART_FIFO_SIZE) {
        sr |= SR_RXFULL;
    }
    if (s->overrun) {
        sr |= SR_OVERRUN;
    }
    return sr;
}

static uint32_t uart_isr(QSD8x50UARTState *s)
{
    uint32_t isr = ISR_TXLEV;       /* TX FIFO is always empty */

    if (s->rx_count && (uint32_t)s->rx_count > s->regs[UART_RFWR / 4]) {
        isr |= ISR_RXLEV;
    }
    if (s->stale) {
        isr |= ISR_RXSTALE;
    }
    return isr;
}

static void uart_update(QSD8x50UARTState *s)
{
    qemu_set_irq(s->irq, (uart_isr(s) & s->imr) != 0);
}

static uint8_t uart_rx_pop(QSD8x50UARTState *s)
{
    uint8_t c = 0;

    if (s->rx_count) {
        c = s->rx_fifo[s->rx_head];
        s->rx_head = (s->rx_head + 1) % UART_FIFO_SIZE;
        s->rx_count--;
    }
    return c;
}

static uint64_t uart_read(void *opaque, hwaddr offset, unsigned size)
{
    QSD8x50UARTState *s = opaque;
    uint32_t r = 0;

    switch (offset) {
    case UART_SR:
        r = uart_sr(s);
        break;
    case UART_RF:
        r = uart_rx_pop(s);
        qemu_chr_fe_accept_input(&s->chr);
        uart_update(s);
        break;
    case UART_MISR:
        r = uart_isr(s) & s->imr;
        break;
    case UART_ISR:
        r = uart_isr(s);
        break;
    default:
        if (offset < UART_REGS_END) {
            r = s->regs[offset / 4];
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "qsd8x50-uart: read of unknown register 0x%02"
                          HWADDR_PRIx "\n", offset);
        }
        break;
    }

    UART_DBG(s, "read  0x%02" HWADDR_PRIx " -> 0x%08x", offset, r);
    return r;
}

static void uart_write(void *opaque, hwaddr offset, uint64_t value,
                       unsigned size)
{
    QSD8x50UARTState *s = opaque;
    uint32_t v = value;
    uint8_t ch;

    UART_DBG(s, "write 0x%02" HWADDR_PRIx " <- 0x%08x", offset, v);

    switch (offset) {
    case UART_TF:
        ch = v;
        /* Blocking write, same simplification as the PL011 model. */
        qemu_chr_fe_write_all(&s->chr, &ch, 1);
        break;
    case UART_CR:
        switch ((v >> 4) & 0xF) {
        case CR_CMD_RESET_RX:
            s->rx_head = 0;
            s->rx_count = 0;
            s->stale = false;
            s->overrun = false;
            break;
        case CR_CMD_RESET_ERR:
            s->overrun = false;
            break;
        case CR_CMD_RESET_STALE:
            s->stale = false;
            break;
        case CR_CMD_RESET_TX:
        case 0:
            break;
        default:
            UART_DBG(s, "ignored CR command 0x%x", (v >> 4) & 0xF);
            break;
        }
        break;
    case UART_IMR:
        s->imr = v;
        break;
    default:
        if (offset < UART_REGS_END) {
            s->regs[offset / 4] = v;    /* MR1, MR2, CSR, IPR, RFWR, ... */
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "qsd8x50-uart: write of unknown register 0x%02"
                          HWADDR_PRIx " <- 0x%08x\n", offset, v);
        }
        break;
    }
    uart_update(s);
}

static const MemoryRegionOps uart_ops = {
    .read = uart_read,
    .write = uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static int uart_can_receive(void *opaque)
{
    QSD8x50UARTState *s = opaque;

    return UART_FIFO_SIZE - s->rx_count;
}

static void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    QSD8x50UARTState *s = opaque;

    for (int i = 0; i < size; i++) {
        if (s->rx_count == UART_FIFO_SIZE) {
            s->overrun = true;
            break;
        }
        s->rx_fifo[(s->rx_head + s->rx_count) % UART_FIFO_SIZE] = buf[i];
        s->rx_count++;
    }
    s->stale = true;
    uart_update(s);
}

static void uart_reset(DeviceState *dev)
{
    QSD8x50UARTState *s = QSD8X50_UART(dev);

    memset(s->regs, 0, sizeof(s->regs));
    s->imr = 0;
    s->rx_head = 0;
    s->rx_count = 0;
    s->stale = false;
    s->overrun = false;
    uart_update(s);
}

static void uart_init(Object *obj)
{
    QSD8x50UARTState *s = QSD8X50_UART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s, TYPE_QSD8X50_UART,
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void uart_realize(DeviceState *dev, Error **errp)
{
    QSD8x50UARTState *s = QSD8X50_UART(dev);

    qemu_chr_fe_set_handlers(&s->chr, uart_can_receive, uart_receive,
                             NULL, NULL, s, NULL, true);
}

static const VMStateDescription vmstate_uart = {
    .name = TYPE_QSD8X50_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, QSD8x50UARTState, UART_REGS_END / 4),
        VMSTATE_UINT32(imr, QSD8x50UARTState),
        VMSTATE_UINT8_ARRAY(rx_fifo, QSD8x50UARTState, UART_FIFO_SIZE),
        VMSTATE_INT32(rx_head, QSD8x50UARTState),
        VMSTATE_INT32(rx_count, QSD8x50UARTState),
        VMSTATE_BOOL(stale, QSD8x50UARTState),
        VMSTATE_BOOL(overrun, QSD8x50UARTState),
        VMSTATE_END_OF_LIST()
    }
};

static const Property uart_properties[] = {
    DEFINE_PROP_CHR("chardev", QSD8x50UARTState, chr),
    DEFINE_PROP_BOOL("debug", QSD8x50UARTState, debug, false),
};

static void uart_class_init(ObjectClass *oc, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = uart_realize;
    device_class_set_legacy_reset(dc, uart_reset);
    dc->vmsd = &vmstate_uart;
    device_class_set_props(dc, uart_properties);
}

static const TypeInfo uart_info = {
    .name          = TYPE_QSD8X50_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(QSD8x50UARTState),
    .instance_init = uart_init,
    .class_init    = uart_class_init,
};

static void uart_register_types(void)
{
    type_register_static(&uart_info);
}

type_init(uart_register_types)
