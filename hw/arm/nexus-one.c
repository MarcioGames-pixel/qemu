/*
 * HTC/Google Nexus One (mahimahi) machine bring-up.
 *
 * Copyright (C) 2026 Nexus-One-QEMU contributors
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * This is an incremental QSD8x50 model, not yet a complete phone SoC.
 * The CSR counter and a minimal MSM UART3 register window are implemented
 * for early-kernel diagnostics. VIC, GPIO, clock/reset, NAND/MTD, display,
 * touchscreen, USB and the remaining SoC blocks are still outstanding.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "system/address-spaces.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/core/boards.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpu-qom.h"

#define NEXUS_ONE_RAM_BASE          0x20000000
#define NEXUS_ONE_KERNEL_LOAD_ADDR  0x20008000
#define NEXUS_ONE_MACHINE_ID        2304
#define NEXUS_ONE_DEFAULT_RAM       (256 * MiB)

/* QSD8x50 physical addresses from the legacy Linux msm_iomap-8x50.h. */
#define QSD8X50_CSR_BASE             0xAC100000
#define QSD8X50_UART3_BASE           0xA9C00000
#define QSD8X50_IO_SIZE              0x1000

/* Legacy MSM UART register offsets (word registers). */
#define MSM_UART_MR1                 0x00
#define MSM_UART_MR2                 0x04
#define MSM_UART_CSR                 0x08
#define MSM_UART_CR                  0x10
#define MSM_UART_IMR                 0x14
#define MSM_UART_ISR                 0x14
#define MSM_UART_TF                  0x0c
#define MSM_UART_SR                  0x08
#define MSM_UART_SR_TX_READY         (1U << 2)
#define MSM_UART_SR_TX_EMPTY         (1U << 3)

/* QSD8x50 GPT runs at 32.768 kHz; DGT runs at 19.2 MHz. */
#define QSD8X50_GPT_HZ               32768ULL
#define QSD8X50_DGT_HZ               19200000ULL
#define QSD8X50_DGT_SHIFT            5
#define QSD8X50_TIMER_MATCH          0x00
#define QSD8X50_TIMER_COUNT          0x04
#define QSD8X50_TIMER_ENABLE         0x08
#define QSD8X50_TIMER_CLEAR          0x0c
#define QSD8X50_DGT_BASE             0x10
#define QSD8X50_TIMER_PROTECTION     0x20

typedef struct NexusOneMachineState {
    MachineState parent;
    struct arm_boot_info bootinfo;
    MemoryRegion ram;
    MemoryRegion csr;
    MemoryRegion uart3;
    uint32_t uart_mr1;
    uint32_t uart_mr2;
    uint32_t uart_csr;
    uint32_t uart_imr;
    uint32_t uart_isr;
    uint32_t uart_tx_count;
    uint32_t csr_regs[QSD8X50_IO_SIZE / sizeof(uint32_t)];
    uint64_t gpt_epoch_ns;
    uint64_t dgt_epoch_ns;
    uint32_t gpt_match;
    uint32_t dgt_match;
    uint32_t gpt_enable;
    uint32_t dgt_enable;
    bool gpt_clear_on_match;
    bool dgt_clear_on_match;
} NexusOneMachineState;

#define TYPE_NEXUS_ONE_MACHINE MACHINE_TYPE_NAME("nexus-one")
OBJECT_DECLARE_SIMPLE_TYPE(NexusOneMachineState, NEXUS_ONE_MACHINE)

static uint32_t nexus_one_timer_count(uint64_t epoch_ns, uint64_t hz)
{
    uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint64_t elapsed_ns = now_ns >= epoch_ns ? now_ns - epoch_ns : 0;

    return (uint32_t)(elapsed_ns * hz / NANOSECONDS_PER_SECOND);
}

static uint64_t nexus_one_csr_read(void *opaque, hwaddr offset, unsigned size)
{
    NexusOneMachineState *s = opaque;
    uint32_t index = offset >> 2;

    if (size != 4 || (offset & 3) || offset >= QSD8X50_IO_SIZE) {
        return 0;
    }

    /* The MSM GPT occupies CSR + 0x00; the debug timer occupies + 0x10. */
    if (offset == QSD8X50_TIMER_COUNT) {
        return nexus_one_timer_count(s->gpt_epoch_ns, QSD8X50_GPT_HZ);
    }
    if (offset == QSD8X50_TIMER_ENABLE) {
        return s->gpt_enable;
    }
    if (offset == QSD8X50_DGT_BASE + QSD8X50_TIMER_COUNT) {
        return nexus_one_timer_count(s->dgt_epoch_ns, QSD8X50_DGT_HZ);
    }
    if (offset == QSD8X50_DGT_BASE + QSD8X50_TIMER_ENABLE) {
        return s->dgt_enable;
    }
    if (offset == QSD8X50_TIMER_MATCH) {
        return s->gpt_match;
    }
    if (offset == QSD8X50_DGT_BASE + QSD8X50_TIMER_MATCH) {
        return s->dgt_match;
    }
    return s->csr_regs[index];
}

static void nexus_one_csr_write(void *opaque, hwaddr offset, uint64_t value,
                                unsigned size)
{
    NexusOneMachineState *s = opaque;
    uint32_t index = offset >> 2;
    uint64_t now_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

    if (size != 4 || (offset & 3) || offset >= QSD8X50_IO_SIZE) {
        return;
    }

    switch (offset) {
    case QSD8X50_TIMER_MATCH:
        s->gpt_match = value;
        break;
    case QSD8X50_TIMER_COUNT:
        /* Writes to COUNT are ignored by the physical counter. */
        break;
    case QSD8X50_TIMER_ENABLE:
        s->gpt_enable = value & 3;
        s->gpt_clear_on_match = (value & 2) != 0;
        break;
    case QSD8X50_TIMER_CLEAR:
        s->gpt_epoch_ns = now_ns;
        break;
    case QSD8X50_DGT_BASE + QSD8X50_TIMER_MATCH:
        s->dgt_match = value;
        break;
    case QSD8X50_DGT_BASE + QSD8X50_TIMER_COUNT:
        break;
    case QSD8X50_DGT_BASE + QSD8X50_TIMER_ENABLE:
        s->dgt_enable = value & 3;
        s->dgt_clear_on_match = (value & 2) != 0;
        break;
    case QSD8X50_DGT_BASE + QSD8X50_TIMER_CLEAR:
        s->dgt_epoch_ns = now_ns;
        break;
    case QSD8X50_TIMER_PROTECTION:
        s->csr_regs[index] = value;
        break;
    default:
        s->csr_regs[index] = value;
        break;
    }
}

static const MemoryRegionOps nexus_one_csr_ops = {
    .read = nexus_one_csr_read,
    .write = nexus_one_csr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static uint64_t nexus_one_uart_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    NexusOneMachineState *s = opaque;

    if (size != 4 || (offset & 3) || offset >= QSD8X50_IO_SIZE) {
        return 0;
    }

    switch (offset) {
    case MSM_UART_MR1:
        return s->uart_mr1;
    case MSM_UART_MR2:
        return s->uart_mr2;
    case MSM_UART_CSR:
        /* SR is read at the CSR offset; CSR is write-only. */
        return MSM_UART_SR_TX_READY | MSM_UART_SR_TX_EMPTY;
    case MSM_UART_IMR:
        return s->uart_imr;
    case MSM_UART_ISR:
        return s->uart_isr;
    case MSM_UART_SR:
        /* Transmitter is always ready; no receive FIFO is modelled. */
        return MSM_UART_SR_TX_READY | MSM_UART_SR_TX_EMPTY;
    default:
        return 0;
    }
}

static void nexus_one_uart_write(void *opaque, hwaddr offset, uint64_t value,
                                 unsigned size)
{
    NexusOneMachineState *s = opaque;

    if (size != 4 || (offset & 3) || offset >= QSD8X50_IO_SIZE) {
        return;
    }

    switch (offset) {
    case MSM_UART_MR1:
        s->uart_mr1 = value;
        break;
    case MSM_UART_MR2:
        s->uart_mr2 = value;
        break;
    case MSM_UART_CSR:
        s->uart_csr = value;
        break;
    case MSM_UART_CR:
        /* Reset/enable commands are accepted; TX/RX remain polling-ready. */
        break;
    case MSM_UART_IMR:
        s->uart_imr = value;
        break;
    case MSM_UART_ISR:
        /* Interrupt status bits are write-one-to-clear. */
        s->uart_isr &= ~value;
        break;
    case MSM_UART_TF:
        /*
         * Until a QEMU CharBackend is wired up, make transmitted debug
         * characters visible when guest-error logging is enabled.
         */
        if ((value & 0xff) == '\n' || (value & 0xff) == '\r' ||
            (value & 0xff) == '\t' ||
            ((value & 0xff) >= 0x20 && (value & 0xff) <= 0x7e)) {
            qemu_log_mask(LOG_GUEST_ERROR, "nexus-one uart3: %c",
                          (char)(value & 0xff));
        }
        s->uart_tx_count++;
        break;
    default:
        break;
    }
}

static const MemoryRegionOps nexus_one_uart_ops = {
    .read = nexus_one_uart_read,
    .write = nexus_one_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
        .unaligned = false,
    },
};

static void nexus_one_init(MachineState *machine)
{
    NexusOneMachineState *s = NEXUS_ONE_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj;
    ARMCPU *cpu;

    if (!machine->cpu_type) {
        error_report("nexus-one: machine CPU type is not configured");
        exit(1);
    }
    cpuobj = object_new(machine->cpu_type);
    if (!object_dynamic_cast(cpuobj, TYPE_DEVICE)) {
        error_report("nexus-one: CPU type '%s' created QOM object '%s', "
                     "which is not a Device",
                     machine->cpu_type, object_get_typename(cpuobj));
        object_unref(cpuobj);
        exit(1);
    }

    memory_region_init_ram(&s->ram, NULL, "nexus-one.ram",
                           machine->ram_size, &error_fatal);
    memory_region_add_subregion(sysmem, NEXUS_ONE_RAM_BASE, &s->ram);

    memory_region_init_io(&s->csr, NULL, &nexus_one_csr_ops, s,
                          "nexus-one.qsd8x50-csr", QSD8X50_IO_SIZE);
    memory_region_add_subregion(sysmem, QSD8X50_CSR_BASE, &s->csr);

    memory_region_init_io(&s->uart3, NULL, &nexus_one_uart_ops, s,
                          "nexus-one.msm-uart3", QSD8X50_IO_SIZE);
    memory_region_add_subregion(sysmem, QSD8X50_UART3_BASE, &s->uart3);

    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);
    cpu = ARM_CPU(cpuobj);

    s->bootinfo.ram_size = machine->ram_size;
    s->bootinfo.board_id = NEXUS_ONE_MACHINE_ID;
    s->bootinfo.loader_start = NEXUS_ONE_RAM_BASE;
    s->bootinfo.kernel_load_addr = NEXUS_ONE_KERNEL_LOAD_ADDR;
    s->bootinfo.initrd_load_addr = 0x21000000;

    arm_load_kernel(cpu, machine, &s->bootinfo);
}

static void nexus_one_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "HTC/Google Nexus One (QSD8x50 early bring-up)";
    mc->init = nexus_one_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a8");
    mc->default_ram_size = NEXUS_ONE_DEFAULT_RAM;
}

static const TypeInfo nexus_one_machine_type = {
    .name = TYPE_NEXUS_ONE_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(NexusOneMachineState),
    .class_init = nexus_one_machine_class_init,
};

static void nexus_one_machine_register_types(void)
{
    type_register_static(&nexus_one_machine_type);
}

type_init(nexus_one_machine_register_types)
