/*
 * HTC/Google Nexus One (mahimahi) machine - phase 1 boot bring-up.
 *
 * Copyright (C) 2026 Nexus-One-QEMU contributors
 *
 * This code is licensed under the GNU GPL, version 2 or later.
 *
 * Phase 1 provides just enough of the Qualcomm QSD8x50 SoC for the legacy
 * 2.6.29 Android kernel to start executing and print to its console:
 *
 *   - ARM CPU (Cortex-A8 stand-in for the Qualcomm Scorpion core)
 *   - RAM at 0x20000000
 *   - Vectored interrupt controller (qsd8x50_vic.c)
 *   - GPT/DGT timers (qsd8x50_timer.c)
 *   - UART1/2/3 (qsd8x50_uart.c)
 *   - A catch-all region that logs (-d unimp) every access to an address no
 *     model claims and returns 0, so that the kernel's probing can be
 *     mapped out one peripheral at a time.
 *
 * NOT yet emulated: GPIO, clocks/PLLs, proc_comm and SMD (shared-memory link
 * to the modem ARM9), NAND/MTD, MDDI/display, touchscreen, ADSP (audio and
 * camera), SD/MMC, USB, I2C, RTC. The machine is a bring-up tool, not a
 * working phone.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "system/address-spaces.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/arm/qsd8x50.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "hw/core/qdev-properties-system.h"
#include "hw/core/sysbus.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpu-qom.h"

#define NEXUS_ONE_RAM_BASE      0x20000000
#define NEXUS_ONE_KERNEL_LOAD_ADDR 0x20008000
#define NEXUS_ONE_MACHINE_ID    2304
#define NEXUS_ONE_DEFAULT_RAM   (256 * MiB)

/* Priority below every real device so it only sees otherwise-unmapped I/O */
#define NEXUS_ONE_CATCHALL_PRIO (-1000)

typedef struct NexusOneMachineState {
    MachineState parent;
    struct arm_boot_info bootinfo;
    MemoryRegion catchall;
} NexusOneMachineState;

#define TYPE_NEXUS_ONE_MACHINE MACHINE_TYPE_NAME("nexus-one")
OBJECT_DECLARE_SIMPLE_TYPE(NexusOneMachineState, NEXUS_ONE_MACHINE)

static uint64_t nexus_one_catchall_read(void *opaque, hwaddr addr,
                                        unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "nexus-one: unmapped read%u  @0x%08" HWADDR_PRIx
                  "\n", size * 8, addr);
    return 0;
}

static void nexus_one_catchall_write(void *opaque, hwaddr addr,
                                     uint64_t value, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "nexus-one: unmapped write%u @0x%08" HWADDR_PRIx
                  " <- 0x%" PRIx64 "\n", size * 8, addr, value);
}

static const MemoryRegionOps nexus_one_catchall_ops = {
    .read = nexus_one_catchall_read,
    .write = nexus_one_catchall_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

static DeviceState *nexus_one_create_uart(hwaddr base, DeviceState *vic,
                                          int irq, int serial_idx)
{
    DeviceState *dev = qdev_new(TYPE_QSD8X50_UART);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    qdev_prop_set_chr(dev, "chardev", serial_hd(serial_idx));
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, base);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(vic, irq));
    return dev;
}

static void nexus_one_init(MachineState *machine)
{
    NexusOneMachineState *s = NEXUS_ONE_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj = object_new(machine->cpu_type);
    ARMCPU *cpu = ARM_CPU(cpuobj);
    DeviceState *vic, *timer;
    SysBusDevice *sbd;

    /*
     * Catch-all for the whole 32-bit physical space. Everything real is
     * mapped on top of it with a higher priority.
     */
    memory_region_init_io(&s->catchall, OBJECT(machine),
                          &nexus_one_catchall_ops, NULL,
                          "nexus-one.catchall", 4 * GiB);
    memory_region_add_subregion_overlap(sysmem, 0, &s->catchall,
                                        NEXUS_ONE_CATCHALL_PRIO);

    /*
     * The original Nexus One kernel expects physical RAM to start at
     * 0x20000000.
     */
    memory_region_add_subregion(sysmem, NEXUS_ONE_RAM_BASE, machine->ram);

    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

    /* Interrupt controller -> CPU IRQ/FIQ */
    vic = qdev_new(TYPE_QSD8X50_VIC);
    sbd = SYS_BUS_DEVICE(vic);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, QSD8X50_VIC_BASE);
    sysbus_connect_irq(sbd, 0, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(sbd, 1, qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ));

    /* GPT + DGT timers */
    timer = qdev_new(TYPE_QSD8X50_TIMER);
    sbd = SYS_BUS_DEVICE(timer);
    sysbus_realize_and_unref(sbd, &error_fatal);
    sysbus_mmio_map(sbd, 0, QSD8X50_CSR_BASE);
    sysbus_connect_irq(sbd, 0,
                       qdev_get_gpio_in(vic, QSD8X50_INT_GP_TIMER_EXP));
    sysbus_connect_irq(sbd, 1,
                       qdev_get_gpio_in(vic, QSD8X50_INT_DEBUG_TIMER_EXP));

    /* UART1 is the kernel console; -serial N maps to UART(N+1) */
    nexus_one_create_uart(QSD8X50_UART1_BASE, vic, QSD8X50_INT_UART1, 0);
    nexus_one_create_uart(QSD8X50_UART2_BASE, vic, QSD8X50_INT_UART2, 1);
    nexus_one_create_uart(QSD8X50_UART3_BASE, vic, QSD8X50_INT_UART3, 2);

    s->bootinfo.ram_size = machine->ram_size;
    s->bootinfo.board_id = NEXUS_ONE_MACHINE_ID;
    s->bootinfo.loader_start = NEXUS_ONE_RAM_BASE;
    s->bootinfo.kernel_load_addr = NEXUS_ONE_KERNEL_LOAD_ADDR;
    /* Original boot.img ramdisk address from the Nexus One header. */
    s->bootinfo.initrd_load_addr = 0x21000000;

    /*
     * -kernel/-initrd/-append boot experiment. The kernel is expected to
     * advance past early init and then stop at the next missing peripheral;
     * use "-d unimp,guest_errors -D qemu.log" to see where.
     */
    arm_load_kernel(cpu, machine, &s->bootinfo);
}

static void nexus_one_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "HTC/Google Nexus One (QSD8x50 phase 1 boot bring-up)";
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
