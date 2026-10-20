/*
 * HTC/Google Nexus One (mahimahi) machine bring-up scaffold.
 *
 * Copyright (C) 2026 Nexus-One-QEMU contributors
 *
 * This code is licensed under the GNU GPL, version 2 or later.
 *
 * IMPORTANT: This is not yet a complete QSD8x50 emulation. It only provides
 * the ARM CPU, the initial RAM window, and legacy Linux kernel loading. The
 * Qualcomm interrupt controller, timers, UART, GPIO, NAND/MTD, display,
 * touchscreen and other SoC peripherals still need real device models.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/core/boards.h"
#include "system/system.h"
#include "target/arm/cpu.h"
#include "target/arm/cpu-qom.h"

#define NEXUS_ONE_RAM_BASE      0x20000000
#define NEXUS_ONE_MACHINE_ID    2304
#define NEXUS_ONE_DEFAULT_RAM   (256 * MiB)

typedef struct NexusOneMachineState {
    MachineState parent;
    struct arm_boot_info bootinfo;
} NexusOneMachineState;

#define TYPE_NEXUS_ONE_MACHINE MACHINE_TYPE_NAME("nexus-one")
OBJECT_DECLARE_SIMPLE_TYPE(NexusOneMachineState, NEXUS_ONE_MACHINE)

static void nexus_one_init(MachineState *machine)
{
    NexusOneMachineState *s = NEXUS_ONE_MACHINE(machine);
    MemoryRegion *sysmem = get_system_memory();
    Object *cpuobj = object_new(machine->cpu_type);
    ARMCPU *cpu = ARM_CPU(cpuobj);

    /*
     * The original Nexus One kernel expects physical RAM to start at
     * 0x20000000. This RAM mapping is grounded in the legacy QSD8K kernel
     * support; no Qualcomm-specific peripherals are claimed here.
     */
    memory_region_add_subregion(sysmem, NEXUS_ONE_RAM_BASE, machine->ram);

    if (object_property_find(cpuobj, "has_el3")) {
        object_property_set_bool(cpuobj, "has_el3", false, &error_fatal);
    }
    qdev_realize(DEVICE(cpuobj), NULL, &error_fatal);

    s->bootinfo.ram_size = machine->ram_size;
    s->bootinfo.board_id = NEXUS_ONE_MACHINE_ID;
    s->bootinfo.loader_start = NEXUS_ONE_RAM_BASE;

    /*
     * This enables a first boot experiment with -kernel/-initrd. Expect the
     * original Android kernel to stop early until its QSD8x50 peripherals
     * and correct boot protocol are emulated.
     */
    arm_load_kernel(cpu, machine, &s->bootinfo);
}

static void nexus_one_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "HTC/Google Nexus One (QSD8x50 bring-up scaffold)";
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
