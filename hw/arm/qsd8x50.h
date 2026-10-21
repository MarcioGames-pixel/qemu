/*
 * Qualcomm QSD8x50 (Snapdragon S1) SoC definitions for the Nexus One
 * (mahimahi) machine.
 *
 * Copyright (C) 2026 Nexus-One-QEMU contributors
 *
 * This code is licensed under the GNU GPL, version 2 or later.
 *
 * !!! UNVERIFIED CONSTANTS !!!
 * The physical addresses and IRQ numbers below were written from memory of
 * the legacy android-msm 2.6.29 kernel (arch/arm/mach-msm/include/mach/
 * msm_iomap-8x50.h and irqs-8x50.h). They MUST be checked against the kernel
 * tree that matches your EPE54B boot.img before being trusted. Running with
 * "-d unimp -D qemu.log" shows every access to an address that no model
 * claims, which is the fastest way to find a wrong constant.
 */

#ifndef HW_ARM_QSD8X50_H
#define HW_ARM_QSD8X50_H

/* ---- Physical memory map (msm_iomap-8x50.h) ---- */
#define QSD8X50_VIC_BASE        0xAC000000  /* vectored interrupt ctrl */
#define QSD8X50_CSR_BASE        0xAC100000  /* GPT/DGT timers */
#define QSD8X50_UART1_BASE      0xA9A00000  /* debug/console UART (non-DM) */
#define QSD8X50_UART2_BASE      0xA9B00000
#define QSD8X50_UART3_BASE      0xA9C00000

/* ---- Interrupt numbers (irqs-8x50.h), index into the 64-line VIC ---- */
#define QSD8X50_INT_GP_TIMER_EXP     7
#define QSD8X50_INT_DEBUG_TIMER_EXP  8
#define QSD8X50_INT_UART1            9
#define QSD8X50_INT_UART2            10
#define QSD8X50_INT_UART3            11

/* ---- QOM type names ---- */
#define TYPE_QSD8X50_VIC        "qsd8x50-vic"
#define TYPE_QSD8X50_TIMER      "qsd8x50-timer"
#define TYPE_QSD8X50_UART       "qsd8x50-uart"

#endif /* HW_ARM_QSD8X50_H */
