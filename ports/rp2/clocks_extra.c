/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "pico.h"
#include "clocks_extra.h"
#include "rp2_flash.h"
#include "hardware/regs/clocks.h"
#include "hardware/platform_defs.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/ticks.h"
#include "hardware/vreg.h"

#if PICO_RP2040
// The RTC clock frequency is 48MHz divided by power of 2 (to ensure an integer
// division ratio will be used in the clocks block).  A divisor of 1024 generates
// an RTC clock tick of 46875Hz.  This frequency is relatively close to the
// customary 32 or 32.768kHz 'slow clock' crystals and provides good timing resolution.
#define RTC_CLOCK_FREQ_HZ       (USB_CLK_KHZ * KHZ / 1024)
#endif

static void start_all_ticks(void) {
    uint32_t cycles = clock_get_hz(clk_ref) / MHZ;
    // Note RP2040 has a single tick generator in the watchdog which serves
    // watchdog, system timer and M0+ SysTick; The tick generator is clocked from clk_ref
    // but is now adapted by the hardware_ticks library for compatibility with RP2350
    // npte: hardware_ticks library now provides an adapter for RP2040

    for (int i = 0; i < (int)TICK_COUNT; ++i) {
        tick_start((tick_gen_num_t)i, cycles);
    }
}

// We ignore SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST and SYS_CLK_VREG_VOLTAGE_MIN from the
// Pico SDK and instead just pick a suitable voltage here based on the requested frequency.
static void rp2_set_vsel_for_freq(uint32_t freq) {
    if (freq <= 125 * MHZ) {
        vreg_set_voltage(VREG_VOLTAGE_1_10);
    } else {
        vreg_set_voltage(VREG_VOLTAGE_1_15);
    }
    // TODO: think about the value and execution of this delay
    busy_wait_at_least_cycles((uint32_t)((SYS_CLK_VREG_VOLTAGE_AUTO_ADJUST_DELAY_US * (uint64_t)XOSC_HZ) / 1000000));
}

bool rp2_set_freq(uint32_t freq) {
    // Figure out PLL settings for requested frequency.
    // TODO: how long does this take when clk_sys=12 MHz?
    uint vco_freq, postdiv1, postdiv2;
    if (!check_sys_clock_hz(freq, &vco_freq, &postdiv1, &postdiv2)) {
        return false;
    }
    freq = vco_freq / (postdiv1 * postdiv2);

    uint32_t old_freq = clock_get_hz(clk_sys);   // TODO: reading an uninitialized var here?
    uint32_t max_freq = MAX(freq, old_freq);
    rp2_set_vsel_for_freq(max_freq);
    rp2_flash_set_timing_for_freq(max_freq);

    // Before we touch PLL_SYS, set CLK_SYS = CLK_REF (12 MHz) to avoid glitches.
    // TODO: why not use clock_configure_undivided?
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1)
        tight_loop_contents();

    pll_init(pll_sys, PLL_SYS_REFDIV, vco_freq, postdiv1, postdiv2);

    // CLK SYS = PLL SYS
    clock_configure_undivided(clk_sys,
        CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
        CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
        freq);

    freq = clock_get_hz(clk_sys);
    rp2_set_vsel_for_freq(freq);
    rp2_flash_set_timing_for_freq(freq);
    return true;
}

// Wrap the SDK's clocks_init() function to save code size
void __wrap_runtime_init_clocks(void) {
    runtime_init_clocks_optional_usb(true);
}

void runtime_init_clocks_optional_usb(bool init_usb) {
    // Disable resus that may be enabled from previous software
    clocks_hw->resus.ctrl = 0;

    // Enable the xosc
    xosc_init();

    // CLK_REF = XOSC (usually 12 MHz)
    clock_configure_undivided(clk_ref,
        CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
        0,
        XOSC_HZ);

    rp2_set_freq(SYS_CLK_HZ);

    if (init_usb) {
        pll_init(pll_usb, PLL_COMMON_REFDIV, PLL_USB_VCO_FREQ_HZ, PLL_USB_POSTDIV1, PLL_USB_POSTDIV2);

        // CLK_USB = PLL_USB (48 MHz).
        clock_configure_undivided(clk_usb,
            0,
            CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
            USB_CLK_HZ);

        // CLK_ADC = PLL_USB (48 MHz).
        clock_configure_undivided(clk_adc,
            0,
            CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
            USB_CLK_HZ);

        // CLK_PERI = PLL_USB (48 MHz).
        // Used as reference clock for UART and SPI serial.
        // Allows us to change clk_sys later without affecting these peripherals.
        clock_configure_undivided(clk_peri,
            0,
            CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
            USB_CLK_HZ);
    }

    #if HAS_RP2040_RTC
    // CLK RTC = PLL USB 48MHz / 1024 = 46875Hz
    #if (USB_CLK_HZ % RTC_CLOCK_FREQ_HZ == 0)
    // this doesn't pull in 64 bit arithmetic
    clock_configure_int_divider(clk_rtc,
        0, // No GLMUX
        CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        USB_CLK_HZ,
        USB_CLK_HZ / RTC_CLOCK_FREQ_HZ);
    #else
    clock_configure(clk_rtc,
        0, // No GLMUX
        CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
        USB_CLK_HZ,
        RTC_CLOCK_FREQ_HZ);
    #endif
    #endif

    #if PICO_RP2350
    // CLK_HSTX = clk_sys. Transmit bit clock for the HSTX peripheral.
    clock_configure(clk_hstx,
        0,
        CLOCKS_CLK_HSTX_CTRL_AUXSRC_VALUE_CLK_SYS,
        SYS_CLK_KHZ * KHZ,
        SYS_CLK_KHZ * KHZ);
    #endif

    // Finally, all clocks are configured so start the ticks
    // The ticks use clk_ref so now that is configured we can start them
    start_all_ticks();
}
