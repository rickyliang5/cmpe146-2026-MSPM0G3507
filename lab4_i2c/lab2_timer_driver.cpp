#include "lab2_timer.hpp"

#include <cstddef>
#include <cstdint>

namespace {

constexpr std::uintptr_t timer_a0_base = 0x4000'0000;

/**
 * @brief Frequency of SMCLK in hertz.
 *
 * SMCLK is sourced from the DCO out of reset, and SystemInit() leaves DCORSEL
 * in its 3 MHz nominal range with DIVS set to /1. Nothing in this project
 * reconfigures the Clock System, so this value holds for the life of the
 * program. See README.md, "HINT: FREQUENCY MATH".
 */
constexpr std::uint32_t smclk_frequency = 3'000'000;

// TAxCTL fields. TRM section 17.3.1.
/// TASSEL (bits 9:8) - clock source select.
constexpr std::uint16_t tassel_smclk = 0b10 << 8;
/// ID (bits 7:6) - input divider.
constexpr std::uint16_t id_divide_1 = 0b00 << 6;
/// MC (bits 5:4) - mode control. Continuous counts up to 0xFFFF and wraps.
constexpr std::uint16_t mc_continuous = 0b10 << 4;
/// TACLR (bit 2) - resets TAxR and the divider. Self-clearing.
constexpr std::uint16_t taclr = 1 << 2;
/// TAIFG (bit 0) - set when the counter wraps. Cleared by software.
constexpr std::uint16_t taifg = 1 << 0;

/// Number of ticks the 16-bit counter covers before it wraps.
constexpr std::uint64_t counter_period = 0x1'0000;

struct timer_a_registers
{
  std::uint16_t volatile ctl;
  std::uint16_t volatile cctl[5];
  std::uint16_t volatile reserved0[2];
  std::uint16_t volatile r;
  std::uint16_t volatile ccr[5];
};



auto* const timer_a0 = reinterpret_cast<timer_a_registers*>(timer_a0_base);

}

timer_a_clock::timer_a_clock()
{

  timer_a0->ctl = tassel_smclk | id_divide_1 | mc_continuous | taclr;

  m_frequency = smclk_frequency / 1;
}

std::uint32_t timer_a_clock::driver_frequency()
{
  return m_frequency;
}

std::uint64_t timer_a_clock::driver_uptime()
{

  auto count = static_cast<std::uint64_t>(timer_a0->r);

  if ((timer_a0->ctl & taifg) != 0) {
    timer_a0->ctl &= static_cast<std::uint16_t>(~taifg);
    m_rollovers++;
    count = static_cast<std::uint64_t>(timer_a0->r);
  }

  return (m_rollovers << 16) |  count;
}

namespace {

constexpr std::uintptr_t timer_a2_base = 0x4000'0800;
/// Base address of the P5/P6 register pair. Datasheet Table 6-21, where P5IN
/// sits at offset 040h.
constexpr std::uintptr_t port_pair_base = 0x4000'4C40;

constexpr std::uint16_t mc_up = 0b01 << 4;

constexpr std::uint16_t outmod_out_bit = 0b000 << 5;

constexpr std::uint16_t outmod_reset_set = 0b111 << 5;

constexpr std::uint16_t out_high = 1 << 2;

constexpr std::uint32_t full_scale = 65535;

struct port_pair_registers
{
  std::uint8_t volatile in[2];
  std::uint8_t volatile out[2];
  std::uint8_t volatile dir[2];
  std::uint8_t volatile ren[2];
  std::uint8_t volatile ds[2];
  std::uint8_t volatile sel0[2];
  std::uint8_t volatile sel1[2];
};


constexpr std::size_t port5 = 0;
constexpr std::size_t port6 = 1;

constexpr std::uint8_t p5_pwm_mask = 1 << 6;

constexpr std::uint8_t p6_pwm_mask = (1 << 6) | (1 << 7);

auto* const timer_a2 = reinterpret_cast<timer_a_registers*>(timer_a2_base);
auto* const port_pair = reinterpret_cast<port_pair_registers*>(port_pair_base);

}

void configure_rgb_pwm_pins()
{

  port_pair->sel0[port5] |= p5_pwm_mask;
  port_pair->dir[port5] |= p5_pwm_mask;

  port_pair->sel0[port6] |= p6_pwm_mask;
  port_pair->dir[port6] |= p6_pwm_mask;
}

timer_a_pwm::timer_a_pwm(std::uint8_t p_channel, std::uint32_t p_frequency_hz)
{

  m_channel = p_channel;

  auto const ticks_per_cycle = smclk_frequency / p_frequency_hz;
  timer_a2->ccr[0] = static_cast<std::uint16_t>(ticks_per_cycle - 1);

  m_frequency = smclk_frequency / ticks_per_cycle;

  timer_a2->ctl = tassel_smclk | id_divide_1 | mc_up | taclr;

  timer_a2->cctl[m_channel] = outmod_out_bit;
}

std::uint32_t timer_a_pwm::driver_frequency()
{
  return m_frequency;
}

void timer_a_pwm::driver_duty_cycle(std::uint16_t p_duty_cycle)
{

  if (p_duty_cycle == 0) {
    timer_a2->cctl[m_channel] = outmod_out_bit;
    return;
  }

  if (p_duty_cycle == full_scale) {
    timer_a2->cctl[m_channel] = outmod_out_bit | out_high;
    return;
  }

  auto const ccr0 = static_cast<std::uint32_t>(timer_a2->ccr[0]);
  auto const ticks = (ccr0 * p_duty_cycle) / full_scale;

  timer_a2->ccr[m_channel] = static_cast<std::uint16_t>(ticks);
  timer_a2->cctl[m_channel] = outmod_reset_set;
}
