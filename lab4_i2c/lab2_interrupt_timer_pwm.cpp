#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <cstdio>

#include <cstdint>

#include "../hal/timer.hpp"
#include "../hal/timer_util.hpp"
#include "lab2_timer.hpp"

extern "C" void initialise_monitor_handles(void);

namespace {

constexpr std::uint16_t full_scale = 65535;

constexpr std::uint32_t pwm_frequency = 1'000;

/// Steps in one leg of the wheel. Three legs make a full revolution.
constexpr std::uint16_t steps_per_leg = 256;
constexpr std::uint16_t phase_count = steps_per_leg * 3;

/// Time spent on each step. At 10 ms a revolution takes about 7.7 seconds,
/// and 100 updates per second is well past the point the eye sees stepping.
constexpr auto step_duration = std::chrono::milliseconds(10);

/// Brightness of the three channels at one point on the wheel.
struct rgb_level
{
  std::uint16_t red;
  std::uint16_t green;
  std::uint16_t blue;
};

constexpr rgb_level color_wheel(std::uint16_t p_phase)
{
  auto const leg = p_phase / steps_per_leg;
  auto const step = p_phase % steps_per_leg;

  auto const rising = static_cast<std::uint16_t>(step * 257);
  auto const falling = static_cast<std::uint16_t>(full_scale - rising);

  switch (leg) {
    case 0:
      return { falling, rising, 0 };
    case 1:
      return { 0, falling, rising };
    default:
      return { rising, 0, falling };
  }
}

}

int main()
{

  MAP_WDT_A_holdTimer();

  initialise_monitor_handles();

  timer_a_clock clock;

  configure_rgb_pwm_pins();

  timer_a_pwm red{ 1, pwm_frequency };
  timer_a_pwm green{ 3, pwm_frequency };
  timer_a_pwm blue{ 4, pwm_frequency };

  std::uint16_t phase = 0;

  while (true) {
    auto const level = color_wheel(phase);

    red.duty_cycle(level.red);
    green.duty_cycle(level.green);
    blue.duty_cycle(level.blue);

    phase = static_cast<std::uint16_t>((phase + 1) % phase_count);

    lab2::delay(clock, step_duration);
  }
}
