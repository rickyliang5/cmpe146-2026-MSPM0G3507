#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "../hal/accelerometer.hpp"
#include "../hal/i2c.hpp"
#include "../hal/timer_util.hpp"
#include "../lab2_interrupt_timer_pwm/lab2_timer.hpp"

using namespace std::chrono_literals;

#define UCB0_BASE   0x40002000
#define B0_CTLW0    (*(volatile uint16_t *)(UCB0_BASE + 0x00))
#define B0_BRW      (*(volatile uint16_t *)(UCB0_BASE + 0x06))
#define B0_RXBUF    (*(volatile uint16_t *)(UCB0_BASE + 0x0C))
#define B0_TXBUF    (*(volatile uint16_t *)(UCB0_BASE + 0x0E))
#define B0_I2CSA    (*(volatile uint16_t *)(UCB0_BASE + 0x20))
#define B0_IFG      (*(volatile uint16_t *)(UCB0_BASE + 0x2C))

#define P1_BASE     0x40004C00
#define P1_SEL0     (*(volatile uint8_t *)(P1_BASE + 0x0A))
#define P1_SEL1     (*(volatile uint8_t *)(P1_BASE + 0x0C))

#define B0_SSEL_SMCLK (2 << 6)
#define B0_MODE_I2C   (3 << 9)

#define BMI160_I2C_ADDR_SDO_GND   0x69
#define BMI160_I2C_ADDR_SDO_VDDIO 0x69

#define BMI160_REG_CHIP_ID     0x00
#define BMI160_REG_ERR_REG     0x02
#define BMI160_REG_PMU_STATUS  0x03
#define BMI160_REG_DATA_GYR_X  0x0C
#define BMI160_REG_DATA_ACC_X  0x12
#define BMI160_REG_STATUS      0x1B
#define BMI160_REG_ACC_CONF    0x40
#define BMI160_REG_ACC_RANGE   0x41
#define BMI160_REG_GYR_CONF    0x42
#define BMI160_REG_GYR_RANGE   0x43
#define BMI160_REG_CMD         0x7E
#define BMI160_REG_SPI_DUMMY   0x7F

#define BMI160_CHIP_ID_VAL     0xD1
#define BMI160_PMU_ACC_GYR_NORMAL 0x14

#define BMI160_CMD_SOFTRESET       0xB6
#define BMI160_CMD_ACC_SUSPEND     0x10
#define BMI160_CMD_ACC_NORMAL      0x11
#define BMI160_CMD_ACC_LOWPOWER    0x12
#define BMI160_CMD_GYR_SUSPEND     0x14
#define BMI160_CMD_GYR_NORMAL      0x15
#define BMI160_CMD_GYR_FASTSTARTUP 0x17

#define BMI160_STATUS_DRDY_ACC  (1 << 7)
#define BMI160_STATUS_DRDY_GYR  (1 << 6)

#define BMI160_ACC_RANGE_2G    0x03
#define BMI160_ACC_RANGE_4G    0x05
#define BMI160_ACC_RANGE_8G    0x08
#define BMI160_ACC_RANGE_16G   0x0C

#define BMI160_GYR_RANGE_2000  0x00
#define BMI160_GYR_RANGE_1000  0x01
#define BMI160_GYR_RANGE_500   0x02
#define BMI160_GYR_RANGE_250   0x03
#define BMI160_GYR_RANGE_125   0x04

#define BMI160_CONF_100HZ      0x28

#define BMI160_ACC_STARTUP_MS  4
#define BMI160_GYR_STARTUP_MS  81
#define BMI160_SUSPEND_WRITE_DELAY_US 450

extern "C" void initialise_monitor_handles(void);

class msp432_i2c : public lab4::i2c
{
public:
  msp432_i2c()
  {
    driver_configure(settings{});
  }

private:
  bool driver_configure(settings const& p_settings) override
  {
    B0_CTLW0 = UCSWRST;
    B0_CTLW0 |= UCMST | B0_MODE_I2C
              | UCSYNC | B0_SSEL_SMCLK;
    P1_SEL0 |= (1 << 6) | (1 << 7);
    P1_SEL1 &= ~((1 << 6) | (1 << 7));
    B0_CTLW0 &= ~UCSWRST;
    return true;
  }

  bool driver_transaction(std::uint8_t p_address,
                          std::span<std::uint8_t const> p_data_out,
                          std::span<std::uint8_t> p_data_in) override
  {
    while (B0_CTLW0 & UCTXSTP);
    B0_IFG &= ~UCTXIFG0;
    B0_I2CSA = p_address;

    if (!p_data_out.empty()) {
      B0_CTLW0 |= UCTR | UCTXSTT;
      while (!(B0_IFG & UCTXIFG0));
      B0_TXBUF = p_data_out.data()[0];
      while (B0_CTLW0 & UCTXSTT);
      if (B0_IFG & UCNACKIFG) {
        return nack();
      }

      for (std::size_t i = 1; i < p_data_out.size(); i++) {
        while (!(B0_IFG & (UCTXIFG0 | UCNACKIFG)));
        if (B0_IFG & UCNACKIFG) {
          return nack();
        }
        B0_TXBUF = p_data_out.data()[i];
      }

      while (!(B0_IFG & (UCTXIFG0 | UCNACKIFG)));
      if (B0_IFG & UCNACKIFG) {
        return nack();
      }
    }

    if (!p_data_in.empty()) {
      B0_CTLW0 &= ~UCTR;
      B0_CTLW0 |= UCTXSTT;
      while (B0_CTLW0 & UCTXSTT);
      if (B0_IFG & UCNACKIFG) {
        return nack();
      }

      for (std::size_t i = 0; i < p_data_in.size(); i++) {
        if (i == p_data_in.size() - 1) {
          B0_CTLW0 |= UCTXSTP;
        }
        while (!(B0_IFG & UCRXIFG0));
        p_data_in.data()[i] = B0_RXBUF;
      }
    } else {
      B0_CTLW0 |= UCTXSTP;
    }

    while (B0_CTLW0 & UCTXSTP);
    return true;
  }

  bool nack()
  {
    B0_CTLW0 |= UCTXSTP;
    B0_IFG &= ~UCNACKIFG;
    return false;
  }
};

class bmi160 : public lab4::accelerometer
{
public:
  bmi160(lab4::i2c& i2c, timer_a_clock& clock)
    : m_i2c(i2c)
  {
    writeReg(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
    lab2::delay(clock, 4ms);

    writeReg(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_2G);
    writeReg(BMI160_REG_ACC_CONF, BMI160_CONF_100HZ);
  }

  bool chipId(uint8_t* id)
  {
    return readReg(BMI160_REG_CHIP_ID, id);
  }

  // Roll = rotation about X. Tilting that way moves gravity between Y and Z,
  // so the angle is atan(y / z). atan2 handles z = 0 and uses the signs of
  // both to give the full -180 to +180 range.
  // * 180 / pi converts radians to degrees.
  float roll(read_t acc)
  {
    return std::atan2(acc.y, acc.z) * 180.0f / 3.14159f;
  }

  // Pitch = rotation about Y. Tilting that way moves gravity onto X.
  // Opposite side = x, adjacent side = sqrt(y^2 + z^2), the part of gravity
  // still in the Y-Z plane. Using sqrt(y^2 + z^2) instead of just z keeps
  // pitch correct when the board is also rolled.
  // -x makes nose-down positive.
  float pitch(read_t acc)
  {
    return std::atan2(-acc.x, std::sqrt(acc.y * acc.y + acc.z * acc.z))
           * 180.0f / 3.14159f;
  }

private:
  read_t driver_read() override
  {
    uint8_t raw[6] = {};
    readReg(BMI160_REG_DATA_ACC_X, raw, 6);

    read_t acc;
    acc.x = toG(raw[0], raw[1]);
    acc.y = toG(raw[2], raw[3]);
    acc.z = toG(raw[4], raw[5]);
    return acc;
  }

  float toG(uint8_t lsb, uint8_t msb)
  {
    // Join the two bytes into 16 bits. The int16_t cast makes the top bit a
    // sign bit, so 0xE000 reads as -8192 instead of 57344.
    int16_t counts = (int16_t)((msb << 8) | lsb);

    // At +/-2 g the 16-bit range -32768..32767 covers -2 g..+2 g,
    // so 1 g = 32768 / 2 = 16384 counts. The .0f forces float division,
    // otherwise integer division would round everything to 0 or +/-1.
    return counts / 16384.0f;
  }

  bool readReg(uint8_t reg, uint8_t* buff, std::size_t count = 1)
  {
    uint8_t out[1] = { reg };
    return m_i2c.transaction(BMI160_I2C_ADDR_SDO_GND,
                             std::span<uint8_t const>(out, 1),
                             std::span<uint8_t>(buff, count));
  }

  bool writeReg(uint8_t reg, uint8_t value)
  {
    uint8_t out[2] = { reg, value };
    return m_i2c.transaction(BMI160_I2C_ADDR_SDO_GND,
                             std::span<uint8_t const>(out, 2),
                             std::span<uint8_t>());
  }

  lab4::i2c& m_i2c;
};

int main()
{
  MAP_WDT_A_holdTimer();

  initialise_monitor_handles();

  std::printf("Hello, World\n");

  timer_a_clock clock;
  msp432_i2c i2c;
  bmi160 imu(i2c, clock);

  uint8_t id = 0;
  bool ok = imu.chipId(&id);
  std::printf("chip ID: %s 0x%02X (expect 0xD1)\n", ok ? "ok" : "failed", id);

  while (true) {
    lab4::accelerometer::read_t acc = imu.read();

    std::printf("roll: %d  pitch: %d\n", (int)imu.roll(acc), (int)imu.pitch(acc));

    lab2::delay(clock, 100ms);
  }
}
