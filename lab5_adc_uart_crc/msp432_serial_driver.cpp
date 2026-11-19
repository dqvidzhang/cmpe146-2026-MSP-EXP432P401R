// cspell:ignore EUSCIA RXIFG TXIFG RXBUF TXBUF CTLW MCTL BRW PRIMASK
#include "msp432_serial_driver.hpp"

#include <ti/devices/msp432p4xx/inc/msp.h>

msp432_serial_driver::msp432_serial_driver(std::uint32_t p_smclk_hz,
                                           settings const& p_settings)
  : m_smclk_hz(p_smclk_hz)
{
  driver_configure(p_settings);
}

msp432_serial_driver::~msp432_serial_driver()
{
  auto const mask = __get_PRIMASK();
  __disable_irq();
  if (m_owner == this) {
    EUSCI_A2->IE = 0;
    NVIC_DisableIRQ(EUSCIA2_IRQn);
    NVIC_ClearPendingIRQ(EUSCIA2_IRQn);
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST;
    m_owner = nullptr;
  }
  __set_PRIMASK(mask);
}

bool msp432_serial_driver::ready() const
{
  return m_owner == this;
}

bool msp432_serial_driver::driver_configure(settings const& p_settings)
{
  if (m_smclk_hz != 3'000'000 || p_settings.baud_rate != 9'600 ||
      p_settings.parity != decltype(p_settings.parity)::none ||
      p_settings.stop != settings::stop_bits::one) {
    return false;
  }

  auto const mask = __get_PRIMASK();
  __disable_irq();
  if (m_owner != nullptr && m_owner != this) {
    __set_PRIMASK(mask);
    return false;
  }
  EUSCI_A2->IE = 0;
  EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;
  // P3.2 = RX (J1.3), P3.3 = TX (J1.4), primary module function.
  constexpr auto uart_pins = 0b11u << 2;
  P3->SEL0 |= uart_pins;
  P3->SEL1 &= ~uart_pins;
  // TI settings for 3 MHz SMCLK, 9600 baud: BRW=19, BRF=8, BRS=0x55, OS16=1.
  EUSCI_A2->BRW = 19;
  EUSCI_A2->MCTLW = 0x5581;
  m_cursor = 0;
  m_owner = this;
  NVIC_ClearPendingIRQ(EUSCIA2_IRQn);
  // SYNC/MODE/MSB/SEVENBIT remain clear: asynchronous, LSB first, 8 bits.
  EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
  EUSCI_A2->IE = EUSCI_A_IE_RXIE;
  NVIC_EnableIRQ(EUSCIA2_IRQn);
  __set_PRIMASK(mask);
  return true;
}

void msp432_serial_driver::driver_write(std::span<std::uint8_t const> p_data)
{
  if (!ready()) {
    return;
  }
  for (auto const byte : p_data) {
    while ((EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG) == 0) {
      // Polled transmit: no DriverLib, TX interrupt or FreeRTOS calls.
    }
    EUSCI_A2->TXBUF = byte;
  }
}

void msp432_serial_driver::handle_interrupt()
{
  if (m_owner == nullptr) {
    EUSCI_A2->IE &= ~EUSCI_A_IE_RXIE;
    return;
  }
  // Reading RXBUF acknowledges RXIFG. Drain any byte arriving during the ISR.
  while ((EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) != 0) {
    auto const cursor = m_owner->m_cursor;
    m_owner->m_buffer[cursor] = static_cast<std::uint8_t>(EUSCI_A2->RXBUF);
    // Publish the byte before advancing the next-write cursor.
    __DMB();
    m_owner->m_cursor = (cursor + 1) % buffer_size;
  }
}

std::span<std::uint8_t const> msp432_serial_driver::driver_receive_buffer()
{
  return { m_buffer.data(), m_buffer.size() };
}

std::size_t msp432_serial_driver::driver_receive_cursor()
{
  auto const cursor = m_cursor;
  __DMB();
  return cursor;
}

extern "C" void EUSCIA2_IRQHandler()
{
  msp432_serial_driver::handle_interrupt();
}
