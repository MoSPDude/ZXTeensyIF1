/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

class HardwareSerialIMXRTPublic : public HardwareSerial
{
public:
	static const uint8_t cnt_tx_pins = 2;
	static const uint8_t cnt_rx_pins = 2;
	typedef struct {
		const uint8_t 		pin;		// The pin number
		const uint32_t 		mux_val;	// Value to set for mux;
		volatile uint32_t	*select_input_register; // Which register controls the selection
		const uint32_t		select_val;	// Value for that selection
	} pin_info_t;

	typedef struct {
		uint8_t serial_index;	// which object are we? 0 based
		IRQ_NUMBER_t irq;
		void (*irq_handler)(void);
		void (* _serialEvent)(void);
		volatile uint32_t &ccm_register;
		const uint32_t ccm_value;
		pin_info_t rx_pins[cnt_rx_pins];
		pin_info_t tx_pins[cnt_tx_pins];
		const uint8_t cts_pin;
		const uint8_t cts_mux_val;
		const uint16_t irq_priority;
		const uint16_t rts_low_watermark;
		const uint16_t rts_high_watermark;
		const uint8_t xbar_out_lpuartX_trig_input;
	} hardware_t;

public:
	uintptr_t port_addr;
	hardware_t * hardware;
	uint8_t				rx_pin_index_ = 0x0;	// default is always first item
	uint8_t				tx_pin_index_ = 0x0;
	uint8_t				half_duplex_mode_ = 0; // are we in half duplex mode?

	volatile BUFTYPE 	*tx_buffer_;
	volatile BUFTYPE 	*rx_buffer_;
	volatile BUFTYPE	*rx_buffer_storage_ = nullptr;
	volatile BUFTYPE	*tx_buffer_storage_ = nullptr;
	size_t				tx_buffer_size_;
	size_t				rx_buffer_size_;
	size_t				tx_buffer_total_size_;
	size_t				rx_buffer_total_size_;
	size_t  			rts_low_watermark_ = 0;
	size_t  			rts_high_watermark_ = 0;
	volatile uint8_t 	transmitting_ = 0;
	volatile uint16_t 	tx_buffer_head_ = 0;
	volatile uint16_t 	tx_buffer_tail_ = 0;
	volatile uint16_t 	rx_buffer_head_ = 0;
	volatile uint16_t 	rx_buffer_tail_ = 0;

	volatile uint32_t 	*transmit_pin_baseReg_ = 0;
	uint32_t 			transmit_pin_bitmask_ = 0;

	volatile uint32_t 	*rts_pin_baseReg_ = 0;
	uint32_t 			rts_pin_bitmask_ = 0;

	#if defined(ARDUINO_TEENSY41)
	static HardwareSerialIMXRT 	*s_serials_with_serial_events[8];
	#else	
	static HardwareSerialIMXRT 	*s_serials_with_serial_events[7];
	#endif
	static uint8_t 			s_count_serials_with_serial_events;

};