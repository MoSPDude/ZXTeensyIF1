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

#include "UartZXTeensy.h"

void UartZXTeensy::begin(uint8_t baud)
{
    if (!enabled)
    {
        int bestdiv, bestosr;
        IMXRT_LPUART_t *port = (IMXRT_LPUART_t *)IMXRT_LPUART5_ADDRESS;
        switch (baud)
        {
            case 1 :
                bestdiv = calculateBestDiv(57600, bestosr);
                break;
            case 2 :
                bestdiv = calculateBestDiv(38400, bestosr);
                break;
            case 3 :
                bestdiv = calculateBestDiv(31250, bestosr);
                break;
            case 4 :
                bestdiv = calculateBestDiv(19200, bestosr);
                break;
            case 5 :
                bestdiv = calculateBestDiv(9600, bestosr);
                break;
            case 6 :
                bestdiv = calculateBestDiv(4800, bestosr);
                break;
            case 7 :
                bestdiv = calculateBestDiv(2400, bestosr);
                break;
            default :
                bestdiv = calculateBestDiv(115200, bestosr);
                break;
        }

        isTransmitting = 0;
        CCM_CCGR3 |= CCM_CCGR3_LPUART5(CCM_CCGR_ON);

        CORE_PIN34_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
        CORE_PIN34_CONFIG = 1;
        IOMUXC_LPUART5_RX_SELECT_INPUT = 1;

        CORE_PIN35_PADCONFIG =  IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
        CORE_PIN35_CONFIG = 1;
        IOMUXC_LPUART5_TX_SELECT_INPUT = 1;

        port->BAUD = LPUART_BAUD_OSR(bestosr - 1) | LPUART_BAUD_SBR(bestdiv) |
            (bestosr <= 8 ? LPUART_BAUD_BOTHEDGE : 0);
        port->PINCFG = 0;

        port->WATER = LPUART_WATER_RXWATER(2) | LPUART_WATER_TXWATER(2);
        port->FIFO |= LPUART_FIFO_TXFE | LPUART_FIFO_RXFE;
        port->CTRL = (LPUART_CTRL_TE | LPUART_CTRL_RE | LPUART_CTRL_RIE | LPUART_CTRL_ILIE);
        port->STAT &= ~LPUART_STAT_RXINV;
        enabled = true;
    }
};

void UartZXTeensy::end(void)
{
    if (enabled)
    {
        if (CCM_CCGR3 & CCM_CCGR3_LPUART5(CCM_CCGR_ON))
        {
            while (isTransmitting)
            {
                // Wait for buffered data to send
                yield();
            }

            // Disable the TX and RX ...
            IMXRT_LPUART_t *port = (IMXRT_LPUART_t *)IMXRT_LPUART5_ADDRESS;
            port->CTRL = 0;
        }

        // Clear the ring buffers
        uartReadBuffer.clear();
        uartWriteBuffer.clear();
        uartFlagsBuffer.clear();
        uartTxDataBuffer.clear();
        enabled = false;
    }
}
