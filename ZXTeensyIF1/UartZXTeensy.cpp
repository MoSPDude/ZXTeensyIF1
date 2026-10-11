
#include "UartZXTeensy.h"

static const uint32_t UART_BAUD_RATES[] = {
    115200, 57600, 38400, 31250, 19200, 9600, 4800, 2400,
    230400, 460800, 576000, 921600, 1152000, 1500000, 2000000,
    2304000, 3456000, 4608000, 5760000, 6912000, 8064000, 9216000,
    10368000, 11520000
};

void UartZXTeensy::begin(uint8_t baudRate, const char* modemUrl)
{
    if (!enabled)
    {
        if (baudRate >= (sizeof(UART_BAUD_RATES) / sizeof(UART_BAUD_RATES[0])))
        {
            baudRate = 0;
        }
        baudCode = baudRate;
        Serial8.begin(UART_BAUD_RATES[baudRate]);
        readByteCycle = ARM_DWT_CYCCNT;
        if (modemUrl != 0)
        {
            isModemPassthrough = true;
            Serial8.println("ATE0");
            if (espWaitFor("OK"))
            {
                Serial8.println("AT+CIPMUX=0");
                espWaitFor("OK");
                Serial8.println("AT+CIPMODE=1");
                espWaitFor("OK");
                Serial8.print("AT+CIPSTART=\"TCP\",");
                Serial8.println(modemUrl);
                if (espWaitFor("OK"))
                {
                    Serial8.println("AT+CIPSEND");
                    if (espWaitFor(">"))
                    {
                        enabled = true;
                    }
                }
            }
        } else {
            enabled = true;
            isModemPassthrough = false;
        }
    }
};

void UartZXTeensy::end(bool hasReset)
{
    if (enabled)
    {
        if (!hasReset && isModemPassthrough)
        {
            Serial8.print("+++");
            delay(1000);
            Serial8.println("AT+CIPCLOSE");
            if (espWaitFor("OK"))
            {
                Serial8.println("AT+CIPMODE=0");
                espWaitFor("OK");
            }
        }
        Serial8.end();
        clear();
        enabled = false;
    }
}
