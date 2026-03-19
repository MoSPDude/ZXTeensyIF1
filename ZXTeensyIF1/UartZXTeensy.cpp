
#include "UartZXTeensy.h"

bool UartZXTeensy::espWaitFor(const char *token, uint32_t timeout = 3000)
{
    String line;
    uint32_t start = millis();
    while ((millis() - start) < timeout)
    {
        while (Serial8.available())
        {
            char c = Serial8.read();
            line += c;
            if (line.indexOf(token) >= 0)
            {
                return true;
            }
        }
    }
    return false;
}

void UartZXTeensy::begin(uint8_t baudRate, const char* modemUrl)
{
    if (!enabled)
    {
        int baud;
        switch (baudRate)
        {
            case 1 :
                baud = 57600;
                break;
            case 2 :
                baud = 38400;
                break;
            case 3 :
                baud = 31250;
                break;
            case 4 :
                baud = 19200;
                break;
            case 5 :
                baud = 9600;
                break;
            case 6 :
                baud = 4800;
                break;
            case 7 :
                baud = 2400;
                break;
            default :
                baud = 115200;
                break;
        }
        Serial8.begin(baud);
        if (modemUrl != 0)
        {
            isPassthrough = true;
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
            isPassthrough = false;
        }
    }
};

void UartZXTeensy::end(void)
{
    if (enabled)
    {
        if (isPassthrough)
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
        flush();
        enabled = false;
    }
}
