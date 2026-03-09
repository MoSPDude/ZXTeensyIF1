
#include "UartZXTeensy.h"

void UartZXTeensy::begin(uint8_t baudRate)
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
        enabled = true;
    }
};

void UartZXTeensy::end(void)
{
    if (enabled)
    {
        Serial8.end();
        flush();
        enabled = false;
    }
}
