
#ifndef ESP_NTP_ZX_TEENSY_H
#define ESP_NTP_ZX_TEENSY_H

#include <TimeLib.h>

//define DEBUG_NTP_OUTPUT

class EspNtpZXTeensy
{
    public :
        typedef enum {
            STATE_IDLE,
            STATE_WAITING,
            STATE_READY,
            STATE_CHECK_NTP,
            STATE_SET_NTP,
            STATE_GET_TIME
        } cmd_state_t;

    protected :
        static const size_t BUFFER_SIZE = 64;
        UartZXTeensy* espUart;
        bool hasSyncTime;
        cmd_state_t currentState;
        char commandResponse;
        char* ascTimePtr;
        char buffer[BUFFER_SIZE];
        uint8_t bufferIdx;

        inline void sendCommand(const char* cmd)
        {
#ifdef DEBUG_NTP_OUTPUT
            Serial.printf("> %s\n", cmd);
#endif
            while (*cmd != 0)
            {
                espUart->writeData(UartZXTeensy::UART_WRITE, *cmd);
                ++cmd;
            }
            espUart->writeData(UartZXTeensy::UART_WRITE, '\r');
            espUart->writeData(UartZXTeensy::UART_WRITE, '\n');
        }

        bool setState(cmd_state_t state)
        {
            currentState = state;
            switch (currentState)
            {
                case STATE_WAITING :
                    break;
                case STATE_READY :
                    sendCommand("ATE0");
                    break;
                case STATE_CHECK_NTP :
                    sendCommand("AT+CIPSNTPCFG?");
                    break;
                case STATE_SET_NTP :
                    sendCommand("AT+CIPSNTPCFG=1,0,\"pool.ntp.org\"");
                    break;
                case STATE_GET_TIME :
                    sendCommand("AT+CIPSNTPTIME?");
                    break;
                default :
                    return true;
            }
            return false;
        }

        bool processResponse()
        {
            if (buffer[0] == '+')
            {
                switch (currentState)
                {
                    case STATE_CHECK_NTP :
                        if (strncmp("+CIPSNTPCFG:", buffer, 12) == 0)
                        {
                            commandResponse = buffer[12];
                        }
                        break;
                    case STATE_GET_TIME :
                        if (strncmp("+CIPSNTPTIME:", buffer, 13) == 0)
                        {
                            if (strstr(buffer, " 1970") != 0)
                            {
                                // Time has not been reported to the ESP-01S yet
                                commandResponse = '0';
                            } else {
                                // NOTE: "OK\r\n" will replace the beginning of
                                // the buffer
                                ascTimePtr = &(buffer[13]);
                                commandResponse = '1';
                                hasSyncTime = true;
                            }
                        }
                        break;
                    default :
                        break;
                }
            } else if (strncmp("WIFI GOT IP", buffer, 11) == 0)
            {
                // Wifi is now connected, so start sending commands
                setState(STATE_READY);
            } else if (currentState != STATE_WAITING)
            {
                if (strncmp("OK", buffer, 2) == 0)
                {
                    cmd_state_t nextState;
                    switch (currentState)
                    {
                        case STATE_WAITING :
                            nextState = STATE_READY;
                            break;
                        case STATE_READY :
                            nextState = STATE_CHECK_NTP;
                            break;
                        case STATE_CHECK_NTP :
                            nextState = ((commandResponse == '1') ? 
                                STATE_GET_TIME : STATE_SET_NTP);
                            break;
                        case STATE_SET_NTP :
                            nextState = STATE_GET_TIME;
                            break;
                        default :
                            nextState = ((commandResponse == '1') ? 
                                STATE_IDLE : STATE_GET_TIME);
                            break;
                    }
                    return setState(nextState);
                } else if ((strncmp("FAIL", buffer, 4) == 0) ||
                    (strncmp("ERROR", buffer, 5) == 0))
                {
                    return setState(STATE_IDLE);
                }
            }
            return false;
        }

    public :
        EspNtpZXTeensy() : espUart(0), hasSyncTime(false), currentState(STATE_IDLE), 
            commandResponse(0), ascTimePtr(0), buffer(), bufferIdx(0)
        {
        }

        inline __attribute__((always_inline)) void begin(UartZXTeensy* uart)
        {
            espUart = uart;
            if (currentState == STATE_IDLE)
            {
                hasSyncTime = false;
                ascTimePtr = 0;
                setState(STATE_WAITING);
            }
#ifdef DEBUG_NTP_OUTPUT
            Serial.begin(115200);
#endif
        }

        inline __attribute__((always_inline)) void end()
        {
            setState(STATE_IDLE);
        }

        inline __attribute__((always_inline)) bool onTick()
        {
            if (currentState != STATE_IDLE)
            {
                while (espUart->hasReadData())
                {
                    uint8_t c = espUart->readData();
                    if (c != '\n')
                    {
                        buffer[bufferIdx] = c;
                        if (bufferIdx < 63)
                        {
                            ++bufferIdx;
                        }
                    } else {
                        buffer[bufferIdx] = 0;
                        bufferIdx = 0;
#ifdef DEBUG_NTP_OUTPUT
                        Serial.printf("< %s\n", buffer);
#endif
                        return processResponse();
                    }
                }
            }
            return false;
        }

        inline const char* getAscTime()
        {
            return (hasSyncTime ? ascTimePtr : 0);
        }
};

#endif
