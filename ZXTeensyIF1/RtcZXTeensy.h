
#ifndef RTC_ZX_TEENSY_H
#define RTC_ZX_TEENSY_H

#include <TimeLib.h>

extern unsigned long rtc_get(void);
extern void rtc_set(unsigned long t);

time_t getTeensy3Time()
{
  return rtc_get();
}

class RtcZXTeensy
{
    protected :
        volatile uint8_t regBank[14];
        volatile bool hasHold;
        volatile bool is24Hour;
        volatile bool hasModified;
        volatile bool needsRtcSet;

        inline __attribute__((always_inline)) void updateRegisters()
        {
            uint8_t secs = second();
            uint8_t mins = minute();
            uint8_t hrs = (is24Hour ? hour() : hourFormat12());
            uint8_t days = day();
            uint8_t mnts = month();
            uint8_t yrs = (year() % 100);
            regBank[0] = secs % 10;
            regBank[1] = secs / 10;
            regBank[2] = mins % 10;
            regBank[3] = mins / 10;
            regBank[4] = hrs % 10;
            regBank[5] = ((hrs / 10) | ((!is24Hour && isPM()) ? 0x04 : 0x00));
            regBank[6] = days % 10;
            regBank[7] = days / 10;
            regBank[8] = mnts % 10;
            regBank[9] = mnts / 10;
            regBank[10] = yrs % 10;
            regBank[11] = yrs / 10;
            regBank[12] = (weekday() - 1);
        }

        inline __attribute__((always_inline)) void updateTime()
        {
            int secs = (regBank[1] * 10) + regBank[0];
            int mins = (regBank[3] * 10) + regBank[2];
            int hrs = ((regBank[5] & 0x03) * 10) + regBank[4];
            if (!is24Hour)
            {
                if (regBank[5] & 0x04)
                {
                    if (hrs < 12)
                    {
                        hrs += 12;
                    } else {
                        hrs = 12;
                    }
                } else if (hrs >= 12)
                {
                    hrs = 0;
                }
            }
            int days = (regBank[7] * 10) + regBank[6];
            int mnts = (regBank[9] * 10) + regBank[8];
            int yrs = (regBank[11] * 10) + regBank[10];
            setTime(hrs, mins, secs, days, mnts, yrs);
            setSyncProvider(0);
            needsRtcSet = true;
        }

    public :
        constexpr RtcZXTeensy() : regBank(), hasHold(false), is24Hour(true),
            hasModified(false), needsRtcSet(false)
        {
            for (int i = 0; i < 14; ++i) {
                regBank[i] = 0;
            }
            setSyncProvider(getTeensy3Time);
            updateRegisters();
        }

        inline __attribute__((always_inline)) uint8_t read(uint8_t value)
        {
            switch (value)
            {
                case 0x0d :
                    return (hasHold ? 0x01 : 0x02);
                case 0x0e :
                    return regBank[13];
                case 0x0f :
                    return (is24Hour ? 0x04 : 0x00);
                default :
                    return regBank[value];
            }
        }

        inline __attribute__((always_inline)) void write(uint8_t value, uint8_t data)
        {
            switch (value)
            {
                case 0x0d :
                    if (hasModified)
                    {
                        updateTime();
                        hasModified = false;
                    } else {
                        updateRegisters();
                    }
                    hasHold = (data & 0x01);
                    break;
                case 0x0e :
                    regBank[13] = (data & 0x0f);
                    break;
                case 0x0f :
                    is24Hour = (data & 0x04);
                    break;
                default :
                    regBank[value] = (data & 0x0f);
                    hasModified = true;
                    break;
            }
        }

        inline __attribute__((always_inline)) void updateRtc()
        {
            if (needsRtcSet)
            {
                time_t timeNow = now();
                rtc_set(timeNow);
                setSyncProvider(getTeensy3Time);
                needsRtcSet = false;
            }
        }

        inline __attribute__((always_inline)) void setAscTime(const char* ascTime, uint8_t ntpTz)
        {
            if (ascTime != 0)
            {
                int secs, mins, hrs, days, mnts, yrs;
                switch (ascTime[4])
                {
                    case 'J' : // Jan, Jun, Jul
                        if (ascTime[5] == 'u')
                        {
                            mnts = (ascTime[6] == 'n') ? 6 : 7;
                        } else {
                            mnts = 1;
                        }
                        break;
                    case 'F' : // Feb
                        mnts = 2;
                        break;
                    case 'M' : // Mar, May
                        mnts = (ascTime[6] == 'r') ? 3 : 5;
                        break;
                    case 'A' : // Apr, Aug
                        mnts = (ascTime[5] == 'p') ? 4 : 8;
                        break;
                    case 'S' : // Sep
                        mnts = 9;
                        break;
                    case 'O' : // Oct
                        mnts = 10;
                        break;
                    case 'N' : // Nov
                        mnts = 11;
                        break;
                    case 'D' : // Dec
                        mnts = 12;
                        break;
                    default : 
                        mnts = 0;
                        break;
                }
                if ((mnts > 0) &&
                    (sscanf(&(ascTime[8]), "%d%d:%d:%d%d", &days, &hrs, &mins, &secs, &yrs) >= 5))
                {
                    setSyncProvider(0);
                    setTime(hrs, mins, secs, days, mnts, yrs);
                    time_t timeNow = now();
                    if (ntpTz >= 48)
                    {
                        time_t adjust = (ntpTz - 48) * 15 * 60;
                        timeNow += adjust;
                        setTime(timeNow);
                    } else if (ntpTz < 48)
                    {
                        time_t adjust = (48 - ntpTz) * 15 * 60;
                        timeNow -= adjust;
                        setTime(timeNow);
                    }
                    updateRegisters();
                    needsRtcSet = true;
                }
            }
        }
};

#endif
