
#ifndef RTC_ZX_TEENSY_H
#define RTC_ZX_TEENSY_H

#include <TimeLib.h>

#define DEFAULT_TIME 1767225600 // 1st January 2026 at 00:00

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

class RtcZXTeensy
{
    protected :
        volatile uint8_t regBank[14];
        volatile bool hasHold;
        volatile bool is24Hour;
        volatile bool hasModified;

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
            regBank[12] = weekday();
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
            time_t timeNow = now();
            Teensy3Clock.set(timeNow);
            setTime(timeNow);
            updateRegisters();
        }
        
    public :
        constexpr RtcZXTeensy() : regBank(), hasHold(false), is24Hour(true), hasModified(false)
        {
            for (int i = 0; i < 14; ++i) {
                regBank[i] = 0;
            }
            setSyncProvider(getTeensy3Time);
            Teensy3Clock.set(DEFAULT_TIME);
            setTime(DEFAULT_TIME);
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
                        hasModified = false;
                        updateTime();
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
};

#endif
