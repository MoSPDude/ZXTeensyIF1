
#include "SdSdioZXTeensy.h"

void SdSdioZXTeensy::begin(RingBuffer<READ_BUFFER_SIZE>* readBuffer, SdCard* card)
{
    // NOTE: Do NOT clear isSdIdle as the DivMMC can warm reset
    sdSpiReadBuffer = readBuffer;
    currentState = SdSdioZXTeensy::STATE_IDLE;
    currentCommand = SdSdioZXTeensy::CMD_IDLE;
    commandAppCmd = false;
    commandArgument = 0;
    dataActive = false;
    dataRegister = 0;
    dataIndex = 0;

#ifdef DEBUG_HDF_OUTPUT
    Serial.begin(115200);
#endif

    sdioCard = static_cast<SdioCard*>(card);
    if (sdioCard != 0)
    {
        numSectors = sdioCard->sectorCount();
        while (sdioCard->isBusy()) { yield(); };
    } else {
        numSectors = 0;
    }
}

void SdSdioZXTeensy::end(void)
{
    if (sdioCard != 0)
    {
        while (sdioCard->isBusy()) { yield(); };
        sdioCard->end();
        sdioCard = 0;
    }
}
