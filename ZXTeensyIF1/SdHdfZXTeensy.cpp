
#include "SdHdfZXTeensy.h"

void SdHdfZXTeensy::begin(RingBuffer<READ_BUFFER_SIZE>* readBuffer, const char* filename)
{
    // NOTE: Do NOT clear isSdIdle as the DivMMC can warm reset
    sdSpiReadBuffer = readBuffer;
    cardSelected = false;
    currentState = SdHdfZXTeensy::STATE_IDLE;
    currentCommand = SdHdfZXTeensy::CMD_IDLE;
    commandAppCmd = false;
    commandArgument = 0;
    dataActive = false;
    dataRegister = 0;
    dataIndex = 0;

#ifdef DEBUG_HDF_OUTPUT
    Serial.begin(115200);
#endif

    // Load the image header
    if (!hasValidPath)
    {
        strcpy(sdCardPath, filename);
        hasValidPath = true;
        numSectors = 0;
        currentSector = 0;
        openSdCard();
        if (sdCard)
        {
            uint8_t header[16];
            if (sdCard.read(header, 16) >= 16)
            {
                if (memcmp(header, "RS-IDE", 6) == 0)
                {
                    // Find start of the data
                    sectorOffset = ((header[10] << 8) | header[9]);
                } else {
                    // Assume as disk image
                    sectorOffset = 0;
                }

                // Determine the card size
                sdCard.seek(0, SeekEnd);
                int filesize = sdCard.position();
                numSectors = (filesize - sectorOffset) / 512;
                currentSector = numSectors;
            }
        }
    }

#ifdef DEBUG_HDF_OUTPUT
    Serial.printf("%x, %x\n", numSectors, sectorOffset);
#endif
}

void SdHdfZXTeensy::end(void)
{
    if (sdCard)
    {
        sdCard.close();
#ifdef DEBUG_HDF_OUTPUT
        Serial.printf("close\n");
#endif
    }
}
