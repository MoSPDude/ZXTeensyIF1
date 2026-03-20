
// Occasionally, receive 2 x MTU into buffer, so size over (2 * 1500)
static const size_t PKT_BUFFER_SIZE = 3072;
volatile bool httpEnabled = false;
char connectionId[8];
bool isReceivingPacket = false;
DMAMEM uint8_t packetBuffer[PKT_BUFFER_SIZE];
size_t packetBufferIndex = 0;
size_t packetLength = 0;
int packetCount = 0;
String httpServerStatus;

File httpUploadFile;
bool httpUploadActive = false;
size_t httpUploadContentLength = 0;
size_t httpUploadBytesWritten = 0;

bool httpWaitFor(const char *token, uint32_t timeout = 3000)
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

void sendData(const uint8_t *data, size_t size)
{
    Serial8.print("AT+CIPSEND=");
    Serial8.print(connectionId);
    Serial8.print(",");
    Serial8.println(size);
    if (httpWaitFor(">"))
    {
        Serial8.write(data, size);
        httpWaitFor("SEND OK");
    }
}

void httpSendString(const String &s)
{
    sendData((const uint8_t*)s.c_str(), s.length());
}

void httpSendHeader(int code, const char *type, int length)
{
    String header = "HTTP/1.1 " + String(code) + " OK\r\n";
    header += "Content-Type: " + String(type) + "\r\n";
    header += "Content-Length: " + String(length) + "\r\n";
    header +="\r\n";
    httpSendString(header);
}

void httpSend404()
{
    String msg = "404 Not Found";
    httpSendHeader(404, "text/plain", msg.length());
    httpSendString(msg);
}

void urldecode2(char *dst, const char *src)
{
    char a, b;
    while (*src)
    {
        if ((*src == '%') &&
            ((a = src[1]) && (b = src[2])) &&
            (isxdigit(a) && isxdigit(b)))
        {
            if (a >= 'a')
                a -= 'a'-'A';
            if (a >= 'A')
                a -= ('A' - 10);
            else
                a -= '0';
            if (b >= 'a')
                b -= 'a'-'A';
            if (b >= 'A')
                b -= ('A' - 10);
            else
                b -= '0';
            *dst++ = (16 * a) + b;
            src += 3;
        } else if (*src == '+')
        {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst++ = '\0';
}

void httpFinishUpload()
{
    String msg;
    if (httpUploadFile)
    {
        httpUploadFile.close();
        msg = "Number of bytes written: ";
        msg += httpUploadBytesWritten;
        msg += "\r\n";
    } else {
        msg = "Error opening file to write";
        msg += "\r\n";
    }
    httpSendHeader(200, "text/plain", msg.length());
    httpSendString(msg);
    httpUploadActive = false;

    // Close
    Serial8.print("AT+CIPCLOSE=");
    Serial8.println(connectionId);
}

void httpContinueUpload(uint8_t* content, size_t size)
{
    if (httpUploadFile)
    {
        httpUploadBytesWritten += httpUploadFile.write(content, size);
    }
    if (size < httpUploadContentLength)
    {
        httpUploadContentLength -= size;
    } else {
        httpUploadContentLength = 0;
        httpFinishUpload();
    }
}

void httpPerformPacket(char action, const char* path, size_t contentLength,
    uint8_t* content, size_t size)
{
    char decodedPath[MAX_PATH];
    urldecode2(decodedPath, path);
    if (action == 'P')
    {
        // Perform upload via HTTP PUT
        bool success = false;
        httpUploadActive = true;
        httpUploadBytesWritten = 0;
        char* ptr = strrchr(decodedPath, '/');
        if ((ptr != 0) && (ptr != decodedPath))
        {
            // Create sub-directories as required
            char parentDir[MAX_PATH];
            size_t index = (ptr - decodedPath);
            memcpy(parentDir, decodedPath, index);
            parentDir[index] = 0;
            if (SD.exists(parentDir) ||
                SD.mkdir(parentDir))
            {
                // Create file in the sub-directory, if given
                success = (strlen(ptr + 1) > 0);
            }
        } else {
            // Only create files in the root directory
            success = ((ptr != 0) && (strlen(ptr + 1) > 0));
        }
        if (success)
        {
            httpUploadFile = SD.open(decodedPath, FILE_WRITE_BEGIN);
        }
        if (httpUploadFile && (size > 0))
        {
            httpUploadBytesWritten += httpUploadFile.write(content, size);
        }
        if (size < contentLength)
        {
            httpUploadContentLength = (contentLength - size);
        } else {
            httpUploadContentLength = 0;
            httpFinishUpload();
        }
    } else {
        // Perform download or list via HTTP GET
        File file = SD.open(decodedPath, FILE_READ);
        if (file)
        {
            if (file.isDirectory())
            {
                // List directories as a simple web page
                bool isSubDirectory = (strlen(decodedPath) > 1);
                String page = "<html><body>";
                File tmpFile = file.openNextFile();
                while (tmpFile)
                {
                    const char* filename = tmpFile.name();
                    page += "<a href=\"";
                    if (isSubDirectory)
                    {
                        page += path;
                    }
                    page += "/";
                    page += filename;
                    page += "\">";
                    page += filename;
                    page += "</a><br>";
                    tmpFile.close();
                    tmpFile = file.openNextFile();
                }
                page += "</body></html>";
                httpSendHeader(200, "text/html", page.length());
                httpSendString(page);
            } else {
                // Download a specific file
                httpSendHeader(200, "application/octet-stream", file.size());
                while (file.available())
                {
                    size = file.read(packetBuffer, PKT_BUFFER_SIZE);
                    sendData(packetBuffer, size);
                }
            }
            file.close();
        } else {
            httpSend404();
        }

        // Close
        Serial8.print("AT+CIPCLOSE=");
        Serial8.println(connectionId);
    }
}

void httpProcessPacket()
{
    if ((packetCount > 0) || httpUploadActive)
    {
        if (httpUploadActive)
        {
            httpContinueUpload(packetBuffer, packetBufferIndex);
        }
    } else {
        // Find end of the HTTP header
        char* content = strstr((const char*)packetBuffer, "\r\n\r\n");
        if (content != 0)
        {
            *content = 0;
            content += 4;
            if ((memcmp(packetBuffer, "GET ", 4) == 0) ||
                (memcmp(packetBuffer, "PUT ", 4) == 0))
            {
                char* ptr = strstr((const char*)&(packetBuffer[4]), " ");
                if (ptr != 0)
                {
                    size_t contentLength;
                    *ptr++ = 0;
                    ptr = strstr(ptr, "Content-Length: ");
                    if (ptr != 0)
                    {
                        char* value = ptr + 16;
                        ptr = strstr(ptr, "\r\n");
                        if (ptr != 0)
                        {
                            *ptr = 0;
                        }
                        contentLength = atoi(value);
                    } else {
                        contentLength = 0;
                    }
                    size_t contentIndex = (content - (char*)packetBuffer);
                    httpPerformPacket(packetBuffer[0], (const char*)&(packetBuffer[4]),
                        contentLength, (uint8_t*)content, (packetBufferIndex - contentIndex));
                }
            }
        }
    }
}

void httpRunServer()
{
    if (httpEnabled)
    {
        while (Serial8.available())
        {
            if (isReceivingPacket)
            {
                // Fetch all available into the buffer
                int size = ((packetLength > PKT_BUFFER_SIZE) ?
                    PKT_BUFFER_SIZE : packetLength) - packetBufferIndex;
                if (size > Serial8.available())
                {
                    size = Serial8.available();
                }
                size = Serial8.readBytes(&(packetBuffer[packetBufferIndex]), size);
                if (size > 0)
                {
                    packetBufferIndex += size;
                }

                // Decide if a packet has been received
                if ((packetBufferIndex >= packetLength) ||
                    (packetBufferIndex >= PKT_BUFFER_SIZE))
                {
                    httpProcessPacket();
                    packetLength -= packetBufferIndex;
                    if (packetLength == 0)
                    {
                        isReceivingPacket = false;
                        packetCount = 0;
                    } else {
                        ++packetCount;
                    }

                    // Clear the buffer
                    packetBufferIndex = 0;
                }
            } else {
                // Fetch a byte into the buffer
                uint8_t c = Serial8.read();
                packetBuffer[packetBufferIndex] = c;
                ++packetBufferIndex;

                // Decide if a line or packet has been received
                if ((packetBufferIndex >= (PKT_BUFFER_SIZE - 1)) ||
                    (c == ':') || (c == '\n'))
                {
                    // Parse for an incoming packet
                    packetBuffer[packetBufferIndex] = 0;
                    if (strncmp("+IPD,", (const char*)packetBuffer, 5) == 0)
                    {
                        char* ptr = (char*)&(packetBuffer[5]);
                        char* lenPtr = strstr(ptr, ",");
                        if (lenPtr != 0)
                        {
                            *lenPtr = 0;
                            strncpy(connectionId, ptr, 8);
                            connectionId[7] = 0;
                            ptr = lenPtr + 1;
                            packetLength = atoi(ptr);
                            isReceivingPacket = true;
                            packetCount = 0;
                        }
                    }

                    // Clear the buffer
                    packetBufferIndex = 0;
                }
            }
        }
    }
}

void httpStartServer()
{
    httpStopServer();
    if (!httpEnabled)
    {
        // Wait for WiFi NTP to complete
        if (wifiNtpEnabled)
        {
            httpServerStatus = " > Waiting for WiFi NTP";
            return;
        }

        // Close the UART, and open the port exclusively
        espUart.end();

        // Open the UART, and start the server
        Serial8.begin(115200);
        Serial8.println("ATE0");
        httpWaitFor("OK");
        Serial8.println("AT+CIFSR");
        if (httpWaitFor("+CIFSR:STAIP,\""))
        {
            String ipAddress = Serial8.readStringUntil('\"');
            if (ipAddress != "0.0.0.0")
            {
                httpServerStatus = " > Address: " + ipAddress;
                httpWaitFor("OK");
                Serial8.println("AT+CIPMUX=1");
                httpWaitFor("OK");
                Serial8.println("AT+CIPSERVER=1,80");
                if (httpWaitFor("OK"))
                {
                    httpEnabled = true;
                }
            } else {
                httpServerStatus = " > Waiting for IP address";
            }
        } else {
            httpServerStatus = " > Waiting for WiFi";
        }
        if (!httpEnabled)
        {
            Serial8.end();
        }
    }
}

void httpStopServer()
{
    if (httpEnabled)
    {
        Serial8.println("AT+CIPSERVER=0");
        httpWaitFor("OK");
        Serial8.end();

        // Close any partial uploads
        if (httpUploadFile)
        {
            httpUploadFile.close();
        }
    }
    httpServerStatus = "";
    httpEnabled = false;
    httpUploadActive = false;
    packetBufferIndex = 0;
}
