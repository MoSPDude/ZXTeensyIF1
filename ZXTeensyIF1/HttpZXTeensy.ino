
#define BUFFER_SIZE 1024

volatile bool httpEnabled = false;
char connectionId[16];
bool isReceivingPacket = false;
uint8_t packetBuffer[BUFFER_SIZE];
size_t packetBufferIndex = 0;
size_t packetLength = 0;
int packetCount = 0;
String httpServerStatus;

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

/*void httpHandleUpload(int conn, String name, int length)
{
    File f = SD.open(name, FILE_WRITE_BEGIN);
    if (f)
    {
        uint8_t buffer[BUFFER_SIZE];
        int remaining = length;
        while (remaining > 0)
        {
            int chunk = min(remaining, BUFFER_SIZE);
            int readBytes = 0;
            while (readBytes < chunk)
            {
                if (ESP.available())
                {
                    buffer[readBytes++] = ESP.read();
                }
            }
            f.write(buffer, readBytes);
            remaining -= readBytes;
        }
        f.close();

        // Success
        String msg = "OK";
        sendHeader(conn, 200, "text/plain", msg.length());
        sendString(conn, msg);
    } else {
        httpSend404(conn);
    }
}*/

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

void httpPerformPacket(char action, const char* path, size_t contentLength,
    uint8_t* content, size_t size)
{
    if (action == 'P')
    {
        // TODO:
    } else {
        char decodedPath[256];
        urldecode2(decodedPath, path);
        File file = SD.open(decodedPath, FILE_READ);
        if (file)
        {
            if (file.isDirectory())
            {
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
                httpSendHeader(200, "application/octet-stream", file.size());
                while (file.available())
                {
                    size = file.read(packetBuffer, BUFFER_SIZE);
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
    if (packetCount > 0)
    {
        // TODO:
        //httpContinueAction(packetBuffer, packetLength);
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
                    size_t contentLength = 0;
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
                    }
                    httpPerformPacket(packetBuffer[0], (const char*)&(packetBuffer[4]), contentLength,
                        (uint8_t*)content, (packetLength - (content - (char*)packetBuffer)));
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
            // Fetch a byte into the buffer
            uint8_t c = Serial8.read();
            packetBuffer[packetBufferIndex] = c;
            ++packetBufferIndex;

            // Decide if all bytes are received
            if (isReceivingPacket)
            {
                if ((packetBufferIndex >= packetLength) ||
                    (packetBufferIndex >= BUFFER_SIZE))
                {
                    packetLength -= packetBufferIndex;
                    httpProcessPacket();
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
                // Receive a line from the ESP
                if ((packetBufferIndex >= (BUFFER_SIZE - 1)) ||
                    (c == ':') || (c == '\n'))
                {
                    // Parse for an incoming packet
                    packetBuffer[packetBufferIndex] = 0;
                    if (strstr((const char*)packetBuffer, "+IPD,") == (char*)packetBuffer)
                    {
                        char* ptr = (char*)&(packetBuffer[5]);
                        char* lenPtr = strstr(ptr, ",");
                        if (lenPtr != 0)
                        {
                            *lenPtr = 0;
                            strncpy(connectionId, ptr, 16);
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
    if (!httpEnabled)
    {
        if (uartPresent)
        {
            if (wifiNtpEnabled)
            {
                httpServerStatus = " > Waiting for WiFi NTP";
                return;
            }

            // Close the UART, and open the port exclusively
            espUart.end();
        }

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
    }
    httpServerStatus = "";
    httpEnabled = false;
}
