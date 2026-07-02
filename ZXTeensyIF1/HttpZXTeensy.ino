
#include "HttpZXTeensy.h"

typedef enum {
    HTTP_ACTION_UNKNOWN,
    HTTP_ACTION_OPTIONS,
    HTTP_ACTION_GET,
    HTTP_ACTION_HEAD,
    HTTP_ACTION_PUT,
    HTTP_ACTION_PROPFIND,
    HTTP_ACTION_PROPPATCH,
    HTTP_ACTION_MKCOL,
    HTTP_ACTION_MOVE,
    HTTP_ACTION_COPY,
    HTTP_ACTION_DELETE,
    HTTP_ACTION_NOT_IMPLEMENTED
} http_action_t;

enum http_dav_property_t {
    HTTP_DAV_DISPLAYNAME       = 1 << 0,
    HTTP_DAV_RESOURCETYPE      = 1 << 1,
    HTTP_DAV_CONTENTLENGTH     = 1 << 2,
    HTTP_DAV_CONTENTTYPE       = 1 << 3,
    HTTP_DAV_LASTMODIFIED      = 1 << 4,
    HTTP_DAV_CREATIONDATE      = 1 << 5,
    HTTP_DAV_ETAG              = 1 << 6
};

static const uint16_t HTTP_DAV_ALL_PROPERTIES =
    HTTP_DAV_DISPLAYNAME | HTTP_DAV_RESOURCETYPE |
    HTTP_DAV_CONTENTLENGTH | HTTP_DAV_CONTENTTYPE |
    HTTP_DAV_LASTMODIFIED | HTTP_DAV_CREATIONDATE | HTTP_DAV_ETAG;

void httpPerformPacket(http_action_t action, const char* path, size_t contentLength,
    char* header, uint8_t* content, size_t size);
void httpPerformDelete(const char* decodedPath, bool isChild);

// Occasionally, receive 2 x MTU into buffer, so size over (2 * 1500)
static const size_t PACKET_BUFFER_SIZE = (RAM_PAGE_SIZE / 2);
static const size_t CONTENT_BUFFER_SIZE = (EXT_RAM_PAGE_COUNT - 1) * RAM_PAGE_SIZE;
static const size_t MAX_TX_PACKET_SIZE = 1500;
uint8_t* const httpPacketBuffer = (uint8_t*)divMmcExtRamArray[0];
char* const httpHeaderBuffer = (char*)&(divMmcExtRamArray[0][PACKET_BUFFER_SIZE]);
uint8_t* const httpContentBuffer = (uint8_t*)divMmcExtRamArray[1];
volatile bool httpEnabled = false;
char httpConnectionId[8];
bool httpReceivingPacket = false;
size_t httpPacketBufferIndex = 0;
size_t httpPacketLength = 0;
int httpPacketCount = 0;
String httpServerStatus;
http_action_t httpAction = HTTP_ACTION_UNKNOWN;
char httpURLPath[MAX_PATH];

File httpUploadFile;
size_t httpUploadBytesWritten = 0;
size_t httpUploadContentLength = 0;
bool httpUploadCreated = false;
size_t httpResponseLength = 0;
bool httpResponseOverflow = false;

const char* httpFindHeaderValue(char* header, const char* name)
{
    size_t nameLength = strlen(name);
    char* candidate = header;
    while ((candidate = strchr(candidate, '\n')) != 0)
    {
        if (strlen(candidate) >= nameLength)
        {
            char saved = candidate[nameLength];
            candidate[nameLength] = 0;
            bool match = (stricmp(candidate, name) == 0);
            candidate[nameLength] = saved;
            if (match)
            {
                const char* value = candidate + nameLength;
                while ((*value == ' ') || (*value == '\t'))
                {
                    ++value;
                }
                return value;
            }
        }
        ++candidate;
    }
    return 0;
}

bool httpBodyContains(const uint8_t* content, size_t size, const char* token)
{
    size_t tokenLength = strlen(token);
    if ((tokenLength > 0) && (tokenLength <= size))
    {
        for (size_t i = 0; i <= (size - tokenLength); ++i)
        {
            size_t j = 0;
            while ((j < tokenLength) && (content[i + j] == token[j]))
            {
                ++j;
            }
            if (j == tokenLength)
            {
                return true;
            }
        }
    }
    return false;
}

bool httpWaitFor(const char *token, uint32_t timeout = 3000)
{
    int index = 0;
    uint32_t start = millis();
    while ((millis() - start) < timeout)
    {
        while (Serial8.available())
        {
            char c = Serial8.read();
            if (c != token[index])
            {
                index = 0;
            } else {
                ++index;
                if (token[index] == 0)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void sendData(const uint8_t *data, size_t size)
{
    while (size > 0)
    {
        size_t bytesSent = ((size >= MAX_TX_PACKET_SIZE) ?
            MAX_TX_PACKET_SIZE : size);
        Serial8.print(HTTP_STRINGS[HTTP_STR_AT_SEND]);
        Serial8.print(httpConnectionId);
        Serial8.print(",");
        Serial8.println(bytesSent);
        if (httpWaitFor(">"))
        {
            Serial8.write(data, bytesSent);
            httpWaitFor(HTTP_STRINGS[HTTP_STR_AT_SEND_OK]);
            data += bytesSent;
            size -= bytesSent;
        } else {
            break;
        }
    }
}

void httpSendText(const char* text)
{
    sendData((const uint8_t*)text, strlen(text));
}

void httpResponseBegin()
{
    httpResponseLength = 0;
    httpResponseOverflow = false;
}

void httpResponseAppend(const char* text, size_t length)
{
    if (httpResponseOverflow ||
        (length > (CONTENT_BUFFER_SIZE - httpResponseLength)))
    {
        httpResponseOverflow = true;
        return;
    }
    memcpy(httpContentBuffer + httpResponseLength, text, length);
    httpResponseLength += length;
}

void httpResponseAppend(const char* text)
{
    httpResponseAppend(text, strlen(text));
}

void httpResponseAppend(char value)
{
    httpResponseAppend(&value, 1);
}

void httpResponseAppendNumber(unsigned long long value,
    bool hexadecimal = false)
{
    char text[24];
    char* end = text + sizeof(text);
    char* begin = end;
    unsigned int base = hexadecimal ? 16 : 10;
    const char* digits = HTTP_STRINGS[HTTP_STR_HEX];
    do
    {
        *--begin = digits[value % base];
        value /= base;
    } while (value != 0);
    httpResponseAppend(begin, end - begin);
}

bool httpSendResponse()
{
    if (httpResponseOverflow)
    {
        return false;
    }
    sendData(httpContentBuffer, httpResponseLength);
    return true;
}

void httpSendHeader(const char *code, const char *type, size_t length)
{
    int size = snprintf(httpHeaderBuffer, PACKET_BUFFER_SIZE,
        "%s%s\r\n%s%d\r\n%s%s\r\n%s",
        HTTP_STRINGS[HTTP_STR_HTTP_PREFIX], code,
        HTTP_STRINGS[HTTP_STR_CONTENT_LENGTH], length,
        HTTP_STRINGS[HTTP_STR_CONTENT_TYPE], type,
        HTTP_STRINGS[HTTP_STR_CONNECTION_CLOSE]);
    if ((size > 0) && (size < (int)PACKET_BUFFER_SIZE))
    {
        sendData((const uint8_t*)httpHeaderBuffer, size);
    }
}

void httpSend404()
{
    httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_404],
        HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
}

void httpSendNotImplemented()
{
    httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_501],
        HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
}

void httpFinishConnection()
{
    httpAction = HTTP_ACTION_UNKNOWN;
}

void httpCloseConnection()
{
    Serial8.print(HTTP_STRINGS[HTTP_STR_AT_CLOSE]);
    Serial8.println(httpConnectionId);
    httpWaitFor(HTTP_STRINGS[HTTP_STR_AT_OK]);
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

size_t httpGetUrlPath(char* destination, const char* source)
{
    urldecode2(destination, source);

    // Trim trailing '/' characters
    size_t pathLength = strlen(destination);
    while ((pathLength > 1) && (destination[pathLength - 1] == '/'))
    {
        destination[--pathLength] = 0;
    }
    return pathLength;
}

void httpPerformOptions()
{
    int size = snprintf(httpHeaderBuffer, PACKET_BUFFER_SIZE,
        "%s%s\r\n%s%s%s%s%s",
        HTTP_STRINGS[HTTP_STR_HTTP_PREFIX], HTTP_STRINGS[HTTP_STR_HTTP_200],
        HTTP_STRINGS[HTTP_STR_ALLOW], HTTP_STRINGS[HTTP_STR_DAV],
        HTTP_STRINGS[HTTP_STR_CONTENT_LENGTH],
        HTTP_STRINGS[HTTP_STR_CONTENT_LENGTH_ZERO],
        HTTP_STRINGS[HTTP_STR_CONNECTION_CLOSE]);
    if ((size > 0) && (size < (int)PACKET_BUFFER_SIZE))
    {
        sendData((const uint8_t*)httpHeaderBuffer, size);
    }

    // Close
    httpFinishConnection();
}

void httpPerformGet(bool sendBody)
{
    // Perform download or list via HTTP GET
    File file = SD.open(httpURLPath, FILE_READ);
    if (file)
    {
        if (file.isDirectory())
        {
            // List directories as a simple web page
            bool isSubDirectory = (strlen(httpURLPath) > 1);
            httpResponseBegin();
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_HTML_BEGIN]);
            File tmpFile = file.openNextFile();
            while (tmpFile)
            {
                const char* filename = tmpFile.name();
                httpResponseAppend(HTTP_STRINGS[HTTP_STR_HTML_LINK]);
                if (isSubDirectory)
                {
                    httpResponseAppend(httpURLPath);
                }
                httpResponseAppend('/');
                httpResponseAppend(filename);
                httpResponseAppend(HTTP_STRINGS[HTTP_STR_HREF_END]);
                httpResponseAppend(filename);
                httpResponseAppend(HTTP_STRINGS[HTTP_STR_HTML_LINK_END]);
                tmpFile.close();
                tmpFile = file.openNextFile();
            }
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_HTML_END]);
            if (httpResponseOverflow)
            {
                httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_507],
                    HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
            } else {
                httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_200],
                    HTTP_STRINGS[HTTP_STR_TEXT_HTML], httpResponseLength);
                if (sendBody)
                {
                    httpSendResponse();
                }
            }
        } else {
            // Download a specific file
            httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_200], HTTP_STRINGS[HTTP_STR_BINARY_TYPE],
                file.size());
            if (sendBody)
            {
                while (file.available())
                {
                    // NOTE: Avoid sending partial packets
                    size_t size = file.read(httpContentBuffer,
                        ((CONTENT_BUFFER_SIZE / MAX_TX_PACKET_SIZE) *
                            MAX_TX_PACKET_SIZE));
                    sendData(httpContentBuffer, size);
                }
            }
        }
        file.close();
    } else {
        httpSend404();
    }

    // Close
    httpFinishConnection();
}

void httpAppendXmlEscaped(const char* text)
{
    while (*text != 0)
    {
        switch (*text)
        {
            case '&': httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_AMP]); break;
            case '<': httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_LT]); break;
            case '>': httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_GT]); break;
            case '\"': httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_QUOT]); break;
            case '\'': httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_APOS]); break;
            default: httpResponseAppend(*text); break;
        }
        ++text;
    }
}

void httpAppendEncodedHref(const char* path, bool directory)
{
    const char* hex = HTTP_STRINGS[HTTP_STR_HEX];
    const uint8_t* ptr = (const uint8_t*)path;
    if (*ptr != '/')
    {
        httpResponseAppend('/');
    }
    while (*ptr != 0)
    {
        uint8_t c = *ptr++;
        if (isalnum(c) || (c == '/') || (c == '-') || (c == '_') ||
            (c == '.') || (c == '~'))
        {
            httpResponseAppend((char)c);
        } else {
            httpResponseAppend('%');
            httpResponseAppend(hex[c >> 4]);
            httpResponseAppend(hex[c & 0x0F]);
        }
    }
    if (directory && (httpResponseLength > 0) &&
        (httpContentBuffer[httpResponseLength - 1] != '/'))
    {
        httpResponseAppend('/');
    }
}

int httpDavFullYear(int year)
{
    return (year < 1900) ? year + 1900 : year;
}

int httpDavWeekday(int year, int month, int day)
{
    static const int offsets[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
    if (month < 3)
    {
        --year;
    }
    return (year + year / 4 - year / 100 + year / 400 +
        offsets[month - 1] + day) % 7;
}

void httpDavAppendHttpDate(const DateTimeFields& date)
{
    char text[40];
    int year = httpDavFullYear(date.year);
    int month = date.mon + 1;
    if ((month < 1) || (month > 12))
    {
        month = 1;
    }
    snprintf(text, sizeof(text), HTTP_STRINGS[HTTP_STR_DATE_HTTP_FORMAT],
        HTTP_STRINGS[HTTP_STR_SUN + httpDavWeekday(year, month, date.mday)],
        date.mday, HTTP_STRINGS[HTTP_STR_JAN + month - 1], year,
        date.hour, date.min, date.sec);
    httpResponseAppend(text);
}

void httpDavAppendIsoDate(const DateTimeFields& date)
{
    char text[32];
    int month = date.mon + 1;
    if ((month < 1) || (month > 12))
    {
        month = 1;
    }
    snprintf(text, sizeof(text), HTTP_STRINGS[HTTP_STR_DATE_ISO_FORMAT],
        httpDavFullYear(date.year), month, date.mday,
        date.hour, date.min, date.sec);
    httpResponseAppend(text);
}

uint16_t httpDavRequestedProperties(const uint8_t* content, size_t size)
{
    if ((size == 0) || httpBodyContains(content, size,
        HTTP_STRINGS[HTTP_STR_ALLPROP]))
    {
        return HTTP_DAV_ALL_PROPERTIES;
    }

    uint16_t properties = 0;
    if (httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_DISPLAYNAME]))
        properties |= HTTP_DAV_DISPLAYNAME;
    if (httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_RESOURCETYPE]))
        properties |= HTTP_DAV_RESOURCETYPE;
    if (httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_CONTENTLENGTH_PROP]))
        properties |= HTTP_DAV_CONTENTLENGTH;
    if (httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_CONTENTTYPE_PROP]))
        properties |= HTTP_DAV_CONTENTTYPE;
    if (httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_LASTMODIFIED]))
        properties |= HTTP_DAV_LASTMODIFIED;
    if (httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_CREATIONDATE]))
        properties |= HTTP_DAV_CREATIONDATE;
    if (httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_ETAG]))
        properties |= HTTP_DAV_ETAG;
    return properties;
}

void httpDavAppendEmptyProperty(uint16_t properties,
    uint16_t property, const char* name)
{
    if ((properties & property) != 0)
    {
        httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_TAG_BEGIN]);
        httpResponseAppend(name);
        httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_EMPTY_END]);
    }
}

void httpDavAppendResponse(File& file, const char* filePath,
    uint16_t properties, bool propertyNames)
{
    bool directory = file.isDirectory();
    httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_RESPONSE_BEGIN]);
    httpAppendEncodedHref(filePath, directory);
    httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_HREF_END]);

    if (propertyNames)
    {
        httpDavAppendEmptyProperty(properties, HTTP_DAV_DISPLAYNAME,
            HTTP_STRINGS[HTTP_STR_DISPLAYNAME]);
        httpDavAppendEmptyProperty(properties, HTTP_DAV_RESOURCETYPE,
            HTTP_STRINGS[HTTP_STR_RESOURCETYPE]);
        httpDavAppendEmptyProperty(properties, HTTP_DAV_CONTENTLENGTH,
            HTTP_STRINGS[HTTP_STR_CONTENTLENGTH_PROP]);
        httpDavAppendEmptyProperty(properties, HTTP_DAV_CONTENTTYPE,
            HTTP_STRINGS[HTTP_STR_CONTENTTYPE_PROP]);
        httpDavAppendEmptyProperty(properties, HTTP_DAV_LASTMODIFIED,
            HTTP_STRINGS[HTTP_STR_LASTMODIFIED]);
        httpDavAppendEmptyProperty(properties, HTTP_DAV_CREATIONDATE,
            HTTP_STRINGS[HTTP_STR_CREATIONDATE]);
        httpDavAppendEmptyProperty(properties, HTTP_DAV_ETAG,
            HTTP_STRINGS[HTTP_STR_ETAG]);
    } else {
        if ((properties & HTTP_DAV_DISPLAYNAME) != 0)
        {
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_DISPLAY_BEGIN]);
            httpAppendXmlEscaped(
                (strcmp(filePath, "/") == 0) ?
                "/" : file.name());
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_DISPLAY_END]);
        }
        if ((properties & HTTP_DAV_RESOURCETYPE) != 0)
        {
            httpResponseAppend(directory ?
                HTTP_STRINGS[HTTP_STR_XML_RESOURCE_DIR] :
                HTTP_STRINGS[HTTP_STR_XML_RESOURCE_FILE]);
        }
        if ((properties & HTTP_DAV_CONTENTLENGTH) != 0)
        {
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_LENGTH_BEGIN]);
            httpResponseAppendNumber(directory ? 0 : file.size());
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_LENGTH_END]);
        }
        if ((properties & HTTP_DAV_CONTENTTYPE) != 0)
        {
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_TYPE_BEGIN]);
            httpResponseAppend(directory ?
                HTTP_STRINGS[HTTP_STR_DIRECTORY_TYPE] :
                HTTP_STRINGS[HTTP_STR_BINARY_TYPE]);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_TYPE_END]);
        }

        DateTimeFields modified;
        bool hasModified = file.getModifyTime(modified);
        if (((properties & HTTP_DAV_LASTMODIFIED) != 0) && hasModified)
        {
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_MODIFIED_BEGIN]);
            httpDavAppendHttpDate(modified);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_MODIFIED_END]);
        }
        DateTimeFields created;
        if (((properties & HTTP_DAV_CREATIONDATE) != 0) && file.getCreateTime(created))
        {
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_CREATED_BEGIN]);
            httpDavAppendIsoDate(created);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_CREATED_END]);
        }
        if ((properties & HTTP_DAV_ETAG) != 0)
        {
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_ETAG_BEGIN]);
            httpResponseAppendNumber(file.size(), true);
            if (hasModified)
            {
                httpResponseAppend('-');
                httpResponseAppendNumber(httpDavFullYear(modified.year), true);
                httpResponseAppendNumber(modified.mon + 1, true);
                httpResponseAppendNumber(modified.mday, true);
                httpResponseAppendNumber(modified.hour, true);
                httpResponseAppendNumber(modified.min, true);
                httpResponseAppendNumber(modified.sec, true);
            }
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_ETAG_END]);
        }
    }

    httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_PROP_OK]);
    httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_PROP_END]);
}

void httpDavSendResponse(File& file, const char* filePath,
    uint16_t properties, bool propertyNames)
{
    httpResponseBegin();
    httpDavAppendResponse(file, filePath, properties, propertyNames);
    httpSendResponse();
}

void httpDavSendDirectory(File& directory, const char* directoryPath,
    int depth, uint16_t properties, bool propertyNames)
{
    if (depth > 0)
    {
        File child = directory.openNextFile();
        while (child)
        {
            char childPath[MAX_PATH];
            int pathLength;
            if (strcmp(directoryPath, "/") == 0)
            {
                pathLength = snprintf(childPath, MAX_PATH, "/%s", child.name());
            } else {
                pathLength = snprintf(childPath, MAX_PATH, "%s/%s",
                    directoryPath, child.name());
            }
            if ((pathLength > 0) && (pathLength < (int)MAX_PATH))
            {
                bool childDirectory = child.isDirectory();
                httpDavSendResponse(child, childPath, properties, propertyNames);
                if (childDirectory && (depth < 0))
                {
                    httpDavSendDirectory(child, childPath, depth,
                        properties, propertyNames);
                }
            }
            child.close();
            child = directory.openNextFile();
        }
    }
}

void httpPerformPropfind(uint8_t* content, size_t size)
{
    bool directory = false;
    if (httpUploadBytesWritten >= CONTENT_BUFFER_SIZE)
    {
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_413],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else {
        File file = SD.open(httpURLPath, FILE_READ);
        if (file)
        {
            int depth = -1;
            const char* depthValue = httpFindHeaderValue(httpHeaderBuffer,
                HTTP_STRINGS[HTTP_STR_DEPTH]);
            if (depthValue != 0)
            {
                if (*depthValue == '0')
                {
                    depth = 0;
                } else if (*depthValue == '1')
                {
                    depth = 1;
                } else if ((tolower((unsigned char)depthValue[0]) != 'i'))
                {
                    file.close();
                    httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_400],
                        HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
                    httpFinishConnection();
                    return;
                }
            }

            bool propertyNames = (httpUploadBytesWritten > 0) &&
                httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_PROPNAME]);
            uint16_t properties = propertyNames ? HTTP_DAV_ALL_PROPERTIES :
                httpDavRequestedProperties(content, size);

            directory = file.isDirectory();
            httpResponseBegin();
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_DECL]);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_MULTI_BEGIN]);
            if (!directory)
            {
                httpDavAppendResponse(file, httpURLPath, properties, propertyNames);
                httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_MULTI_END]);
            }

            if (httpResponseOverflow)
            {
                httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_507],
                    HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
            } else {
                int headerSize;
                if (directory)
                {
                    // Directory output is streamed, so its length is not known up front
                    headerSize = snprintf(httpHeaderBuffer, PACKET_BUFFER_SIZE,
                        "%s%s%s%s", HTTP_STRINGS[HTTP_STR_HTTP_207],
                        HTTP_STRINGS[HTTP_STR_CONTENT_TYPE],
                        HTTP_STRINGS[HTTP_STR_XML_CONTENT_TYPE],
                        HTTP_STRINGS[HTTP_STR_CONNECTION_CLOSE]);
                } else {
                    headerSize = snprintf(httpHeaderBuffer, PACKET_BUFFER_SIZE,
                        "%s%s%d\r\n%s%s%s",
                        HTTP_STRINGS[HTTP_STR_HTTP_207],
                        HTTP_STRINGS[HTTP_STR_CONTENT_LENGTH], httpResponseLength,
                        HTTP_STRINGS[HTTP_STR_CONTENT_TYPE],
                        HTTP_STRINGS[HTTP_STR_XML_CONTENT_TYPE],
                        HTTP_STRINGS[HTTP_STR_CONNECTION_CLOSE]);
                }
                if ((headerSize > 0) &&
                    (headerSize < (int)PACKET_BUFFER_SIZE))
                {
                    sendData((const uint8_t*)httpHeaderBuffer, headerSize);
                    httpSendResponse();
                    if (directory)
                    {
                        httpDavSendResponse(file, httpURLPath,
                            properties, propertyNames);
                        httpDavSendDirectory(file, httpURLPath, depth,
                            properties, propertyNames);
                        httpSendText(HTTP_STRINGS[HTTP_STR_XML_MULTI_END]);
                    }
                }
            }
            file.close();
        } else {
            httpSend404();
        }
    }

    // Close
    httpFinishConnection();
    if (directory)
    {
        // Directory output is streamed, so its length is not known up front -
        // requires the connection to close
        httpCloseConnection();
    }
}

void httpPerformProppatch(uint8_t* content, size_t size)
{
    if (httpUploadBytesWritten >= CONTENT_BUFFER_SIZE)
    {
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_413],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else {
        File file = SD.open(httpURLPath, FILE_READ);
        if (file)
        {
            bool directory = file.isDirectory();
            file.close();
            if ((httpUploadBytesWritten == 0) ||
                (!httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_PROPERTYUPDATE])) ||
                ((!httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_SET_TAG])) &&
                 (!httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_SET_PREFIX])) &&
                 (!httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_REMOVE_TAG])) &&
                 (!httpBodyContains(content, size, HTTP_STRINGS[HTTP_STR_REMOVE_PREFIX]))))
            {
                httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_400],
                    HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
                httpFinishConnection();
                return;
            }

            uint16_t properties = httpDavRequestedProperties(content, size);
            httpResponseBegin();
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_DECL]);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_PATCH_BEGIN]);
            httpAppendEncodedHref(httpURLPath, directory);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_HREF_END]);
            httpDavAppendEmptyProperty(properties, HTTP_DAV_DISPLAYNAME,
                HTTP_STRINGS[HTTP_STR_DISPLAYNAME]);
            httpDavAppendEmptyProperty(properties, HTTP_DAV_RESOURCETYPE,
                HTTP_STRINGS[HTTP_STR_RESOURCETYPE]);
            httpDavAppendEmptyProperty(properties, HTTP_DAV_CONTENTLENGTH,
                HTTP_STRINGS[HTTP_STR_CONTENTLENGTH_PROP]);
            httpDavAppendEmptyProperty(properties, HTTP_DAV_CONTENTTYPE,
                HTTP_STRINGS[HTTP_STR_CONTENTTYPE_PROP]);
            httpDavAppendEmptyProperty(properties, HTTP_DAV_LASTMODIFIED,
                HTTP_STRINGS[HTTP_STR_LASTMODIFIED]);
            httpDavAppendEmptyProperty(properties, HTTP_DAV_CREATIONDATE,
                HTTP_STRINGS[HTTP_STR_CREATIONDATE]);
            httpDavAppendEmptyProperty(properties, HTTP_DAV_ETAG,
                HTTP_STRINGS[HTTP_STR_ETAG]);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_PROP_FORBIDDEN]);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_PROTECTED]);
            httpResponseAppend(HTTP_STRINGS[HTTP_STR_XML_PATCH_END]);
            if (httpResponseOverflow)
            {
                httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_507],
                    HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
            } else {
                int headerSize = snprintf(httpHeaderBuffer, PACKET_BUFFER_SIZE,
                    "%s%s%d\r\n%s%s%s",
                    HTTP_STRINGS[HTTP_STR_HTTP_207],
                    HTTP_STRINGS[HTTP_STR_CONTENT_LENGTH], httpResponseLength,
                    HTTP_STRINGS[HTTP_STR_CONTENT_TYPE],
                    HTTP_STRINGS[HTTP_STR_XML_CONTENT_TYPE],
                    HTTP_STRINGS[HTTP_STR_CONNECTION_CLOSE]);
                if ((headerSize > 0) &&
                    (headerSize < (int)PACKET_BUFFER_SIZE))
                {
                    sendData((const uint8_t*)httpHeaderBuffer, headerSize);
                    httpSendResponse();
                }
            }
        } else {
            httpSend404();
        }
    }

    // Close
    httpFinishConnection();
}

void httpPerformMkcol()
{
    if (SD.exists(httpURLPath))
    {
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_405],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else if (SD.mkdir(httpURLPath))
    {
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_201],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else {
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_409],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    }

    // Close
    httpFinishConnection();
}

bool httpPathIsCanonical(const char* path)
{
    if (*path != '/')
    {
        return false;
    }
    while (*path != 0)
    {
        if ((*path == '\\') ||
            ((*path == '/') && (path[1] == '/')) ||
            ((*path == '/') && (path[1] == '.') &&
             ((path[2] == 0) || (path[2] == '/'))) ||
            ((*path == '/') && (path[1] == '.') && (path[2] == '.') &&
             ((path[3] == 0) || (path[3] == '/'))))
        {
            return false;
        }
        ++path;
    }
    return true;
}

bool httpGetDestination(char* destination)
{
    const char* value = httpFindHeaderValue(httpHeaderBuffer,
        HTTP_STRINGS[HTTP_STR_DESTINATION]);
    if (value != 0)
    {
        // Obtain the encoded path
        char encoded[MAX_PATH];
        size_t length = 0;
        if (*value == '<')
        {
            ++value;
        }
        while ((value[length] != 0) && (value[length] != '\r') &&
            (value[length] != '\n') && (value[length] != '>'))
        {
            if (length >= (MAX_PATH - 1))
            {
                return false;
            }
            encoded[length] = value[length];
            ++length;
        }
        encoded[length] = 0;

        // Remove the "http://"
        const char* path = encoded;
        const char* scheme = strstr(encoded, "://");
        if (scheme != 0)
        {
            path = strchr(scheme + 3, '/');
            if (path == 0)
            {
                return false;
            }
        }
        if (*path != '/')
        {
            return false;
        }

        // Decode the URL from the Destination
        size_t pathLength = strcspn(path, "?#");
        if ((pathLength > 0) && (pathLength < MAX_PATH))
        {
            memcpy(destination, path, pathLength);
            destination[pathLength] = 0;
            pathLength = httpGetUrlPath(destination, destination);
            return (pathLength > 0) && httpPathIsCanonical(destination);
        }
    }
    return false;
}

bool httpPathIsDescendant(const char* path, const char* parent)
{
    size_t parentLength = strlen(parent);
    if ((parentLength == 0) || (strlen(path) <= parentLength) ||
        (path[parentLength] != '/'))
    {
        return false;
    }
    for (size_t i = 0; i < parentLength; ++i)
    {
        if (tolower((unsigned char)path[i]) !=
            tolower((unsigned char)parent[i]))
        {
            return false;
        }
    }
    return true;
}

bool httpDestinationParentExists(const char* destination)
{
    char parent[MAX_PATH];
    snprintf(parent, MAX_PATH, "%s", destination);
    char* separator = strrchr(parent, '/');
    if (separator == 0)
    {
        return false;
    }
    if (separator == parent)
    {
        parent[1] = 0;
    } else {
        *separator = 0;
    }

    File file = SD.open(parent, FILE_READ);
    bool exists = file && file.isDirectory();
    file.close();
    return exists;
}

bool httpCopyPath(const char* sourcePath, const char* destinationPath,
    int depth)
{
    bool success = false;
    File source = SD.open(sourcePath, FILE_READ);
    if (source)
    {
        if (source.isDirectory())
        {
            if (!SD.mkdir(destinationPath))
            {
                source.close();
            } else {
                if (depth != 0)
                {
                    File child = source.openNextFile();
                    while (child)
                    {
                        char sourceChild[MAX_PATH];
                        char destinationChild[MAX_PATH];
                        int sourceLength = snprintf(sourceChild, sizeof(sourceChild),
                            "%s/%s", sourcePath, child.name());
                        int destinationLength = snprintf(destinationChild,
                            sizeof(destinationChild), "%s/%s", destinationPath,
                            child.name());
                        child.close();
                        if ((sourceLength <= 0) ||
                            (sourceLength >= (int)sizeof(sourceChild)) ||
                            (destinationLength <= 0) ||
                            (destinationLength >= (int)sizeof(destinationChild)) ||
                            !httpCopyPath(sourceChild, destinationChild, depth))
                        {
                            source.close();
                            httpPerformDelete(destinationPath, true);
                            return false;
                        }
                        child = source.openNextFile();
                    }
                }
                source.close();
                success = true;
            }
        } else {
            File destination = SD.open(destinationPath, FILE_WRITE_BEGIN);
            success = destination;
            while (success && source.available())
            {
                int count = source.read(httpPacketBuffer, PACKET_BUFFER_SIZE);
                success = (count > 0) &&
                    (destination.write(httpPacketBuffer, count) == (size_t)count);
            }
            destination.close();
            source.close();
            if (!success)
            {
                SD.remove(destinationPath);
            }
        }
    }
    return success;
}

void httpSendCopyMoveResult(bool created)
{
    if (created)
    {
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_201],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else {
        httpSendText(HTTP_STRINGS[HTTP_STR_HTTP_204]);
    }
}

void httpPerformMove()
{
    char destination[MAX_PATH];
    if (!httpGetDestination(destination))
    {
        // No destination given
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_400],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else if (!SD.exists(httpURLPath))
    {
        // Source does not exist
        httpSend404();
    } else if (!httpPathIsCanonical(httpURLPath) ||
        (strcmp(httpURLPath, "/") == 0) ||
        (stricmp(httpURLPath, destination) == 0) ||
        httpPathIsDescendant(destination, httpURLPath) ||
        httpPathIsDescendant(httpURLPath, destination))
    {
        // Path is invalid
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_403],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else if (!httpDestinationParentExists(destination))
    {
        // Target destination does not exist
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_409],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else {
        bool destinationExists = SD.exists(destination);
        const char* overwrite = httpFindHeaderValue(httpHeaderBuffer,
            HTTP_STRINGS[HTTP_STR_OVERWRITE]);
        if (destinationExists && (overwrite != 0) &&
            (toupper((unsigned char)*overwrite) == 'F'))
        {
            // Destination already exists, and not overwriting
            httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_412],
                HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
        } else {
            if (destinationExists)
            {
                httpPerformDelete(destination, true);
            }
            if (SD.exists(destination) || !SD.rename(httpURLPath, destination))
            {
                httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_507],
                    HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
            } else {
                httpSendCopyMoveResult(!destinationExists);
            }
        }
    }

    // Close
    httpFinishConnection();
}

void httpPerformCopy()
{
    char destination[MAX_PATH];
    const char* depthValue = httpFindHeaderValue(httpHeaderBuffer,
        HTTP_STRINGS[HTTP_STR_DEPTH]);
    bool validDepth = (depthValue == 0) || (*depthValue == '0') ||
        (tolower((unsigned char)*depthValue) == 'i');
    if (!validDepth || !httpGetDestination(destination))
    {
        // Invalid depth, or no destination given
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_400],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else if (!SD.exists(httpURLPath))
    {
        // Source does not exist
        httpSend404();
    } else if (!httpPathIsCanonical(httpURLPath) ||
        (strcmp(httpURLPath, "/") == 0) ||
        (stricmp(httpURLPath, destination) == 0) ||
        httpPathIsDescendant(destination, httpURLPath) ||
        httpPathIsDescendant(httpURLPath, destination))
    {
        // Path is invalid
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_403],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else if (!httpDestinationParentExists(destination))
    {
        // Target destination does not exist
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_409],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    } else {
        bool destinationExists = SD.exists(destination);
        const char* overwrite = httpFindHeaderValue(httpHeaderBuffer,
            HTTP_STRINGS[HTTP_STR_OVERWRITE]);
        if (destinationExists && (overwrite != 0) &&
            (toupper((unsigned char)*overwrite) == 'F'))
        {
            // Destination already exists, and not overwriting
            httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_412],
                HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
        } else {
            if (destinationExists)
            {
                httpPerformDelete(destination, true);
            }
            int depth = ((depthValue != 0) && (*depthValue == '0')) ? 0 : -1;
            if (SD.exists(destination) ||
                !httpCopyPath(httpURLPath, destination, depth))
            {
                httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_507],
                    HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
            } else {
                httpSendCopyMoveResult(!destinationExists);
            }
        }
    }

    // Close
    httpFinishConnection();
}

void httpPerformDelete(const char* decodedPath, bool isChild)
{
    File file = SD.open(decodedPath);
    if (file)
    {
        // Recurse directories, and delete all files
        if (file.isDirectory())
        {
            File child = file.openNextFile();
            while (child)
            {
                char childPath[MAX_PATH];
                if (snprintf(childPath, MAX_PATH, "%s/%s", decodedPath,
                    child.name()) >= (int)MAX_PATH)
                {
                    child.close();
                } else {
                    child.close();
                    httpPerformDelete(childPath, true);
                }
                child = file.openNextFile();
            }
            file.close();

            // Remove the empty directory
            SD.rmdir(decodedPath);
        } else {
            file.close();

            // Remove the file
            SD.remove(decodedPath);
        }
    }

    // Return status, and close
    if (!isChild)
    {
        httpSendText(HTTP_STRINGS[HTTP_STR_HTTP_204]);
        httpFinishConnection();
    }
}

void httpFinishPut()
{
    // Close the file, and return status
    if (httpUploadFile)
    {
        httpUploadFile.close();
        if (httpUploadCreated)
        {
            httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_201],
                HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
        } else {
            httpSendText(HTTP_STRINGS[HTTP_STR_HTTP_204]);
        }
    } else {
        httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_507],
            HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
    }

    // Close
    httpFinishConnection();
}

void httpContinuePut(uint8_t* data, size_t size)
{
    if ((size > 0) && httpUploadFile)
    {
        size_t bytesWritten = httpUploadFile.write(data, size);
        if (bytesWritten != size)
        {
            // Close the file on write error
            httpUploadFile.close();
        } else {
            httpUploadBytesWritten += bytesWritten;
        }
    }
    if (size < httpUploadContentLength)
    {
        httpUploadContentLength -= size;
    } else {
        httpUploadContentLength = 0;
        httpFinishPut();
    }
}

void httpPerformPut(size_t contentLength, uint8_t* data, size_t size)
{
    // Prepare to receive data
    httpUploadBytesWritten = 0;
    httpUploadContentLength = contentLength;

    // Determine if the file exists, or is being overwritten
    if (SD.exists(httpURLPath))
    {
        bool validPath = false;
        httpUploadCreated = false;
        httpUploadFile = SD.open(httpURLPath, FILE_READ);
        if (httpUploadFile)
        {
            if (!httpUploadFile.isDirectory())
            {
                validPath = true;
            }
            httpUploadFile.close();
        }
        if (!validPath)
        {
            httpSendHeader(HTTP_STRINGS[HTTP_STR_HTTP_405],
                HTTP_STRINGS[HTTP_STR_TEXT_PLAIN], 0);
            httpFinishConnection();
            return;
        }
    } else {
        httpUploadCreated = true;
    }

    // Open the file for writing
    httpUploadFile = SD.open(httpURLPath, FILE_WRITE_BEGIN);
    if (httpUploadFile && !httpUploadCreated)
    {
        httpUploadFile.seek(0, SeekSet);
        httpUploadFile.truncate();
    }

    // Windows WebClient commonly waits for this interim response before it
    // transmits the request body
    if ((contentLength > size) &&
        (httpFindHeaderValue(httpHeaderBuffer, HTTP_STRINGS[HTTP_STR_EXPECT]) != 0))
    {
        httpSendText(HTTP_STRINGS[HTTP_STR_HTTP_100]);
    }

    // Write any existing content
    httpContinuePut(data, size);
}

void httpFinishAction()
{
    switch (httpAction)
    {
        case HTTP_ACTION_OPTIONS :
            httpPerformOptions();
            break;
        case HTTP_ACTION_GET :
            httpPerformGet(true);
            break;
        case HTTP_ACTION_HEAD :
            httpPerformGet(false);
            break;
        case HTTP_ACTION_PROPFIND :
            httpPerformPropfind(httpContentBuffer, httpUploadBytesWritten);
            break;
        case HTTP_ACTION_PROPPATCH :
            httpPerformProppatch(httpContentBuffer, httpUploadBytesWritten);
            break;
        case HTTP_ACTION_NOT_IMPLEMENTED :
            httpSendNotImplemented();
            httpFinishConnection();
            break;
        case HTTP_ACTION_MKCOL :
            httpPerformMkcol();
            break;
        case HTTP_ACTION_MOVE :
            httpPerformMove();
            break;
        case HTTP_ACTION_COPY :
            httpPerformCopy();
            break;
        case HTTP_ACTION_DELETE :
            httpPerformDelete(httpURLPath, false);
            break;
        default :
            // Unknown action, and close
            httpSend404();
            httpFinishConnection();
            break;
    }
}

void httpContinueAction(uint8_t* data, size_t size)
{
    // Write data into buffer
    if ((size > 0) && (httpUploadBytesWritten < CONTENT_BUFFER_SIZE))
    {
        size_t bytesWritten;
        if (size < (CONTENT_BUFFER_SIZE - httpUploadBytesWritten))
        {
            bytesWritten = size;
        } else {
            bytesWritten = (CONTENT_BUFFER_SIZE - httpUploadBytesWritten);
        }
        memcpy(&(httpContentBuffer[httpUploadBytesWritten]), data, bytesWritten);
        httpUploadBytesWritten += bytesWritten;
    }
    if (size < httpUploadContentLength)
    {
        httpUploadContentLength -= size;
    } else {
        httpUploadContentLength = 0;
        httpFinishAction();
    }
}

void httpPerformAction(size_t contentLength, uint8_t* data, size_t size)
{
    // Prepare to receive data
    httpUploadBytesWritten = 0;
    httpUploadContentLength = contentLength;

    // Write any existing content
    httpContinueAction(data, size);
}

char* httpGetAction(void* action)
{
    http_action_t* httpAction = (http_action_t*)action;
    char* request = (char*)httpPacketBuffer;
    switch (httpPacketBuffer[0])
    {
        case 'C' :
            if (strncmp(HTTP_STRINGS[HTTP_STR_COPY], (const char*)httpPacketBuffer, 5) == 0)
            {
                *httpAction = HTTP_ACTION_COPY;
                return request + 5;
            }
            break;
        case 'D' :
            if (strncmp(HTTP_STRINGS[HTTP_STR_DELETE], (const char*)httpPacketBuffer, 7) == 0)
            {
                *httpAction = HTTP_ACTION_DELETE;
                return request + 7;
            }
            break;
        case 'G' :
            if (strncmp(HTTP_STRINGS[HTTP_STR_GET], (const char*)httpPacketBuffer, 4) == 0)
            {
                *httpAction = HTTP_ACTION_GET;
                return request + 4;
            }
            break;
        case 'H' :
            if (strncmp(HTTP_STRINGS[HTTP_STR_HEAD], (const char*)httpPacketBuffer, 5) == 0)
            {
                *httpAction = HTTP_ACTION_HEAD;
                return request + 5;
            }
            break;
        case 'M' :
            if (strncmp(HTTP_STRINGS[HTTP_STR_MOVE], (const char*)httpPacketBuffer, 5) == 0)
            {
                *httpAction = HTTP_ACTION_MOVE;
                return request + 5;
            } else if (strncmp(HTTP_STRINGS[HTTP_STR_MKCOL], (const char*)httpPacketBuffer, 6) == 0)
            {
                *httpAction = HTTP_ACTION_MKCOL;
                return request + 6;
            }
            break;
        case 'O' :
            if (strncmp(HTTP_STRINGS[HTTP_STR_OPTIONS], (const char*)httpPacketBuffer, 8) == 0)
            {
                *httpAction = HTTP_ACTION_OPTIONS;
                return request + 8;
            }
            break;
        case 'P' :
            if (strncmp(HTTP_STRINGS[HTTP_STR_PUT], (const char*)httpPacketBuffer, 4) == 0)
            {
                *httpAction = HTTP_ACTION_PUT;
                return request + 4;
            } else if (strncmp(HTTP_STRINGS[HTTP_STR_PROPFIND], (const char*)httpPacketBuffer, 9) == 0)
            {
                *httpAction = HTTP_ACTION_PROPFIND;
                return request + 9;
            } else if (strncmp(HTTP_STRINGS[HTTP_STR_PROPPATCH], (const char*)httpPacketBuffer, 10) == 0)
            {
                *httpAction = HTTP_ACTION_PROPPATCH;
                return request + 10;
            }
            break;
        default :
            break;
    }
    *httpAction = HTTP_ACTION_NOT_IMPLEMENTED;
    char* path = strchr(request, ' ');
    return (path == 0) ? request : path + 1;
}

void httpProcessPacket()
{
    if ((httpPacketCount > 0) || (httpAction != HTTP_ACTION_UNKNOWN))
    {
        switch (httpAction)
        {
            case HTTP_ACTION_PUT :
                httpContinuePut(httpPacketBuffer, httpPacketBufferIndex);
                break;
            default :
                httpContinueAction(httpPacketBuffer, httpPacketBufferIndex);
                break;
        }
    } else {
        // Find end of the HTTP header
        uint8_t* content = (uint8_t*)strstr((const char*)httpPacketBuffer, "\r\n\r\n");
        if (content != 0)
        {
            *content = 0;
            content += 4;
            char* path = httpGetAction(&httpAction);
            if (httpAction != HTTP_ACTION_UNKNOWN)
            {
                char* ptr = strstr(path, " ");
                if (ptr != 0)
                {
                    // Decode the URL from the header
                    *ptr = 0;
                    httpGetUrlPath(httpURLPath, path);

                    // Store the HTTP header
                    snprintf(httpHeaderBuffer, PACKET_BUFFER_SIZE, "%s", ptr + 1);

                    // Obtain the length of content
                    size_t contentLength;
                    ptr = strstr(ptr + 1, HTTP_STRINGS[HTTP_STR_CONTENT_LENGTH]);
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

                    // Capture the content
                    size_t size = (&(httpPacketBuffer[httpPacketBufferIndex]) - content);
                    switch (httpAction)
                    {
                        case HTTP_ACTION_PUT :
                            httpPerformPut(contentLength, content, size);
                            break;
                        default :
                            httpPerformAction(contentLength, content, size);
                            break;
                    }
                }
            }
        }
    }
}

void httpOnTick()
{
    if (httpEnabled)
    {
        while (Serial8.available())
        {
            if (httpReceivingPacket)
            {
                // Fetch all available into the buffer
                int size = ((httpPacketLength > PACKET_BUFFER_SIZE) ?
                    PACKET_BUFFER_SIZE : httpPacketLength) - httpPacketBufferIndex;
                if (size > Serial8.available())
                {
                    size = Serial8.available();
                }
                size = Serial8.readBytes(&(httpPacketBuffer[httpPacketBufferIndex]), size);
                if (size > 0)
                {
                    httpPacketBufferIndex += size;
                }

                // Decide if a packet has been received
                if ((httpPacketBufferIndex >= httpPacketLength) ||
                    (httpPacketBufferIndex >= PACKET_BUFFER_SIZE))
                {
                    httpProcessPacket();
                    httpPacketLength -= httpPacketBufferIndex;
                    if (httpPacketLength == 0)
                    {
                        httpReceivingPacket = false;
                        httpPacketCount = 0;
                    } else {
                        ++httpPacketCount;
                    }

                    // Clear the buffer
                    httpPacketBufferIndex = 0;
                }
            } else {
                // Fetch a byte into the buffer
                uint8_t c = Serial8.read();
                httpPacketBuffer[httpPacketBufferIndex] = c;
                ++httpPacketBufferIndex;

                // Decide if a line or packet has been received
                if ((httpPacketBufferIndex >= (PACKET_BUFFER_SIZE - 1)) ||
                    (c == ':') || (c == '\n'))
                {
                    // Parse for an incoming packet
                    httpPacketBuffer[httpPacketBufferIndex] = 0;
                    if (strncmp("+IPD,", (const char*)httpPacketBuffer, 5) == 0)
                    {
                        char* ptr = (char*)&(httpPacketBuffer[5]);
                        char* lenPtr = strstr(ptr, ",");
                        if (lenPtr != 0)
                        {
                            *lenPtr = 0;
                            strncpy(httpConnectionId, ptr, 8);
                            httpConnectionId[7] = 0;
                            ptr = lenPtr + 1;
                            httpPacketLength = atoi(ptr);
                            httpReceivingPacket = true;
                            httpPacketCount = 0;
                        }
                    }

                    // Clear the buffer
                    httpPacketBufferIndex = 0;
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
            httpServerStatus = HTTP_STRINGS[HTTP_STR_WAIT_NTP];
            return;
        }

        // Close the UART, and open the port exclusively
        espUart.end();

        // Open the UART, and start the server
        Serial8.begin(115200);
        Serial8.println(HTTP_STRINGS[HTTP_STR_AT_ECHO_OFF]);
        httpWaitFor(HTTP_STRINGS[HTTP_STR_AT_OK]);
        Serial8.println(HTTP_STRINGS[HTTP_STR_AT_IP]);
        if (httpWaitFor(HTTP_STRINGS[HTTP_STR_AT_IP_REPLY]))
        {
            String ipAddress = Serial8.readStringUntil('\"');
            if (ipAddress != HTTP_STRINGS[HTTP_STR_NO_IP])
            {
                httpServerStatus = HTTP_STRINGS[HTTP_STR_ADDRESS] + ipAddress;
                httpWaitFor(HTTP_STRINGS[HTTP_STR_AT_OK]);
                Serial8.println(HTTP_STRINGS[HTTP_STR_AT_MUX]);
                httpWaitFor(HTTP_STRINGS[HTTP_STR_AT_OK]);
                Serial8.println(HTTP_STRINGS[HTTP_STR_AT_SERVER_START]);
                if (httpWaitFor(HTTP_STRINGS[HTTP_STR_AT_OK]))
                {
                    httpEnabled = true;
                }
            } else {
                httpServerStatus = HTTP_STRINGS[HTTP_STR_WAIT_IP];
            }
        } else {
            httpServerStatus = HTTP_STRINGS[HTTP_STR_WAIT_WIFI];
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
        Serial8.println(HTTP_STRINGS[HTTP_STR_AT_SERVER_STOP]);
        httpWaitFor(HTTP_STRINGS[HTTP_STR_AT_OK]);
        Serial8.end();
    }

    // Close any partial upload even if the server has already transitioned
    // out of its enabled state.
    if (httpUploadFile)
    {
        httpUploadFile.close();
    }
    httpServerStatus.remove(0);
    httpEnabled = false;
    httpAction = HTTP_ACTION_UNKNOWN;
    httpReceivingPacket = false;
    httpPacketBufferIndex = 0;
    httpPacketLength = 0;
    httpPacketCount = 0;
}
