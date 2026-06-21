
#ifndef PRINTABLE_STRING_H
#define PRINTABLE_STRING_H

class PrintableString : public Print
{
    protected :
        String buffer;

    public :
        size_t write(uint8_t c)
        {
            buffer.concat(char(c));
            return 1;
        }

        size_t write(uint8_t* str, uint8_t length)
        {
            buffer.concat((char*)str);
            return length;
        }

        size_t size()
        {
            return buffer.length();
        }

        void clear()
        {
            buffer = "";
        }

        const char* c_str()
        {
            return buffer.c_str();
        }
};

#endif
