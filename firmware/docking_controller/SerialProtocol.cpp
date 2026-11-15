#include "SerialProtocol.h"
#include "Config.h"
#include <stdlib.h>
#include <string.h>

namespace
{
    // Fixed buffer, no String: the Uno has 2 KB of RAM.
    char sLine[48];
    uint8_t sLen = 0;
    bool sOverflow = false;

    HostInput parse(char* aLine)
    {
        HostInput in;
        if (aLine[0] == 'V' && aLine[1] == ',')
        {
            char* p = aLine + 2;
            char* end = nullptr;
            float closing = (float)strtod(p, &end);
            if (end == p || *end != ',') return in;
            p = end + 1;
            float gap = (float)strtod(p, &end);
            if (end == p) return in;
            in.gotVelocity = true;
            in.closing = closing;
            in.gap = gap;
        }
        else if (aLine[0] == 'C' && aLine[1] == ',')
        {
            const char* c = aLine + 2;
            if      (strcmp(c, "ARM") == 0)    in.command = HostCommand::ARM;
            else if (strcmp(c, "DISARM") == 0) in.command = HostCommand::DISARM;
            else if (strcmp(c, "LOCK") == 0)   in.command = HostCommand::LOCK;
            else if (strcmp(c, "UNLOCK") == 0) in.command = HostCommand::UNLOCK;
            else SerialProtocol::log("unknown command");
        }
        return in;
    }
}

namespace SerialProtocol
{
    void begin()
    {
        Serial.begin(SERIAL_BAUD);
    }

    HostInput poll()
    {
        while (Serial.available() > 0)
        {
            char ch = (char)Serial.read();
            if (ch == '\r') continue;
            if (ch == '\n')
            {
                HostInput in;
                if (!sOverflow && sLen > 0)
                {
                    sLine[sLen] = '\0';
                    in = parse(sLine);
                }
                sLen = 0;
                sOverflow = false;
                if (in.gotVelocity || in.command != HostCommand::NONE) return in;
                continue;
            }
            if (sLen < sizeof(sLine) - 1) sLine[sLen++] = ch;
            else sOverflow = true;  // drop the rest of an over-long line
        }
        return HostInput();
    }

    void sendStatus(uint8_t aState, bool aLocked, bool aHall1, bool aHall2, bool aEstop,
                    uint16_t aCycles, uint8_t aPwm, uint8_t aFault)
    {
        Serial.print(F("S,"));
        Serial.print(aState);   Serial.print(',');
        Serial.print(aLocked ? 1 : 0); Serial.print(',');
        Serial.print(aHall1 ? 1 : 0);  Serial.print(',');
        Serial.print(aHall2 ? 1 : 0);  Serial.print(',');
        Serial.print(aEstop ? 1 : 0);  Serial.print(',');
        Serial.print(aCycles);  Serial.print(',');
        Serial.print(aPwm);     Serial.print(',');
        Serial.print(aFault);   Serial.print('\n');
    }

    void log(const char* aText)
    {
        Serial.print(F("L,"));
        Serial.print(aText);
        Serial.print('\n');
    }
}
