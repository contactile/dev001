
#include "Commands.h"
#include <Arduino.h>

namespace Commands
{       
    SerialParser parser;
    static const int bufferSize = 128;
    static char buffer[bufferSize];
    static char lastCommand[bufferSize];
    static char cmdResponse[SerialParser::MAX_RESPONSE_SIZE];
    static int serialIn = 0;

    /// @brief SerialParser command handler
    /// @param c byte read from serial port
    void handler(char c)
    {  
        if(serialIn > bufferSize)
        {
        Serial.println("Error: Maximum buffer size exceeded");
        serialIn = 0;
        return;
        }

        // ignore some chars
        if(c == '\r')
        return;

        // Add all chars to buffer except the terminator
        if(c != '\n')
        {
        // Add char to buffer
        buffer[serialIn++] = c;
        return;
        }

        // Add a null terminator
        buffer[serialIn] = '\0';

        // Copy command if it is not the repeat command
        if (buffer[0] != '.')
        strcpy(lastCommand, buffer);

        // Parse buffer for commands    
        parser.processCommand(buffer, cmdResponse);
        if(cmdResponse[0] != 0)        
        Serial.println(cmdResponse);        

        // Reset pointer after handling command
        serialIn = 0;     
    }
}