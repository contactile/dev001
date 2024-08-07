/*
  CommandParser.h - Library for parsing commands of the form "COMMAND_NAME ARG1 ARG2 ARG3 ...".

  Copyright 2020 Anthony Zhang (Uberi) <me@anthonyz.ca>

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __COMMAND_PARSER_H__
#define __COMMAND_PARSER_H__

#include <limits.h>
#include <stdint.h>
#include <cstring>
#include <cstdio>

/*
#include <cstring>
size_t strlcpy(char *dst, const char *src, size_t size) {
    *dst = '\0';
    strncat(dst, src, size - 1);
    return strlen(dst);
}
*/

// avr-libc lacks strtoll and strtoull (see https://www.nongnu.org/avr-libc/user-manual/group__avr__stdlib.html), so we'll implement our own to be compatible with AVR boards such as the Arduino Uno
// typically you would use this like: `int64_t result; size_t bytesRead = strToInt<int64_t>("-0x123", &result, std::numeric_limits<int64_t>::min(), std::numeric_limits<int64_t>::max())`
// if an error occurs during parsing, `bytesRead` will be 0 and `result` will be an arbitrary value
template<typename T> size_t strToInt(const char* buf, T *value, T min_value, T max_value) {
    size_t position = 0;

    // parse sign if necessary
    bool isNegative = false;
    if ((min_value < 0) && ((buf[position] == '+') || (buf[position] == '-')) ) {
        isNegative = buf[position] == '-';
        position ++;
    }

    // parse base identifier if necessary
    int base = 10;
    if (buf[position] == '0' && buf[position + 1] == 'b') {
        base = 2;
        position += 2;
    } else if (buf[position] == '0' && buf[position + 1] == 'o') {
        base = 8;
        position += 2;
    } else if (buf[position] == '0' && buf[position + 1] == 'x') {
        base = 16;
        position += 2;
    }

    int digit = -1;
    *value = 0;
    while (true) {
        // obtain the next digit of the number
        if      (base >= 2  && '0' <= buf[position] && buf[position] <= '1') { digit = buf[position] - '0'; }
        else if (base >= 8  && '2' <= buf[position] && buf[position] <= '7') { digit = (buf[position] - '2') + 2; }
        else if (base >= 10 && '8' <= buf[position] && buf[position] <= '9') { digit = (buf[position] - '8') + 8; }
        else if (base >= 16 && 'a' <= buf[position] && buf[position] <= 'f') { digit = (buf[position] - 'a') + 10; }
        else if (base >= 16 && 'A' <= buf[position] && buf[position] <= 'F') { digit = (buf[position] - 'A') + 10; }
        else { break; }

        if (*value < min_value / base || *value > max_value / base) { return 0; } // integer multiplication underflow/overflow, fail gracefully
        *value *= base;
        if (isNegative ? *value < min_value + digit : *value > max_value - digit) { return 0; } // integer subtraction-underflow/addition-overflow, fail gracefully
        *value += digit;

        position ++;
    }
    return digit == -1 ? 0 : position; // ensure that there is at least one digit
}

template<size_t COMMANDS = 30, size_t COMMAND_ARGS = 6, size_t COMMAND_NAME_LENGTH = 16, size_t COMMAND_ARG_SIZE = 32, size_t RESPONSE_SIZE = 64>
class CommandParser {
    public:
        static const size_t MAX_COMMANDS = COMMANDS;
        static const size_t MAX_COMMAND_ARGS = COMMAND_ARGS;
        static const size_t MAX_COMMAND_NAME_LENGTH = COMMAND_NAME_LENGTH;
        static const size_t MAX_COMMAND_ARG_SIZE = COMMAND_ARG_SIZE;
        static const size_t MAX_RESPONSE_SIZE = RESPONSE_SIZE;

        union Argument {
            double asDouble;
            uint64_t asUInt64;
            int64_t asInt64;
            char asString[MAX_COMMAND_ARG_SIZE + 1];
        };

        void printErrors(bool enable)
        {
            _printErrors = enable;
        }
    private:
        struct Command {
            char name[MAX_COMMAND_NAME_LENGTH + 1];
            char argTypes[MAX_COMMAND_ARGS + 1];
            char argOptional[MAX_COMMAND_ARGS + 1];
            void (*callback)(union Argument *args, char numArgs, char *response);
        };

        union Argument commandArgs[MAX_COMMAND_ARGS];
        struct Command commandDefinitions[MAX_COMMANDS];
        size_t numCommands = 0;
        bool _printErrors = true;

        size_t parseString(const char *buf, char *output) {
            size_t readCount = 0;
            bool isQuoted = buf[0] == '"'; // whether the string is quoted or just a plain word
            if (isQuoted) {
                readCount ++; // move past the opening quote
            }

            size_t i = 0;
            for (; i < MAX_COMMAND_ARG_SIZE && buf[readCount] != '\0'; i ++) { // loop through each character of the string literal
                if (isQuoted ? buf[readCount] == '"' : buf[readCount] == ' ') {
                    break;
                }
                if (buf[readCount] == '\\') { // start of the escape sequence
                    readCount ++; // move past the backslash
                    switch (buf[readCount]) { // check what kind of escape sequence it is, turn it into the correct character
                        case 'n': output[i] = '\n'; readCount ++; break;
                        case 'r': output[i] = '\r'; readCount ++; break;
                        case 't': output[i] = '\t'; readCount ++; break;
                        case '"': output[i] = '"'; readCount ++; break;
                        case '\\': output[i] = '\\'; readCount ++; break;
                        case 'x': { // hex escape, of the form \xNN where NN is a byte in hex
                            readCount ++; // move past the "x" character
                            output[i] = 0;
                            for (size_t j = 0; j < 2; j ++, readCount ++) {
                                if      ('0' <= buf[readCount] && buf[readCount] <= '9') { output[i] = output[i] * 16 + (buf[readCount] - '0'); }
                                else if ('a' <= buf[readCount] && buf[readCount] <= 'f') { output[i] = output[i] * 16 + (buf[readCount] - 'a') + 10; }
                                else if ('A' <= buf[readCount] && buf[readCount] <= 'F') { output[i] = output[i] * 16 + (buf[readCount] - 'A') + 10; }
                                else { return 0; }
                            }
                            break;
                        }
                        default: // unknown escape sequence
                            return 0;
                    }
                } else { // non-escaped character
                    output[i] = buf[readCount];
                    readCount ++;
                }
            }
            if (isQuoted) {
                if (buf[readCount] != '"') { return 0; }
                readCount ++; // move past the closing quote
            }

            output[i] = '\0';
            return readCount;
        }
    public:
        bool registerCommand(const char *name, const char *argTypes, void (*callback)(union Argument *args, char numArgs, char *response), const char* optional = 0) {
            if (numCommands == MAX_COMMANDS) { return false; }
            if (strlen(name) > MAX_COMMAND_NAME_LENGTH) { return false; }
            if (strlen(argTypes) > MAX_COMMAND_ARGS) { return false; }
            if (callback == nullptr) { return false; }
            for (size_t i = 0; argTypes[i] != '\0'; i ++) {
                switch (argTypes[i]) {
                    case 'd':
                    case 'u':
                    case 'i':
                    case 's':
                        break;
                    default:
                        return false;
                }
            }

            strlcpy(commandDefinitions[numCommands].name, name, MAX_COMMAND_NAME_LENGTH + 1);
            strlcpy(commandDefinitions[numCommands].argTypes, argTypes, MAX_COMMAND_ARGS + 1);
            strlcpy(commandDefinitions[numCommands].argOptional, optional, MAX_COMMAND_ARGS + 1);
            commandDefinitions[numCommands].callback = callback;
            numCommands ++;
            return true;
        }

        bool processCommand(const char *command, char *response) {
            // retrieve the command name
            char name[MAX_COMMAND_NAME_LENGTH + 1];
            size_t i = 0;
            for (; i < MAX_COMMAND_NAME_LENGTH && *command != ' ' && *command != '\0'; i ++, command ++) { name[i] = *command; }
            name[i] = '\0';

            // look up the command argument types and callback
            char numArgs = 0;
            char *argTypes = nullptr;
            char *argOptional = nullptr;
            void (*callback)(union Argument *, char, char *) = nullptr;
            for (size_t i = 0; i < numCommands; i ++) {
                if (strcmp(commandDefinitions[i].name, name) == 0) {
                    argTypes = commandDefinitions[i].argTypes;
                    argOptional = commandDefinitions[i].argOptional;
                    callback = commandDefinitions[i].callback;
                    break;
                }
            }
            if (argTypes == nullptr) {

                if(_printErrors)
                    snprintf(response, MAX_RESPONSE_SIZE, "parse error: unknown command name %s", name);
                return false;
            }

            // parse each command
            for (size_t i = 0; argTypes[i] != '\0'; i ++) {

                // require and skip 1 or more whitespace characters
                if (*command != ' ') 
                {
                    if(argOptional != 0 && argOptional[i] == 'y')
                    {
                        // skip whitespace
                        while (*command == ' ') { command ++; }

                        // ensure that we're at the end of the command
                        if (*command != '\0') {
                            if(_printErrors)
                                snprintf(response, MAX_RESPONSE_SIZE, "parse error: too many args (expected max %d)", strlen(argTypes));
                            return false;
                        }

                        // set response to empty string
                        response[0] = '\0';

                        // invoke the command
                        (*callback)(commandArgs, numArgs, response);
                        return true;
                    }
                
                    // Argument is not optional
                    if(_printErrors)
                        snprintf(response, MAX_RESPONSE_SIZE, "parse error: missing whitespace before arg %d", i + 1);
                    return false;
                }
                do { command ++; } while (*command == ' ');

                switch (argTypes[i]) {
                    case 'd': { // double argument
                        char *after;
                        commandArgs[i].asDouble = strtod(command, &after);
                        if (after == command || (*after != ' ' && *after != '\0')) {
                            if(_printErrors)
                                snprintf(response, MAX_RESPONSE_SIZE, "parse error: invalid double for arg %d", i + 1);
                            return false;
                        }
                        command = after;
                        break;
                    }
                    case 'u': { // uint64_t argument
                        size_t bytesRead = strToInt<uint64_t>(command, &commandArgs[i].asUInt64, 0, ULONG_LONG_MAX);
                        if (bytesRead == 0 || ( (command[bytesRead] != ' ') && (command[bytesRead] != '\0') )) {
                            if(_printErrors)
                                snprintf(response, MAX_RESPONSE_SIZE, "parse error: invalid uint64_t for arg %d", i + 1);
                            return false;
                        }
                        command += bytesRead;
                        break;
                    }
                    case 'i': { // int64_t argument
                    
                        // This doesn't work, so replaced it with below (but keeping the call so bytesRead is returned).
                         size_t bytesRead = strToInt<int64_t>(command, &commandArgs[i].asInt64, LONG_LONG_MIN, LONG_LONG_MAX);                        
                         
                        // replacement
                            // Get base radix
                            int base = 10;
                            if (*command == '0' && *command + 1 == 'b') {
                                base = 2;
                            } else if (*command == '0' && *command + 1 == 'o') {
                                base = 8;
                            } else if (*command == '0' && *command + 1 == 'x') {
                                base = 16;
                            }

                            commandArgs[i].asInt64 = strtoll(command, nullptr, base);    


                        if (bytesRead == 0 || ((command[bytesRead] != ' ') && (command[bytesRead] != '\0'))) {
                            if(_printErrors)
                                snprintf(response, MAX_RESPONSE_SIZE, "parse error: invalid int64_t for arg %d", i + 1);
                            return false;
                        }
                        command += bytesRead;
                        break;
                    }
                    case 's': {
                        size_t readCount = parseString(command, commandArgs[i].asString);
                        if (readCount == 0) {
                            if(_printErrors)
                                snprintf(response, MAX_RESPONSE_SIZE, "parse error: invalid string for arg %d", i + 1);
                            return false;
                        }
                        command += readCount;
                        break;
                    }
                    default:
                        if(_printErrors)
                            snprintf(response, MAX_RESPONSE_SIZE, "parse error: invalid argtype %c for arg %d", argTypes[i], i + 1);
                        return false;
                }
                numArgs++;
            }

            // skip whitespace
            while (*command == ' ') { command ++; }

            // ensure that we're at the end of the command
            if (*command != '\0') {
                if(_printErrors)
                    snprintf(response, MAX_RESPONSE_SIZE, "parse error: too many args (expected max %d)", strlen(argTypes));
                return false;
            }

            // set response to empty string
            response[0] = '\0';

            // invoke the command
            (*callback)(commandArgs, numArgs, response);
            return true;
        }
};

#endif
