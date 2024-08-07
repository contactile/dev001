#ifndef _COMMANDS_H
#define _COMMANDS_H

#include <CommandParser.h>

typedef CommandParser<20, 10> SerialParser;

namespace Commands
{
    extern SerialParser parser;
    void handler(char c);
}

// Command declarations
void bias(SerialParser::Argument *args, char numArgs, char *response);
void bias_and_start(SerialParser::Argument *args, char numArgs, char *response);
void help(SerialParser::Argument *args, char numArgs, char *response);
void info(SerialParser::Argument *args, char numArgs, char *response);
void address(SerialParser::Argument *args, char numArgs, char *response);
void reset(SerialParser::Argument *args, char numArgs, char *response);
void mode(SerialParser::Argument *args, char numArgs, char *response);
void hub_comms(SerialParser::Argument *args, char numArgs, char *response);
void scan(SerialParser::Argument *args, char numArgs, char *response);
void stream(SerialParser::Argument *args, char numArgs, char *response);

#endif // _COMMANDS_H