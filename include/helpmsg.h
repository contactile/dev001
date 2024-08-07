#ifndef HELP_MSG_H
#define HELP_MSG_H

const char *help_msg =
    R"(
-- HELP --

Available commands:
  help, h                             print this help message
  address, a {[port] [address] -d}    query or modify the I2C address of a sensor on the given port. Type 'help address' for more usage info. 
  bias, b [port (default: all)]       remove the bias for the sensor on the given port
  hub, mux [i2c/spi]                  set the development board analog mux to this communication mode
  info, i                             print information about connected sensors and the development board
  mode, m [i2c/spi]                   set all connected sensors to this communication mode
  reset, r [port (default: all)]      reset the sensor on the given port
  scan                                scan the i2c bus for connected devices
  stream, s [start/stop]              start/stop stream. Type 'help stream' for more usage info.    
)";

const char *stream_help =
    R"(
-- STREAMING --
  
  Control data streaming from connected sensors.

  Usage: stream, s [port (default: all)] { optional: [action] -d [data flags] -f [stream freq] -p [print format]}

  Valid [action] args:
    start    - start streaming
    stop     - stop streaming  

  Valid [data flags] args:
    force   - force data       
    all     - all data (force and temperature)        
    fx      - force x
    fy      - force y
    fz      - force z
    temp    - temperature  

  Data flags should be provided as a comma-separated list, without spaces.

  Valid [print format] args:
    binary  - binary data (default)
    ascii   - human-readable characters (will limit streaming speed)
    none    - stream without printing

  Example - stream forces from all ports at 1khZ:
    stream start -d force -f 1000 -p binary

  Example - print human-readable force data from port 0, at 100Hz:
    stream 0 start -d force -f 100 -p ascii

  Example - stop streaming all ports
    stream stop

)";

const char *address_help =
    R"(
-- ADDRESS --
  
  Query or modify the I2C address of a sensor on the given port

  Usage: address, a { optional: [port] [address] -d }

  Valid [port] args:
    0 - 4         - port number

  Valid [address] args:
    57 - 72   - the new i2c address as a HEX (base 16) string

  Passing the optional flag '-d' will write the default I2C addresses to each port:
    Port 0: 0x57
    Port 1: 0x58
    Port 2: 0x59
    Port 3: 0x5A
    Port 4: 0x5B

  If the '-d' flag is provided, the other arguments will be ignored.

)";

#endif // HELP_MSG_H