/*

  Quick start: Open a serial terminal, and type help for a list of commands.

*/

#include "helpmsg.h"
#include "version.h"
#include <Arduino.h>
#include <SensorHub.h>
#include <Commands.h>
#include <algorithm>

// Create namespace aliases for brevity
namespace Hub = SensorHub;
namespace Cmd = Commands;

#pragma region Function Prototypes

void setupParser();
void bias(int port = -1);
void stream(bool action = true, int port = -1, uint32_t fields = C3DFBS::AllForce, int freq = 1000);
void start_stream(int port = -1);
void stop_stream(int port = -1);
void i2c_bug_workaround();
void print_ascii();
void print_helper(uint8_t *buffer, uint32_t &offset, bool condition, bool print_comma = true);

#pragma endregion

#pragma region Variables

typedef enum
{
  PrintFormatNone,
  PrintFormatBinary,
  PrintFormatAscii,
} print_format_t;

print_format_t _print_format = PrintFormatNone;

uint32_t data_fields[Hub::num_ports];
uint32_t data_sizes[Hub::num_ports];

// Port number, timestamp, data flags [3*uint32], force and temperature data [4*float]
static constexpr uint32_t port_header_size = (3 * sizeof(uint32_t));
static constexpr uint32_t port_data_size = (4 * sizeof(float));
static constexpr uint32_t port_packet_size = (port_header_size + port_data_size);

static uint8_t data_buffer[Hub::num_ports * port_packet_size];
static uint32_t data_length = 0;

#pragma endregion

#pragma region Setup

void setup()
{
  Serial.begin(115200);
  Hub::begin();
  setupParser();
}

#pragma endregion

#pragma region Loop

void loop()
{
  /* Wrapping loop() in a while(1) loop prevents yielding to the ESP32 FreeRTOS scheduler.
   * This is a workaround for a known bug that introduces a 5ms delay every 2 seconds
   * https://www.esp32.com/viewtopic.php?f=19&t=39134&sid=a1e6dce5078c01bfa8a3ae1dc6019bac.
   */
  while (1)
  {
    Hub::update();

    // Handle received serial commands
    while (Serial.available())
    {
      // Read a character
      auto c = Serial.read();

      // Run serial parser
      Cmd::handler(c);
    }

    // Maintain the i2c clock speed setting
    i2c_bug_workaround();

    // Acquire data
    data_length = 0;
    for (int i = 0; i < Hub::num_ports; i++)
    {
      auto sensor = Hub::sensor(i);

      if (sensor->isStreaming() && data_sizes[i] > 0)
      {
        // Add all available data to one large buffer
        auto buffer_offset = data_length;

        auto status = sensor->readDataStream(&data_buffer[buffer_offset + port_header_size]);

        if (status == C3DFBS::SUCCESS)
        {
          auto timestamp = millis();
          uint32_t header_offset = 0;

          // WRITE HEADER: Port #, timestamp, data flags
          *(uint32_t *)&data_buffer[buffer_offset + header_offset] = (uint32_t)i; // Port number
          header_offset += sizeof(uint32_t);

          *(uint32_t *)&data_buffer[buffer_offset + header_offset] = timestamp; // Timestamp
          header_offset += sizeof(uint32_t);

          *(uint32_t *)&data_buffer[buffer_offset + header_offset] = data_fields[i]; // Stream flags
          header_offset += sizeof(uint32_t);

          data_length += data_sizes[i] + port_header_size;
        }
      }
    }

    // Print pending data
    if (data_length > 0)
    {
      switch (_print_format)
      {
      default:
      case PrintFormatBinary:
      {
        // To print all force and temperature data for all ports (140 bytes) takes ~ 500us
        Serial.write((uint8_t *)data_buffer, data_length);
        break;
      }

      case PrintFormatAscii:
      {
        // WARNING: This will be slow (~10ms to print), so the sensor sample rate will be reduced
        print_ascii();
        break;
      }
      }
    }
  }
}

#pragma endregion

#pragma region Serial Command Functions

void setupParser()
{
  // Don't print errors as this can interfere with other comms
  Cmd::parser.printErrors(false);

  // Register serial commands
  Cmd::parser.registerCommand("help", "s", &help, "y");
  Cmd::parser.registerCommand("h", "s", &help, "y");

  Cmd::parser.registerCommand("info", "s", &info, "y");
  Cmd::parser.registerCommand("i", "s", &info, "y");

  Cmd::parser.registerCommand("address", "sss", &address, "yyy");
  Cmd::parser.registerCommand("a", "sss", &address, "yyy");

  Cmd::parser.registerCommand("bias", "u", &bias, "y");
  Cmd::parser.registerCommand("b", "u", &bias, "y");

  Cmd::parser.registerCommand("z", "u", &bias_and_start, "y");

  Cmd::parser.registerCommand("hub", "s", &hub_comms, "n");
  Cmd::parser.registerCommand("mux", "s", &hub_comms, "n");

  Cmd::parser.registerCommand("mode", "s", &mode, "n");
  Cmd::parser.registerCommand("m", "s", &mode, "n");

  Cmd::parser.registerCommand("reset", "u", &reset, "y");
  Cmd::parser.registerCommand("r", "u", &reset, "y");

  Cmd::parser.registerCommand("scan", "u", &scan, "y");

  Cmd::parser.registerCommand("stream", "sssssssss", &stream, "yyyyyyyyy");
  Cmd::parser.registerCommand("s", "sssssssss", &stream, "yyyyyyyyy");
}

void bias(int port)
{
  int max_ports;

  Serial.println("Biasing...");

  if (port < 0)
  { // Apply to all sensors
    port = 0;
    max_ports = Hub::num_ports;
  }
  else
  {
    max_ports = port + 1;
  }

  for (int i = port; i < max_ports; i++)
  {
    auto sensor = Hub::sensor(i);
    bool streaming = sensor->isStreaming();

    // Stop streaming to perform bias
    if (streaming)
      sensor->stopDataStream();

    if (!sensor->isAlive())
    {
      Serial.printf(" Port %d... No response\n", i);
      continue;
    }
    Serial.printf(" Port %d...", i);

    auto status = sensor->removeBias();
    if (status == C3DFBS::SUCCESS)
      Serial.println(" OK!");
    else if (status == C3DFBS::COMMAND_FAILED)
      Serial.println(" Check calibration.");
    else
      Serial.println(" Failed.");

    // Restore previous streaming state
    if (streaming)
      sensor->startDataStream();
  }
}

void bias(SerialParser::Argument *args, char numArgs, char *response)
{
  int port = -1;
  int max_ports;

  if (numArgs > 0)
  {
    port = args[0].asUInt64;
    if (port >= Hub::num_ports)
    {
      strlcpy(response, "Invalid port number", SerialParser::MAX_RESPONSE_SIZE);
      return;
    }
  }

  bias(port);
}

void bias_and_start(SerialParser::Argument *args, char numArgs, char *response)
{
  _print_format = PrintFormatBinary;
  bias();
  stream();
}

void help(SerialParser::Argument *args, char numArgs, char *response)
{
  // Print detailed help message
  if (numArgs > 0)
  {
    String param = args[0].asString;
    param.toLowerCase();

    if (param == "stream")
    {
      Serial.printf(stream_help);
      return;
    }
    else if (param == "address")
    {
      Serial.printf(address_help);
      return;
    }
  }

  // Print default help message
  Serial.printf(help_msg);
}

void address(SerialParser::Argument *args, char numArgs, char *response)
{
  int port = -1;
  int address = -1;

  std::vector<String> arg_s;
  for (int i = 0; i < numArgs; i++)
  {
    arg_s.push_back(args[i].asString);
  }

  auto mode = Hub::getMode();
  if (mode == C3DFBS::COMMS_I2C)
  {
    Serial.print("Querying ports... ");
    Hub::refreshPorts();
    Serial.println("Done.");
  }

  if (arg_s.empty())
  {

    if (mode != C3DFBS::COMMS_I2C)
    {
      Serial.println("To read port I2C addresses, set the hub to I2C mode by typing 'hub i2c'.");
      return;
    }

    // If no args are provided, print all port addresses
    Serial.println("-- Port Addresses --");
    for (int i = 0; i < Hub::num_ports; i++)
    {
      auto addr = Hub::getPortAddress(i);

      if (addr < 0 || addr >= Hub::invalid_port_address)
      {
        Serial.printf("Port %d: [Unknown]\n", i);
      }
      else
      {
        Serial.printf("Port %d: 0x%02X\n", i, addr);
      }
    }
    return;
  }

  if (auto it = std::find(arg_s.begin(), arg_s.end(), "-d"); it != arg_s.end())
  {
    // Restore defaults and ignore other arguments
    Serial.print("Assigning default port addresses... ");
    Hub::assignDefaultPortAddresses();
    Serial.println("Done.");
    return;
  }

  int position = 0;
  for (int i = 0; i < arg_s.size(); i++)
  {
    auto arg = arg_s[i];
    arg.toLowerCase();

    // Arguments are positional
    if (position == 0)
    {
      position++;
      if (arg.length() > 1 || arg[0] < '0' || arg[0] > '4')
      {
        Serial.printf("ERROR: Invalid port '%s'. Valid range is 0 - %d.\n", arg.c_str(), (Hub::num_ports - 1));
        return;
      }

      port = arg.toInt();

      if (port < 0 || port > (Hub::num_ports - 1))
      {
        // Error: Invalid port
        Serial.printf("ERROR: Invalid port '%s'. Valid range is 0 - %d.\n", arg.c_str(), (Hub::num_ports - 1));
        return;
      }
    }
    else if (position == 1)
    {
      position++;
      std::string str = std::string(arg.c_str());

      for (char ch : str)
      {
        if (!isdigit(ch))
        {
          Serial.printf("ERROR: Invalid address '%s'. Address should contain only numeric characters in hex (base 16) format(e.g. 57).\n", arg.c_str());
          return;
        }
      }

      // Convert string to integer
      address = std::stoi(str, 0, 16);

      if (address < C3DFBS::c3dfbs_i2c_address_range_min || address > C3DFBS::c3dfbs_i2c_address_range_max)
      {
        // Error: Invalid address
        Serial.printf("ERROR: Invalid address '%s'. Valid range is 0x%x - 0x%x.\n",
                      C3DFBS::c3dfbs_i2c_address_range_min, C3DFBS::c3dfbs_i2c_address_range_max, arg.c_str());
        return;
      }
    }
  }

  if (address < 0)
  {
    // Read port address
    Serial.println("-- Port Address --");
    auto addr = Hub::getPortAddress(port);

    if (addr < 0 || addr >= Hub::invalid_port_address)
    {
      Serial.printf("Port %d: [Unknown]\n", port);
    }
    else
    {
      Serial.printf("Port %d: 0x%02X\n", port, addr);
    }
    return;
  }

  // Modify port address

  // Remove other devices from the bus to eliminate collisions
  for (int i = 0; i < Hub::num_ports; i++)
  {
    Hub::sensor(i)->assertReset();
  }
  Hub::sensor(port)->deassertReset();
  delay(50);
  // Change the address
  Serial.printf("Setting port %d I2C address to 0x%02X... ", port, address);
  auto stat = Hub::sensor(port)->changeI2CAddress(address, Hub::getPortAddress(port));
  if (stat == C3DFBS::SUCCESS)
  {
    Serial.println("OK.");
    Serial.print("Refreshing ports... ");
    Hub::refreshPorts();
    Serial.println("Done.");
  }
  else
  {
    Serial.printf("ERROR: Unable to change I2C address. Error code %d.\n", stat);
  }
}

void info(SerialParser::Argument *args, char numArgs, char *response)
{
  // FIRMWARE_VERSION
  Serial.println();
  Serial.println("-- Contactile C3DFBS Development Board --");
  Serial.println("SensorHub FW " + String(Hub::firmware_version));
  Serial.print("Communication mode: ");
  if (Hub::getMode() == C3DFBS::COMMS_I2C)
    Serial.println("I2C");
  else
    Serial.println("SPI");

  Serial.println();
  Serial.println("-- Sensors --");

  Serial.println("Querying ports...");
  Hub::printPortInfo();
}

void scan(SerialParser::Argument *args, char numArgs, char *response)
{
  Serial.println("Scanning I2C bus...");
  int devices = 0;

  auto mode = Hub::getMode();

  if (mode != C3DFBS::COMMS_I2C)
  {
    // Setup the I2C bus
    SPI.end();
    Wire.begin();
  }

  // Scan the i2c bus
  for (int scan_address = 1; scan_address <= 127; scan_address++)
  {
    // Ping a device and check for ACK
    Wire.beginTransmission(scan_address);
    auto error = Wire.endTransmission();

    if (error == 0)
    { // A device on the bus has replied with an ACK
      devices++;
      Serial.printf("Found device at address 0x%02X\n", scan_address);
    }
  }

  Serial.printf("Scan complete. Found %d device(s).\n", devices);

  if (mode != C3DFBS::COMMS_I2C)
  {
    // Restore the SPI bus
    Wire.end();
    SPI.begin();
  }
}

void hub_comms(SerialParser::Argument *args, char numArgs, char *response)
{
  String mode = args[0].asString;
  mode.toLowerCase();

  int protocol = -1;

  if (mode == "i2c")
  {
    protocol = C3DFBS::COMMS_I2C;
  }
  else if (mode == "spi" || mode == "uart")
  {
    protocol = C3DFBS::COMMS_SPI;
  }
  else
  {
    strlcpy(response, "Invalid communication mode", SerialParser::MAX_RESPONSE_SIZE);
    return;
  }

  if (protocol >= 0)
  {
    if (Hub::getMode() == protocol)
    {
      Serial.print("Hub configured for ");
      if (protocol == C3DFBS::COMMS_I2C)
        Serial.println("I2C mode.");
      else
        Serial.println("SPI mode.");

      return;
    }

    // Change the hub comms mode
    if (protocol == C3DFBS::COMMS_I2C)
    {
      Serial.print("Configuring Hub for I2C mode -> ");
      Hub::setMode(C3DFBS::COMMS_I2C);
    }
    else
    {
      Serial.print("Configuring Hub for SPI mode -> ");
      Hub::setMode(C3DFBS::COMMS_SPI);
    }
  }
}

void mode(SerialParser::Argument *args, char numArgs, char *response)
{
  String mode = args[0].asString;
  mode.toLowerCase();

  int protocol = -1;

  if (mode == "i2c")
  {
    protocol = C3DFBS::COMMS_I2C;
  }
  else if (mode == "spi")
  {
    protocol = C3DFBS::COMMS_SPI;
  }
  else
  {
    strlcpy(response, "Invalid communication mode", SerialParser::MAX_RESPONSE_SIZE);
    return;
  }

  if (protocol >= 0)
  {
    // Set comms mode for all sensors
    Serial.print("Setting communication mode to ");
    if (protocol == C3DFBS::COMMS_I2C)
      Serial.println("I2C...");
    else
      Serial.println("SPI...");

    for (int i = 0; i < Hub::num_ports; i++)
    {
      Serial.printf(" Port %d...", i);
      auto status = Hub::sensor(i)->setCommsMode(protocol);
      if (status == C3DFBS::SUCCESS)
        Serial.println(" OK!");
      else
        Serial.println(" Failed.");
    }
  }
}

void reset(SerialParser::Argument *args, char numArgs, char *response)
{
  int port = -1;
  int max_ports;

  if (numArgs > 0)
  {
    port = args[0].asUInt64;
    if (port >= Hub::num_ports)
    {
      strlcpy(response, "Invalid port number", SerialParser::MAX_RESPONSE_SIZE);
      return;
    }
  }

  if (port < 0)
  { // Apply to all sensors
    port = 0;
    max_ports = Hub::num_ports;
  }
  else
  {
    max_ports = port + 1;
  }

  for (int i = port; i < max_ports; i++)
  {
    // Reset port(s)
    Serial.printf("Port %d reset\n", i);
    Hub::sensor(i)->reset();
  }
}

void stream(bool action, int port, uint32_t fields, int freq)
{
  C3DFBS::status_t status;
  int max_ports;

  // If port number < 0, apply to all sensors
  if (port < 0)
  {
    port = 0;
    max_ports = Hub::num_ports;
  }
  else
  {
    max_ports = port + 1;
  }

  for (int i = port; i < max_ports; i++)
  {
    Serial.println("Port " + String(i) + ":");
    auto active_sensor = Hub::sensor(i);

    // Skip unresponsive (unconnected) ports
    if (!active_sensor->isStreaming())
    {
      // Not streaming. OK to query.
      if (!active_sensor->isAlive())
      {
        Serial.println(" No response.");
        continue; // No response
      }
    }

    // Set the requested data fields if argument is provided
    if (fields > 0)
    {
      if (active_sensor->isStreaming())
        active_sensor->stopDataStream();

      status = active_sensor->setDataFields(fields);
      if (status != C3DFBS::SUCCESS)
      {
        Serial.printf(" ERROR: Unable to set stream contents. Error %d\n", status);
        continue;
        ;
      }
      Serial.printf(" Set stream contents to 0x%X. Expecting %d bytes in data stream\n", fields, active_sensor->getDataStreamSize());

      // Store data_fields
      data_fields[i] = fields;
      data_sizes[i] = active_sensor->getDataStreamSize();
    }

    // Set the stream frequency if argument is provided
    if (freq > 0)
    {
      if (active_sensor->isStreaming())
        active_sensor->stopDataStream();

      status = active_sensor->setDataFrequency(freq);
      if (status != C3DFBS::SUCCESS)
      {
        Serial.printf(" ERROR: Unable to set stream frequency. Error %d\n", status);
        continue;
        ;
      }
      Serial.printf(" Set streaming frequency to %d Hz\n", freq);
    }

    if (action > 0)
    {
      // START STREAM
      if (active_sensor->isStreaming())
        active_sensor->stopDataStream();

      status = active_sensor->startDataStream();
      if (status != C3DFBS::SUCCESS)
      {
        Serial.printf(" ERROR: Unable to start data stream.");
        if (status == C3DFBS::OUT_OF_BOUNDS)
        {
          Serial.println(" Requested data fields are invalid.");
        }
        else
        {
          Serial.printf(" Error %d\n", status);
        }
        continue;
      }

      Serial.println(" Starting data stream");
    }
    else
    {
      // STOP STREAM
      Serial.println(" Stopping data stream");

      status = active_sensor->stopDataStream();
      if (status != C3DFBS::SUCCESS)
      {
        Serial.printf(" ERROR: Data stream did not stop!\n");
        continue;
      }
      Serial.println(" Data stream stopped");
    }
  }
}

void start_stream(int port)
{
  C3DFBS::status_t status;
  int max_ports;

  // If port number < 0, apply to all sensors
  if (port < 0)
  {
    port = 0;
    max_ports = Hub::num_ports;
  }
  else
  {
    max_ports = port + 1;
  }

  for (int i = port; i < max_ports; i++)
  {
    Serial.println("Port " + String(i) + ":");
    auto active_sensor = Hub::sensor(i);

    // Skip unresponsive (unconnected) ports
    if (!active_sensor->isStreaming())
    {
      // Not streaming. OK to query.
      if (!active_sensor->isAlive())
      {
        Serial.println(" No response.");
        continue; // No response
      }
    }

    // START STREAM
    if (active_sensor->isStreaming())
      active_sensor->stopDataStream();

    status = active_sensor->startDataStream();
    if (status != C3DFBS::SUCCESS)
    {
      Serial.printf(" ERROR: Unable to start data stream.");
      if (status == C3DFBS::OUT_OF_BOUNDS)
      {
        Serial.println(" Requested data fields are invalid.");
      }
      else
      {
        Serial.printf(" Error %d\n", status);
      }
      continue;
    }

    Serial.println(" Starting data stream");
  }
}

void stop_stream(int port)
{
  C3DFBS::status_t status;
  int max_ports;

  // If port number < 0, apply to all sensors
  if (port < 0)
  {
    port = 0;
    max_ports = Hub::num_ports;
  }
  else
  {
    max_ports = port + 1;
  }

  for (int i = port; i < max_ports; i++)
  {
    Serial.println("Port " + String(i) + ":");
    auto active_sensor = Hub::sensor(i);

    // Skip unresponsive (unconnected) ports
    if (!active_sensor->isStreaming())
    {
      // Not streaming. OK to query.
      if (!active_sensor->isAlive())
      {
        Serial.println(" No response.");
        continue; // No response
      }
    }

    // STOP STREAM
    Serial.println(" Stopping data stream");

    status = active_sensor->stopDataStream();
    if (status != C3DFBS::SUCCESS)
    {
      Serial.printf(" ERROR: Data stream did not stop!\n");
      continue;
    }
    Serial.println(" Data stream stopped");
  }
}

void stream(SerialParser::Argument *args, char numArgs, char *response)
{
  int freq = -1;
  uint32_t data_flags = 0;
  bool setFlags = false;
  String action;
  int port = -1;

  // Fetch all arguments
  String args_str[9];
  for (int i = 0; i < numArgs; i++)
  {
    args_str[i] = args[i].asString;
    args_str[i].toLowerCase();
    // Serial.println("Arg [" + String(i) + "] " + args_str[i]);
  }

  // Search for argument flags and extract values
  for (int arg = 0; arg < numArgs; arg++)
  {
    if (args_str[arg] == "-d")
    {
      // The next argument must be the desired stream data fields
      if ((arg + 1) < numArgs)
      {
        // Parse stream contents and convert to flags
        String contents = args_str[++arg];

        /* Flags should be provided as a comma separated list (without spaces)

        Valid flags:
        [Groups]
          force   - force data
          all     - all data (force and temperature)

        [Individual values]
          fx      - force x
          fy      - force y
          fz      - force z
          temp    - temperature

          Example: Request only z force and temperature
            stream -f fz,temp
        */

        // R[0-7] are reserved bytes
        String flag_ind[12] =
            {
                "r0",
                "r1",
                "r2",
                "r3",
                "r4",
                "temp",
                "r5",
                "r6",
                "r7",
                "fz",
                "fy",
                "fx",
            };

        String flag_groups[2] =
            {
                "force", "all"};

        // Search for individual flags
        for (int i = 0; i < 12; i++)
        {
          if (contents.indexOf(flag_ind[i]) >= 0)
          {
            // Add this flag
            data_flags |= (1 << i);
          }
        }

        // Search for group flags
        for (int i = 0; i < 2; i++)
        {
          if (contents.indexOf(flag_groups[i]) >= 0)
          {
            switch (i)
            {
            case 0:
              // All force data
              data_flags |= C3DFBS::AllForce;
              break;

            case 1:
              // All data
              data_flags = C3DFBS::AllForce | C3DFBS::Temperature;
              break;

            default:
              break;
            }
          }
        }

        setFlags = true;
      }
    }
    else if (args_str[arg] == "-f")
    {
      // The next argument must be the frequency.
      if (arg + 1 < numArgs)
      {
        // Convert string to number
        String f = args_str[++arg];
        freq = f.toInt();
        Serial.printf("Frequency: %d\n", freq);

        if (freq == 0)
        {
          // args_str[i] is either zero, or invalid
          Serial.printf("ERROR: Invalid frequency %s\n", f);
          return;
        }
      }
    }
    else if (args_str[arg] == "-p")
    {
      // The next argument must be the print format.
      if (arg + 1 < numArgs)
      {
        // Get the print format
        String format = args_str[++arg];

        if (format == "ascii")
          _print_format = PrintFormatAscii;
        else if (format == "binary")
          _print_format = PrintFormatBinary;
        else
          _print_format = PrintFormatNone;
      }
    }
    else if (args_str[arg] == "start" || args_str[arg] == "stop")
    {
      // This is an action command
      action = args_str[arg];
    }
    else
    {
      // Any other value will be interpreted as the port number
      port = args_str[arg].toInt();
      // Serial.printf("Port number: %d\n", port);
    }
  }
  // Parsing done.
  stream(action == "start", port, data_flags, freq);
}

#pragma endregion

#pragma region Helper Functions

void i2c_bug_workaround()
{
  // https://esp32.com/viewtopic.php?t=33934
  // https://github.com/espressif/arduino-esp32/issues/8480
  // There is a bug in the ESP32-S2 ESP-IDF core that causes the I2C clock to be set to 5kHz after 10 failed attempts to read from a device.
  // The workaround is to set the clock to 'something', then back to the desired freq.
  if (Hub::getMode() == C3DFBS::COMMS_I2C)
  {
    // If Wire.begin() has not been called, these function calls will take a long time to execute
    Wire.setClock(100000);
    Wire.setClock(400000);
  }
}

void print_ascii()
{
  uint32_t offset = 0;
  while (offset < data_length)
  {
    // Retrieve header
    uint32_t port = *(uint32_t *)&data_buffer[offset];
    offset += sizeof(uint32_t);

    uint32_t timestamp = *(uint32_t *)&data_buffer[offset];
    offset += sizeof(uint32_t);

    uint32_t flags = *(uint32_t *)&data_buffer[offset];
    offset += sizeof(uint32_t);

    // Print header
    Serial.printf("Port %d: Timestamp: %d, ", port, timestamp);

    // Print data
    Serial.print("Temp: ");
    print_helper(data_buffer, offset, flags & C3DFBS::Temperature);

    Serial.print("Fz: ");
    print_helper(data_buffer, offset, flags & C3DFBS::ForceZ);

    Serial.print("Fy: ");
    print_helper(data_buffer, offset, flags & C3DFBS::ForceY);

    Serial.print("Fx: ");
    print_helper(data_buffer, offset, flags & C3DFBS::ForceX, false);

    Serial.println();
  }
}

void print_helper(uint8_t *buffer, uint32_t &offset, bool condition, bool print_comma)
{
  // Format a float and print
  if (condition)
  {
    Serial.printf("%+3.2f", *(float *)(data_buffer + offset));
    offset += sizeof(float);
  }
  else
  {
    Serial.printf("--");
  }

  if (print_comma)
    Serial.printf(", ");
}

#pragma endregion
