#include <SensorHub.h>
#include <Arduino.h>
#include "hub_pins.h"
#include <Preferences.h>
#include <set>
#include <vector>
#include <memory>

namespace SensorHub
{
    // Set this to a nullptr to disable serial output
    static Print *_output = &Serial;

    /* Definitions */
    constexpr int bootloader_i2c_address = 0x55;
    constexpr int port_0_addr = C3DFBS::c3dfbs_default_i2c_address;
    constexpr int port_1_addr = port_0_addr + 1;
    constexpr int port_2_addr = port_0_addr + 2;
    constexpr int port_3_addr = port_0_addr + 3;
    constexpr int port_4_addr = port_0_addr + 4;
    constexpr const char *settings_hub_comms_mode_key = "mode";

    typedef enum
    {
        HUB_IDLE,
        HUB_ACTIVE,
        HUB_BUSY,
        HUB_ERROR,
    } HubStatus;

    /* Function Prototypes */
    static void set_comms_mode(C3DFBS::comms_protocol_t mode);
    static void switch_mux(C3DFBS::comms_protocol_t mode);
    static int refresh_port_i2c_addresses(bool verbose = true);
    static int assign_port_i2c_addresses(int port = -1);
    static void hold_all_sensors_in_reset();
    static void hold_idle_sensors_in_reset();
    static void release_all_sensors_from_reset();
    static void setHubStatus(HubStatus status);
    static void ledUpdate();
    static void reset(int port = -1);
    static void onStreamStarted(C3DFBS *sender);
    static void onStreamStopped(C3DFBS *sender);

    static const int cs_pins[num_ports] = {CS_0, CS_1, CS_2, CS_3, CS_4};
    static const int reset_pins[num_ports] = {RESET_0, RESET_1, RESET_2, RESET_3, RESET_4};
    static const int int_pins[num_ports] = {INT_0, INT_1, INT_2, INT_3, INT_4};
    static std::array<int, num_ports> port_i2c_addresses = {invalid_port_address, invalid_port_address, invalid_port_address, invalid_port_address, invalid_port_address};

    // Manager class to handle sensor arrays
    class SensorManager
    {
    public:
        SensorManager()
        {
            // C3DFBS does not have a default constructor, so we need to build the vector and assign default values here
            for (size_t i = 0; i < num_ports; i++)
            {
                sensors.push_back(C3DFBS(C3DFBS::COMMS_I2C, invalid_port_address, int_pins[i], reset_pins[i], &onStreamStarted, &onStreamStopped));
            }
        }

        std::vector<C3DFBS> sensors;

        void select_I2C()
        {
            for (size_t i = 0; i < num_ports; i++)
            {
                sensors[i] = C3DFBS(C3DFBS::COMMS_I2C, port_i2c_addresses[i], int_pins[i], reset_pins[i], &onStreamStarted, &onStreamStopped);
            }
        }

        void select_SPI()
        {
            for (size_t i = 0; i < num_ports; i++)
            {
                sensors[i] = C3DFBS(C3DFBS::COMMS_SPI, cs_pins[i], int_pins[i], reset_pins[i], &onStreamStarted, &onStreamStopped);
            }
        }

    private:
    };

    static bool _control_status_led = true;
    static HubStatus _hub_status = HUB_IDLE;

    static uint32_t _last_update = 0;
    static uint32_t _last_poll = 0;
    static int _streaming_ports = 0;
    static const int _poll_interval_ms = 3000;
    static C3DFBS::comms_protocol_t _comms_mode = C3DFBS::COMMS_UNKNOWN;

    static Preferences settings;
    SensorManager sensor_manager;

    /* Public Functions */

    void begin()
    {
        // Load persistent variables
        settings.begin("hub");
        // Load hub comms mode (default SPI)
        int mode = settings.getInt(settings_hub_comms_mode_key, C3DFBS::COMMS_SPI);

        // Configure hub pins
        pinMode(STATUS_LED, OUTPUT);
        pinMode(MUX_SELECT, OUTPUT);
        pinMode(PORT_RESET, INPUT);

        // After upload, the INT LED on port 4 stays lit. Switch it off.
        pinMode(INT_4, INPUT);

        // Set Hub status
        setHubStatus(HUB_IDLE);

        // Configure all reset ports
        for (int i = 0; i < num_ports; i++)
        {
            pinMode(reset_pins[i], OUTPUT);
        }

        // Configure hub comms mode (loaded from persistent storage)
        set_comms_mode((C3DFBS::comms_protocol_t)mode);
    }

    /**
     * Call periodically to update SensorHub internal state.
     */
    void update()
    {
        // Write the reset switch state to all ports
        uint8_t reset = !digitalRead(PORT_RESET);
        for (int i = 0; i < num_ports; i++)
        {
            if (reset)
                sensor_manager.sensors.at(i).assertReset();
            else
                sensor_manager.sensors.at(i).deassertReset();
        }

        // Update LED.
        ledUpdate();
    }

    void setHubStatus(HubStatus status)
    {
        pinMode(STATUS_LED, OUTPUT);
        _hub_status = status;
    }

    /**
     * Returns a pointer to a C3DFBS based on the specified port number.
     *
     * @param port the port number of the sensor
     *
     * @return a pointer to the C3DFBS at the specified port
     */
    C3DFBS *sensor(int port)
    {
        if (port < 0 || port > num_ports - 1)
        {
            // Attempted to access invalid address
            setHubStatus(HUB_ERROR);
            return nullptr;
        }

        return &sensor_manager.sensors[port];
    }

    /**
     * Prints port information for each sensor in the SensorHub.
     *
     * @param dest The destination to which to print port information.
     *
     */
    void printPortInfo(Print *dest)
    {
        String info;
        C3DFBS *sensor;
        C3DFBS::status_t status;

        if (_comms_mode == C3DFBS::COMMS_I2C)
        {
            // Run bus scan to detect addresses and bus collisions
            if (refresh_port_i2c_addresses(false) != 0)
            {
                dest->printf("WARNING: I2C bus collision detected!\n");
            }

            // Recreate C3DFBS with detected I2C addresses
            sensor_manager.select_I2C();

            // Call begin on all sensors
            for (int i = 0; i < num_ports; i++)
                sensor_manager.sensors.at(i).begin();
        }

        for (int i = 0; i < num_ports; i++)
        {
            // Query port info
            sensor = &sensor_manager.sensors.at(i);

            if (sensor->isStreaming())
            {
                // Do not query the sensor or the stream will be stopped
                info = "Port " + String(i) + String(": [Streaming; unable to query]\n");
                dest->write(info.c_str());
                info = "";
                continue;
            }

            if (!sensor->isAlive())
            {
                info = "Port " + String(i) + String(": [Not connected]\n");
                dest->write(info.c_str());
                info = "";
                continue;
            }

            // Print a header for each port
            info += "Port " + String(i) + String(":");
            if (_comms_mode == C3DFBS::COMMS_I2C)
            {
                info += String(" 0x") + String(port_i2c_addresses[i], HEX);
            }

            info += String("\n");
            dest->write(info.c_str());
            info = "";

            String ver, whoami;
            status = sensor->getVersion(&ver);
            if (status != C3DFBS::SUCCESS)
            {
                ver = "--";
            }

            status = sensor->whoAmI(&whoami);
            if (status != C3DFBS::SUCCESS)
            {
                whoami = "Unknown";
            }

            // Print version info
            info += "     " + whoami + ", version " + ver + '\n';
            dest->write(info.c_str());
            info = "";

            // Get stream frequency
            uint32_t value;
            String freq, fields;
            status = sensor->getDataFrequency(&value);
            if (status == C3DFBS::SUCCESS)
            {
                freq = "Stream freq : " + String(value) + " Hz";
            }

            // Get stream stream contents
            status = sensor->getDataFields(&value);
            if (status == C3DFBS::SUCCESS)
            {
                fields = "Stream flags: 0x" + String(value, HEX);
            }
            else
            {
                fields = "Stream flags: --";
            }

            if (freq.length() > 0)
            {
                info += "     " + freq + '\n';
            }

            if (fields.length() > 0)
            {
                info += "     " + fields + '\n';
            }

            dest->write(info.c_str());
            info = "";
        }
    }

    /* Private Functions */

    /**
     * Update the count of streaming ports and change LED color to indicate the active state.
     *
     * @param sender pointer to the C3DFBS object
     *
     * @return void
     *
     * @throws No exceptions
     */
    void onStreamStarted(C3DFBS *sender)
    {
        _streaming_ports++;
        if (_streaming_ports > num_ports)
            _streaming_ports = num_ports;

        setHubStatus(HUB_ACTIVE);
    }

    /**
     * Update the streaming ports count in the SensorHub and change LED color if no longer in an active state.
     *
     * @param sender Pointer to the C3DFBS object triggering the function
     *
     * @return void
     *
     * @throws None
     */
    void onStreamStopped(C3DFBS *sender)
    {
        _streaming_ports--;
        if (_streaming_ports < 0)
            _streaming_ports = 0;

        if (_streaming_ports == 0)
        {
            setHubStatus(HUB_IDLE);
        }
    }

    /**
     * Set the communication mode of the SensorHub.
     *
     * @param mode The communication protocol mode to set
     *
     * @return void
     *
     * @throws None
     */
    void set_comms_mode(C3DFBS::comms_protocol_t mode)
    {
        // Set the analog mux for the new comms mode
        switch_mux(mode);

        // Configure the hub ports for the new comms mode
        switch (mode)
        {
        default: // Default to SPI
        case C3DFBS::COMMS_SPI:
            _output->println("Comms mode set to SPI");

            // Make I2C and UART pins high-Z
            Wire.end();
            pinMode(TX, INPUT);
            pinMode(RX, INPUT);

            sensor_manager.select_SPI();

            for (int i = 0; i < num_ports; i++)
                sensor_manager.sensors.at(i).begin();

            break;

        case C3DFBS::COMMS_I2C:
            _output->println("Comms mode set to I2C");

            // Make SPI and UART pins high-Z
            SPI.end();
            pinMode(TX, INPUT);
            pinMode(RX, INPUT);
            Wire.begin();
            Wire.setClock(400000);

            // Run bus scan to detect addresses and bus collisions
            if (refresh_port_i2c_addresses(false) != 0)
            {
                _output->printf("WARNING: I2C bus collision detected!\n");
            }

            sensor_manager.select_I2C();

            // Call begin on all sensors
            for (int i = 0; i < num_ports; i++)
                sensor_manager.sensors.at(i).begin();

            break;

        case C3DFBS::COMMS_UNKNOWN:
            _output->println("ERROR: UART mode not implemented.");
            break;

            // // Not implemented yet
            // case C3DFBS::COMMS_UART:
            // Make SPI pins high-Z
            // Wire.end();
            // SPI.end();
            //     for(int i=0; i< num_ports; i++)
            //         sensors[i].begin();
            //     break;
        }

        // Assign the new comms mode
        _comms_mode = mode;

        // Save this mode to persistent storage
        settings.putInt(settings_hub_comms_mode_key, _comms_mode);
    }

    /**
     * Switches the analog multiplexer to the specified communication mode.
     *
     * @param mode The communication protocol to switch to
     *
     * @return void
     *
     * @throws None
     */
    void switch_mux(C3DFBS::comms_protocol_t mode)
    {
        // Do nothing if the mode hasn't changed
        if (mode == _comms_mode)
            return;

        switch (mode)
        {
        default: // Default to I2C
        case C3DFBS::COMMS_I2C:
            // Select I2C
            digitalWrite(MUX_SELECT, LOW);
            break;

        // UART and SPI share the same mux pins
        // case C3DFBS::COMMS_UART:
        case C3DFBS::COMMS_SPI:
            // Select SPI/UART
            digitalWrite(MUX_SELECT, HIGH);
            break;
        }

        // Allow ample switching time
        delay(10);
    }

    C3DFBS::comms_protocol_t getMode(void)
    {
        return _comms_mode;
    }

    void setMode(C3DFBS::comms_protocol_t mode)
    {
        set_comms_mode(mode);
    }

    /// @brief Take control of the status LED
    /// @param control if true, the user application may control the STATUS LED, and the SensorHub will not modify it.
    void ledControl(bool control)
    {
        _control_status_led = !control;
    }

    /**
     * Call periodically to update the LED 'breathing' animation
     */
    void ledUpdate()
    {
        if (_control_status_led)
        {
            switch (_hub_status)
            {
            default:
            case HUB_IDLE:
                digitalWrite(STATUS_LED, LOW);
                break;

            case HUB_ACTIVE:
            {
                if (millis() - _last_update > 30)
                {
                    // A pleasing breathing function
                    // https://thingpulse.com/breathing-leds-cracking-the-algorithm-behind-our-breathing-pattern/
                    float b = (exp(sin(millis() / 2000.0 * PI)) - 0.368) * 108.49;
                    analogWrite(STATUS_LED, (uint8_t)b);
                    _last_update = millis();
                }
                break;
            }

            case HUB_BUSY:
                digitalWrite(STATUS_LED, HIGH);
                break;

            case HUB_ERROR:
            {
                if (millis() - _last_update > 30)
                {
                    // Continuous flashing
                    float t = millis() / 1000.0f;
                    float b = 2.0 * abs(fmod(t, 1) - 0.5f);
                    digitalWrite(STATUS_LED, b > 0.5);
                    _last_update = millis();
                }
                break;
            }
            }
        }
    }

    /**
     * Reset one or more ports
     *
     * @param port the port number to reset. (-1 to reset all)
     *
     * @return void
     *
     * @throws None
     */
    void reset(int port)
    {
        int i, max;

        // If port out of range, do nothing
        if (port > num_ports - 1)
            return;

        if (port < 0)
        { // Reset all ports
            i = 0;
            max = num_ports;
        }
        else
        { // Reset specified port
            i = port;
            max = port + 1;
        }

        // Reset one or more ports
        for (i; i < max; i++)
        {
            digitalWrite(reset_pins[i], LOW);
            delay(50);
            digitalWrite(reset_pins[i], HIGH);
            delay(50);
        }
    }

    /**
     * Holds all sensors in reset.
     *
     * @param None
     *
     * @return None
     *
     * @throws None
     */
    void hold_all_sensors_in_reset()
    {
        for (uint8_t i = 0; i < num_ports; i++)
            sensor_manager.sensors.at(i).assertReset();

        // TODO: Handle port streaming
        //  streaming = false;

        // Ensure enough time for sensors to go offline
        delay(50);
    }

    void hold_idle_sensors_in_reset()
    {
        for (int i = 0; i < num_ports; i++)
        {
            C3DFBS *sensor = &sensor_manager.sensors.at(i);

            if (!sensor->isStreaming())
                sensor->assertReset();
        }
    }

    /**
     * Release all sensors from reset.
     *
     * @param None
     *
     * @return None
     *
     * @throws None
     */
    void release_all_sensors_from_reset()
    {
        for (uint8_t i = 0; i < num_ports; i++)
            sensor_manager.sensors.at(i).deassertReset();

        // Ensure enough time for sensors to come online
        delay(50);
    }

    /// @brief Query the I2C bus to find devices and detect a possible bus collision due to duplicate addresses
    /// @return
    auto refresh_port_i2c_addresses(bool verbose) -> int
    {
        if (verbose)
            _output->println("Querying port I2C addresses...");

        constexpr int start = 0;
        constexpr int stop = 127;
        uint32_t duplicates = 0;

        std::set<int> ignored_addresses;
        ignored_addresses.insert(bootloader_i2c_address);

        // First remove all sensors from the bus and get a list of any other I2C devices present.
        hold_all_sensors_in_reset();

        // Scan the I2C bus to check what is connected here
        for (int scan_address = start; scan_address <= stop; scan_address++)
        {
            auto it = ignored_addresses.find(scan_address);
            if (it == ignored_addresses.end())
            {
                continue; // This is an ignored address.
            }

            // Ping a device and check for ACK
            Wire.beginTransmission(scan_address);
            auto error = Wire.endTransmission();

            if (error == 0)
            { // A device on the bus has replied with an ACK
                ignored_addresses.insert(scan_address);
            }
        }

        if (verbose)
            _output->printf("Searching...\n");

        // Next release one sensor at a time and try to determine its address
        // Fill port_i2c_addresses array with default values
        for (int i = 0; i < num_ports; i++)
            port_i2c_addresses[i] = invalid_port_address;

        for (int i = 0; i < num_ports; i++)
        {
            hold_all_sensors_in_reset();

            auto sensor = &sensor_manager.sensors.at(i);
            sensor->deassertReset();
            delay(50);

            if (verbose)
                _output->printf(" Port %d: ", i);

            bool found = false;
            for (int ping_addr = start; ping_addr <= stop; ping_addr++)
            {
                auto it = ignored_addresses.find(ping_addr);
                if (it != ignored_addresses.end())
                {
                    continue; // This is an ignored address.
                }

                // Ping a device and check for ACK
                Wire.beginTransmission(ping_addr);
                auto error = Wire.endTransmission();

                if (error == 0)
                {
                    found = true;
                    if (verbose)
                        _output->printf("Found sensor with I2C address 0x%02X", ping_addr);

                    if (std::find(port_i2c_addresses.begin(), port_i2c_addresses.end(), ping_addr) != port_i2c_addresses.end())
                    {
                        // Duplicate address!
                        duplicates++;
                        if (verbose)
                            _output->println(" (duplicate)");
                    }
                    else
                    {
                        if (verbose)
                            _output->println();
                    }

                    port_i2c_addresses[i] = ping_addr;
                    break;
                }
            }

            if (!found)
            {
                if (verbose)
                    _output->println("Not connected");
            }
        }

        release_all_sensors_from_reset();

        if (duplicates)
        {
            if (verbose)
                _output->printf("WARNING: I2C bus collision detected!\n");
        }

        return duplicates;
    }

    int getPortAddress(int port)
    {
        return port_i2c_addresses[port];
    }

    auto refreshPorts() -> void
    {
        refresh_port_i2c_addresses(false);
    }

    void assignDefaultPortAddresses()
    {
        assign_port_i2c_addresses();
    }

    /**
     * Assigns I2C addresses to sensors connected to the SensorHub.
     *
     * @return The number of ports to which I2C addresses were successfully assigned. This may be zero if all ports already have the correct address, and doesn't indicate a failure.
     *
     * @throws None
     */
    int assign_port_i2c_addresses(int port)
    {
        int max_ports;
        int ports_assigned = 0;
        C3DFBS::status_t status;

        if (port < 0)
        { // Assign addresses for all ports
            port = 0;
            max_ports = num_ports;
        }
        else
        { // Assign address for this port only
            max_ports = port + 1;
        }

        for (int i = port; i < max_ports; i++)
        {
            hold_all_sensors_in_reset();

            // Release the i-th sensor so we can assign a new address
            sensor_manager.sensors.at(i).deassertReset();
            delay(50); // Allow time for the sensor to boot

            bool found = false;

            // Scan the possible addresses to check what is connected here
            for (int scan_address = 1; scan_address <= 127; scan_address++)
            {
                if (scan_address == bootloader_i2c_address)
                {
                    // This is a reserved address.
                    continue;
                }

                // Ping a device and check for ACK
                Wire.beginTransmission(scan_address);
                auto error = Wire.endTransmission();

                if (error == 0)
                { // A device on the bus has replied with an ACK
                    auto expected_address = invalid_port_address;

                    // Try to change the address for this device if it does not match the expected_address
                    if (scan_address != expected_address)
                    {
                        status = sensor_manager.sensors.at(i).changeI2CAddress(expected_address, scan_address);
                        if (status != C3DFBS::SUCCESS)
                        {
                            // printdf("Error: Unable to assign new address. Error code %d\n", status);
                        }
                        else
                        {
                            // printdln("OK");
                            ports_assigned++;
                        }
                    }

                    found = true;
                    break;
                }
            }

            if (!found)
            {
                //   printdf("No device connected on port %d\n", i + 1);
            }

            release_all_sensors_from_reset();
        }

        return ports_assigned;
    }

} // namespace SensorHub