#include <C3DFBS.h>

namespace SensorHub
{
    /* Definitions */
    static constexpr uint32_t num_ports = 5;
    static constexpr int invalid_port_address = 255;

    void begin();
    void update();
    C3DFBS *sensor(int port);
    void printPortInfo(Print *dest = &Serial);
    C3DFBS::comms_protocol_t getMode(void);
    void setMode(C3DFBS::comms_protocol_t mode);
    void ledControl(bool);
    void assignDefaultPortAddresses();
    int getPortAddress(int port);
    void refreshPorts();

}