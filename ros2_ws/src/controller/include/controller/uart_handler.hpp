#ifndef UART_HANDLER_HPP
#define UART_HANDLER_HPP

#include <vector>
#include <cstdint>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class UARTHandler {
public:
    UARTHandler(const std::string &port, int baud_rate);
    ~UARTHandler();

    void sendSpeedCommand(float x, float y, float w);
    bool receiveSpeedData(float &x, float &y, float &w);

private:
    int serial_port;
    void sendFrame(uint8_t frameType, uint8_t seq, const std::vector<uint8_t> &payload);
    bool receiveFrame(uint8_t &frameType, std::vector<uint8_t> &payload);
};

#endif // UART_HANDLER_HPP
