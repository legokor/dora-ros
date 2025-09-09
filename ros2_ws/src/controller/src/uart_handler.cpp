#include "controller/uart_handler.hpp"
#include <cstring>

#define UART_SOF 42
#define UART_ESC 123
#define UART_EOF 69

void configure_serial_port(int fd, int speed) {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        exit(1);
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;                                          // 8-bit chars
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | INLCR | IGNCR | ICRNL); // disable break processing
    tty.c_lflag = 0;                                                                     // no signaling chars, no echo
    tty.c_oflag = 0;                                                                     // no remapping, no delays
    tty.c_cc[VMIN] = 1;  // read doesn't return until at least 1 byte is received
    tty.c_cc[VTIME] = 1; // 0.1 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);        // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // no parity
    tty.c_cflag &= ~CSTOPB;                 // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                // no hardware flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        exit(1);
    }
}

UARTHandler::UARTHandler(const std::string& port, int baud_rate) {
    serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port == -1) {
        std::cerr << "Error opening UART port!" << std::endl;
        exit(1);
    }

    configure_serial_port(serial_port, baud_rate);
}

UARTHandler::~UARTHandler() {
    close(serial_port);
}

void UARTHandler::sendSpeedCommand(float x, float y, float w) {
    std::vector<uint8_t> payload(sizeof(float) * 3);
    float speeds[3] = { x, y, w };
    std::memcpy(payload.data(), speeds, sizeof(speeds));

    sendFrame(0b00000001, 0x01, payload); // Frame type: speed command
}

bool UARTHandler::receiveSpeedData(float& x, float& y, float& w) {
    uint8_t frameType;
    std::vector<uint8_t> payload;

    if (!receiveFrame(frameType, payload) || frameType != 0b10000010) {
        return false;
    }

    if (payload.size() != sizeof(float) * 3) {
        return false; // Error: Incorrect payload size
    }

    float speeds[3];
    std::memcpy(speeds, payload.data(), sizeof(speeds));

    x = speeds[0];
    y = speeds[1];
    w = speeds[2];

    return true;
}

void UARTHandler::sendFrame(uint8_t frameType, uint8_t seq, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> frame;
    frame.push_back(UART_SOF); // SOF
    frame.push_back(frameType);
    frame.push_back(seq);

    for (uint8_t byte : payload) {
        if (byte == UART_SOF || byte == UART_EOF || byte == UART_ESC) {
            frame.push_back(UART_ESC);
            frame.push_back(byte == UART_SOF ? 1 : (byte == UART_EOF ? 3 : 2));
        } else {
            frame.push_back(byte);
        }
    }

    uint8_t checksum = 0;
    for (size_t i = 1; i < frame.size(); i++) {
        checksum += frame[i];
    }

    frame.push_back(checksum);
    frame.push_back(UART_EOF); // EOF

    write(serial_port, frame.data(), frame.size());
}

bool UARTHandler::receiveFrame(uint8_t& frameType, std::vector<uint8_t>& payload) {
    uint8_t buffer[256];
    int len;
    bool readingFrame = false;
    bool escapeNext = false;
    uint8_t checksum = 0;
    payload.clear();

    while ((len = read(serial_port, buffer, sizeof(buffer))) > 0) {
        for (int i = 0; i < len; i++) {
            uint8_t byte = buffer[i];

            if (byte == UART_ESC) {
                escapeNext = true;
                continue;
            }

            if (escapeNext) {
                byte = (byte == 1) ? UART_SOF : (byte == 2) ? UART_ESC : (byte == 3) ? UART_EOF : byte;
                escapeNext = false;
            }

            if (byte == UART_SOF) {
                payload.clear();
                readingFrame = true;
                checksum = 0;
                continue;
            }

            if (readingFrame) {
                payload.push_back(byte);
                if (payload.size() > 1) {
                    checksum += byte;
                }
            }

            if (byte == UART_EOF && readingFrame) {
                if (payload.size() < 2) {
                    readingFrame = false;
                    continue;
                }

                uint8_t receivedChecksum = payload.back();
                payload.pop_back();

                if (checksum == receivedChecksum) {
                    frameType = payload[0];
                    payload.erase(payload.begin());
                    return true;
                } else {
                    readingFrame = false;
                }
            }
        }
    }

    return false;
}

