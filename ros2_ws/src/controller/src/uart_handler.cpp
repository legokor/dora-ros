#include "controller/uart_handler.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <span>
#include <variant>

namespace dora {

constexpr static std::byte UART_SOF = std::byte(42);
constexpr static std::byte UART_ESC = std::byte(123);
constexpr static std::byte UART_EOF = std::byte(69);

static void configure_serial_port(int fd, int speed) {
    struct termios t;

    if (tcgetattr(fd, &t) != 0) {
        perror("tcgetattr");
        exit(1);
    }

    cfsetospeed(&t, speed);
    cfsetispeed(&t, speed);

    t.c_cflag = (t.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    t.c_cflag |= (CLOCAL | CREAD);          // ignore modem controls, enable reading
    t.c_cflag &= ~(PARENB | PARODD);        // no parity
    t.c_cflag &= ~CSTOPB;                   // 1 stop bit
    t.c_cflag &= ~CRTSCTS;                  // no hardware flow control
    t.c_lflag = 0;                          // no signaling chars, no echo
    t.c_cc[VMIN] = 8;                       // read doesn't return until at least 8 byte is received
    t.c_cc[VTIME] = 1;                      // 0.1 seconds read timeout between bytes

    // disable break processing and shut off xon / xoff ctrl
    t.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);

    // no remapping, no delays
    t.c_oflag = 0;

    if (tcsetattr(fd, TCSANOW, &t) != 0) {
        perror("tcsetattr");
        exit(1);
    }
}

UARTHandler::UARTHandler(const std::string& port, uint32_t baud_rate) {
    serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port == -1) {
        std::cerr << "Error opening UART port!" << std::endl;
        exit(1);
    }

    configure_serial_port(serial_port, baud_rate);

    read_buffer.resize(1024);
    write_buffer.resize(4096);
}

std::expected<ReceivedMessage, std::string> UARTHandler::receiveMessage() {
    if (!parsedMessages.empty()) {
        auto m = parsedMessages.front();
        parsedMessages.pop();
        return m;
    }

    // receive any amount of data
    auto e = receiveData(std::nullopt);
    if (e)
        return std::unexpected(*e);

    while (true) {
        // try parsing frames
        std::expected<size_t, std::string> p = parseFrames();

        // success, return one
        if (!parsedMessages.empty()) {
            auto m = parsedMessages.front();
            parsedMessages.pop();
            return m;
        }

        if (p) {
            // incomplete message, we need more data to parse it
            auto e = receiveData(*p);

            // error while reading from serial
            if (e)
                return std::unexpected(e.value());
        } else // invalid data, return error
            // TODO: drop data until next SOF
            return std::unexpected(p.error());
    }
}

void UARTHandler::sendSpeedCommand(float x, float y, float w) {
    const float speeds[3] = { x, y, w };
    auto float_span = std::span(speeds);

    sendFrame(std::byte((uint8_t) FrameCategory::Request | (uint8_t) RequestFrameTypeID::SetSpeed),
              std::as_bytes(float_span));
}

static std::byte escape(std::byte b) {
    switch (b) {
        case UART_SOF: return std::byte(1);
        case UART_ESC: return std::byte(2);
        case UART_EOF: return std::byte(3);

        default: return b;
    }
}

static std::byte unescape(std::byte b) {
    switch (b) {
        case std::byte(1): return UART_SOF;
        case std::byte(2): return UART_ESC;
        case std::byte(3): return UART_EOF;

        default: return b;
    }
}

void UARTHandler::sendFrame(std::byte frameType, std::span<const std::byte> payload) {
    write_buffer.push_back(UART_SOF);

    uint8_t checksum = 0;
    write_buffer.push_back(std::byte(frameType));
    checksum += (uint8_t) write_buffer.back();

    // TODO: does seq really work like this?
    write_buffer.push_back(std::byte(seq++));
    checksum += (uint8_t) write_buffer.back();

    for (std::byte byte : payload) {
        if (byte == UART_SOF || byte == UART_EOF || byte == UART_ESC) {
            write_buffer.push_back(UART_ESC);
            write_buffer.push_back(escape(byte));
        } else {
            write_buffer.push_back(byte);
        }

        checksum += (uint8_t) write_buffer.back();
    }

    write_buffer.push_back(std::byte(checksum));
    write_buffer.push_back(UART_EOF);

    write(serial_port, write_buffer.data(), write_buffer.size());

    write_buffer.clear();
}

size_t unescapeBuffer(std::span<std::byte> buf) {
    size_t new_size = 0;

    for (size_t i = 0; i < buf.size(); i++) {
        if (buf[i] == UART_ESC)
            buf[new_size] = unescape(buf[++i]);
        else
            buf[new_size] = buf[i];

        new_size++;
    }

    return new_size;
}

std::optional<std::string> UARTHandler::receiveData(std::optional<size_t> min_count) {
    size_t offset = read_buffer.size();
    size_t remaining = read_buffer.capacity() - offset;

    // have at least min_count remaining
    if (min_count && remaining < *min_count) {
        remaining = *min_count;
    }

    // set size to fit new data
    read_buffer.resize(offset + remaining);

    // read data
    int len = read(serial_port,                           //
                   (void*) (read_buffer.data() + offset), //
                   (int) remaining);

    // read error
    if (len < 0)
        return std::string(std::strerror(errno));

    // unescape to make parsing easier
    size_t new_size = unescapeBuffer(std::span(read_buffer).subspan(offset));
    read_buffer.resize(new_size);

    return {};
}

std::expected<size_t, std::string> UARTHandler::parseFrames() {
    auto it = read_buffer.begin();

    auto require_bytes = [&](size_t n) { return read_buffer.end() - it <= n; };

    while (require_bytes(sizeof(UART_SOF) + sizeof(std::byte) + sizeof(UART_EOF))) {
        if (*it++ != UART_SOF)
            return std::unexpected("Frame doesn't start with SOF");

        std::byte frame_type = *it++;

        FrameCategory frame_category = (FrameCategory) (frame_type & FRAME_CATEGORY_MASK);
        uint8_t frame_type_id = (uint8_t) (frame_type & FRAME_TYPE_ID_MASK);

        switch (frame_category) {
            case FrameCategory::Reply: {
                std::byte mseq;

                if (!require_bytes(sizeof(mseq)))
                    return sizeof(mseq);

                mseq = *it++;

                // TODO: handle replies
                switch ((RequestFrameTypeID) frame_type_id) {
                    case RequestFrameTypeID::SetSpeed: break;
                    case RequestFrameTypeID::SetSettings: break;
                }

                break;
            }

            case FrameCategory::Stream: {
                switch ((StreamFrameTypeID) frame_type_id) {
                    case StreamFrameTypeID::Speed:
                        SpeedData spd;

                        if (!require_bytes(sizeof(spd)))
                            return sizeof(spd);

                        std::memcpy(&spd, &*it, sizeof(spd));
                        it += sizeof(spd);
                        break;

                    case StreamFrameTypeID::Status:
                        RobotStatus stat;

                        if (!require_bytes(sizeof(stat)))
                            return sizeof(stat);

                        // TODO: more type safe solution...
                        std::memcpy(&stat, &*it, sizeof(stat));
                        it += sizeof(stat);
                        break;
                }

                break;
            }

            case FrameCategory::Request: return std::unexpected("Nucleo sent request???");
            default: return std::unexpected("Invalid frame category");
        }

        // TODO: checksum
        uint8_t checksum;

        if (!require_bytes(sizeof(UART_EOF) + si))
            return sizeof(UART_EOF);

        if (*it++ != UART_EOF)
            return std::unexpected("Frame doesn't end with EOF");
    }

    return sizeof(UART_SOF) + sizeof(std::byte) + sizeof(UART_EOF);
}

UARTHandler::~UARTHandler() {
    close(serial_port);
}

} // namespace dora
