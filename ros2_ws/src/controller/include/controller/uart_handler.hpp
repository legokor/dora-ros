#ifndef UART_HANDLER_HPP
#define UART_HANDLER_HPP

#include <cstdint>
#include <expected>
#include <optional>
#include <queue>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace dora {

class SpeedData {
public:
    float x, y, w;
};

class RobotStatus {
public:
    float voltage;
};

using ReceivedMessage = std::variant<SpeedData, RobotStatus>;

constexpr std::byte FRAME_CATEGORY_MASK = std::byte(0b11000000);
constexpr std::byte FRAME_TYPE_ID_MASK = ~FRAME_CATEGORY_MASK;

enum class FrameCategory : uint8_t {
    Request = 0b00000000,
    Reply = 0b01000000,
    Stream = 0b10000000,
};

enum class RequestFrameTypeID : uint8_t {
    SetSpeed = 0b00000001,
    SetSettings = 0b00000010,
};

enum class StreamFrameTypeID : uint8_t {
    Speed = 0b0000000,
    Status = 0b00000010,
};

class UARTHandler {
public:
    UARTHandler(const std::string& port, uint32_t baud_rate);
    ~UARTHandler();

    void sendSpeedCommand(float x, float y, float w);

    std::expected<dora::ReceivedMessage, std::string> receiveMessage();

private:
    int serial_port;
    uint8_t seq;

    std::vector<std::byte> read_buffer, write_buffer;

    std::queue<dora::ReceivedMessage> parsedMessages;

    std::optional<std::string> receiveData(std::optional<size_t> min_count);

    // parses from read_buffer to parsedMessages and returns the estimated amount of bytes to receive for the next
    // message
    std::expected<size_t, std::string> parseFrames();

    void sendFrame(std::byte type, std::span<const std::byte> payload);
};

} // namespace dora

#endif // UART_HANDLER_HPP
