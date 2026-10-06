#include "controller/uart_handler.hpp"

#include <asm-generic/errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>
#include <expected>
#include <format>
#include <optional>
#include <ostream>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <span>
#include <variant>

// CHANGE miért nem volt itt eddig a vektor?
#include <vector>

// #define DEBUG_WITHOUT_UART
#define DEBUG_CERR

// CHANGE ezek a függvények
// wrapper around std::cerr which can be turned on with defining DEBUG_CERR
void cerrdebug(std::string s) {
#ifdef DEBUG_CERR
    std::cerr << s << std::endl;
#endif
}

void debugbyte(std::byte b) {
    static int hanyadik = 0;
    std::cerr << std::format("byte{}: {:02x}", hanyadik++, std::to_integer<unsigned char>(b)) << std::endl;
}

void printreadbuffer(std::vector<std::byte> read_buffer) {
    std::cerr << "--- Start of read buffer ---" << std::endl;
    for (std::byte b : read_buffer) {
        // thx gemini
        // átcastolja a byteot char-á, majd formázza szigorúan kétjegyű hexa számként
        std::cerr << std::format("{:02x}", std::to_integer<unsigned char>(b));
    }
    std::cerr << std::endl;
}

namespace dora {
constexpr static std::byte UART_SOF = std::byte(42);
constexpr static std::byte UART_ESC = std::byte(123);
constexpr static std::byte UART_EOF = std::byte(69);

// maximum length of read buffer
// 1024 should be enough for message lengths of ~30 bytes
const int READ_BUFFER_MAX_LEN = 1024;
const int TRY_AMOUNT = 100;

// R: saját comment, Remove
// C: Comment, ami jó ha bennemarad

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

#ifdef DEBUG_WITHOUT_UART
    std::cerr << "Dont forget to open port after debugging!" << std::endl;
#endif

#ifndef DEBUG_WITHOUT_UART
    serial_port = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    // R: Itt hiba lesz, mert az én gépemen nincs UART port, de megoldom
    if (serial_port == -1) {
        std::cerr << "Error opening UART port!" << std::endl;
        exit(1);
    }

    configure_serial_port(serial_port, baud_rate);
#endif

    // reserve 1024 and have size 512
    // reserving twice the length because unescaping can double the length of the vector
    // WRONG: Unescaping in this context only makes the message shorter
    read_buffer.reserve(READ_BUFFER_MAX_LEN);
    // resizing is no longer needed
    // read_buffer.resize(READ_BUFFER_MAX_LEN / 2);
    write_buffer.resize(4096);
}

// This function returns one of the already parsed messages, or reads from UART until a message is parsed, if the parsed
// messages queue is empty.
std::expected<ReceivedMessage, std::string> UARTHandler::receiveMessage() {
    cerrdebug("Started receiveMessage function");

    int tries = 0;
    // try to read and parse messages until we get something
    while (parsedMessages.empty()) {
        cerrdebug("Parsed messages are empty, reading and parsing...");

        // try to fill read_buffer from UART
        auto e = receiveData(std::nullopt);

        cerrdebug("Received data, starting parsing...");

        // try parsing frames from read_buffer to parsedMessages
        std::expected<size_t, std::string> p = parseFrames();

        // return error from parsing
        if (!p.has_value()) {
            cerrdebug("Error parsing message, returning error message.");

            std::string s = p.error();
            return std::unexpected(p.error());
        }

        if (*p == -1) {
            return std::unexpected("What? (Code escaped while(true) in parseFrames())");
        }

        // TRY_AMOUNT olvasás-parsolás után sincs új üzenet, hibát jelzünk
        if (tries++ > TRY_AMOUNT) {
            return std::unexpected("No new messages on UART");
        }
    }
    cerrdebug("Returning message");
    auto m = parsedMessages.front();
    parsedMessages.pop();
    return m;
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

// elméletben ez működik, nem kell vele foglalkozni
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

    ssize_t w = write(serial_port, write_buffer.data(), write_buffer.size());
    if (w == -1) {
        // TODO: error
    }

    write_buffer.clear();
}

size_t unescapeBuffer(std::span<std::byte> buf) {
    // R: new_size a ciklusváltozója az új buffernek, amiben már nincsenek escape / egyéb kontroll karakterek
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
    cerrdebug("\tReceiving data from UART");
    // TODO ha az előző beolvasásnak a vége egy UART_ESC volt, akkor nem fog tudni unescapelni rendesen
    // valszeg megoldható azzal, ha az unescapinget a messageParsing-ba rakjuk bele

    size_t offset = read_buffer.size();

    size_t remaining = READ_BUFFER_MAX_LEN - offset; // remaining space for reading in data

    // TODO anticipációs valami, hogy visszaadja valamiben a parsing, hogy menny byte kell még az üzenet végéig
    // min count is no longer used, the function is only called for null_opt
    // if (min_count && remaining < *min_count) {
    //     remaining = *min_count;
    // }

    // set size to fit new data
    read_buffer.resize(offset + remaining);

#ifdef DEBUG_WITHOUT_UART
    // kiírja honnan kezdődnek a beolvasások
    // char* buf = new char[1000];
    // getcwd(buf, 1000);
    // std::cerr<<buf<<std::endl;
    static int fd = open("/workspaces/dora-ros/raw_data.bin", O_RDONLY);

    if (fd == -1) {
        std::cerr << "Error opening test raw_data.bin" << std::endl;
        exit(0);
    }

    // Beolvasás a fileból
    int len = read(fd, (void*) (read_buffer.data() + offset), (int) remaining);
#endif

#ifndef DEBUG_WITHOUT_UART
    // read data
    int len = read(serial_port, (void*) (read_buffer.data() + offset), (int) remaining);
#endif

    cerrdebug(std::format("\tRead {} bytes", len));

    // read error
    if (len < 0)
        return std::string(std::strerror(errno));

    // Note on read_buffer len and unescapeing:
    // When reading into read buffer, the receiveData() function tries to fill it to 64 bites.
    // This can generate up to 128 bytes when fully unescaped, but because it only tries to fill it to 64,
    // this means that 32 escaped bytes can fill that space so this approach can't read messages longer than 32 bytes.
    read_buffer.resize(offset + len);

    // unescape to make parsing easier
    size_t unescaped_size = unescapeBuffer(std::span(read_buffer).subspan(offset));

    // the data was unescaped last pass to the point off the offset
    // resize the buffer to contain the unescaped size
    read_buffer.resize(offset + unescaped_size);

    return {};
}

// parse data from read_buffer to parsedMessages()
// the processed bytes get removed from the read_buffer, and moved forward
std::expected<size_t, std::string> UARTHandler::parseFrames() {
    while (true) {
        auto ret = parseSingleFrame();

        // unexpected error
        if (!ret.has_value()) {
            // printf("%s\n", ret.error().c_str());
            return std::unexpected(ret.error());
        }

        size_t return_value = *ret;

        // temporary return values for unfinished messages
        if (ret == 101 || ret == 102) {
            cerrdebug("\tParsing read_buffer is stopped, because the last message is not finished:");
            if (ret == 101)
                cerrdebug("\t\tNot enough data from UART for parsing a single message");
            if (ret == 102)
                cerrdebug("\t\tHalf finished stream message");

            return 0;
        }
    }

    return -1;
}
// Parsing based on https://github.com/legokor/DORA-Nucleo-firmware/blob/feature%2Fjetson_comm/TestUtils%2Fpackets.md
// when this function is called, it's assumed that the read_buffer starts with UART_SOF, and the characters are escaped
// after this function finishes, the read_buffer's start will be the next frame's UART_SOF
//
// Returns 100 if a message was pushed to parsedMessages. This should be replaced by a better return type
//
// Quick, temporary fixes for unfinished message parsing:
// Returns 101 if error message would be: Not enough data from UART for parsing a single message
// Returns 102 if error message would be: Half finished stream message
std::expected<size_t, std::string> UARTHandler::parseSingleFrame() {
    cerrdebug("\tStarted parsing a single frame");
    // printf("In parseSingleFrame\n");
    // printreadbuffer(read_buffer);
    // Frame felépítése:
    // SOF, type, [ha request/reply typus, akkor sequence number], <data> ... ,  checksum, EOF
    // type lehet request, reply, stream

    auto it = read_buffer.begin();

    // lamdba function, returns true if there are at least n bytes remaining from the buffer
    auto require_bytes = [&](size_t n) {
        // printf("remaining %d, needed: %d\n OK: %s", read_buffer.end()-it, n, read_buffer.end() - it >= n ? "ok" :
        // "WARNINGNWARNINGNWARNINGNWARNINGNWARNINGNWARNINGNWARNINGNWARNINGNWARNINGNWARNINGN"); if(read_buffer.end() -
        // it == 0 & n == 2){ exit(0);
        // }
        return read_buffer.end() - it >= n;
    };

    // cuts off bytes alread read off the read_buffer's end with an offset
    // the function is unchecked, so it can cause errors if offset is incorrect
    auto cut_buffer = [&](int offset = 0) {
        // printf("Starting cut\n");
        // printreadbuffer(read_buffer);

        // printf("Cut");
        size_t remaining_size = read_buffer.end() - (it + offset);
        std::memmove(read_buffer.data(), &*it + offset, remaining_size);
        read_buffer.resize(remaining_size);

        // printreadbuffer(read_buffer);
    };

    char checksum = 0;

    // UART_SOF and frame type
    if (!require_bytes(2)) {
        return 101;
        // cut_buffer();
        return std::unexpected("Not enough data from UART for parsing a single message");
    }

    if (*it++ != UART_SOF) {
        // no need to check UART_ESC, because other EOF characters are replaced by (ESC, id) bytes

        // read in until EOF is read, then cut it do discard whole message
        // hope this works

        while (it < read_buffer.end() - 1 && *it++ != UART_EOF) {
        };
        cut_buffer();
        return std::unexpected("Frame doesn't start with SOF, read_buffer cut to next SOF");
    }

    cerrdebug("\tStart of frame found");

    std::byte frame_type = *it++;
    checksum += (char) frame_type;

    FrameCategory frame_category = (FrameCategory) (frame_type & FRAME_CATEGORY_MASK);
    uint8_t frame_type_id = (uint8_t) (frame_type & FRAME_TYPE_ID_MASK);
    switch (frame_category) {

        case FrameCategory::Request:
            // cut off unexpected frame
            cut_buffer();
            return std::unexpected("Nucleo shouldn't send request");

        case FrameCategory::Reply:

            // cut off the SOF message, so the message is discarded next parseSingleFrame is called
            cut_buffer();
            return std::unexpected(
                "Reply frames are not implemented, get the parsing of these messages from the old file");

            // TODO checksumba beletartozik az unescape?

        case FrameCategory::Stream:
            // if stream, use the rest of the frame_type byte to figure out what kind of message it is
            switch ((StreamFrameTypeID) frame_type_id) {
                case StreamFrameTypeID::Speed: {
                    cerrdebug("\t\tStarted parsing speed frame");
                    SpeedData spd;

                    // data + checksum + EOF
                    if (!require_bytes(sizeof(spd) + sizeof(std::byte) + sizeof(UART_EOF))) {
                        return 102;
                        // return std::unexpected("Half finished stream message");
                    }

                    std::memcpy(&spd, &*it, sizeof(spd));
                    // printf("%d", sizeof(spd));
                    it += sizeof(spd);

                    // checksum, discarded
                    std::byte checksum_byte = *it++;
                    std::byte eof_byte = *it++;

                    if (eof_byte != UART_EOF) {
                        // dont cut the message, to get the EOF

                        // Itt levágom az üzenetet, mert a bemenetem nincs escapelve, és ha itt levágom, eldobom a
                        // hibákat. Dórán nem kéne előjönnie, mert ott rendesen vannak escapelve az üzenetek. És amúgy
                        // is, ha nincs itt EOF, akkor a frame rosszul formált, eldobjuk.
                        cut_buffer();
                        return std::unexpected("Speed message is not closed with EOF");
                    }

                    cerrdebug(std::format("\t\tPushed speed message [x: {}, y: {}, w: {}] to parsedMessages", spd.x,
                                          spd.y, spd.w));

                    parsedMessages.push(spd);
                    // printf("Pushed speed message: ");
                    // printf("x: %f, y: %f, w: %f\n", spd.x, spd.y, spd.w);
                    cut_buffer();
                    return 100;
                }

                case StreamFrameTypeID::Status: {
                    cerrdebug("\t\tStarted parsing status frame");
                    RobotStatus stat;

                    // data + checksum + EOF
                    if (!require_bytes(sizeof(stat) + sizeof(std::byte) + sizeof(UART_EOF)))
                        return 102;

                    std::memcpy(&stat, &*it, sizeof(stat));
                    it += sizeof(stat);
                    std::byte checksum_byte = *it++;
                    std::byte eof_byte = *it++;
                    if (eof_byte != UART_EOF) {
                        // lehet ide kéne egy cut, hogy azért menjen

                        // Itt levágom az üzenetet, mert a bemenetem nincs escapelve, és ha itt levágom, eldobom a
                        // hibákat. Dórán nem kéne előjönnie, mert ott rendesen vannak escapelve az üzenetek. És amúgy
                        // is, ha nincs itt EOF, akkor a frame rosszul formált, eldobjuk.
                        cut_buffer();
                        return std::unexpected("Status message is not closed with EOF");
                    }

                    cerrdebug(std::format("\t\tPushed status message [{}] to parsedMessages", stat.voltage));

                    parsedMessages.push(stat);
                    cut_buffer();
                    return 100;
                }
                default:
                    // cut off undiagnosable data?
                    cut_buffer();
            }

        default:
            // cut off undiagnosable data
            cut_buffer();
            return std::unexpected(
                std::format("Couldn't identify frame category from byte {:08b}", std::to_integer<uint8_t>(frame_type)));
            break;
    }

    return std::unexpected(
        "Control flow should not reach this point. (A switch statement before this return has returns in all it's "
        "branches.)");
}

UARTHandler::~UARTHandler() {
    close(serial_port);
}

} // namespace dora
