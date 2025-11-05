#include "ppk2.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ios>
#include <iostream>
#include <fstream>
#include <limits>
#include <string>
#include <system_error>
#include <sstream>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>


using namespace std;


#define BAUDRATE B115200
#define TOCHAR(x) (static_cast<char>(x))

static constexpr unsigned int MIN_VOLTAGE = 800;
static constexpr unsigned int MAX_VOLTAGE = 5000;
static const std::string AMPERE_MODE = "AMPERE_MODE";
static const std::string SOURCE_MODE = "SOURCE_MODE";


enum class Command : char
{
    NO_OP = 0x00,
    TRIGGER_SET = 0x01,
    AVG_NUM_SET = 0x02,
    TRIGGER_WINDOW_SET = 0x03,
    TRIGGER_INTERVAL_SET = 0x04,
    TRIGGER_SINGLE_SET = 0x05,
    AVERAGE_START = 0x06,
    AVERAGE_STOP = 0x07,
    RANGE_SET = 0x08,
    LCD_SET = 0x09,
    TRIGGER_STOP = 0x0a,
    DEVICE_RUNNING_SET = 0x0c,
    REGULATOR_SET = 0x0d,
    SWITCH_POINT_DOWN = 0x0e,
    SWITCH_POINT_UP = 0x0f,
    TRIGGER_EXT_TOGGLE = 0x11,
    SET_POWER_MODE = 0x11,
    RES_USER_SET = 0x12,
    SPIKE_FILTERING_ON = 0x15,
    SPIKE_FILTERING_OFF = 0x16,
    GET_META_DATA = 0x19,
    RESET = 0x20,
    SET_USER_GAINS = 0x25,
};


/**
 * Serial class
 * Handles low-level serial port communication
 */

Serial::Serial(const std::string &path)
    : m_path(path)
{
    m_fd = open(m_path.c_str(), O_RDWR | O_NOCTTY);
    printf("Opening: %s\n", m_path.c_str());
    if (m_fd < 0) throw system_error(errno, generic_category(), "Failed to open serial port");

    tcgetattr(m_fd, &m_oldtio); /* save current port settings */

    memset(&m_newtio, 0, sizeof(m_newtio));
    // m_newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    m_newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    m_newtio.c_iflag = IGNPAR;
    m_newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    m_newtio.c_lflag = 0;

    m_newtio.c_cc[VTIME] = 5; /* inter-character timer unused */
    m_newtio.c_cc[VMIN] = 0;  /* blocking read until 5 chars received */

    tcflush(m_fd, TCIFLUSH);
    tcsetattr(m_fd, TCSANOW, &m_newtio);
}


Serial::~Serial()
{
    tcsetattr(m_fd, TCSANOW, &m_oldtio);
    if (m_fd >= 0) close(m_fd);
    cout << "Cleanup serial" << endl;
}


// TODO(noxet): Maybe return the amount written instead of bool?
bool Serial::write(char *data, size_t len)
{
    size_t totalWrite{0};
    do
    {
        ssize_t dataWritten = ::write(m_fd, &data[totalWrite], len - totalWrite);
        if (dataWritten < 0) return false;
        totalWrite += dataWritten;
    } while (totalWrite < len);

    return true;
}


ssize_t Serial::read(char *buf, size_t len)
{
    struct pollfd pfd = { .fd = m_fd, .events = POLLIN };
    int res = poll(&pfd, 1, 500);
    if (res == 0)
    {
        // timeout
        cout << "TIMEOUT" << endl;
        return 0;
    }

    ssize_t count = ::read(m_fd, buf, len);
    if (count  < 0) 
    {
        cout << "READ FAILED" << endl;
    }

    return count;
}


/**
 * PPK2 Class
 * Manages the Power Profiler device.
 */

PPK2::PPK2(const string &path)
    : m_serial(path)
{
    
}


PPK2::~PPK2()
{
    //reset();
}


bool PPK2::setMode(enum Mode mode)
{
    char data[2];
    data[0] = TOCHAR(Command::SET_POWER_MODE);
    data[1] = mode == Mode::SRC_MODE ? TOCHAR(Command::AVG_NUM_SET) : TOCHAR(Command::TRIGGER_SET);
    return m_serial.write(data, sizeof(data));
}


bool PPK2::setSourceVoltage(unsigned int mv)
{
    unsigned int setVoltage = std::clamp(mv, MIN_VOLTAGE, MAX_VOLTAGE);
    char data[3];
    data[0] = TOCHAR(Command::REGULATOR_SET);
    data[1] = (setVoltage & 0xff00) >> 8;
    data[2] = setVoltage & 0xff;
    return m_serial.write(data, sizeof(data));
}


bool PPK2::setDUTPower(bool on)
{
    char data[2];
    data[0] = TOCHAR(Command::DEVICE_RUNNING_SET);
    data[1] = on ? TOCHAR(Command::TRIGGER_SET) : TOCHAR(Command::NO_OP);
    return m_serial.write(data, sizeof(data));
}


void PPK2::convertADC(uint8_t *data, size_t len, double *result, size_t &cnt)
{
    // TODO(noxet): save this in class instead, no need to calc the same shit again and again
    double adcMult = 1.8 / 163840;
    // TODO(noxet): Handle this in a better way.
    // Save a small buffer with the remainder if len % 4 != 0, and handle it at the next call
    assert(len % 4 == 0);
    cnt = 0;
    for (size_t i = 0; i + 3 < len; i += 4)
    {
        uint32_t adcVal = (data[i + 3] << 24) | (data[i + 2] << 16) | (data[i + 1] << 8) | (data[i + 0]);

        uint32_t adcRes = (adcVal & 0x3FFF) * 4;
        uint32_t currMask = ((1 << 3) - 1) << 14;
        uint32_t currRange = (adcVal & currMask) >> 14;
        currRange = min(currRange, (uint32_t) 4);
        double analogValNoGain = (adcRes - m_meta.O[currRange]) * (adcMult / m_meta.R[currRange]);
        double current = m_meta.UG[currRange] * (analogValNoGain * (m_meta.GS[currRange] * analogValNoGain + m_meta.GI[currRange])
                + (m_meta.S[currRange] * (m_meta.vdd / 1000.0) + m_meta.I[currRange]));
        current *= 1000000.0;

        result[cnt] = current;
        cnt++;
    }
}


bool PPK2::stopMeasure()
{
    char data[1] = { TOCHAR(Command::AVERAGE_STOP) };
    if (!m_serial.write(data, sizeof(data)))
    {
        cout << "Failed to stop measure" << endl;
        return false;
    }

    // clear remaining data in serial buffer
    char buf[128];
    while (m_serial.read(buf, sizeof(buf)) != 0);

    return true;
}


// TODO(noxet): return status code instead, make sure to handle errors from write and read
void PPK2::startMeasure(size_t duration)
{
    // TODO(noxet): use a smaller buffer and write to file in chunks.
    double *result = (double *) malloc(1024 * 1024 * sizeof(*result));
    assert(result);

    char data[1] = { TOCHAR(Command::AVERAGE_START) };
    m_serial.write(data, sizeof(data));

    uint8_t buf[128]{};

    int reads = 0;
    size_t totCnt = 0;
    auto runTime = chrono::duration<double>(duration);
    auto start = chrono::steady_clock::now();
    while (chrono::steady_clock::now() - start < runTime)
    {
        // TODO(noxet): Keep track of the amount of timeouts.
        // If we get too many, maybe the device has been disconnected, we need to handle it.
        // Also, reset the timeouts counter when we get actual data
        ssize_t count = m_serial.read((char *)buf, sizeof(buf));
        if (count == 0) continue;
        size_t cnt = 0;
        convertADC(buf, count, &result[totCnt], cnt);
        totCnt += cnt;
        reads++;
    }

    cout << "TOTAL COUNT: " << totCnt << endl;

    ofstream file("out.txt");
    if (!file)
    {
        cerr << "Failed to open file" << endl;
    }

    for (size_t i = 0; i < totCnt; i++)
    {
        file << result[i] << ", ";
    }

    free(result);
}


// TODO(noxet): return status code instead
void PPK2::reset()
{
    char data[1] = { TOCHAR(Command::RESET) };
    if (!m_serial.write(data, sizeof(data)))
    {
        cerr << "Failed to write RESET" << endl;
    }
}

// TODO(noxet): return status code instead
void PPK2::getMeta()
{
    char data[] = { TOCHAR(Command::GET_META_DATA) };
    if (!m_serial.write(data, sizeof(data)))
    {
        cerr << "Failed to get meta data\n";
    }

    constexpr size_t bufSize = 10 * 1024;
    char *buf = (char *) malloc(bufSize);
    ssize_t count = 0;
    size_t totalCount = 0;
    do
    {
        // read until we get a serial timeout
        count = m_serial.read(&buf[totalCount], bufSize - totalCount);
        totalCount += count;
        if (totalCount >= bufSize)
        {
            cout << format("Buffer full while reading meta data, please increase the buffer size\n");
            break;
        }
    } while (count != 0);

    buf[totalCount] = 0;
    parseMeta(buf);
    free(buf);
}


template<typename T>
T PPK2::parseRow(istringstream &iss)
{
    iss.ignore(numeric_limits<streamsize>::max(), ':');

    T v{};
    iss >> v;
    return v;
}


void PPK2::parseMeta(const string &meta)
{
    istringstream iss(meta);
    int calibrated = parseRow<int>(iss);

    m_meta.R[0] = parseRow<double>(iss);
    m_meta.R[1] = parseRow<double>(iss);
    m_meta.R[2] = parseRow<double>(iss);
    m_meta.R[3] = parseRow<double>(iss);
    m_meta.R[4] = parseRow<double>(iss);

    m_meta.GS[0] = parseRow<double>(iss);
    m_meta.GS[1] = parseRow<double>(iss);
    m_meta.GS[2] = parseRow<double>(iss);
    m_meta.GS[3] = parseRow<double>(iss);
    m_meta.GS[4] = parseRow<double>(iss);

    m_meta.GI[0] = parseRow<double>(iss);
    m_meta.GI[1] = parseRow<double>(iss);
    m_meta.GI[2] = parseRow<double>(iss);
    m_meta.GI[3] = parseRow<double>(iss);
    m_meta.GI[4] = parseRow<double>(iss);

    m_meta.O[0] = parseRow<double>(iss);
    m_meta.O[1] = parseRow<double>(iss);
    m_meta.O[2] = parseRow<double>(iss);
    m_meta.O[3] = parseRow<double>(iss);
    m_meta.O[4] = parseRow<double>(iss);

    m_meta.vdd = parseRow<int>(iss);
    m_meta.hw = parseRow<int>(iss);
    m_meta.mode = parseRow<int>(iss);

    m_meta.S[0] = parseRow<double>(iss);
    m_meta.S[1] = parseRow<double>(iss);
    m_meta.S[2] = parseRow<double>(iss);
    m_meta.S[3] = parseRow<double>(iss);
    m_meta.S[4] = parseRow<double>(iss);

    m_meta.I[0] = parseRow<double>(iss);
    m_meta.I[1] = parseRow<double>(iss);
    m_meta.I[2] = parseRow<double>(iss);
    m_meta.I[3] = parseRow<double>(iss);
    m_meta.I[4] = parseRow<double>(iss);

    m_meta.UG[0] = parseRow<double>(iss);
    m_meta.UG[1] = parseRow<double>(iss);
    m_meta.UG[2] = parseRow<double>(iss);
    m_meta.UG[3] = parseRow<double>(iss);
    m_meta.UG[4] = parseRow<double>(iss);

    m_meta.ia = parseRow<int>(iss);

    // The meta data ends with an "END" string
    string end{};
    iss >> end;
    if (end != "END")
    {
        cerr << format("Did not parse entire meta data\n");
    }
}


void PPK2::printMeta()
{
    printf("R: [");
    for (const auto &i : m_meta.R)
    {
        printf("%f ", i);
    }

    printf("]\nGS: [");
    for (const auto &i : m_meta.GS)
    {
        printf("%f ", i);
    }

    printf("]\nGI: [");
    for (const auto &i : m_meta.GI)
    {
        printf("%f ", i);
    }

    printf("]\nO: [");
    for (const auto &i : m_meta.O)
    {
        printf("%f ", i);
    }

    printf("]\nS: [");
    for (const auto &i : m_meta.S)
    {
        printf("%f ", i);
    }

    printf("]\nI: [");
    for (const auto &i : m_meta.I)
    {
        printf("%f ", i);
    }

    printf("]\nUG: [");
    for (const auto &i : m_meta.UG)
    {
        printf("%f ", i);
    }
    printf("]\n");
}


int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        cerr << format("Usage {} <serial port> <sampling duration in seconds (double)>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    double runTime = atof(argv[2]);
    PPK2 ppk{argv[1]};
    if (!ppk.stopMeasure())
    {
        cout << "Failed to stop measure" << endl;
    }

    ppk.getMeta();

    ppk.setMode(Mode::SRC_MODE);
    ppk.setSourceVoltage(3300);
    ppk.setDUTPower(true);

    cout << "START MEASURE" << endl;
    ppk.startMeasure(runTime);
    ppk.stopMeasure();

    ppk.setDUTPower(false);
}
