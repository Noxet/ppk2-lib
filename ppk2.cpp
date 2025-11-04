#include "ppk2.h"
#include "profiler.cpp"

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

#define BAUDRATE B115200
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1


#define TOCHAR(x) (static_cast<char>(x))

constexpr unsigned int MIN_VOLTAGE = 800;
constexpr unsigned int MAX_VOLTAGE = 5000;

using namespace std;



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
    double adcMult = 1.8 / 163840;
    assert(len % 4 == 0);
    // size_t values = len / 4;
    // cout << "values: " << values << endl;
    cnt = 0;
    for (size_t i = 0; i + 3 < len; i += 4)
    {
        uint32_t adcVal = (data[i + 3] << 24) | (data[i + 2] << 16) | (data[i + 1] << 8) | (data[i + 0]);

        uint32_t adcRes = (adcVal & 0x3FFF) * 4;
        uint32_t currMask = ((1 << 3) - 1) << 14;
        uint32_t currRange = (adcVal & currMask) >> 14;
        currRange = min(currRange, (uint32_t) 4);
        double analogValNoGain = (adcRes - m_O[currRange]) * (adcMult / m_R[currRange]);
        double current = m_UG[currRange] * (analogValNoGain * (m_GS[currRange] * analogValNoGain + m_GI[currRange])
                + (m_S[currRange] * (m_vdd / 1000.0) + m_I[currRange]));
        current *= 1000000;

        result[cnt] = current;
        cnt++;
        // printf(" %.4f ", current);
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

void PPK2::startMeasure()
{
    double *result = (double *) malloc(1024 * 1024 * sizeof(*result));
    assert(result);

    TimeFunction;

    char data[1] = { TOCHAR(Command::AVERAGE_START) };
    m_serial.write(data, sizeof(data));

    uint8_t buf[128]{};

    int reads = 0;
    size_t totCnt = 0;
    {
        TimeBlock("Read and convert");
        auto start = chrono::steady_clock::now();
        while (chrono::steady_clock::now() - start < 5s)
        {
            ssize_t count = m_serial.read((char *)buf, sizeof(buf));
            if (count == 0) continue;
            // printf("RAW (%ld): [", count);
            // for (int i = 0; i < count; i++)
            // {
            //     printf("%02x, ", buf[i]);
            // }
            // printf("]\n");
            size_t cnt = 0;
            // printf("[");
            convertADC(buf, count, &result[totCnt], cnt);
            totCnt += cnt;
            reads++;
        }
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
}


void PPK2::reset()
{
    char data[1] = { TOCHAR(Command::RESET) };
    if (!m_serial.write(data, sizeof(data)))
    {
        cerr << "Failed to write RESET" << endl;
    }
}


void PPK2::getMeta(char *buf, size_t len, ssize_t *dataRead)
{
    TimeFunction;

    char data[] = { TOCHAR(Command::GET_META_DATA) };
    if (!m_serial.write(data, sizeof(data)))
    {
        cerr << "Failed to get meta data\n";
    }

    ssize_t count = 0;
    size_t totalCount = 0;
    do
    {
        count = m_serial.read(&buf[totalCount], len);
        // printf("Data during get meta: [");
        // for (int i = totalCount; i < totalCount + count; i++)
        // {
        //     printf("%x ", buf[i]);
        // }
        // printf("]\n");
        totalCount += count;
        // cout << "tot: " << totalCount << endl;
    } while (count != 0);

    *dataRead = totalCount;
}

template<typename T>
T parseRow(istringstream &iss)
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

    m_R[0] = parseRow<double>(iss);
    m_R[1] = parseRow<double>(iss);
    m_R[2] = parseRow<double>(iss);
    m_R[3] = parseRow<double>(iss);
    m_R[4] = parseRow<double>(iss);

    m_GS[0] = parseRow<double>(iss);
    m_GS[1] = parseRow<double>(iss);
    m_GS[2] = parseRow<double>(iss);
    m_GS[3] = parseRow<double>(iss);
    m_GS[4] = parseRow<double>(iss);

    m_GI[0] = parseRow<double>(iss);
    m_GI[1] = parseRow<double>(iss);
    m_GI[2] = parseRow<double>(iss);
    m_GI[3] = parseRow<double>(iss);
    m_GI[4] = parseRow<double>(iss);

    m_O[0] = parseRow<double>(iss);
    m_O[1] = parseRow<double>(iss);
    m_O[2] = parseRow<double>(iss);
    m_O[3] = parseRow<double>(iss);
    m_O[4] = parseRow<double>(iss);

    m_vdd = parseRow<int>(iss);
    int hw = parseRow<int>(iss);
    int mode = parseRow<int>(iss);

    m_S[0] = parseRow<double>(iss);
    m_S[1] = parseRow<double>(iss);
    m_S[2] = parseRow<double>(iss);
    m_S[3] = parseRow<double>(iss);
    m_S[4] = parseRow<double>(iss);

    m_I[0] = parseRow<double>(iss);
    m_I[1] = parseRow<double>(iss);
    m_I[2] = parseRow<double>(iss);
    m_I[3] = parseRow<double>(iss);
    m_I[4] = parseRow<double>(iss);

    m_UG[0] = parseRow<double>(iss);
    m_UG[1] = parseRow<double>(iss);
    m_UG[2] = parseRow<double>(iss);
    m_UG[3] = parseRow<double>(iss);
    m_UG[4] = parseRow<double>(iss);

    int ia = parseRow<int>(iss);
}


void PPK2::printMeta()
{
    printf("R: [");
    for (const auto &i : m_R)
    {
        printf("%f ", i);
    }

    printf("]\nGS: [");
    for (const auto &i : m_GS)
    {
        printf("%f ", i);
    }

    printf("]\nGI: [");
    for (const auto &i : m_GI)
    {
        printf("%f ", i);
    }

    printf("]\nO: [");
    for (const auto &i : m_O)
    {
        printf("%f ", i);
    }

    printf("]\nS: [");
    for (const auto &i : m_S)
    {
        printf("%f ", i);
    }

    printf("]\nI: [");
    for (const auto &i : m_I)
    {
        printf("%f ", i);
    }

    printf("]\nUG: [");
    for (const auto &i : m_UG)
    {
        printf("%f ", i);
    }
    printf("]\n");
}


int main(int argc, char *argv[])
{
    beginProfiler();

    if (argc < 2)
    {
        cerr << format("Usage {} <serial port>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    PPK2 ppk{argv[1]};
    if (!ppk.stopMeasure())
    {
        cout << "Failed to stop measure" << endl;
    }
    char *buf = (char *) malloc(100 * 1024);
    ssize_t count;
    size_t tries = 0;
    do
    {
        ppk.getMeta(buf, sizeof(buf) - 1, &count);
        if (count == 0)
        {
            cout << "Could not read meta" << endl;
        }
        tries++;
    } while (count == 0 && tries < 5);

    if (tries >= 5)
    {
        cout << "Failed to get meta, quitting..." << endl;
        return EXIT_FAILURE;
    }

    buf[count] = 0;
    // cout << buf << endl;
    // cout << "DONE" << endl;

    ppk.parseMeta(buf);
    // ppk.printMeta();
    // ppk.reset();
    ppk.setMode(Mode::SRC_MODE);
    ppk.setSourceVoltage(3300);
    ppk.setDUTPower(true);
    // sleep(2);


    cout << "START MEASURE" << endl;
    ppk.startMeasure();
    ppk.stopMeasure();

    ppk.setDUTPower(false);
    free(buf);

    endProfiler();
}
