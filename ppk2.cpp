#include "ppk2.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>

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
    m_newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
    m_newtio.c_iflag = IGNPAR;
    m_newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    m_newtio.c_lflag = 0;

    m_newtio.c_cc[VTIME] = 0; /* inter-character timer unused */
    m_newtio.c_cc[VMIN] = 5;  /* blocking read until 5 chars received */

    tcflush(m_fd, TCIFLUSH);
    tcsetattr(m_fd, TCSANOW, &m_newtio);
}

Serial::~Serial()
{
    if (m_fd >= 0) close(m_fd);
    tcsetattr(m_fd, TCSANOW, &m_oldtio);
    cout << "Cleanup serial" << endl;
}

bool Serial::write(char *data, size_t len)
{
    size_t totalWrite{0};
    do
    {
        size_t dataWritten = ::write(m_fd, &data[totalWrite], len - totalWrite);
        if (dataWritten < 0) return false;
        totalWrite += dataWritten;
    } while (totalWrite < len);

    return true;
}


ssize_t Serial::read(char *buf, size_t len)
{
    ssize_t count = ::read(m_fd, buf, len);
    if (count  < 0) 
    {
        cout << "READ FAILED" << endl;
    }

    return count;
}


PPK2::PPK2(const string &path)
    : m_serial(path)
{
    
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


void PPK2::startMeasure()
{
    char data[1] = { TOCHAR(Command::AVERAGE_START) };
    m_serial.write(data, sizeof(data));

    char buf[1024];
    while (true)
    {
        ssize_t count = m_serial.read(buf, sizeof(buf) - 1);
        if (count == 0) continue;
        buf[count] = 0;
        cout << buf << endl;
    }
}


int main(int argc, char *argv[]) {
    int fd, c, res;

    PPK2 ppk{"/dev/ttyACM0"};
    ppk.setMode(Mode::SRC_MODE);
    ppk.setSourceVoltage(3200);
    ppk.setDUTPower(true);


    // ppk.startMeasure();
    // char get_meta[1] = {0x19};
    // write(fd, get_meta, 1);

}
