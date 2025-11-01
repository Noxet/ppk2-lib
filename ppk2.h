#pragma once

#include <cstdint>
#include <string>
#include <termios.h>


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

enum class Mode
{
    AMP_MODE,
    SRC_MODE,
};


class Serial
{
public:
    Serial(const std::string &path);
    ~Serial();

    ssize_t read(char *buf, size_t len);
    bool write(char *data, size_t len);

private:
    std::string m_path;
    int m_fd;

    struct termios m_oldtio;
    struct termios m_newtio;
};



class PPK2
{
public:
    PPK2(const std::string &path);
    ~PPK2();

    bool setSourceVoltage(unsigned int voltage);
    bool setDUTPower(bool on);
    bool setMode(enum Mode mode);
    void startMeasure();
    bool stopMeasure();
    void reset();
    void getMeta(char *buf, size_t len, ssize_t *dataRead);
    void parseMeta(const std::string &meta);
    void printMeta();

private:

    // void convertADC(uint8_t *data, size_t len);
    void convertADC(uint8_t *data, size_t len, double *result, size_t &cnt);

private:
    Serial m_serial;

    double m_R[5];
    double m_GS[5];
    double m_GI[5];
    double m_O[5];
    double m_S[5];
    double m_I[5];
    double m_UG[5];

    int m_vdd;
};
