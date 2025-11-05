#pragma once

#include <cstdint>
#include <string>
#include <termios.h>



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
    void startMeasure(size_t duration);
    bool stopMeasure();
    void reset();
    void getMeta();
    void printMeta();

private:

    // void convertADC(uint8_t *data, size_t len);
    void convertADC(uint8_t *data, size_t len, double *result, size_t &cnt);
    void parseMeta(const std::string &meta);
    template<typename T> T parseRow(std::istringstream &iss);

private:
    Serial m_serial;

    struct Meta
    {
        int vdd;
        int hw;
        int mode;
        int ia;

        double R[5];
        double GS[5];
        double GI[5];
        double O[5];
        double S[5];
        double I[5];
        double UG[5];
    };

    struct Meta m_meta;
};
