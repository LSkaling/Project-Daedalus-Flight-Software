#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>
#include <SD.h>

class Logging {
public:
    Logging(bool debug, bool logToSD, int SD_CS);

    void log(const char* message);
    void log(const char* names[], const float values[], size_t size);

    bool begin(const char *names[], size_t size);

    void flush();


private:
    bool debug;
    bool logToSD;
    int SD_CS;
    int getNextLogFileNumber();
    int logNumber;
    String logFileName;
    File dataFile;
};

#endif