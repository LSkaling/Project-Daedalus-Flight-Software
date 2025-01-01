#include "Logging.h"

Logging::Logging(bool debug, bool logToSD, int SD_CS)
{
    this->debug = debug;
    this->logToSD = logToSD;
    this->SD_CS = SD_CS;
}

int Logging::getNextLogFileNumber()
{
    int logNumber = 0;

    File root = SD.open("/");
    Serial1.println("Files found on the SD card:");
    while (true)
    {
        File entry = root.openNextFile();
        Serial1.print("  ");
        Serial1.println(entry.name());
        if (!entry)
        {
            // No more files
            break;
        }

        String fileName = entry.name();
        if (fileName.startsWith("LOG") && fileName.endsWith(".TXT"))
        {
            // Extract the number part of the filename (e.g., "log3.txt" -> 3)
            int currentLogNumber = fileName.substring(3, fileName.length() - 4).toInt();
            if (currentLogNumber > logNumber)
            {
                logNumber = currentLogNumber;
            }
        }
        entry.close();
    }

    // Return the next available log number
    return logNumber + 1;
}

bool Logging::begin(const char* names[], size_t size)
{
    if (debug)
    {
        int start_wait = millis();
        while (!Serial1)
        {
            delay(10);
            if (millis() - start_wait > 5000)
            {
                return false;
            }
        }
    }
    if (logToSD)
    {
        if (!SD.begin(SD_CS))
        {
            return false;
        }
        logNumber = getNextLogFileNumber();
        logFileName = "log" + String(logNumber) + ".txt";
        dataFile = SD.open(logFileName, FILE_WRITE);
        if (!dataFile)
        {
            return false;
        }
    }
    return true;
}

void Logging::log(const char* message)
{
    if (debug)
    {
        Serial1.println(message);
    }
    if (logToSD)
    {
        dataFile.println(message);
    }
}

void Logging::log(const char* names[], const float values[], size_t size)
{
    char buffer[256]; // Pre-allocated buffer for printing
    for (size_t i = 0; i < size; i++)
    {
        snprintf(buffer, sizeof(buffer), "%s: %.2f,", names[i], values[i]);
        if (debug)
        {
            Serial1.print(buffer);
        }
        if (logToSD && dataFile)
        {
            dataFile.print(buffer);
        }
    }
    if (debug)
    {
        Serial1.println();
    }
    if (logToSD && dataFile)
    {
        dataFile.println();
    }
}

void Logging::flush()
{
    if (logToSD)
    {
        dataFile.flush();
    }
}




