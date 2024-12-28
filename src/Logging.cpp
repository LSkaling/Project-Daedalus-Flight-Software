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

bool Logging::begin(String names[], size_t size)
{
    if (debug)
    {
        Serial1.begin(115200);
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

void Logging::log(String message)
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

void Logging::log(String values[], String names[], size_t size)
{
    if (sizeof(names) != sizeof(values))
    {
        return;
    }
    if (debug)
    {
        for (int i = 0; i < size; i++)
        {
            Serial1.print(names[i]);
            Serial1.print(": ");
            Serial1.print(values[i]);
            Serial1.print(",");
        }
        Serial1.println();
    }
    if (logToSD)
    {
        for (int i = 0; i < sizeof(values) / sizeof(values[0]); i++)
        {
            dataFile.print(values[i]);
            dataFile.print(",");
        }
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




