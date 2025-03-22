#include "Logger.h"

String Logger::generate_unique_file_name(const String &prefix, const String &extension)
{
    int counter = 0;
    String new_file_name;
    while (true)
    {
        new_file_name = prefix;
        if (counter < 10)
        {
            new_file_name += "00";
        }
        else if (counter < 100)
        {
            new_file_name += "0";
        }
        new_file_name += String(counter);
        new_file_name += extension;
        
        if (!SD.exists(new_file_name))
        {
            break;
        }
        counter++;
    }
    return new_file_name;
}

bool Logger::init(const String &prefix, const String &extension)
{
    file_name = generate_unique_file_name(prefix, extension);
    log_file = SD.open(file_name, FILE_WRITE);
    if (!log_file)
    {
        #if DEBUG
        Serial.println("Failed to open log file: " + log_file);
        #endif
        return false;
    }
    return true;
}

// --- EventLogger implementation --- 
bool EventLogger::init(const String &prefix)
{
    return Logger::init(prefix, ".TXT");
}

void EventLogger::log(const String &data)
{
    if (log_file)
    {
        String message = "["+ String(micros()) + "] " + data;
        log_file.println(message);
        log_file.flush();
    }
}

EventLogger::~EventLogger()
{
    if (log_file)
    {
        log_file.close();
    }
}

// --- DataLogger Implementation ---

bool DataLogger::init(unsigned long interval, const String &prefix)
{
    log_interval = interval;
    last_log_time = 0;
    return Logger::init(prefix, ".CSV");
}

void DataLogger::log(const String &data)
{
    unsigned long current_time = millis();
    if (current_time - last_log_time >= log_interval)
    {
        if (log_file)
        {
            log_file.println(data);
            log_file.flush();
        }
        last_log_time = current_time;
    }
}

DataLogger::~DataLogger()
{
    if (log_file)
    {
        log_file.close();
    }
}