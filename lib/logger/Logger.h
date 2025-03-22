// logger.h

#pragma once

#include <Arduino.h>
#include <SD.h>


class Logger
{
    protected:
        File log_file;
        String file_name;

        String generate_unique_file_name(const String &prefix, const String &extension);
    
    public:
        virtual bool init(const String &prefix, const String &extension);
        virtual void log(const String &data) = 0;
};


class EventLogger: public Logger
{
    public: 
        bool init(const String &prefix = "EVENT");
        void log(const String &data) override;
        ~EventLogger();
};


class DataLogger: public Logger
{
    private:
        unsigned long log_interval;
        unsigned long last_log_time;

    public: 
        bool init(unsigned long interval, const String &prefix = "DATA");
        void log(const String &data) override;
        ~DataLogger();
};