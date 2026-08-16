#pragma once

#include "WooDriveSdk.h"
#include <cstdint>
#include <windows.h>

class WindowsSerialTransport : public ITransport
{
public:
    explicit WindowsSerialTransport(const char* portName, int baudrate = 9600);
    ~WindowsSerialTransport() override;

    bool isOpen() const;
    int baudrate() const;

    size_t write(const uint8_t* data, size_t len) override;
    int available() override;
    int read() override;
    void flush() override;
    void clearRx() override;

private:
    HANDLE handle_;
    int baudrate_;
};

class WindowsClock : public IClock
{
public:
    WindowsClock();
    uint32_t nowMs() const override;

private:
    ULONGLONG startMs_;
};
