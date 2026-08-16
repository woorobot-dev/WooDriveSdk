#include "WindowsSerialTransport.h"

#include <algorithm>
#include <string>

WindowsSerialTransport::WindowsSerialTransport(const char* portName, int baudrate)
    : handle_(INVALID_HANDLE_VALUE), baudrate_(baudrate)
{
    std::string device = "\\\\.\\";
    device += portName;
    handle_ = CreateFileA(device.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                          OPEN_EXISTING, 0, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) return;

    DCB dcb{};
    COMMTIMEOUTS timeouts{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle_, &dcb)) goto fail;
    dcb.BaudRate = static_cast<DWORD>(baudrate_);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    if (!SetCommState(handle_, &dcb)) goto fail;

    timeouts.ReadIntervalTimeout = MAXDWORD;
    if (!SetCommTimeouts(handle_, &timeouts)) goto fail;
    SetupComm(handle_, 4096, 4096);
    PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return;

fail:
    CloseHandle(handle_);
    handle_ = INVALID_HANDLE_VALUE;
}

WindowsSerialTransport::~WindowsSerialTransport()
{
    if (handle_ != INVALID_HANDLE_VALUE) CloseHandle(handle_);
}

bool WindowsSerialTransport::isOpen() const { return handle_ != INVALID_HANDLE_VALUE; }
int WindowsSerialTransport::baudrate() const { return baudrate_; }

size_t WindowsSerialTransport::write(const uint8_t* data, size_t len)
{
    if (!isOpen()) return 0;
    DWORD written = 0;
    const DWORD requested = static_cast<DWORD>(std::min<size_t>(len, MAXDWORD));
    return WriteFile(handle_, data, requested, &written, nullptr) ? written : 0;
}

int WindowsSerialTransport::available()
{
    if (!isOpen()) return 0;
    COMSTAT stat{};
    DWORD errors = 0;
    return ClearCommError(handle_, &errors, &stat) ? static_cast<int>(stat.cbInQue) : 0;
}

int WindowsSerialTransport::read()
{
    if (!isOpen()) return -1;
    uint8_t value = 0;
    DWORD count = 0;
    return ReadFile(handle_, &value, 1, &count, nullptr) && count == 1 ? value : -1;
}

void WindowsSerialTransport::flush()
{
    if (isOpen()) FlushFileBuffers(handle_);
}

void WindowsSerialTransport::clearRx()
{
    if (isOpen()) PurgeComm(handle_, PURGE_RXABORT | PURGE_RXCLEAR);
}

WindowsClock::WindowsClock() : startMs_(GetTickCount64()) {}

uint32_t WindowsClock::nowMs() const
{
    return static_cast<uint32_t>(GetTickCount64() - startMs_);
}
