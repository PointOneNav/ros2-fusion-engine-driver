#pragma once

#include <functional>
#include <cstdint>

class DataListener
{
    public:
        virtual ~DataListener() = default;

        virtual void listen() = 0;
        virtual void setCallback(const std::function<void(uint8_t*, size_t)>&func) = 0;
    private:
};