#ifndef DATA_LISTENER_HPP_
#define DATA_LISTENER_HPP_

#include <functional>

class DataListener
{
    public:
        virtual ~DataListener() = default;

        virtual void listen() = 0;
        virtual void setCallback(const std::function<void(uint8_t*, size_t)>&func) = 0;
    private:
};

#endif /* !DATA_LISTENER_HPP_ */