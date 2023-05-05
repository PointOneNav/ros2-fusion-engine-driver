#pragma once

#include <cstdint>
#include <functional>

class DataListener {
 public:
  virtual ~DataListener() = default;

  virtual void listen() = 0;
  virtual void write(uint8_t* data, size_t size) = 0;
  virtual void setCallback(
      const std::function<void(uint8_t*, size_t)>& func) = 0;

 private:
};
