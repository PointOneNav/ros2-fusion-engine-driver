#pragma once

#include <cstdint>
#include <functional>

/**
 * @brief An abstract base class that defines a data listener interface.
 *
 * The DataListener class is used as a base class for implementing different
 * types of data listeners, such as a TCP listener or a serial listener. It
 * defines a set of pure virtual functions that must be implemented by any
 * subclass, including functions for starting and stopping the listener, writing
 * data to the listener, and setting a callback function for handling incoming
 * data.
 */
class DataListener {
 public:
  virtual ~DataListener() = default;

  /**
   * @brief Starts the data listener.
   *
   * This pure virtual function must be implemented by any subclass of
   * DataListener. It is responsible for starting the data listener and
   * listening for incoming data.
   */
  virtual void listen() = 0;

  /**
   * @brief Stops the data listener.
   *
   * This virtual function can be overridden by any subclass of DataListener to
   * provide additional functionality for stopping the data listener. By
   * default, it simply sets the `running_` flag to false.
   */
  virtual void stop() { running_ = false; }

  /**
   * @brief Writes data to the data listener.
   *
   * This pure virtual function must be implemented by any subclass of
   * DataListener. It is responsible for writing data to the data listener,
   * typically to send rtcm correction for the serial mode.
   *
   * @param data A pointer to the data buffer to be written.
   * @param size The size of the data buffer in bytes.
   */
  virtual void write(uint8_t* data, size_t size) = 0;

  /**
   * @brief Sets a callback function to handle incoming data.
   *
   * This pure virtual function must be implemented by any subclass of
   * DataListener. It is responsible for setting a callback function to handle
   * incoming data from the sensor or device.
   *
   * @param func The callback function to be set.
   */
  virtual void setCallback(
      const std::function<void(uint8_t*, size_t)>& func) = 0;

 protected:
  bool running_{
      false};  ///< A flag indicating whether the data listener is running.
};