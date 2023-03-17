#ifndef FUSION_ENGINE_BYTE_FRAME_EVENT_HPP
#define FUSION_ENGINE_BYTE_FRAME_EVENT_HPP

#include <cstring>
#include <arpa/inet.h>

/**
 * Data class that wraps a received byte frame into a generic object.
 */
class FusionEngineByteFrameEvent {
public:
  uint8_t * frame;
  size_t bytes_read;

  FusionEngineByteFrameEvent(uint8_t * frame, size_t bytes_read) 
      : frame{frame}, bytes_read{bytes_read} {}
  
};

#endif