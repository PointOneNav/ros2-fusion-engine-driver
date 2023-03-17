#ifndef FUSION_ENGINE_BYTE_FRAME_LISTENER_HPP
#define FUSION_ENGINE_BYTE_FRAME_LISTENER_HPP

#include "fusion_engine_byte_frame_event.hpp"

/**
 * Abstract interface for clients listening to byte frames.
 */
class FusionEngineByteFrameListener {
public: 
  /**
   * Triggers when FusionEngine internal port receives a byte frame.
   * @param evt Event that wraps the message data received.
   */
  virtual void receivedFusionEngineByteFrame(FusionEngineByteFrameEvent & evt) = 0;
};

#endif