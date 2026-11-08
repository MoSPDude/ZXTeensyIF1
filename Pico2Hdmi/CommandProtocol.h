#pragma once

#include <stddef.h>
#include <stdint.h>

namespace zx {

struct ProtocolCounters {
  uint32_t receivedBytes;
  uint32_t completedCommands;
  uint32_t unknownCommands;
  uint32_t framingErrors;
  uint32_t audioSamples;
};

// Volatile keeps these symbols useful in a debugger without adding output to
// the same CDC stream that carries the binary protocol. Counters wrap at 2^32.
extern volatile ProtocolCounters protocolCounters;

class CommandParser {
 public:
  void push(const uint8_t *data, size_t size);
  void pushPacket(const uint8_t *data, size_t size);
  void push(uint8_t byte);
  uint16_t pendingByteCount() const { return pendingByteCount_; }

 private:
  void dispatchWrite(uint16_t address, uint8_t data);

  enum class State : uint8_t {
    kMessage,
    kIncrementalData,
    kIncrementalEscape,
    kBeeperPcmData,
  };

  static constexpr uint8_t kMessageSize = 4;
  uint8_t message_[kMessageSize] = {};
  uint8_t pendingByteCount_ = 0;
  uint16_t incrementalAddress_ = 0;
  uint8_t ulaPlusSelectedRegister_ = 0;
  uint8_t beeperPcmRemaining_ = 0;
  State state_ = State::kMessage;
};

extern CommandParser commandParser;

}  // namespace zx
