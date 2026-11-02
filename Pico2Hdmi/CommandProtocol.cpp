#include "CommandProtocol.h"

#include "EmulationState.h"
#include "RasterEvents.h"

namespace zx {

volatile ProtocolCounters protocolCounters = {};
CommandParser commandParser;

void CommandParser::push(const uint8_t *data, size_t size) {
  while (size-- != 0) {
    push(*data++);
  }
}

void CommandParser::pushPacket(const uint8_t *data, size_t size) {
  while (size != 0) {
    // A fixed-size packet contains as many complete no-op headers as fit after
    // its useful commands. Ignore the final one to three filler bytes rather
    // than carrying a partial header into the next packet.
    if (state_ == State::kMessage && pendingByteCount_ == 0 &&
        size < kMessageSize) {
      protocolCounters.receivedBytes =
          protocolCounters.receivedBytes + size;
      break;
    }
    push(*data++);
    --size;
  }

  // Incremental runs must terminate within their packet. Bound any damaged
  // packet to that packet instead of allowing it to corrupt the next one.
  if (state_ != State::kMessage || pendingByteCount_ != 0) {
    protocolCounters.framingErrors = protocolCounters.framingErrors + 1;
    pendingByteCount_ = 0;
    state_ = State::kMessage;
  }
}

void CommandParser::push(uint8_t byte) {
  constexpr uint8_t kIncrementalFlag = 0x01;
  constexpr uint8_t kAudioValidFlag = 0x02;
  constexpr uint8_t kEscape = 0xb6;
  constexpr uint8_t kRunEnd = 0xe0;
  constexpr uint16_t kAudioOnlyAddress = 0xffff;
  constexpr uint8_t kAudioOnlyData = 0x00;
  constexpr uint8_t kMicMask = 0x08;
  constexpr uint8_t kEarMask = 0x10;
  constexpr uint8_t kBeeperMask = kMicMask | kEarMask;

  protocolCounters.receivedBytes = protocolCounters.receivedBytes + 1;
  switch (state_) {
    case State::kMessage:
      message_[pendingByteCount_++] = byte;
      if (pendingByteCount_ == kMessageSize) {
        incrementalAddress_ = static_cast<uint16_t>(message_[0]) |
                              (static_cast<uint16_t>(message_[1]) << 8);
        const uint8_t extension = message_[3];
        const bool audioOnly = incrementalAddress_ == kAudioOnlyAddress &&
                               message_[2] == kAudioOnlyData;
        if (!audioOnly) {
          dispatchWrite(incrementalAddress_, message_[2]);
        }
        if ((extension & kAudioValidFlag) != 0) {
          queueBeeperSample(extension & kBeeperMask);
          protocolCounters.audioSamples = protocolCounters.audioSamples + 1;
        }
        pendingByteCount_ = 0;
        if (!audioOnly && (extension & kIncrementalFlag) != 0) {
          state_ = State::kIncrementalData;
        }
      }
      break;

    case State::kIncrementalData:
      if (byte == kEscape) {
        state_ = State::kIncrementalEscape;
      } else {
        incrementalAddress_ = static_cast<uint16_t>(incrementalAddress_ + 1);
        dispatchWrite(incrementalAddress_, byte);
      }
      break;

    case State::kIncrementalEscape:
      if (byte == kEscape) {
        incrementalAddress_ = static_cast<uint16_t>(incrementalAddress_ + 1);
        dispatchWrite(incrementalAddress_, kEscape);
        state_ = State::kIncrementalData;
      } else if (byte == kRunEnd) {
        state_ = State::kMessage;
      } else {
        // Only B6 B6 (escaped data) and B6 E0 (end of run) are valid. Treat
        // the unexpected byte as the first byte of a new message so a damaged
        // run has a bounded effect on the following stream.
        protocolCounters.framingErrors =
            protocolCounters.framingErrors + 1;
        message_[0] = byte;
        pendingByteCount_ = 1;
        state_ = State::kMessage;
      }
      break;
  }
}

void CommandParser::dispatchWrite(uint16_t address, uint8_t data) {
  constexpr uint16_t kRasterTimestampMask = 0xff00;
  constexpr uint16_t kRasterTimestampBase = 0xfc00;
  constexpr uint16_t kFrameMarkerAddress = 0xfffc;
  constexpr uint16_t kTimexControlAddress = 0xfffb;
  constexpr uint16_t kUlaPlusSelectAddress = 0xbf3b;
  constexpr uint16_t kUlaPlusDataAddress = 0xff3b;
  constexpr uint16_t kResetAddress = 0xffff;
  constexpr uint8_t kResetData = 0xff;

  protocolCounters.completedCommands =
      protocolCounters.completedCommands + 1;

  if ((address & kRasterTimestampMask) == kRasterTimestampBase) {
    const uint16_t timestamp = static_cast<uint16_t>(address & 0x00ff) |
                               (static_cast<uint16_t>(data) << 8);
    setRasterTimestamp(timestamp);
    return;
  }

  if (address == kFrameMarkerAddress) {
    beginRasterSourceFrame(data);
    return;
  }

  if (address == kTimexControlAddress) {
    if (!queueRasterTimexWrite(data)) {
      applyTimexControl(data);
    }
    return;
  }

  if (address == kUlaPlusSelectAddress) {
    ulaPlusSelectedRegister_ = data;
    applyCommand(address, data);
    return;
  }

  if (address == kUlaPlusDataAddress) {
    if (!queueRasterUlaPlusWrite(ulaPlusSelectedRegister_, data)) {
      applyUlaPlusRegister(ulaPlusSelectedRegister_, data);
    }
    return;
  }

  if (address == kResetAddress && data == kResetData) {
    ulaPlusSelectedRegister_ = 0;
    resetRasterEvents();
  }

  if (!applyCommand(address, data)) {
    protocolCounters.unknownCommands =
        protocolCounters.unknownCommands + 1;
  }
}

}  // namespace zx
