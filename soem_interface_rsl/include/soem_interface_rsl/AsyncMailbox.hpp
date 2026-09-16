#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

namespace soem_interface_rsl {

enum class MailboxStatus { Pending, Success, Abort, Timeout, TransportError, Invalid, Unavailable, Cancelled };
inline const char* mailboxStatusName(MailboxStatus status) {
  switch (status) {
    case MailboxStatus::Pending: return "pending";
    case MailboxStatus::Success: return "confirmed";
    case MailboxStatus::Abort: return "SDO abort";
    case MailboxStatus::Timeout: return "mailbox timeout";
    case MailboxStatus::TransportError: return "datagram failed";
    case MailboxStatus::Invalid: return "invalid request or response";
    case MailboxStatus::Unavailable: return "mailbox unavailable (inactive, full, or quarantined)";
    case MailboxStatus::Cancelled: return "bus left OP before completion";
  }
  return "unknown mailbox status";
}
struct MailboxRequest {
  using Ptr = std::shared_ptr<MailboxRequest>;
  enum class Kind { Read, Write, Register };
  const Kind kind;
  const uint16_t slave, index;
  const uint8_t subindex, size;
  const uint32_t writeValue;
  std::atomic<MailboxStatus> status{MailboxStatus::Pending};
  // Published by the release store to status; read only after an acquire load.
  uint32_t value{0}, abortCode{0};
  std::array<uint8_t, 32> registers{};
  MailboxRequest(Kind k, uint16_t s, uint16_t i, uint8_t sub, uint8_t n, uint32_t v = 0)
      : kind(k), slave(s), index(i), subindex(sub), size(n), writeValue(v) {}
};

struct MailboxEndpoint { uint16_t writeOffset, writeSize, readOffset, readSize; };
// One datagram in flight. start/poll must never wait for a network response.
class MailboxTransport {
 public:
  virtual ~MailboxTransport() = default;
  virtual MailboxEndpoint endpoint(uint16_t slave) const = 0;
  virtual uint8_t nextCounter(uint16_t slave) = 0;
  virtual bool start(uint16_t slave, uint16_t reg, bool write, const uint8_t* data, uint16_t size) = 0;
  // -1 pending, 0 unsuccessful WKC, 1 complete. Copies exactly the requested size.
  virtual int poll(uint8_t* data) = 0;
  virtual void cancel() = 0;
};

// Expedited CoE only (1..4-byte objects); segmented/complete-access startup
// configuration remains synchronous outside OP. One network poll per tick.
class AsyncMailbox {
 public:
  using Clock = std::chrono::steady_clock;
  // SOEM EC_TIMEOUTRXM: transaction lifetime, never a per-cycle blocking wait.
  static constexpr auto kTimeout = std::chrono::microseconds(700000);
  static constexpr size_t kQueueCapacity = 64;
  static constexpr size_t kMaxMailbox = 1486; // SOEM EC_MAXMBX
  explicit AsyncMailbox(MailboxTransport& transport) : transport_(transport) {}

  void enable(size_t slaves) {
    std::lock_guard<std::mutex> lock(mutex_);
    cancelLocked();
    // SAFE_OP/OP transitions do not reset a slave mailbox or resolve late replies.
    poisoned_.resize(slaves + 1, false);
    enabled_ = true;
  }
  void disable() {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = false;
    cancelLocked();
  }
  MailboxRequest::Ptr submit(MailboxRequest::Kind kind, uint16_t slave, uint16_t index,
                             uint8_t sub, uint8_t size, uint32_t value = 0) {
    auto request = std::make_shared<MailboxRequest>(kind, slave, index, sub, size, value);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || slave == 0 || slave >= poisoned_.size() || (kind != MailboxRequest::Kind::Register && poisoned_[slave]) || queue_.size() >= kQueueCapacity) {
      request->status = MailboxStatus::Unavailable;
    } else if (size == 0 || size > (kind == MailboxRequest::Kind::Register ? 32 : 4)) {
      request->status = MailboxStatus::Invalid;
    } else {
      queue_.push_back({request, Clock::now()});
    }
    return request;
  }

  void tick(Clock::time_point now = Clock::now()) {
    // Producers cannot make the cyclic owner wait for the queue lock.
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock || !enabled_) return;
    if (!active_) {
      if (queue_.empty()) return;
      active_ = queue_.front().request;
      started_ = queue_.front().queued;
      queue_.pop_front();
      if (active_->kind != MailboxRequest::Kind::Register && poisoned_[active_->slave]) { finish(MailboxStatus::Unavailable); return; }
      endpoint_ = transport_.endpoint(active_->slave);
      phase_ = active_->kind == MailboxRequest::Kind::Register ? Phase::Register : Phase::DrainStatus;
      if (phase_ != Phase::Register && (endpoint_.writeSize < 16 || endpoint_.readSize < 16 ||
          endpoint_.writeSize > buffer_.size() || endpoint_.readSize > buffer_.size())) {
        finish(MailboxStatus::Invalid); return;
      }
    }
    if (now - started_ >= kTimeout) { finish(MailboxStatus::Timeout, sentMailbox_); return; }
    if (inFlight_) {
      const int result = transport_.poll(buffer_.data());
      if (result < 0) return;
      inFlight_ = false;
      if (result == 0) { finish(MailboxStatus::TransportError, sentMailbox_); return; }
      switch (phase_) {
        case Phase::DrainStatus: phase_ = (buffer_[0] & 8) ? Phase::Drain : Phase::SendStatus; break;
        case Phase::Drain: phase_ = Phase::DrainStatus; break;
        case Phase::SendStatus: if (!(buffer_[0] & 8)) phase_ = Phase::Send; break;
        case Phase::Send: phase_ = Phase::ReceiveStatus; break;
        case Phase::ReceiveStatus: if (buffer_[0] & 8) phase_ = Phase::Receive; break;
        case Phase::Receive: consume(); break;
        case Phase::Register:
          std::copy_n(buffer_.begin(), active_->size, active_->registers.begin());
          finish(MailboxStatus::Success); break;
      }
      return; // At most one completion or submission per tick.
    }
    buffer_.fill(0);
    uint16_t reg = 0, size = 1;
    bool write = false;
    switch (phase_) {
      case Phase::DrainStatus: case Phase::ReceiveStatus: reg = 0x080d; break; // SM1 status
      case Phase::Drain: case Phase::Receive: reg = endpoint_.readOffset; size = endpoint_.readSize; break;
      case Phase::SendStatus: reg = 0x0805; break; // SM0 status
      case Phase::Send:
        reg = endpoint_.writeOffset; size = endpoint_.writeSize; write = true;
        counter_ = transport_.nextCounter(active_->slave);
        put16(0, 10); buffer_[5] = 3 | (counter_ << 4); put16(6, 0x2000);
        buffer_[8] = active_->kind == MailboxRequest::Kind::Read ? 0x40 : 0x23 | ((4 - active_->size) << 2);
        put16(9, active_->index); buffer_[11] = active_->subindex;
        for (unsigned i = 0; i < active_->size; ++i) buffer_[12 + i] = active_->writeValue >> (8 * i);
        break;
      case Phase::Register: reg = active_->index; size = active_->size; break;
    }
    inFlight_ = transport_.start(active_->slave, reg, write, buffer_.data(), size);
    if (inFlight_ && phase_ == Phase::Send) sentMailbox_ = true;
    if (!inFlight_) finish(MailboxStatus::TransportError, sentMailbox_);
  }

 private:
  enum class Phase { DrainStatus, Drain, SendStatus, Send, ReceiveStatus, Receive, Register };
  struct Queued { MailboxRequest::Ptr request; Clock::time_point queued; };
  void put16(size_t at, uint16_t value) { buffer_[at] = value; buffer_[at+1] = value >> 8; }
  uint16_t get16(size_t at) const { return buffer_[at] | (uint16_t(buffer_[at+1]) << 8); }
  uint32_t get32(size_t at) const { return get16(at) | (uint32_t(get16(at+2)) << 16); }
  void consume() {
    // Ignore unrelated mailbox traffic, including emergencies, without extending
    // the transaction deadline or mistaking a previous reply for this operation.
    if ((buffer_[5] & 15) != 3 || (buffer_[5] >> 4) != counter_ ||
        (get16(6) >> 12) != 3 || get16(9) != active_->index || buffer_[11] != active_->subindex) {
      phase_ = Phase::ReceiveStatus; return;
    }
    if (get16(0) < 10 || get16(0) + 6 > endpoint_.readSize) {
      finish(MailboxStatus::Invalid, sentMailbox_); return;
    }
    if (buffer_[8] == 0x80) {
      active_->abortCode = get32(12); finish(MailboxStatus::Abort); return;
    }
    if (active_->kind == MailboxRequest::Kind::Write) {
      if (buffer_[8] != 0x60) { finish(MailboxStatus::Invalid, true); return; }
    } else {
      const uint8_t command = buffer_[8];
      if ((command & 0xf3) != 0x43 || 4 - ((command >> 2) & 3) != active_->size) {
        finish(MailboxStatus::Invalid); return;
      }
      active_->value = get32(12);
      if (active_->size < 4) active_->value &= (uint32_t(1) << (8 * active_->size)) - 1;
    }
    finish(MailboxStatus::Success);
  }
  void finish(MailboxStatus status, bool poison = false) {
    transport_.cancel();
    if (poison) poisoned_[active_->slave] = true;
    active_->status.store(status, std::memory_order_release);
    active_.reset(); inFlight_ = false; sentMailbox_ = false;
  }
  void cancelLocked() {
    if (active_) finish(MailboxStatus::Cancelled, sentMailbox_);
    for (auto& queued : queue_) queued.request->status.store(MailboxStatus::Cancelled, std::memory_order_release);
    queue_.clear();
  }
  MailboxTransport& transport_;
  std::mutex mutex_;
  std::deque<Queued> queue_;
  std::vector<bool> poisoned_;
  bool enabled_{false}, inFlight_{false}, sentMailbox_{false};
  MailboxRequest::Ptr active_;
  Clock::time_point started_;
  Phase phase_{Phase::DrainStatus};
  MailboxEndpoint endpoint_{};
  uint8_t counter_{0};
  std::array<uint8_t, kMaxMailbox> buffer_{};
};
} // namespace soem_interface_rsl
