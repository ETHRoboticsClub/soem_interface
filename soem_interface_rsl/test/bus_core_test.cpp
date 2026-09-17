// The bus core above BusTransport: lifecycle order, sync/cyclic SDO
// ownership, the working-counter gate and the AL observation cache, proven
// against a scripted transport so no adapter can quietly reimplement them.
#include <gtest/gtest.h>

#include <soem_interface_rsl/EthercatBusBase.hpp>
#include <soem_interface_rsl/EthercatSlaveBase.hpp>

#include <algorithm>
#include <deque>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace soem_interface_rsl;

namespace {

constexpr uint16_t kInit = 0x01, kPreOp = 0x02, kSafeOp = 0x04, kOp = 0x08, kNone = 0x00;

struct Log {
  std::vector<std::string> events;
  void add(const std::string& event) { events.push_back(event); }
};

// A mailbox wire whose replies are scripted per submitted request.
struct ScriptedMailbox final : MailboxTransport {
  explicit ScriptedMailbox(Log& log) : log(log) {}
  Log& log;
  bool flight{false}, frozen{false};
  uint16_t reg{0}, size{0};
  uint8_t count{0};
  std::array<uint8_t, 32> registerReply{};
  MailboxEndpoint endpoint(uint16_t) const override { return {0x1000, 128, 0x1100, 128}; }
  uint8_t nextCounter(uint16_t) override {
    count = count % 7 + 1;
    return count;
  }
  bool start(uint16_t slave, uint16_t r, bool, const uint8_t*, uint16_t n) override {
    flight = true;
    reg = r;
    size = n;
    log.add("mbx:start:" + std::to_string(slave) + ":" + std::to_string(r));
    return true;
  }
  int poll(uint8_t* data) override {
    if (frozen)
      return -1;
    flight = false;
    std::copy_n(registerReply.begin(), size, data);
    return 1;
  }
  void cancel() override { flight = false; }
};

struct ScriptedTransport final : BusTransport {
  explicit ScriptedTransport(Log& log, int slaves) : log_(log), count_(slaves), mailbox_(log) {
    outputs_.resize(slaves + 1, std::vector<uint8_t>(15, 0xAA));
    inputs_.resize(slaves + 1, std::vector<uint8_t>(20, 0xBB));
    al_.resize(slaves + 1);
  }
  Log& log_;
  std::string name_{"scripted"};
  int count_;
  int detected_{-1}, enumerated_{-1};
  bool open_{false}, available_{true};
  int wkc_{0};
  std::deque<int> wkcScript_;
  std::vector<std::vector<uint8_t>> outputs_, inputs_;
  std::vector<AlStatus> al_;
  std::deque<TransportError> errors_;
  int sdoResult_{1};
  int sdoReadSize_{-1}; // bytes a read answers; -1 answers exactly what was asked
  ScriptedMailbox mailbox_;

  const std::string& name() const override { return name_; }
  bool available() const override { return available_; }
  bool open() override {
    log_.add("open");
    open_ = true;
    return true;
  }
  void close() override {
    log_.add("close");
    open_ = false;
  }
  int detectSlaves() override {
    log_.add("detect");
    return detected_ < 0 ? count_ : detected_;
  }
  int enumerate() override {
    log_.add("enumerate");
    return enumerated_ < 0 ? count_ : enumerated_;
  }
  int slaveCount() const override { return count_; }
  DiscoveredSlave slave(uint16_t address) const override {
    return {address, "slave" + std::to_string(address),      0xfb, 0x63500000,
            1,       static_cast<uint16_t>(0x1000 + address)};
  }
  int mapProcessImage() override {
    log_.add("map");
    return 35 * count_;
  }
  ProcessDataView outputs(uint16_t slave) override { return {outputs_[slave].data(), 15}; }
  ProcessDataView inputs(uint16_t slave) override { return {inputs_[slave].data(), 20}; }
  int expectedWorkingCounter() const override { return 3 * count_; }
  void sendProcessData() override { log_.add("send"); }
  int receiveProcessData(std::chrono::microseconds) override {
    log_.add("receive");
    if (!wkcScript_.empty()) {
      wkc_ = wkcScript_.front();
      wkcScript_.pop_front();
    }
    return wkc_;
  }
  void requestAlState(uint16_t slave, uint16_t state) override {
    log_.add("al:" + std::to_string(slave) + ":" + std::to_string(state));
    for (size_t i = 1; i < al_.size(); ++i)
      if (slave == 0 || slave == i)
        al_[i].state = state;
  }
  uint16_t readAlStatus() override {
    log_.add("readstate");
    return lowest();
  }
  uint16_t awaitAlState(uint16_t slave, uint16_t, std::chrono::microseconds) override {
    return slave == 0 ? lowest() : al_[slave].state;
  }
  AlStatus alStatus(uint16_t slave) const override {
    return slave == 0 ? AlStatus{lowest(), 0} : al_[slave];
  }
  void recordAlStatus(uint16_t slave, AlStatus status) override { al_[slave] = status; }
  int sdoWrite(uint16_t slave, uint16_t index, uint8_t, bool, int, const void*,
               std::chrono::microseconds) override {
    log_.add("sdoWrite:" + std::to_string(slave) + ":" + std::to_string(index));
    return sdoResult_;
  }
  int sdoRead(uint16_t slave, uint16_t index, uint8_t, bool, int& size, void* data,
              std::chrono::microseconds) override {
    log_.add("sdoRead:" + std::to_string(slave) + ":" + std::to_string(index));
    if (sdoReadSize_ >= 0)
      size = std::min(size, sdoReadSize_);
    std::fill_n(static_cast<uint8_t*>(data), size, 0x5A);
    return sdoResult_;
  }
  int readRegister(uint16_t, uint16_t, uint16_t, void*, std::chrono::microseconds) override {
    return 1;
  }
  bool popError(TransportError& error) override {
    if (errors_.empty())
      return false;
    error = errors_.front();
    errors_.pop_front();
    return true;
  }
  void syncDistributedClock0(uint16_t, bool, uint32_t, int32_t) override {}
  MailboxTransport& mailbox() override { return mailbox_; }

  uint16_t lowest() const {
    int state = kOp;
    for (size_t i = 1; i < al_.size(); ++i)
      state = std::min(state & 0x0f, al_[i].state & 0x0f) | ((state | al_[i].state) & 0x10);
    return static_cast<uint16_t>(state);
  }
};

struct ScriptedSlave final : EthercatSlaveBase {
  ScriptedSlave(Log& log, uint32_t address, std::string name) : log_(log), name_(std::move(name)) {
    address_ = address;
  }
  Log& log_;
  std::string name_;
  bool startupResult{true};
  PdoInfo info{0, 0, 15, 20, 0};
  std::vector<uint8_t> lastRead;
  std::string getName() const override { return name_; }
  bool startup() override {
    log_.add("slave.startup:" + name_);
    return startupResult;
  }
  void updateRead() override {
    log_.add("slave.read:" + name_);
    lastRead.resize(20);
    bus_->readTxPdo(static_cast<uint16_t>(address_),
                    *reinterpret_cast<std::array<uint8_t, 20>*>(lastRead.data()));
  }
  void updateWrite() override {
    log_.add("slave.write:" + name_);
    std::array<uint8_t, 15> pdo{};
    pdo.fill(static_cast<uint8_t>(address_));
    bus_->writeRxPdo(static_cast<uint16_t>(address_), pdo);
  }
  void shutdown() override { log_.add("slave.shutdown:" + name_); }
  PdoInfo getCurrentPdoInfo() const override { return info; }
};

struct Fixture {
  Log log;
  ScriptedTransport* transport;
  std::unique_ptr<EthercatBusBase> bus;
  std::vector<std::shared_ptr<ScriptedSlave>> slaves;
  explicit Fixture(int count = 2) {
    auto owned = std::make_unique<ScriptedTransport>(log, count);
    transport = owned.get();
    bus = std::make_unique<EthercatBusBase>(std::move(owned));
    for (int i = 1; i <= count; ++i) {
      auto slave = std::make_shared<ScriptedSlave>(log, i, "s" + std::to_string(i));
      slave->setEthercatBusBasePointer(bus.get());
      bus->addSlave(slave);
      slaves.push_back(slave);
    }
  }
  bool startup() {
    std::atomic<bool> abort{false};
    return bus->startup(abort, true, 0);
  }
  void activate() {
    transport->wkc_ = transport->expectedWorkingCounter();
    bus->setState(ETHERCAT_SM_STATE::OPERATIONAL);
    ASSERT_TRUE(bus->waitForState(ETHERCAT_SM_STATE::OPERATIONAL, 0, 0));
  }
};

} // namespace

TEST(BusCore, StartupOrdersDiscoveryPreOpSlaveStartupThenMapping) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  const std::vector<std::string> expected{
      "open", "detect", "enumerate", "al:0:2", "slave.startup:s1", "slave.startup:s2", "map"};
  std::vector<std::string> seen;
  for (const auto& e : f.log.events)
    if (std::find(expected.begin(), expected.end(), e) != expected.end())
      seen.push_back(e);
  EXPECT_EQ(seen, expected);
  // The image is zeroed after mapping, before any slave callback wrote to it.
  for (uint16_t s = 1; s <= 2; ++s) {
    EXPECT_EQ(f.transport->outputs_[s], std::vector<uint8_t>(15, 0));
    EXPECT_EQ(f.transport->inputs_[s], std::vector<uint8_t>(20, 0));
  }
}

TEST(BusCore, EnumerationShortOfExpectedSlavesFailsStartupAndClosesThePort) {
  Fixture f;
  f.transport->enumerated_ = 1;
  EXPECT_FALSE(f.startup());
  EXPECT_EQ(f.log.events.back(), "close");
  EXPECT_EQ(std::count(f.log.events.begin(), f.log.events.end(), "map"), 0);
  EXPECT_EQ(f.bus->getNumberOfSlaves(), 0);
}

TEST(BusCore, SlaveStartupFailureStopsBeforeMapping) {
  Fixture f;
  f.slaves[0]->startupResult = false;
  EXPECT_FALSE(f.startup());
  EXPECT_EQ(std::count(f.log.events.begin(), f.log.events.end(), "slave.startup:s2"), 0);
  EXPECT_EQ(std::count(f.log.events.begin(), f.log.events.end(), "map"), 0);
}

TEST(BusCore, PdoSizeMismatchFailsStartup) {
  Fixture f;
  f.slaves[0]->info.rxPdoSize_ = 16;
  EXPECT_FALSE(f.startup());
  EXPECT_EQ(std::count(f.log.events.begin(), f.log.events.end(), "map"), 1)
      << "the mismatch is detected from the mapped image";
}

TEST(BusCore, CyclicExchangeOrdersWriteCallbacksSendReceiveThenReadCallbacks) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  f.activate();
  f.log.events.clear();
  f.bus->updateWrite();
  f.bus->updateRead();
  EXPECT_EQ(f.log.events, (std::vector<std::string>{"slave.write:s1", "slave.write:s2", "send",
                                                    "receive", "slave.read:s1", "slave.read:s2"}));
  EXPECT_EQ(f.transport->outputs_[2], std::vector<uint8_t>(15, 2));
  EXPECT_EQ(f.bus->getWorkingCounter(), 6);
}

TEST(BusCore, SynchronousSdoRefusedWhileCyclicAndAllowedInSafeOp) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  uint32_t value = 0;
  EXPECT_TRUE(f.bus->sendSdoRead(1, 0x1018, 4, false, value));
  EXPECT_EQ(value, 0x5A5A5A5Au);
  EXPECT_TRUE(f.bus->sendSdoWrite<uint32_t>(1, 0x34C6, 1, false, 7));
  f.activate();
  f.log.events.clear();
  EXPECT_FALSE(f.bus->sendSdoRead(1, 0x1018, 4, false, value));
  EXPECT_FALSE(f.bus->sendSdoWrite<uint32_t>(1, 0x34C6, 1, false, 7));
  std::string text;
  EXPECT_FALSE(f.bus->sendSdoReadVisibleString(1, 0x1008, 0, text));
  EXPECT_TRUE(f.log.events.empty()) << "no transport transfer may be attempted while cyclic";
  // The mailbox is the OP path.
  auto request = f.bus->requestSdo(1, 0x1018, 4, 4);
  EXPECT_EQ(request->status.load(), MailboxStatus::Pending);
  f.bus->setState(ETHERCAT_SM_STATE::SAFE_OP);
  EXPECT_EQ(request->status.load(), MailboxStatus::Cancelled)
      << "leaving OP cancels queued mailbox work";
  EXPECT_TRUE(f.bus->sendSdoRead(1, 0x1018, 4, false, value));
  EXPECT_EQ(f.bus->requestSdo(1, 0x1018, 4, 4)->status.load(), MailboxStatus::Unavailable);
}

TEST(BusCore, SdoSizeMismatchIsAFailureNotATruncation) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  f.transport->sdoReadSize_ = 2;
  uint32_t value = 0;
  EXPECT_FALSE(f.bus->sendSdoRead(1, 0x1018, 4, false, value));
  std::string text;
  EXPECT_TRUE(f.bus->sendSdoReadVisibleString(1, 0x1008, 0, text))
      << "the size-returning read accepts shorter answers";
  EXPECT_EQ(text.size(), 2u);
}

TEST(BusCore, LowWorkingCounterSuppressesEveryReadCallbackAndTripsHealthAtOneHundred) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  f.activate();
  const int full = f.transport->expectedWorkingCounter();
  auto cycle = [&](int wkc) {
    f.transport->wkc_ = wkc;
    f.log.events.clear();
    f.bus->updateWrite();
    f.bus->updateRead();
    return std::count_if(f.log.events.begin(), f.log.events.end(),
                         [](const std::string& e) { return e.rfind("slave.read", 0) == 0; });
  };
  EXPECT_EQ(cycle(full), 2);
  for (int i = 1; i <= 99; ++i) {
    EXPECT_EQ(cycle(full - 1), 0) << "cycle " << i;
    EXPECT_TRUE(f.bus->busIsOk()) << "cycle " << i;
  }
  EXPECT_EQ(cycle(full - 1), 0);
  EXPECT_FALSE(f.bus->busIsOk()) << "the hundredth consecutive low cycle";
  EXPECT_EQ(cycle(full - 1), 0);
  EXPECT_FALSE(f.bus->busIsOk());
  // One complete exchange restores health and the callbacks together.
  EXPECT_EQ(cycle(full), 2);
  EXPECT_TRUE(f.bus->busIsOk());
}

TEST(BusCore, FirstLowCycleSchedulesAnAlSweepWhoseAnswerIsObservedPerSlave) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  f.activate();
  EXPECT_FALSE(f.bus->getSlaveALStatus(1).observed) << "nothing observed before a sweep";
  f.transport->mailbox_.registerReply = {0x14, 0x00, 0,
                                         0,    0x1b, 0x00}; // SAFE-OP + error, code 0x001b
  f.transport->wkc_ = f.transport->expectedWorkingCounter() - 1;
  for (int i = 0; i < 6; ++i) {
    f.bus->updateWrite();
    f.bus->updateRead();
  }
  const auto al1 = f.bus->getSlaveALStatus(1);
  EXPECT_TRUE(al1.observed);
  EXPECT_EQ(al1.state, 0x14);
  EXPECT_EQ(al1.code, 0x1b);
  EXPECT_EQ(f.bus->getEthercatState(1), ETHERCAT_SM_STATE::ERROR)
      << "cyclic state reads use the sweep's observation";
  // A frozen reply is a timeout: observed NONE, not unobserved.
  f.transport->mailbox_.frozen = true;
  const auto al2 = f.bus->getSlaveALStatus(2);
  EXPECT_TRUE(al2.observed);
  EXPECT_EQ(al2.state, 0x14);
}

TEST(BusCore, LeavingOpForgetsObservationsAndSilentSlaveIsObservedOffTheBus) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  f.activate();
  f.transport->mailbox_.frozen = true;
  f.transport->wkc_ = f.transport->expectedWorkingCounter() - 1;
  f.bus->updateWrite();
  f.bus->updateRead(); // schedules the sweep and starts the first datagram
  f.bus->updateWrite();
  f.bus->updateRead();
  EXPECT_FALSE(f.bus->getSlaveALStatus(1).observed);
  // The mailbox lifetime expires on a later tick; the frozen wire never answers.
  const auto deadline =
      std::chrono::steady_clock::now() + AsyncMailbox::kTimeout + std::chrono::milliseconds(50);
  while (std::chrono::steady_clock::now() < deadline && !f.bus->getSlaveALStatus(1).observed) {
    f.bus->updateWrite();
    f.bus->updateRead();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto al = f.bus->getSlaveALStatus(1);
  ASSERT_TRUE(al.observed);
  EXPECT_EQ(al.state, kNone);
  f.bus->setState(ETHERCAT_SM_STATE::SAFE_OP);
  EXPECT_FALSE(f.bus->getSlaveALStatus(1).observed)
      << "a state request invalidates every observation";
}

TEST(BusCore, ShutdownRequestsInitBeforeSlaveShutdownAndClosesLast) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  f.log.events.clear();
  f.bus->shutdown();
  auto pos = [&](const std::string& e) {
    return std::distance(f.log.events.begin(),
                         std::find(f.log.events.begin(), f.log.events.end(), e));
  };
  EXPECT_LT(pos("al:0:1"), pos("slave.shutdown:s1"));
  EXPECT_LT(pos("slave.shutdown:s2"), pos("close"));
  EXPECT_EQ(f.log.events.back(), "close");
}

TEST(BusCore, NonCyclicMonitoringReadsTheSegmentAndCyclicMonitoringOnlySchedules) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  f.transport->al_[2].state = kSafeOp;
  f.log.events.clear();
  EXPECT_FALSE(f.bus->doBusMonitoring(false));
  EXPECT_EQ(std::count(f.log.events.begin(), f.log.events.end(), "readstate"), 1);
  f.activate();
  f.log.events.clear();
  EXPECT_TRUE(f.bus->doBusMonitoring(false));
  EXPECT_EQ(std::count(f.log.events.begin(), f.log.events.end(), "readstate"), 0)
      << "no blocking segment read on the cyclic path";
}

TEST(BusCore, SdoFailureDrainsTheTransportErrorQueue) {
  Fixture f;
  ASSERT_TRUE(f.startup());
  f.transport->sdoResult_ = 0;
  f.transport->errors_.push_back({TransportError::Type::SdoAbort, 1, 0x34C6, 1, 0x06090030, 0.0});
  EXPECT_FALSE(f.bus->sendSdoWrite<uint32_t>(1, 0x34C6, 1, false, 5));
  TransportError leftover;
  EXPECT_FALSE(f.transport->popError(leftover));
}
