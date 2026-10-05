// The SOEM adapter behind BusTransport: every ecx_* call of the bus lives here.
#include <soem_interface_rsl/BusTransport.hpp>

#include <message_logger/message_logger.hpp>
#include <soem_rsl/ethercat.h>

#include <algorithm>
#include <cstring>

namespace soem_interface_rsl {
namespace {

// Accessed only by the bus owner while its context lock is held.
class SoemMailboxTransport final : public MailboxTransport {
public:
  explicit SoemMailboxTransport(ecx_contextt& context) : context_(context) {}
  MailboxEndpoint endpoint(uint16_t slave) const override {
    const auto& s = context_.slavelist[slave];
    return {s.mbx_wo, s.mbx_l, s.mbx_ro, s.mbx_rl};
  }
  uint8_t nextCounter(uint16_t slave) override {
    auto& count = context_.slavelist[slave].mbx_cnt;
    count = ec_nextmbxcnt(count);
    return count;
  }
  bool start(uint16_t slave, uint16_t reg, bool write, const uint8_t* data,
             uint16_t size) override {
    index_ = ecx_getindex(context_.port);
    size_ = size;
    ecx_setupdatagram(context_.port, &context_.port->txbuf[index_],
                      write ? EC_CMD_FPWR : EC_CMD_FPRD, index_,
                      context_.slavelist[slave].configadr, reg, size, const_cast<uint8_t*>(data));
    if (ecx_outframe(context_.port, index_, 0) <= 0) {
      cancel();
      return false;
    }
    return true;
  }
  int poll(uint8_t* data) override {
    const int wkc = ecx_inframe(context_.port, index_, 0);
    if (wkc < 0)
      return -1;
    if (wkc == 1)
      std::memcpy(data, &context_.port->rxbuf[index_][EC_HEADERSIZE], size_);
    cancel();
    return wkc == 1 ? 1 : 0;
  }
  void cancel() override {
    if (index_ >= 0)
      ecx_setbufstat(context_.port, index_, EC_BUF_EMPTY);
    index_ = -1;
  }

private:
  ecx_contextt& context_;
  int index_{-1};
  uint16_t size_{0};
};
static_assert(AsyncMailbox::kMaxMailbox == EC_MAXMBX);

class SoemTransport final : public BusTransport {
public:
  explicit SoemTransport(std::string networkInterface) : name_(std::move(networkInterface)) {
    // Initialize all soem_rsl context data pointers that are not used with null.
    context_.elist->head = 0;
    context_.elist->tail = 0;
    context_.port->stack.sock = nullptr;
    context_.port->stack.txbuf = nullptr;
    context_.port->stack.txbuflength = nullptr;
    context_.port->stack.tempbuf = nullptr;
    context_.port->stack.rxbuf = nullptr;
    context_.port->stack.rxbufstat = nullptr;
    context_.port->stack.rxsa = nullptr;
    context_.port->redport = nullptr;
    context_.FOEhook = nullptr;
  }
  ~SoemTransport() override { close(); }

  const std::string& name() const override { return name_; }
  bool available() const override { return soemInterfaceExists(name_); }
  bool open() override {
    open_ = ecx_init(&context_, name_.c_str()) > 0;
    return open_;
  }
  void close() override {
    if (!open_)
      return;
    ecx_close(&context_);
    open_ = false;
  }

  int detectSlaves() override { return ecx_detect_slaves(&context_); }
  int enumerate() override {
    const int count = ecx_config_init(&context_, FALSE);
    // Disable symmetrical transfers.
    context_.grouplist[0].blockLRW = 1;
    return count;
  }
  int slaveCount() const override { return *context_.slavecount; }
  DiscoveredSlave slave(uint16_t address) const override {
    const auto& s = context_.slavelist[address];
    return {address, std::string(s.name), s.eep_man, s.eep_id, s.eep_rev, s.configadr};
  }

  int mapProcessImage() override { return ecx_config_map_group(&context_, &ioMap_, 0); }
  ProcessDataView outputs(uint16_t slave) override {
    auto& s = context_.slavelist[slave];
    return {s.outputs, static_cast<uint16_t>(s.Obytes)};
  }
  ProcessDataView inputs(uint16_t slave) override {
    auto& s = context_.slavelist[slave];
    return {s.inputs, static_cast<uint16_t>(s.Ibytes)};
  }
  int expectedWorkingCounter() const override {
    return context_.grouplist[0].outputsWKC * 2 + context_.grouplist[0].inputsWKC;
  }
  void sendProcessData() override { ecx_send_processdata(&context_); }
  int receiveProcessData(std::chrono::microseconds timeout) override {
    return ecx_receive_processdata(&context_, static_cast<int>(timeout.count()));
  }

  void requestAlState(uint16_t slave, uint16_t state) override {
    context_.slavelist[slave].state = state;
    ecx_writestate(&context_, slave);
  }
  uint16_t readAlStatus() override { return static_cast<uint16_t>(ecx_readstate(&context_)); }
  uint16_t awaitAlState(uint16_t slave, uint16_t state,
                        std::chrono::microseconds timeout) override {
    return ecx_statecheck(&context_, slave, state, static_cast<int>(timeout.count()));
  }
  AlStatus alStatus(uint16_t slave) const override {
    const auto& s = context_.slavelist[slave];
    return {s.state, s.ALstatuscode};
  }
  void recordAlStatus(uint16_t slave, AlStatus status) override {
    context_.slavelist[slave].state = status.state;
    context_.slavelist[slave].ALstatuscode = status.code;
  }

  int sdoWrite(uint16_t slave, uint16_t index, uint8_t subindex, bool completeAccess, int size,
               const void* data, std::chrono::microseconds timeout) override {
    return ecx_SDOwrite(&context_, slave, index, subindex, static_cast<boolean>(completeAccess),
                        size, const_cast<void*>(data), static_cast<int>(timeout.count()));
  }
  int sdoRead(uint16_t slave, uint16_t index, uint8_t subindex, bool completeAccess, int& size,
              void* data, std::chrono::microseconds timeout) override {
    return ecx_SDOread(&context_, slave, index, subindex, static_cast<boolean>(completeAccess),
                       &size, data, static_cast<int>(timeout.count()));
  }
  int readRegister(uint16_t slave, uint16_t reg, uint16_t size, void* data,
                   std::chrono::microseconds timeout) override {
    return ecx_FPRD(context_.port, context_.slavelist[slave].configadr, reg, size, data,
                    static_cast<int>(timeout.count()));
  }
  bool popError(TransportError& out) override {
    if (!ecx_iserror(&context_))
      return false;
    ec_errort error;
    if (!ecx_poperror(&context_, &error))
      return false;
    out.slave = error.Slave;
    out.index = error.Index;
    out.subindex = error.SubIdx;
    out.timeSec =
        static_cast<double>(error.Time.sec) + static_cast<double>(error.Time.usec) / 1000000.0;
    switch (error.Etype) {
    case EC_ERR_TYPE_SDO_ERROR:
      out.type = TransportError::Type::SdoAbort;
      out.code = error.AbortCode;
      break;
    case EC_ERR_TYPE_EMERGENCY:
      out.type = TransportError::Type::Emergency;
      out.code = error.ErrorCode;
      break;
    case EC_ERR_TYPE_PACKET_ERROR:
      out.type = TransportError::Type::Packet;
      out.code = error.ErrorCode;
      break;
    case EC_ERR_TYPE_SDOINFO_ERROR:
      out.type = TransportError::Type::SdoInfo;
      out.code = error.AbortCode;
      break;
    case EC_ERR_TYPE_SOE_ERROR:
      out.type = TransportError::Type::SoE;
      out.code = error.ErrorCode;
      break;
    case EC_ERR_TYPE_MBX_ERROR:
      out.type = TransportError::Type::Mailbox;
      out.code = error.ErrorCode;
      break;
    default:
      out.type = TransportError::Type::Other;
      out.code = error.AbortCode;
      break;
    }
    return true;
  }

  void syncDistributedClock0(uint16_t slave, bool activate, uint32_t cycleTimeNs,
                             int32_t cycleShiftNs) override {
    ecx_dcsync0(&context_, slave, static_cast<uint8_t>(activate), cycleTimeNs, cycleShiftNs);
  }

  MailboxTransport& mailbox() override { return mailbox_; }

private:
  std::string name_;
  bool open_{false};

  // EtherCAT input/output mapping of the slaves within the datagrams.
  char ioMap_[4096];

  // EtherCAT context data elements:

  // Port reference.
  ecx_portt port_;
  // List of slave data. Index 0 is reserved for the master, higher indices for the slaves.
  ec_slavet slavelist_[EC_MAXSLAVE];
  // Number of slaves found in the network.
  int slavecount_{0};
  // Slave group structure.
  ec_groupt grouplist_[EC_MAXGROUP];
  // Internal, reference to EEPROM cache buffer.
  uint8 esiBuf_[EC_MAXEEPBUF];
  // Internal, reference to EEPROM cache map.
  uint32 esiMap_[EC_MAXEEPBITMAP];
  // Internal, reference to error list.
  ec_eringt eList_;
  // Internal, reference to processdata stack buffer info.
  ec_idxstackT idxStack_;
  // Boolean indicating if an error is available in error stack.
  boolean error_{FALSE};
  // Reference to last DC time from slaves.
  int64 dcTime_{0};
  // Internal, SM buffer.
  ec_SMcommtypet smCommtype_[EC_MAX_MAPT];
  // Internal, PDO assign list.
  ec_PDOassignt pdoAssign_[EC_MAX_MAPT];
  // Internal, PDO description list.
  ec_PDOdesct pdoDesc_[EC_MAX_MAPT];
  // Internal, SM list from EEPROM.
  ec_eepromSMt sm_;
  // Internal, FMMU list from EEPROM.
  ec_eepromFMMUt fmmu_;

  // EtherCAT context data.
  // Note: soem_rsl does not use dynamic memory allocation (new/delete). Therefore
  // all context pointers must be null or point to an existing member.
  ecx_contextt context_ = {&port_,
                           &slavelist_[0],
                           &slavecount_,
                           EC_MAXSLAVE,
                           &grouplist_[0],
                           EC_MAXGROUP,
                           &esiBuf_[0],
                           &esiMap_[0],
                           0,
                           &eList_,
                           &idxStack_,
                           &error_,
                           &dcTime_,
                           &smCommtype_[0],
                           &pdoAssign_[0],
                           &pdoDesc_[0],
                           &sm_,
                           &fmmu_,
                           nullptr,
                           nullptr,
                           0};
  SoemMailboxTransport mailbox_{context_};
};

} // namespace

std::unique_ptr<BusTransport> makeSoemTransport(const std::string& networkInterface) {
  return std::make_unique<SoemTransport>(networkInterface);
}

bool soemInterfaceExists(const std::string& networkInterface) {
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr) {
    if (networkInterface == std::string(adapter->name))
      return true;
    adapter = adapter->next;
  }
  return false;
}

void printSoemInterfaces() {
  MELO_INFO_STREAM("Available adapters:");
  ec_adaptert* adapter = ec_find_adapters();
  while (adapter != nullptr) {
    MELO_INFO_STREAM("- Name: '" << adapter->name << "', description: '" << adapter->desc << "'");
    adapter = adapter->next;
  }
}

} // namespace soem_interface_rsl
