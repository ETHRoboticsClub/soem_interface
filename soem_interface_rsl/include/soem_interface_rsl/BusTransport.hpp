#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <soem_interface_rsl/AsyncMailbox.hpp>

namespace soem_interface_rsl {

// Identity a segment scan reports for one slot. `serial` is not part of it:
// the CoE identity object (0x1018:04) is what the SDK protocols read.
struct DiscoveredSlave {
  uint16_t address{0}; // slot on the segment, 1-based; 0 is never a slave
  std::string name;
  uint32_t vendorId{0}, productCode{0}, revision{0};
  uint16_t stationAddress{0}; // the configured address FPRD/FPWR datagrams use
};

struct ProcessDataView {
  uint8_t* data{nullptr};
  uint16_t size{0};
};

// AL status of one slave as last refreshed or observed. `state` carries the
// ETG.1000 error bit (0x10) when set; `code` is the AL status code register.
struct AlStatus {
  uint16_t state{0}, code{0};
};

struct TransportError {
  enum class Type { SdoAbort, Emergency, Packet, SdoInfo, SoE, Mailbox, Other };
  Type type{Type::Other};
  uint16_t slave{0}, index{0};
  uint8_t subindex{0};
  uint32_t code{0};
  double timeSec{0.0};
};

// Everything below the bus core that touches a segment: discovery and IO
// mapping, process-image exchange, synchronous CoE, AL requests/readback and
// the mailbox datagrams the AsyncMailbox pump issues. The core owns the slave
// registry, callback order, the WKC gate, sync/cyclic ownership and the AL
// observation cache; an adapter must not reimplement any of those.
//
// Calls are serialized by the core (it holds its context lock); an adapter
// needs no locking of its own. `sendProcessData`/`receiveProcessData` and the
// mailbox transport run on the cyclic path and must not allocate or block
// beyond the stated timeout.
class BusTransport {
public:
  virtual ~BusTransport() = default;

  virtual const std::string& name() const = 0;
  // The segment can be opened: the NIC exists, or the fake session endpoint answers.
  virtual bool available() const = 0;
  // Why it is not, for the bring-up log; empty when the adapter has nothing to add.
  virtual std::string availabilityDiagnosis() const { return {}; }
  virtual bool open() = 0;
  virtual void close() = 0;

  // Slaves answering on the segment; configures nothing.
  virtual int detectSlaves() = 0;
  // Reads every slave's identity, sets up mailboxes and requests PRE-OP.
  // Returns the slave count; <= 0 means the scan failed.
  virtual int enumerate() = 0;
  virtual int slaveCount() const = 0;
  virtual DiscoveredSlave slave(uint16_t address) const = 0;

  // Builds the process image from the mapping each slave holds and requests
  // SAFE-OP. Returns the image size in bytes.
  virtual int mapProcessImage() = 0;
  virtual ProcessDataView outputs(uint16_t slave) = 0;
  virtual ProcessDataView inputs(uint16_t slave) = 0;
  // The working counter a complete exchange returns: 2 per slave with outputs
  // plus 1 per slave with inputs.
  virtual int expectedWorkingCounter() const = 0;
  virtual void sendProcessData() = 0;
  // Working counter of the returned frame; < 0 when nothing came back.
  virtual int receiveProcessData(std::chrono::microseconds timeout) = 0;

  // AL control write; slave 0 addresses every slave.
  virtual void requestAlState(uint16_t slave, uint16_t state) = 0;
  // Refreshes every slave's AL status and returns the lowest state with the
  // error bit or'ed in.
  virtual uint16_t readAlStatus() = 0;
  // Polls one slave (0: all) until it reports `state` or the timeout passes;
  // returns the state it ended on.
  virtual uint16_t awaitAlState(uint16_t slave, uint16_t state,
                                std::chrono::microseconds timeout) = 0;
  // Last refreshed or recorded status. Slave 0 is the lowest state and the
  // or'ed status codes.
  virtual AlStatus alStatus(uint16_t slave) const = 0;
  // An observation the bus core made through the mailbox pump while cyclic.
  virtual void recordAlStatus(uint16_t slave, AlStatus status) = 0;

  // Synchronous CoE. Both return the transfer's working counter; <= 0 is a
  // failure with the cause on the error queue. `size` is in/out for reads.
  virtual int sdoWrite(uint16_t slave, uint16_t index, uint8_t subindex, bool completeAccess,
                       int size, const void* data, std::chrono::microseconds timeout) = 0;
  virtual int sdoRead(uint16_t slave, uint16_t index, uint8_t subindex, bool completeAccess,
                      int& size, void* data, std::chrono::microseconds timeout) = 0;
  // FPRD of an ESC register block by station address; returns the WKC.
  virtual int readRegister(uint16_t slave, uint16_t reg, uint16_t size, void* data,
                           std::chrono::microseconds timeout) = 0;
  virtual bool popError(TransportError& error) = 0;

  virtual void syncDistributedClock0(uint16_t slave, bool activate, uint32_t cycleTimeNs,
                                     int32_t cycleShiftNs) = 0;

  virtual MailboxTransport& mailbox() = 0;
};

// The SOEM adapter for a NIC. Declared here so the bus core can offer the
// legacy name-only constructor; defined in SoemTransport.cpp.
std::unique_ptr<BusTransport> makeSoemTransport(const std::string& networkInterface);
bool soemInterfaceExists(const std::string& networkInterface);
void printSoemInterfaces();

} // namespace soem_interface_rsl
