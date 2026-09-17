/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Markus Staeuble, Jonas Junger, Johannes Pankert, Philipp Leemann,
** Tom Lankhorst, Samuel Bachmann, Gabriel Hottiger, Lennert Nachtigall,
** Mario Mauerer, Remo Diethelm
**
** This file is part of the soem_interface_rsl.
**
** The soem_interface_rsl is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The seom_interface is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the soem_interface_rsl.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <soem_interface_rsl/EthercatBusBase.hpp>
#include <soem_interface_rsl/EthercatSlaveBase.hpp>

// Only the ETG constants, error-string tables and SOEM timeouts; every segment
// access goes through the BusTransport.
#include <soem_rsl/ethercat.h>

#include <set>
#include <sstream>

namespace soem_interface_rsl {

struct EthercatBusBaseTemplateAdapter::EthercatSlaveBaseImpl {
  EthercatSlaveBaseImpl() = delete;
  explicit EthercatSlaveBaseImpl(std::unique_ptr<BusTransport> transport)
      : transport_(std::move(transport)), mailbox_(transport_->mailbox()), wkc_(0) {
    for (auto& status : observedAL_) status.store(kUnobservedAL);
  }

  const std::string& getName() const { return transport_->name(); }

  bool busIsAvailable() const { return transport_->available(); }

  int getNumberOfSlaves() const {
    if (!initlialized_) {
      MELO_WARN_STREAM("[SOEM_Interface] requesting number of slaves on not inited bus.")
      return 0;
    }
    std::lock_guard<std::mutex> contextLock(contextMutex_);
    return transport_->slaveCount();
  }

  bool addSlave(const EthercatSlaveBasePtr& slave) {
    for (const auto& existingSlave : slaves_) {
      if (slave->getAddress() == existingSlave->getAddress()) {
        MELO_ERROR_STREAM("[" << getName() << "] "
                              << "Slave '" << existingSlave->getName() << "' and slave '" << slave->getName()
                              << "' have identical addresses (" << slave->getAddress() << ").");
        return false;
      }
    }

    slaves_.push_back(slave);
    // ensure that they are sorted in adress order. this makes access simpler (access via slaveaddress -1)
    std::sort(slaves_.begin(), slaves_.end(),
              [](const EthercatSlaveBasePtr& a, const EthercatSlaveBasePtr& b) -> bool { return a->getAddress() < b->getAddress(); });
    return true;
  }

  bool startup(std::atomic<bool>& abortFlag, const bool sizeCheck, int maxDiscoverRetries) {
    const std::string& name_ = getName();
    if (!busIsAvailable()) {
      const std::string why = transport_->availabilityDiagnosis();
      MELO_ERROR_STREAM("[" << name_ << "] "
                            << "Bus is not available." << (why.empty() ? "" : " " + why));
      if (why.empty()) printSoemInterfaces();
      return false;
    }

    {
      std::lock_guard<std::mutex> contextLock(contextMutex_);
      if (!transport_->open()) {
        MELO_ERROR_STREAM("[" << name_ << "] "
                              << "No socket connection. Execute as root.");
        return false;
      }
      for (int retry = 0; retry <= maxDiscoverRetries; retry++) {
        if (abortFlag) {
          MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                   << "Shutdown during waiting for slaves.");
          transport_->close();
          return false;  // avoid that executation continues.
        }
        if (transport_->detectSlaves() == static_cast<int>(slaves_.size())) {
          // on some of the older (rsl) anydrives there seems to be a short race between bus is responsive and slave is fully ready...
          // so give them this 1 sec to be fully ready to be started...
          soem_interface_rsl::threadSleep(1.0);
          break;
        }
        if (retry == maxDiscoverRetries) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "No slaves have been found.");
          transport_->close();
          return false;
        }
        // Sleep and retry.
        soem_interface_rsl::threadSleep(ecatConfigRetrySleep_);
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] No slaves have been found, retrying " << retry + 1 << "/"
                                                 << maxDiscoverRetries << " ...");
      }

      // A slot that answered the scan but failed its identity/mailbox setup
      // leaves the segment unusable; the bus cannot be configured around it.
      if (transport_->enumerate() != static_cast<int>(slaves_.size())) {
        MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                  << "Slave enumeration did not configure every expected slave.");
        transport_->close();
        return false;
      }

      int nSlaves = transport_->slaveCount();
      // Print the slaves which have been detected.
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] The following " << nSlaves << " slaves have been found and configured:");
      for (int slave = 1; slave <= nSlaves; slave++) {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Address: " << slave << " - Name: '"
                                                 << transport_->slave(static_cast<uint16_t>(slave)).name << "'");
      }

      // Check if the given slave addresses are valid.
      bool slaveAddressesAreOk = true;
      for (const auto& slave : slaves_) {
        auto address = static_cast<int>(slave->getAddress());
        if (address == 0) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "Slave '" << slave->getName() << "': Invalid address " << address << ".");
          slaveAddressesAreOk = false;
        }
        if (address > nSlaves) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "Slave '" << slave->getName() << "': Invalid address " << address << ", "
                                                    << "only " << nSlaves << " slave(s) found.");
          slaveAddressesAreOk = false;
        }
      }
      if (!slaveAddressesAreOk) {
        transport_->close();
        return false;
      }

      // some slave might require SAFE_OP during setup...
      busDiagnosisLog_.errorCounters_.resize(slaves_.size());
      nSlaves_ = slaves_.size();
      initlialized_ = true;
      setStateLocked(EC_STATE_PRE_OP);
      waitForStateLocked(EC_STATE_PRE_OP, 0);
    }

    // Initialize the communication interfaces of all slaves.
    for (auto& slave : slaves_) {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Starting slave: " << slave->getName())
      if (!slave->startup()) {
        MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] Slave '" << slave->getName() << "' was not initialized successfully.");
        return false;
      } else {
        MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Successfully started slave: " << slave->getName())
      }
    }

    std::lock_guard<std::mutex> contextLock(contextMutex_);
    // Set up the communication IO mapping.
    // Note: mapProcessImage requests the slaves to go to SAFE-OP.
    [[maybe_unused]] int ioMapSize = transport_->mapProcessImage();
    MELO_DEBUG_STREAM("[soem_interface_rsl::" << name_ << "] Configured ioMap with size: " << ioMapSize)

    // Check if the size of the IO mapping fits our slaves.
    bool ioMapIsOk = true;
    // do this check only if 'sizeCheck' is true
    if (sizeCheck) {
      for (const auto& slave : slaves_) {
        const EthercatSlaveBase::PdoInfo pdoInfo = slave->getCurrentPdoInfo();
        const auto address = static_cast<uint16_t>(slave->getAddress());
        const auto rxSize = transport_->outputs(address).size;
        const auto txSize = transport_->inputs(address).size;
        if (pdoInfo.rxPdoSize_ != rxSize) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "RxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of "
                                                    << pdoInfo.rxPdoSize_ << " bytes but the slave found at its address "
                                                    << slave->getAddress() << " requests " << rxSize << " bytes).");
          ioMapIsOk = false;
        }
        if (pdoInfo.txPdoSize_ != txSize) {
          MELO_ERROR_STREAM("[soem_interface_rsl::" << name_ << "] "
                                                    << "TxPDO size mismatch: The slave '" << slave->getName() << "' expects a size of "
                                                    << pdoInfo.txPdoSize_ << " bytes but the slave found at its address "
                                                    << slave->getAddress() << " requests " << txSize << " bytes).");
          ioMapIsOk = false;
        }
      }
    }
    if (!ioMapIsOk) {
      return false;
    }

    // Initialize the memory with zeroes.
    for (int slave = 1; slave <= transport_->slaveCount(); slave++) {
      const auto address = static_cast<uint16_t>(slave);
      auto in = transport_->inputs(address);
      auto out = transport_->outputs(address);
      if (in.data != nullptr) memset(in.data, 0, in.size);
      if (out.data != nullptr) memset(out.data, 0, out.size);
    }

    workingCounterTooLowCounter_ = 0;

    return true;
  }

  void updateRead() {
    if (!sentProcessData_) {
      MELO_DEBUG_STREAM("No process data to read.");
      return;
    }

    //! Receive the EtherCAT data.
    updateReadStamp_ = std::chrono::high_resolution_clock::now();
    {
      std::lock_guard<std::mutex> guard(contextMutex_);
      wkc_ = transport_->receiveProcessData(std::chrono::microseconds(EC_TIMEOUTRET));
    }
    sentProcessData_ = false;
    serviceMailbox();

    int expectedWorkingCounter = transport_->expectedWorkingCounter();
    //! Check the working counter.
    if (wkc_ < expectedWorkingCounter) {
      ++workingCounterTooLowCounter_;
      if (workingCounterTooLowCounter_ == 1) {
        diagnosticSweep_ = true;
      }
      // The running count is part of the message, so one line per period already tells how long the bus has been degraded.
      MELO_WARN_THROTTLE_STREAM(wkcTooLowLogPeriodSec_, "[soem_interface_rsl::" << getName() << "] Working counter is too low: " << wkc_.load()
                                                        << " < " << expectedWorkingCounter << " (" << workingCounterTooLowCounter_
                                                        << " cycles in a row)");
      if (workingCounterTooLowCounter_ > maxWorkingCounterTooLow_) {
        MELO_ERROR_THROTTLE_STREAM(wkcTooLowLogPeriodSec_, "[soem_interface_rsl::" << getName() << "] Bus is not ok. Too many working counter too low in a row: "
                                                           << workingCounterTooLowCounter_)
      }
      return;
    }
    // Reset working counter too low counter.
    workingCounterTooLowCounter_ = 0;

    //! Each slave attached to this bus reads its data to the buffer.
    for (auto& slave : slaves_) {
      slave->updateRead();
    }
  }

  void updateWrite() {
    if (sentProcessData_) {
      MELO_DEBUG_STREAM("[soem_interface_rsl] Sending new process data without reading the previous one.");
    }

    //! Each slave attached to this bus write its data to the buffer.
    for (auto& slave : slaves_) {
      slave->updateWrite();
    }

    //! Send the EtherCAT data.
    updateWriteStamp_ = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> guard(contextMutex_);
    transport_->sendProcessData();
    sentProcessData_ = true;
  }

  const std::chrono::time_point<std::chrono::high_resolution_clock>& getUpdateReadStamp() const { return updateReadStamp_; }

  const std::chrono::time_point<std::chrono::high_resolution_clock>& getUpateWriteStamp() const { return updateWriteStamp_; }

  void shutdown() {
    if (initlialized_) {
      {
        std::lock_guard<std::mutex> guard(contextMutex_);
        // Set the slaves to state Init.
        if (transport_->slaveCount() > 0) {
          setStateLocked(EC_STATE_INIT);
          waitForStateLocked(EC_STATE_INIT);
        }
      }  // release the contextMutex_ in case slave wants to do low_level commands at shutdown.
      for (auto& slave : slaves_) {
        slave->shutdown();
      }
    }

    // Close the port.
    std::lock_guard<std::mutex> guard(contextMutex_);
    MELO_INFO_STREAM("[soem_interface_rsl::" << getName() << "] Closing socket ...");
    transport_->close();
    // Sleep to make sure the socket is closed, because ecx_close is non-blocking.
    soem_interface_rsl::threadSleep(0.5);
    initlialized_ = false;
  }

  void setState(const uint16_t state, const uint16_t slave = 0) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    setStateLocked(state, slave);
  }

  bool waitForState(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 20) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    return waitForStateLocked(state, slave, maxRetries);
  }

  ETHERCAT_SM_STATE getEthercatState(const uint16_t slave = 0) {
    uint16_t stateRaw = getState(slave);
    // check if Error
    if ((stateRaw & 0xf0) == EC_STATE_ERROR) {
      return ETHERCAT_SM_STATE::ERROR;
    }
    switch (stateRaw & 0x0f) {
      case EC_STATE_INIT:
        return ETHERCAT_SM_STATE::INIT;
      case EC_STATE_BOOT:
        return ETHERCAT_SM_STATE::BOOT;
      case EC_STATE_PRE_OP:
        return ETHERCAT_SM_STATE::PRE_OP;
      case EC_STATE_SAFE_OP:
        return ETHERCAT_SM_STATE::SAFE_OP;
      case EC_STATE_OPERATIONAL:
        return ETHERCAT_SM_STATE::OPERATIONAL;
      default:
        return ETHERCAT_SM_STATE::NONE;  // should not happen.
    }
    return ETHERCAT_SM_STATE::NONE;
  }

  bool busIsOk() const { return workingCounterTooLowCounter_ < maxWorkingCounterTooLow_; }

  MailboxRequest::Ptr requestSdo(uint16_t slave, uint16_t index, uint8_t sub, uint8_t size,
                                 bool write, uint32_t value) {
    return mailbox_.submit(write ? MailboxRequest::Kind::Write : MailboxRequest::Kind::Read,
                           slave, index, sub, size, value);
  }

  void serviceMailbox() {
    std::lock_guard<std::mutex> guard(contextMutex_);
    if (!cyclicActive_) return;
    mailbox_.tick();
    if (diagnosticRequest_) {
      const auto status = diagnosticRequest_->status.load(std::memory_order_acquire);
      if (status == MailboxStatus::Pending) return;
      const auto address = static_cast<uint16_t>(slaves_[diagnosticSlave_]->getAddress());
      if (diagnosticRequest_->index == ECT_REG_ALSTAT) {
        if (status == MailboxStatus::Success) {
          const auto& data = diagnosticRequest_->registers;
          const AlStatus observed{static_cast<uint16_t>(data[0] | (uint16_t(data[1]) << 8)),
                                  static_cast<uint16_t>(data[4] | (uint16_t(data[5]) << 8))};
          transport_->recordAlStatus(address, observed);
          observedAL_[address].store(uint32_t(observed.state) | (uint32_t(observed.code) << 16));
        } else {
          // No answer is an observation: the slave is off the bus (Timeout,
          // TransportError). A cancelled or unavailable read says nothing.
          transport_->recordAlStatus(address, {EC_STATE_NONE, transport_->alStatus(address).code});
          const bool silent = status == MailboxStatus::Timeout || status == MailboxStatus::TransportError;
          observedAL_[address].store(silent ? uint32_t(EC_STATE_NONE) : kUnobservedAL);
        }
        if (diagnosticCounters_) {
          diagnosticRequest_ = mailbox_.submit(MailboxRequest::Kind::Register, address,
              static_cast<uint16_t>(REG::ERROR_COUNTERS::FRAME_ERROR_PORT0_ADDR), 0,
              REG::ERROR_COUNTERS_LIST.memorySize());
          return;
        }
      } else if (status == MailboxStatus::Success) {
        size_t i = 0;
        auto* raw = reinterpret_cast<std::byte*>(diagnosticRequest_->registers.data());
        for (const auto& reg : REG::ERROR_COUNTERS_LIST) {
          const auto value = REG::ERROR_COUNTERS_LIST.getValueFromRawAs<uint8_t>(
              reg.addrEnum, raw, REG::ERROR_COUNTERS_LIST.memorySize());
          auto& counter = busDiagnosisLog_.errorCounters_[diagnosticSlave_][i++];
          counter.fullValue += static_cast<uint8_t>(value - counter.previousValue);
          counter.previousValue = value;
        }
      }
      diagnosticRequest_.reset();
      if (++diagnosticSlave_ == slaves_.size()) {
        diagnosticSlave_ = 0;
        diagnosticSweep_ = false;
        busDiagnosisLog_.ecatApplicationLayerStatus = EC_STATE_OPERATIONAL;
        for (const auto& device : slaves_) {
          const auto previous = busDiagnosisLog_.ecatApplicationLayerStatus;
          const auto state = transport_->alStatus(static_cast<uint16_t>(device->getAddress())).state;
          busDiagnosisLog_.ecatApplicationLayerStatus = std::min(int(previous & 0x0f), int(state & 0x0f)) |
                                                       ((previous | state) & EC_STATE_ERROR);
        }
        busDiagnosisLog_.fullyUpdated = true;
      }
    }
    if (diagnosticSweep_ && !slaves_.empty()) {
      diagnosticRequest_ = mailbox_.submit(MailboxRequest::Kind::Register,
          static_cast<uint16_t>(slaves_[diagnosticSlave_]->getAddress()), ECT_REG_ALSTAT, 0, 6);
    }
  }

  EthercatBusBase::SlaveALStatus getSlaveALStatus(uint16_t slave) const {
    const uint32_t value = slave < EC_MAXSLAVE ? observedAL_[slave].load() : kUnobservedAL;
    return {value != kUnobservedAL, uint16_t(value), uint16_t(value >> 16)};
  }
  int getWorkingCounter() const { return wkc_.load(); }

  bool doBusMonitoring(bool logErrorCounterForDiagnosis) {
    if (cyclicActive_) {
      diagnosticSweep_ = true;
      diagnosticCounters_ = logErrorCounterForDiagnosis;
      return busIsOk();
    }
    if (!initlialized_) {
      return false;
    }
    const std::string& name_ = getName();
    bool allFine = true;
    busDiagnosisLog_.fullyUpdated = false;
    BusDiagState nextBusDiagState{BusDiagState::StateReading};
    MELO_DEBUG_STREAM("[DriveManager::DoBusMonitoring::" << name_ << "] Running Bus Monitoring/Diagnosis")

    if (busDiagState_ == BusDiagState::StateReading) {
      if (logErrorCounterForDiagnosis) {
        nextBusDiagState = BusDiagState::CounterReading;
      }
      // read all the states from all slaves.
      MELO_DEBUG_STREAM("[DriveManager::DoBusMonitoring::" << name_ << "] Running Bus Monitoring/Diagnosis State/AlstatusCode")

      int lowestSlaveState = getState(0);  // one datagram iff all slaves in the same state, otherwise one datagram per slave.

      std::lock_guard<std::mutex> guard(contextMutex_);
      if ((lowestSlaveState & 0x0f) < EC_STATE_OPERATIONAL) {  // if ECAT Error bus state is e.g. 0x14 = 0x10 (error) + 0x04 (safeOP)
        MELO_WARN_STREAM("[EthercatBus::BusMonitoring::" << name_ << "] No all slaves in EC_STATE_OPERATIONAL")
        for (const auto& slave : slaves_) {
          const auto address = static_cast<uint16_t>(slave->getAddress());
          const auto al = transport_->alStatus(address);
          MELO_WARN_STREAM("[EthercatBus::BusMonitoring::"
                           << name_ << "] Slave: " << slave->getName()
                           << " in state: " << EthercatBusBase::getStateString(al.state))

          if ((al.state & 0x0f) < EC_STATE_OPERATIONAL) {
            MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave->getName() << " alStatusCode: 0x" << std::setfill('0')
                                                     << std::setw(8) << std::hex << al.code << " "
                                                     << ec_ALstatuscode2string(al.code));

            if (al.state == EC_STATE_NONE && !lostLogged_.count(address)) {
              lostLogged_.insert(address);
              MELO_ERROR_STREAM("[EthercatBus::BusMonitoring] Slave: "
                                << slave->getName() << " no valid state read - slave probably lost - check your cables ;-) !")
              // todo  Trying to recover the lost slave. !NOT IMPLEMENTED! example: in soem_rsl simple_test.c
              // slave (sdks) would require an optional virtual method, something like: slave->recover() in case they loose connection.
              // (could fix partially shacky cables in software..)
            }
          }
        }
        allFine = false;
      }
    }

    // only reached if errorCounterDiagnosis enabled.
    if (busDiagState_ == BusDiagState::CounterReading) {
      MELO_DEBUG_STREAM(
          "[DriveManager::DoBusMonitoring::" << name_ << "] Running Bus Monitoring/Diagnosis counter slave no: " << busDiagOfCurrentSlave_)
      nextBusDiagState = BusDiagState::StateReading;
      auto& selectedSlave = slaves_[busDiagOfCurrentSlave_];
      // Reading from registers: e.g. ECT_REG_RXERR, addr_0x0300 - 0x0307 (check Ethercat Specification ETG1000.4 for details)
      // one frame per slave.( BRD call would OR together the error counters, therefore for exact number one frame per slave required.)
      // we therefore continously update our saved values by mixing in a single datagram per call to this function and sweeping over the
      // slaves.
      std::byte rawData[REG::ERROR_COUNTERS_LIST.memorySize()];
      memset(rawData, 0xbe, REG::ERROR_COUNTERS_LIST.memorySize());
      std::lock_guard<std::mutex> guard(contextMutex_);
      if (transport_->readRegister(static_cast<uint16_t>(selectedSlave->getAddress()),
                                   static_cast<uint16_t>(REG::ERROR_COUNTERS::FRAME_ERROR_PORT0_ADDR),
                                   REG::ERROR_COUNTERS_LIST.memorySize(), rawData, std::chrono::microseconds(EC_TIMEOUTRET3))) {
        size_t currentRegNo{0};
        for (const auto& reg : REG::ERROR_COUNTERS_LIST) {
          uint8_t value = REG::ERROR_COUNTERS_LIST.getValueFromRawAs<uint8_t>(reg.addrEnum, rawData, REG::ERROR_COUNTERS_LIST.memorySize());
          if (busDiagnosisLog_.errorCounters_[busDiagOfCurrentSlave_][currentRegNo].previousValue > value) {
            // we had an overflow, (or multiple..) lets assume it was one, we just have to diagnose fast enough..
            busDiagnosisLog_.errorCounters_[busDiagOfCurrentSlave_][currentRegNo].fullValue +=
                (255 - busDiagnosisLog_.errorCounters_[busDiagOfCurrentSlave_][currentRegNo].previousValue) + value;
          } else {
            busDiagnosisLog_.errorCounters_[busDiagOfCurrentSlave_][currentRegNo].fullValue += value;
          }
          currentRegNo++;
        }
      } else {
        MELO_WARN_STREAM(
            "[soem_interface_rsl::BusMonitoring::" << name_ << "] Could not read Error counters for slave: " << selectedSlave->getName())
      }
      busDiagOfCurrentSlave_++;
      if (busDiagOfCurrentSlave_ >= nSlaves_) {
        busDiagOfCurrentSlave_ = 0;
        busDiagnosisLog_.fullyUpdated = true;
      }
    }
    busDiagState_ = nextBusDiagState;
    return allFine;
  }

  bool getBusDiagnosisLog(BusDiagnosisLog& busDiagnosisLogOut) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    if (busDiagnosisLog_.fullyUpdated) {
      busDiagnosisLogOut = busDiagnosisLog_;
      busDiagnosisLog_.fullyUpdated = false;
      return true;
    }
    return false;
  }

  void syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift) {
    // todo verify!
    MELO_INFO_STREAM("Bus '" << getName() << "', slave " << slave << ":  " << (activate ? "Activating" : "Deactivating")
                             << " distributed clock synchronization...");

    transport_->syncDistributedClock0(slave, activate, static_cast<uint32_t>(cycleTime * 1e9),
                                      static_cast<int32_t>(1e9 * cycleShift));

    MELO_INFO_STREAM("Bus '" << getName() << "', slave " << slave << ":  " << (activate ? "Activated" : "Deactivated")
                             << " distributed clock synchronization.");
  }

  EthercatBusBase::PdoSizePair getHardwarePdoSizes(const uint16_t slave) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    return std::make_pair(transport_->outputs(slave).size, transport_->inputs(slave).size);
  }

  EthercatBusBase::PdoSizeMap getHardwarePdoSizes() {
    EthercatBusBase::PdoSizeMap pdoMap;

    for (const auto& slave : slaves_) {
      pdoMap.insert(std::make_pair(slave->getName(), getHardwarePdoSizes(slave->getAddress())));
    }
    return pdoMap;
  }

  // A mailbox transfer to a slave the started bus does not hold (the bus not
  // started yet, or an address past the scan) fails like any other transfer;
  // it must not take the master down.
  bool slaveOnBus(const uint16_t slave, const char* what) const {
    const int count = initlialized_ ? transport_->slaveCount() : 0;
    if (slave >= 1 && static_cast<int>(slave) <= count) return true;
    MELO_ERROR_STREAM("[soem_interface_rsl::" << getName() << "] " << what << ": slave " << slave << " is not on the bus ("
                                              << count << " slave(s) configured).");
    return false;
  }

  void logSdoFailure(const uint16_t slave, const uint16_t index, const uint8_t subindex, const int wkc, const char* what) {
    MELO_ERROR_STREAM("Slave " << slave << ": Working counter too low (" << wkc << ") for " << what << " SDO (ID: 0x" << std::setfill('0')
                               << std::setw(4) << std::hex << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                               << static_cast<uint16_t>(subindex) << ").");
    checkForSdoErrors(slave, index);
    const auto al = transport_->alStatus(slave);
    if (slave == 0) {
      MELO_INFO_STREAM("[soem_interface_rsl::" << getName() << "] Worst AL status code of all slaves, alStatusCode: 0x" << std::setfill('0')
                                               << std::setw(8) << std::hex << al.code << " " << ec_ALstatuscode2string(al.code));
    } else {
      MELO_INFO_STREAM("[soem_interface_rsl::" << getName() << "] Slave: " << slaves_[slave - 1]->getName() << " alStatusCode: 0x"
                                               << std::setfill('0') << std::setw(8) << std::hex << al.code << " "
                                               << ec_ALstatuscode2string(al.code));
    }
  }

  bool sdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size, void* buf) {
    int wkc = 0;
    {
      if (!slaveOnBus(slave, __func__)) return false;
      std::lock_guard<std::mutex> guard(contextMutex_);
      if (cyclicActive_) return false; // Runtime callers must use requestSdo().
      wkc = transport_->sdoWrite(slave, index, subindex, completeAccess, size, buf, std::chrono::microseconds(EC_TIMEOUTRXM));
    }
    if (wkc <= 0) {
      logSdoFailure(slave, index, subindex, wkc, "writing");
      return false;
    }
    return true;
  }

  bool sdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size, void* buf) {
    int requestedSize = size;
    int wkc = 0;
    {
      if (!slaveOnBus(slave, __func__)) return false;
      std::lock_guard<std::mutex> guard(contextMutex_);
      if (cyclicActive_) return false; // Runtime callers must use requestSdo().
      wkc = transport_->sdoRead(slave, index, subindex, completeAccess, size, buf, std::chrono::microseconds(EC_TIMEOUTRXM));
    }
    if (wkc <= 0) {
      logSdoFailure(slave, index, subindex, wkc, "reading");
      return false;
    }
    if (size != requestedSize) {
      MELO_ERROR_STREAM("Slave " << slave << ": Size mismatch (expected " << requestedSize << " bytes, read " << size
                                 << " bytes) for reading SDO (ID: 0x" << std::setfill('0') << std::setw(4) << std::hex << index
                                 << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(subindex) << ").");
      return false;
    }
    return true;
  }

  int sdoReadSize(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess, int size, void* buf) {
    int wkc = 0;
    {
      if (!slaveOnBus(slave, __func__)) return 0;
      std::lock_guard<std::mutex> guard(contextMutex_);
      if (cyclicActive_) return 0; // Runtime callers must use requestSdo().
      wkc = transport_->sdoRead(slave, index, subindex, completeAccess, size, buf, std::chrono::microseconds(EC_TIMEOUTRXM));
    }
    if (wkc <= 0) {
      logSdoFailure(slave, index, subindex, wkc, "reading");
      return 0;
    }
    return size;
  }

  void readTxPdo(const uint16_t slave, int size, void* buf) const {
    assert(static_cast<int>(slave) <= transport_->slaveCount());
    std::lock_guard<std::mutex> guard(contextMutex_);
    const auto in = transport_->inputs(slave);
    assert(size == (int)in.size);
    memcpy(buf, in.data, size);
  }

  void writeRxPdo(const uint16_t slave, int size, const void* buf) {
    assert(static_cast<int>(slave) <= transport_->slaveCount());
    std::lock_guard<std::mutex> guard(contextMutex_);
    const auto out = transport_->outputs(slave);
    assert((int)out.size == size);
    memcpy(out.data, buf, size);
  }

 private:
  uint16_t getState(const uint16_t slave) {
    std::lock_guard<std::mutex> guard(contextMutex_);
    int lowest_state = EC_STATE_OPERATIONAL;
    if (cyclicActive_) {
      for (int i = 1; i <= transport_->slaveCount(); ++i) {
        const auto state = transport_->alStatus(static_cast<uint16_t>(i)).state;
        lowest_state = std::min(lowest_state & 0x0f, int(state & 0x0f)) |
                       ((lowest_state | state) & EC_STATE_ERROR);
      }
    } else {
      lowest_state = transport_->readAlStatus();
    }

    // readAlStatus refreshes every slave, in the worst case with one datagram per slave, and or's all the ALStatusCodes.
    // therefore we update here the bus AL StatusCode which is the OR of all slave's ALStatusCode!
    busDiagnosisLog_.ecatApplicationLayerStatus = transport_->alStatus(0).code;
    if (slave == 0) {
      return static_cast<uint16_t>(lowest_state);
    }
    return transport_->alStatus(slave).state;
  }

  void setStateLocked(const uint16_t state, const uint16_t slave = 0) {
    if (!initlialized_) {
      MELO_WARN_STREAM("[soem_interface_rsl::" << getName() << "] Bus " << getName() << " was not successfully initialized, skipping operation");
      return;
    }
    if (slave == 0) {
      cyclicActive_ = state == EC_STATE_OPERATIONAL;
      if (cyclicActive_) mailbox_.enable(transport_->slaveCount());
      else mailbox_.disable();
      diagnosticRequest_.reset();
      for (auto& status : observedAL_) status.store(kUnobservedAL);
      diagnosticSlave_ = 0;
      diagnosticSweep_ = false;
    }
    if (state == EC_STATE_OPERATIONAL) {
      transport_->sendProcessData();
      wkc_ = transport_->receiveProcessData(std::chrono::microseconds(EC_TIMEOUTRET));
    }
    transport_->requestAlState(slave, state);
    if (slave == 0) {
      MELO_DEBUG_STREAM("[soem_interface_rsl::" << getName() << "] All slaves on State " << EthercatBusBase::getStateString(state)
                                                << " has been set.");
    } else {
      MELO_DEBUG_STREAM("[soem_interface_rsl::" << getName() << "] Slave " << slaves_[slave - 1]->getName() << " State "
                                                << EthercatBusBase::getStateString(state) << " has been set.");
    }
  }

  bool waitForStateLocked(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 20) {
    const std::string& name_ = getName();
    if (!initlialized_) {
      MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Bus " << name_ << " was not successfully initialized, skipping operation");
      return false;
    }
    uint16_t returnedState = 0;
    uint16_t currentState = transport_->awaitAlState(slave, state, std::chrono::microseconds(20000));
    if (currentState == state) {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave << ": State " << EthercatBusBase::getStateString(state)
                                               << " has been reached directly")
      return true;
    }
    for (unsigned int retry = 0; retry <= maxRetries; retry++) {
      int timeout = EC_TIMEOUTSTATE;
      switch (static_cast<ec_state>(state)) {
        case EC_STATE_NONE:
        case EC_STATE_INIT:
        case EC_STATE_PRE_OP:
        case EC_STATE_BOOT:
          break;
        case EC_STATE_SAFE_OP:
          timeout = EC_TIMEOUTSTATE * 4;
          break;
        case EC_STATE_OPERATIONAL:
          transport_->sendProcessData();
          wkc_ = transport_->receiveProcessData(std::chrono::microseconds(EC_TIMEOUTRET));
          timeout = 20000;
          break;
        case EC_STATE_ACK:
          break;
      }
      returnedState = transport_->awaitAlState(slave, state, std::chrono::microseconds(timeout));
      if (returnedState == state) {
        MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slave << ": State " << EthercatBusBase::getStateString(state)
                                                 << " has been reached after " << retry << " retries");
        return true;
      }
    }
    MELO_WARN_STREAM("[soem_interface_rsl::" << name_ << "] Slave " << slave << ": Targetstate " << EthercatBusBase::getStateString(state)
                                             << " has not been reached. Current State: " << returnedState);

    const auto al = transport_->alStatus(slave);
    if (slave == 0) {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Worst AL status code of all slaves, alStatusCode: 0x" << std::setfill('0')
                                               << std::setw(8) << std::hex << al.code << " " << ec_ALstatuscode2string(al.code));
    } else {
      MELO_INFO_STREAM("[soem_interface_rsl::" << name_ << "] Slave: " << slaves_[slave - 1]->getName() << " alStatusCode: 0x"
                                               << std::setfill('0') << std::setw(8) << std::hex << al.code << " "
                                               << ec_ALstatuscode2string(al.code));
    }
    return false;
  }

  std::string getErrorString(const TransportError& error) {
    std::stringstream stream;
    stream << "Time: " << error.timeSec;

    switch (error.type) {
      case TransportError::Type::SdoAbort:
        stream << " SDO slave: " << error.slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.index << "."
               << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.subindex) << " error: 0x" << std::setfill('0')
               << std::setw(8) << std::hex << error.code << " " << ec_sdoerror2string(error.code);
        break;
      case TransportError::Type::Emergency:
        stream << " EMERGENCY slave: " << error.slave << " error: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.code;
        break;
      case TransportError::Type::Packet:
        stream << " PACKET slave: " << error.slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.index << "."
               << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.subindex) << " error: 0x" << std::setfill('0')
               << std::setw(8) << std::hex << error.code;
        break;
      case TransportError::Type::SdoInfo:
        stream << " SDO slave: " << error.slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.index << "."
               << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(error.subindex) << " error: 0x" << std::setfill('0')
               << std::setw(8) << std::hex << error.code << " " << ec_sdoerror2string(error.code);
        break;
      case TransportError::Type::SoE:
        stream << " SoE slave: " << error.slave << " index: 0x" << std::setfill('0') << std::setw(4) << std::hex << error.index
               << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex << error.code << " "
               << ec_soeerror2string(static_cast<uint16_t>(error.code));
        break;
      case TransportError::Type::Mailbox:
        stream << " MBX slave: " << error.slave << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex << error.code << " "
               << ec_mbxerror2string(static_cast<uint16_t>(error.code));
        break;
      default:
        stream << " MBX slave: " << error.slave << " error: 0x" << std::setfill('0') << std::setw(8) << std::hex << error.code;
        break;
    }
    return stream.str();
  }

  /*!
   * Check if an error for the SDO index of the slave exists.
   * @param slave   Address of the slave.
   * @param index   Index of the SDO.
   * @return True if an error for the index exists.
   */
  bool checkForSdoErrors(const uint16_t slave, const uint16_t index) {
    TransportError error;
    while (transport_->popError(error)) {
      std::string errorStr = getErrorString(error);
      MELO_ERROR_STREAM(errorStr);
      if (error.slave == slave && error.index == index) {
        soem_interface_rsl::common::MessageLog::insertMessage(message_logger::log::levels::Level::Error, errorStr);
        return true;
      }
    }
    return false;
  }

  std::unique_ptr<BusTransport> transport_;
  AsyncMailbox mailbox_;

  //! Whether the bus has been initialized successfully
  bool initlialized_{false};

  //! List of slaves.
  std::vector<EthercatSlaveBasePtr> slaves_;

  //! Bool indicating whether PDO data has been sent and not read yet.
  bool sentProcessData_{false};

  //! Working counter of the most recent PDO.
  std::atomic<int> wkc_;

  //! Time of the last successful PDO reading.
  std::chrono::time_point<std::chrono::high_resolution_clock> updateReadStamp_;
  //! Time of the last successful PDO writing.
  std::chrono::time_point<std::chrono::high_resolution_clock> updateWriteStamp_;

  //! Time to sleep between the retries.
  const double ecatConfigRetrySleep_{1.0};

  //! Count working counter too low in a row.
  unsigned int workingCounterTooLowCounter_{0};
  //! Maximal number of working counter to low.
  const unsigned int maxWorkingCounterTooLow_{100};
  //! Minimum spacing of the working-counter warnings; the cyclic loop would otherwise emit one line per bus cycle.
  const double wkcTooLowLogPeriodSec_{1.0};

  //! Bus Diagnosis Counters, and dl status log
  BusDiagnosisLog busDiagnosisLog_{};
  enum class BusDiagState { StateReading = 0, CounterReading = 1 };
  BusDiagState busDiagState_{BusDiagState::StateReading};
  size_t nSlaves_{0};                // number of slaves on the bus - set after startup.
  size_t busDiagOfCurrentSlave_{0};  // running variable to send only one frame per slave.
  std::set<uint16_t> lostLogged_;    // slaves already reported lost by the non-cyclic monitoring.

  mutable std::mutex contextMutex_;

  bool cyclicActive_{false}; // protected by contextMutex_
  bool diagnosticSweep_{false}, diagnosticCounters_{false};
  static constexpr uint32_t kUnobservedAL = 0xffffffff;
  std::array<std::atomic<uint32_t>, EC_MAXSLAVE> observedAL_{};
  size_t diagnosticSlave_{0};
  MailboxRequest::Ptr diagnosticRequest_;

};

EthercatBusBaseTemplateAdapter::EthercatBusBaseTemplateAdapter(std::unique_ptr<BusTransport> transport)
    : pImpl_(std::make_unique<EthercatSlaveBaseImpl>(std::move(transport))) {}

// has to be defined in cpp! otherwise EthercatBusBaseTemplateAdapter is incomplete type.
EthercatBusBaseTemplateAdapter::~EthercatBusBaseTemplateAdapter() = default;

bool EthercatBusBaseTemplateAdapter::sdoWriteForward(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                     const bool completeAccess, int size, void* buf) {
  return pImpl_->sdoWrite(slave, index, subindex, completeAccess, size, buf);
}

bool EthercatBusBaseTemplateAdapter::sdoReadForward(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                    const bool completeAccess, int size, void* buf) {
  return pImpl_->sdoRead(slave, index, subindex, completeAccess, size, buf);
}

int EthercatBusBaseTemplateAdapter::sdoReadSizeForward(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                       const bool completeAccess, int size, void* buf) {
  return pImpl_->sdoReadSize(slave, index, subindex, completeAccess, size, buf);
}

void EthercatBusBaseTemplateAdapter::readTxPdoForward(const uint16_t slave, int size, void* buf) const {
  pImpl_->readTxPdo(slave, size, buf);
}

void EthercatBusBaseTemplateAdapter::writeRxPdoForward(const uint16_t slave, int size, const void* buf) {
  pImpl_->writeRxPdo(slave, size, buf);
}

//***************************

EthercatBusBase::EthercatBusBase(const std::string& name) : EthercatBusBaseTemplateAdapter(makeSoemTransport(name)) {}

EthercatBusBase::EthercatBusBase(std::unique_ptr<BusTransport> transport) : EthercatBusBaseTemplateAdapter(std::move(transport)) {}

EthercatBusBase::~EthercatBusBase() = default;

bool EthercatBusBase::busIsAvailable(const std::string& name) {
  return soemInterfaceExists(name);
}

void EthercatBusBase::printAvailableBusses() {
  printSoemInterfaces();
}

const std::string& EthercatBusBase::getName() const {
  return pImpl_->getName();
}

bool EthercatBusBase::busIsAvailable() const {
  return pImpl_->busIsAvailable();
}

int EthercatBusBase::getNumberOfSlaves() const {
  return pImpl_->getNumberOfSlaves();
}

bool EthercatBusBase::addSlave(const EthercatSlaveBasePtr& slave) {
  return pImpl_->addSlave(slave);
}

bool EthercatBusBase::startup(const bool sizeCheck, int maxDiscoverRetries) {
  std::atomic<bool> tmpAtomicForStart{false};
  return pImpl_->startup(tmpAtomicForStart, sizeCheck, maxDiscoverRetries);
}

bool EthercatBusBase::startup(std::atomic<bool>& abortFlag, const bool sizeCheck, int maxDiscoverRetries) {
  return pImpl_->startup(abortFlag, sizeCheck, maxDiscoverRetries);
}

void EthercatBusBase::updateRead() {
  pImpl_->updateRead();
}

void EthercatBusBase::updateWrite() {
  pImpl_->updateWrite();
}

void EthercatBusBase::shutdown() {
  pImpl_->shutdown();
  pImpl_.reset(nullptr);
}

void EthercatBusBase::setState(const uint16_t state, const uint16_t slave) {
  pImpl_->setState(state, slave);
}

void EthercatBusBase::setState(soem_interface_rsl::ETHERCAT_SM_STATE state, const uint16_t slave) {
  pImpl_->setState(static_cast<uint16_t>(state), slave);
}

bool EthercatBusBase::waitForState(const uint16_t state, const uint16_t slave, const unsigned int maxRetries) {
  return pImpl_->waitForState(state, slave, maxRetries);
}
bool EthercatBusBase::waitForState(soem_interface_rsl::ETHERCAT_SM_STATE state, const uint16_t slave, const unsigned int maxRetries) {
  return pImpl_->waitForState(static_cast<uint16_t>(state), slave, maxRetries);
}

bool EthercatBusBase::busIsOk() const {
  return pImpl_->busIsOk();
}

std::string EthercatBusBase::getStateString(uint16_t state) {
  std::string stateStr{};
  switch (state & 0x0f) {
    case EC_STATE_INIT:
      stateStr += "EC_STATE_INIT";
      break;
    case EC_STATE_PRE_OP:
      stateStr += "EC_STATE_PRE_OP";
      break;
    case EC_STATE_SAFE_OP:
      stateStr += "EC_STATE_SAFE_OP";
      break;
    case EC_STATE_OPERATIONAL:
      stateStr += "EC_STATE_OPERATIONAL";
      break;
    case EC_STATE_BOOT:
      stateStr += "EC_STATE_BOOT";
      break;
    default:
      break;
  }
  if ((state & 0xf0) == EC_STATE_ERROR) {
    stateStr += " + EC_STATE_ERROR";
  }
  if ((state & 0xf0) == (EC_STATE_ERROR + EC_STATE_ACK)) {
    stateStr += " + EC_STATE_ERROR + EC_STATE_ACK";
  }
  return stateStr;
}

void EthercatBusBase::syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime, const double cycleShift) {
  pImpl_->syncDistributedClock0(slave, activate, cycleTime, cycleShift);
}

EthercatBusBase::PdoSizeMap EthercatBusBase::getHardwarePdoSizes() {
  return pImpl_->getHardwarePdoSizes();
}

EthercatBusBase::PdoSizePair EthercatBusBase::getHardwarePdoSizes(const uint16_t slave) {
  return pImpl_->getHardwarePdoSizes(slave);
}

soem_interface_rsl::ETHERCAT_SM_STATE EthercatBusBase::getEthercatState(const uint16_t slave) {
  return pImpl_->getEthercatState(slave);
}

bool EthercatBusBase::doBusMonitoring(bool logErrorCounterForDiagnosis) {
  return pImpl_->doBusMonitoring(logErrorCounterForDiagnosis);
}

bool EthercatBusBase::getBusDiagnosisLog(BusDiagnosisLog& busDiagnosisLogOut) {
  return pImpl_->getBusDiagnosisLog(busDiagnosisLogOut);
}

bool EthercatBusBase::sendSdoReadVisibleString(const uint16_t slave, const uint16_t index, const uint8_t subindex, std::string& value) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  char buffer[128];
  int length = sizeof(buffer) - 1;

  int readLength = sdoReadSizeForward(slave, index, subindex, false, length, &buffer);
  if (readLength == 0) {
    return false;
  }

  value.clear();
  for (int i = 0; i < readLength; ++i) {
    if (buffer[i] != 0x0) {
      value += buffer[i];
    } else {
      break;
    }
  }
  return true;
}

const std::chrono::time_point<std::chrono::high_resolution_clock>& EthercatBusBase::getUpdateReadStamp() const {
  return pImpl_->getUpdateReadStamp();
}

const std::chrono::time_point<std::chrono::high_resolution_clock>& EthercatBusBase::getUpdateWriteStamp() const {
  return pImpl_->getUpateWriteStamp();
}

template <>
bool EthercatBusBase::sendSdoRead<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                               const bool completeAccess, std::string& value) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  // Expected length of the string. String needs to be preallocated
  int size = value.length();
  // Create buffer with the length of the string
  char buffer[size];
  bool success = sdoReadForward(slave, index, subindex, completeAccess, size, &buffer);
  value = std::string(buffer, size);
  return success;
}

template <>
bool EthercatBusBase::sendSdoWrite<std::string>(const uint16_t slave, const uint16_t index, const uint8_t subindex,
                                                const bool completeAccess, const std::string value) {
  assert(static_cast<int>(slave) <= getNumberOfSlaves());
  const int size = value.length();
  std::string valueCopy{value};
  char* dataPtr = valueCopy.data();
  return sdoWriteForward(slave, index, subindex, completeAccess, size, dataPtr);
}

MailboxRequest::Ptr EthercatBusBase::requestSdo(uint16_t slave, uint16_t index, uint8_t subindex,
                                              uint8_t size, bool write, uint32_t value) {
  return pImpl_->requestSdo(slave, index, subindex, size, write, value);
}

EthercatBusBase::SlaveALStatus EthercatBusBase::getSlaveALStatus(uint16_t slave) const {
  return pImpl_->getSlaveALStatus(slave);
}
int EthercatBusBase::getWorkingCounter() const { return pImpl_->getWorkingCounter(); }

}  // namespace soem_interface_rsl
