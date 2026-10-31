// SPDX-License-Identifier: MIT
//
// IgH EtherCAT Master backend (ecrt API). Compiled with real IgH support
// only when CMake found ecrt.h / libethercat (ETHERCAT_MOTION_HAVE_IGH).
// Written against the API common to IgH 1.5.2 and 1.6.x; return values that
// changed type between versions (void -> int) are deliberately ignored.
#include "ethercat_motion/ethercat_backend.hpp"

#include <chrono>
#include <ctime>
#include <cstdio>
#include <thread>

#ifdef ETHERCAT_MOTION_HAVE_IGH
#include <ecrt.h>
#endif

namespace ethercat_motion {

#ifdef ETHERCAT_MOTION_HAVE_IGH

namespace {

class IghBackend final : public EthercatBackend {
public:
    ~IghBackend() override {
        deactivate();
        close();
    }

    const char* name() const noexcept override { return "igh"; }

    Status open(unsigned master_index) override {
        if (master_ != nullptr) {
            return {ErrorCode::InvalidState, "master already requested"};
        }
        master_ = ecrt_request_master(master_index);
        if (master_ == nullptr) {
            return {ErrorCode::EthercatError,
                    "ecrt_request_master(" + std::to_string(master_index) +
                        ") failed: is the 'ethercat' service running, the NIC bound "
                        "(/etc/ethercat.conf) and /dev/EtherCAT" +
                        std::to_string(master_index) + " accessible?"};
        }
        domain_ = ecrt_master_create_domain(master_);
        if (domain_ == nullptr) {
            return {ErrorCode::EthercatError, "ecrt_master_create_domain() failed"};
        }
        return Status::ok();
    }

    Result<std::vector<SlaveIdentity>> scanSlaves() override {
        ec_master_info_t info{};
        // The master may still be scanning the bus right after start-up.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        for (;;) {
            if (ecrt_master(master_, &info) != 0) {
                return Status(ErrorCode::EthercatError, "ecrt_master() failed");
            }
            if (!info.scan_busy || std::chrono::steady_clock::now() > deadline) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (!info.link_up) {
            return Status(ErrorCode::EthercatError, "EtherCAT link is down (cable / NIC binding?)");
        }
        std::vector<SlaveIdentity> slaves;
        for (unsigned i = 0; i < info.slave_count; ++i) {
            ec_slave_info_t si{};
            if (ecrt_master_get_slave(master_, static_cast<std::uint16_t>(i), &si) != 0) {
                return Status(ErrorCode::EthercatError, "ecrt_master_get_slave(" + std::to_string(i) + ") failed");
            }
            SlaveIdentity id;
            id.ring_position = si.position;
            id.alias = si.alias;
            id.vendor_id = si.vendor_id;
            id.product_code = si.product_code;
            id.revision = si.revision_number;
            id.serial = si.serial_number;
            id.al_state = si.al_state;
            id.name = si.name;
            slaves.push_back(id);
        }
        return slaves;
    }

    Result<int> configureSlave(const SlaveDeviceConfig& config, std::uint32_t cycle_time_ns,
                               PdoLayout& layout) override {
        ec_slave_config_t* sc =
            ecrt_master_slave_config(master_, config.alias, config.position, config.vendor_id, config.product_code);
        if (sc == nullptr) {
            return Status(ErrorCode::EthercatError, "ecrt_master_slave_config() failed (conflicting configuration?)");
        }

        // Build the IgH PDO description. Storage must stay alive during the call.
        std::vector<std::vector<ec_pdo_entry_info_t>> entries;
        std::vector<std::vector<ec_pdo_info_t>> pdos;
        std::vector<ec_sync_info_t> syncs;
        entries.reserve(64);
        pdos.reserve(config.syncs.size());
        for (const auto& sm : config.syncs) {
            pdos.emplace_back();
            for (const auto& pdo : sm.pdos) {
                entries.emplace_back();
                for (const auto& e : pdo.entries) {
                    entries.back().push_back({e.index, e.subindex, e.bit_length});
                }
                ec_pdo_info_t info{};
                info.index = pdo.index;
                info.n_entries = static_cast<unsigned>(entries.back().size());
                info.entries = entries.back().empty() ? nullptr : entries.back().data();
                pdos.back().push_back(info);
            }
            ec_sync_info_t sync{};
            sync.index = sm.index;
            sync.dir = sm.direction == PdoDirection::Output ? EC_DIR_OUTPUT : EC_DIR_INPUT;
            sync.n_pdos = static_cast<unsigned>(pdos.back().size());
            sync.pdos = pdos.back().empty() ? nullptr : pdos.back().data();
            sync.watchdog_mode = sm.direction == PdoDirection::Output ? (sm.watchdog ? EC_WD_ENABLE : EC_WD_DISABLE)
                                                                      : EC_WD_DEFAULT;
            syncs.push_back(sync);
        }
        if (ecrt_slave_config_pdos(sc, static_cast<unsigned>(syncs.size()), syncs.data()) != 0) {
            return Status(ErrorCode::EthercatError, "ecrt_slave_config_pdos() failed");
        }

        for (const auto& sdo : config.sdo_init) {
            int rc = -1;
            switch (sdo.size) {
                case 1:
                    rc = ecrt_slave_config_sdo8(sc, sdo.index, sdo.subindex, static_cast<std::uint8_t>(sdo.value));
                    break;
                case 2:
                    rc = ecrt_slave_config_sdo16(sc, sdo.index, sdo.subindex, static_cast<std::uint16_t>(sdo.value));
                    break;
                case 4:
                    rc = ecrt_slave_config_sdo32(sc, sdo.index, sdo.subindex, static_cast<std::uint32_t>(sdo.value));
                    break;
                default:
                    break;
            }
            if (rc != 0) {
                char buf[80];
                std::snprintf(buf, sizeof(buf), "startup SDO 0x%04X:%02X (size %u) rejected", sdo.index, sdo.subindex,
                              sdo.size);
                return Status(ErrorCode::EthercatError, buf);
            }
        }

        if (config.dc.enabled) {
            ecrt_slave_config_dc(sc, config.dc.assign_activate, cycle_time_ns, config.dc.sync0_shift_ns, 0, 0);
            dc_used_ = true;
        }

        bool has_outputs = false;
        bool has_inputs = false;
        for (const auto& [entry, direction] : collectEntries(config.syncs)) {
            unsigned int bit_position = 0;
            const int offset =
                ecrt_slave_config_reg_pdo_entry(sc, entry.index, entry.subindex, domain_, &bit_position);
            if (offset < 0) {
                char buf[96];
                std::snprintf(buf, sizeof(buf), "registering PDO entry 0x%04X:%02X failed (%d)", entry.index,
                              entry.subindex, offset);
                return Status(ErrorCode::EthercatError, buf);
            }
            PdoEntryLocation loc;
            loc.index = entry.index;
            loc.subindex = entry.subindex;
            loc.bit_length = entry.bit_length;
            loc.direction = direction;
            loc.byte_offset = static_cast<std::uint32_t>(offset);
            loc.bit_offset = static_cast<std::uint8_t>(bit_position);
            layout.add(loc);
            has_outputs = has_outputs || direction == PdoDirection::Output;
            has_inputs = has_inputs || direction == PdoDirection::Input;
        }
        expected_wc_ += (has_outputs ? 2U : 0U) + (has_inputs ? 1U : 0U);

        configs_.push_back(sc);
        return static_cast<int>(configs_.size() - 1);
    }

    Result<int> createSdoRequest(int slave, std::uint16_t index, std::uint8_t subindex, std::size_t size,
                                 std::uint32_t timeout_ms) override {
        if (slave < 0 || static_cast<std::size_t>(slave) >= configs_.size()) {
            return Status(ErrorCode::InvalidArgument, "invalid slave handle");
        }
        ec_sdo_request_t* request =
            ecrt_slave_config_create_sdo_request(configs_[static_cast<std::size_t>(slave)], index, subindex, size);
        if (request == nullptr) {
            return Status(ErrorCode::EthercatError, "ecrt_slave_config_create_sdo_request() failed");
        }
        ecrt_sdo_request_timeout(request, timeout_ms);
        requests_.push_back(request);
        return static_cast<int>(requests_.size() - 1);
    }

    Result<std::uint64_t> sdoUploadBlocking(std::uint16_t ring_position, std::uint16_t index, std::uint8_t subindex,
                                            std::size_t size) override {
        if (active_) {
            return Status(ErrorCode::InvalidState, "blocking SDO upload is only allowed before activation");
        }
        std::uint8_t buffer[8] = {};
        std::size_t result_size = 0;
        std::uint32_t abort_code = 0;
        const int rc = ecrt_master_sdo_upload(master_, ring_position, index, subindex, buffer,
                                              size > sizeof(buffer) ? sizeof(buffer) : size, &result_size, &abort_code);
        if (rc < 0) {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "SDO upload 0x%04X:%02X failed (rc %d, abort 0x%08X)", index, subindex, rc,
                          abort_code);
            return Status(ErrorCode::EthercatError, buf);
        }
        std::uint64_t value = 0;
        for (std::size_t i = 0; i < result_size && i < sizeof(buffer); ++i) {
            value |= static_cast<std::uint64_t>(buffer[i]) << (8 * i);
        }
        return value;
    }

    Status activate() override {
        if (dc_used_) {
            ecrt_master_application_time(master_, static_cast<std::uint64_t>(monotonic()));
        }
        if (ecrt_master_activate(master_) != 0) {
            return {ErrorCode::EthercatError, "ecrt_master_activate() failed"};
        }
        active_ = true;
        domain_pd_ = ecrt_domain_data(domain_);
        domain_size_ = ecrt_domain_size(domain_);
        if (domain_pd_ == nullptr) {
            return {ErrorCode::EthercatError, "ecrt_domain_data() returned no process image"};
        }
        return Status::ok();
    }

    void deactivate() noexcept override {
        if (active_ && master_ != nullptr) {
            ecrt_master_deactivate(master_);
            active_ = false;
        }
    }

    void close() noexcept override {
        if (master_ != nullptr) {
            ecrt_release_master(master_);
            master_ = nullptr;
            domain_ = nullptr;
            domain_pd_ = nullptr;
            configs_.clear();
            requests_.clear();
        }
    }

    std::uint8_t* domainData() noexcept override { return domain_pd_; }
    std::size_t domainSize() const noexcept override { return domain_size_; }

    void receive() noexcept override {
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
    }

    void send(std::uint64_t app_time_ns, bool sync_dc) noexcept override {
        ecrt_domain_queue(domain_);
        if (sync_dc && dc_used_) {
            ecrt_master_application_time(master_, app_time_ns);
            ecrt_master_sync_reference_clock(master_);
            ecrt_master_sync_slave_clocks(master_);
        }
        ecrt_master_send(master_);
    }

    DomainState domainState() noexcept override {
        ec_domain_state_t ds{};
        ecrt_domain_state(domain_, &ds);
        DomainState out;
        out.working_counter = ds.working_counter;
        switch (ds.wc_state) {
            case EC_WC_ZERO: out.wc_state = WorkingCounterState::Zero; break;
            case EC_WC_INCOMPLETE: out.wc_state = WorkingCounterState::Incomplete; break;
            case EC_WC_COMPLETE: out.wc_state = WorkingCounterState::Complete; break;
        }
        return out;
    }

    std::uint32_t expectedWorkingCounter() const noexcept override { return expected_wc_; }

    MasterLinkState masterState() noexcept override {
        ec_master_state_t ms{};
        ecrt_master_state(master_, &ms);
        MasterLinkState out;
        out.slaves_responding = ms.slaves_responding;
        out.al_states = static_cast<std::uint8_t>(ms.al_states);
        out.link_up = ms.link_up != 0;
        return out;
    }

    SlaveRuntimeState slaveState(int slave) noexcept override {
        SlaveRuntimeState out;
        if (slave < 0 || static_cast<std::size_t>(slave) >= configs_.size()) {
            return out;
        }
        ec_slave_config_state_t s{};
        ecrt_slave_config_state(configs_[static_cast<std::size_t>(slave)], &s);
        out.online = s.online != 0;
        out.operational = s.operational != 0;
        out.al_state = static_cast<std::uint8_t>(s.al_state);
        return out;
    }

    SdoRequestState sdoState(int request) noexcept override {
        if (!validRequest(request)) {
            return SdoRequestState::Error;
        }
        switch (ecrt_sdo_request_state(requests_[static_cast<std::size_t>(request)])) {
            case EC_REQUEST_UNUSED: return SdoRequestState::Unused;
            case EC_REQUEST_BUSY: return SdoRequestState::Busy;
            case EC_REQUEST_SUCCESS: return SdoRequestState::Success;
            case EC_REQUEST_ERROR: return SdoRequestState::Error;
        }
        return SdoRequestState::Error;
    }

    std::uint8_t* sdoData(int request) noexcept override {
        return validRequest(request) ? ecrt_sdo_request_data(requests_[static_cast<std::size_t>(request)]) : nullptr;
    }

    void sdoWrite(int request) noexcept override {
        if (validRequest(request)) {
            ecrt_sdo_request_write(requests_[static_cast<std::size_t>(request)]);
        }
    }

    void sdoRead(int request) noexcept override {
        if (validRequest(request)) {
            ecrt_sdo_request_read(requests_[static_cast<std::size_t>(request)]);
        }
    }

private:
    bool validRequest(int request) const noexcept {
        return request >= 0 && static_cast<std::size_t>(request) < requests_.size();
    }

    static std::int64_t monotonic() noexcept {
        timespec ts{};
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
    }

    ec_master_t* master_ = nullptr;
    ec_domain_t* domain_ = nullptr;
    std::uint8_t* domain_pd_ = nullptr;
    std::size_t domain_size_ = 0;
    std::vector<ec_slave_config_t*> configs_;
    std::vector<ec_sdo_request_t*> requests_;
    std::uint32_t expected_wc_ = 0;
    bool dc_used_ = false;
    bool active_ = false;
};

}  // namespace

bool ighBackendAvailable() noexcept { return true; }

Result<std::unique_ptr<EthercatBackend>> createIghBackend() {
    return std::unique_ptr<EthercatBackend>(std::make_unique<IghBackend>());
}

#else  // !ETHERCAT_MOTION_HAVE_IGH

bool ighBackendAvailable() noexcept { return false; }

Result<std::unique_ptr<EthercatBackend>> createIghBackend() {
    return Status(ErrorCode::NotSupported,
                  "library built without IgH EtherCAT Master support (ecrt.h not found at configure time); "
                  "install IgH EtherLab and rebuild, or use ethercat.backend: simulation");
}

#endif

}  // namespace ethercat_motion
