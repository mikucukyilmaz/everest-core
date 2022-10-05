// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "powermeterImpl.hpp"

#include "lib/transport.hpp"
#include <utils/date.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>

using namespace std::chrono_literals;

namespace module {
namespace main {

//////////////////////////////////////////////////////////////////////
//
// module related stuff

void powermeterImpl::init() {
    EVLOG_verbose << "init: ";
    m_module_start_timestamp =
        std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch();
}

void powermeterImpl::ready() {

    m_watchdog_thread = std::thread(&powermeterImpl::watchdog, this);
}

std::string powermeterImpl::handle_get_signed_meter_value(std::string& auth_token) {

    // your code for cmd get_signed_meter_value goes here

    try {

        std::scoped_lock lock(mod->get_device_mutex());

        transport::AbstractDataTransport::Spt transport = mod->get_data_transport();

        transport->trigger_snapshot_generation_BSM_OCMF();

        transport::DataVector data = transport->fetch(known_model::BSM_OCMF_CurrentSnapshot);

        bsm::SignedOCMFSnapshot signed_snapshot(data);

        return signed_snapshot.O();

    } catch (const std::runtime_error& e) {
        EVLOG_error << __PRETTY_FUNCTION__ << " Error: " << e.what() << std::endl;
        return "get_signed_meter_value_error";
    }
};

//////////////////////////////////////////////////////////////////////
//
// module module implementation

void powermeterImpl::watchdog() {

    // poor mans watchdog
    m_publish_variables_worker_thread = std::thread(&powermeterImpl::worker, this);

    while (true) {

        std::this_thread::sleep_for(mod->get_watchdog_interval());

        if (m_publish_variables_worker_thread.joinable()) {
            EVLOG_warning << __PRETTY_FUNCTION__ << " worker thread has died. restarting... " << std::endl;
            m_publish_variables_worker_thread.join();
            m_publish_variables_worker_thread = std::thread(&powermeterImpl::worker, this);
        }
    }

    EVLOG_error << __PRETTY_FUNCTION__ << " watchdog thread terminating, module is dead." << std::endl;
}

void powermeterImpl::worker() {

    try {

        EVLOG_verbose << __PRETTY_FUNCTION__ << " start " << std::endl;

        mod->dump_config_to_log();

        while (true) {

            std::this_thread::sleep_for(mod->get_update_interval());

            try {

                // lock device mutex
                std::scoped_lock lock(mod->get_device_mutex());

                transport::AbstractDataTransport::Spt transport = mod->get_data_transport();

                EVLOG_debug << __PRETTY_FUNCTION__ << " wakeup. " << std::endl;

                transport::DataVector data = transport->fetch(known_model::Sunspec_ACMeter);

                sunspec_model::ACMeter acmeter(data);
                types::powermeter::Powermeter result;

                result.timestamp = 0; // FIXME: current implementation is a float that cant store a 32/31 bit
                result.timestamp_RFC3339_UTC = Everest::Date::to_rfc3339(date::utc_clock::now());

                result.meter_id = mod->config.meter_id;

                float scale_factor_Wh_import = pow(10, acmeter.TotWh_SF());
                result.energy_Wh_import.total = acmeter.TotWhIm() * scale_factor_Wh_import;

                float scale_factor_W = pow(10, acmeter.W_SF());
                result.power_W = types::units::Power{.total = static_cast<float>(acmeter.W() * scale_factor_W)};

                float scale_factor_current = pow(10, acmeter.A_SF());
                result.current_A = types::units::Current{.L1 = static_cast<float>(acmeter.A() * scale_factor_current)};

                float scale_factor_voltage = pow(10, acmeter.V_SF());
                result.voltage_V =
                    types::units::Voltage{.L1 = static_cast<float>(acmeter.PhVphA() * scale_factor_voltage),
                                          .L2 = static_cast<float>(acmeter.PhVphB() * scale_factor_voltage),
                                          .L3 = static_cast<float>(acmeter.PhVphC() * scale_factor_voltage)};

                float scale_factor_frequency = pow(10, acmeter.Hz_SF());
                result.frequency_Hz =
                    types::units::Frequency{.L1 = static_cast<float>(acmeter.Hz() * scale_factor_frequency)};

                float scale_factor_reactive_power = pow(10, acmeter.VAR_SF());
                result.VAR = types::units::ReactivePower{
                    .total = static_cast<float>(acmeter.VAR() * scale_factor_reactive_power),
                    .VARphA = static_cast<float>(acmeter.VARphA() * scale_factor_reactive_power),
                    .VARphB = static_cast<float>(acmeter.VARphB() * scale_factor_reactive_power),
                    .VARphC = static_cast<float>(acmeter.VARphC() * scale_factor_reactive_power)};

                publish_powermeter(result);

            } catch (const std::exception& e) {
                // we catch std::exception& here since there may be other exceotions than std::runtime_error
                EVLOG_warning << __PRETTY_FUNCTION__ << " Exception caught: " << e.what() << std::endl;
            }
        }
    } catch (const std::runtime_error& e) {
        EVLOG_warning << __PRETTY_FUNCTION__ << " workerthread dies: " << e.what() << std::endl;
    }
}

} // namespace main
} // namespace module
