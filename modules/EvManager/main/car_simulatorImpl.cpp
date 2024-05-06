// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "car_simulatorImpl.hpp"
#include "Constants.hpp"
#include "SimulationCommand.hpp"
#include <everest/logging.hpp>

namespace module::main {

void car_simulatorImpl::init() {
    enabled = false;
    loopIntervalMs = DEFAULT_LOOP_INTERVAL_MS;
    resetCarSimulationDefaults();
    registerAllCommands();
    subscribeToExternalMQTT();
    subscribeToVariablesOnInit();
}

void car_simulatorImpl::ready() {
    setupEVParameters();

    if (mod->config.auto_enable) {
        auto enableCopy = mod->config.auto_enable;
        handle_enable(enableCopy);
    }
    if (mod->config.auto_exec) {
        auto valueCopy = mod->config.auto_exec_commands;
        handle_executeChargingSession(valueCopy);
    }
}

const size_t& car_simulatorImpl::getLoopIntervalMs() const {
    return loopIntervalMs;
}

const Everest::PtrContainer<EvManager>& car_simulatorImpl::getMod() const {
    return mod;
}

void car_simulatorImpl::handle_enable(bool& value) {
    if (enabled == value) {
        // ignore if value is the same
        EVLOG_warning << "Enabled value didn't change, ignoring enable!";
        return;
    }

    resetCarSimulationDefaults();

    callEVBoardSupportFunctions();

    if (value) {
        enabled = true;
        simulationThread = std::thread{&car_simulatorImpl::handleSimulationLoop, this};
    } else {
        enabled = false;
        simulationThread.join();
    }

    mod->r_ev_board_support->call_enable(value);
    publish_enabled(value);
}

void car_simulatorImpl::handle_executeChargingSession(std::string& value) {
    if (!checkCanExecute()) {
        return;
    }

    executionActive = false;
    resetCarSimulationDefaults();

    updateCommandQueue(value);

    std::lock_guard<std::mutex> lock{carSimulationMutex};
    if (!commandQueue.empty()) {
        executionActive = true;
    }
}

void car_simulatorImpl::handle_modifyChargingSession(std::string& value) {
    if (!enabled) {
        EVLOG_warning << "Simulation disabled, cannot execute charging simulation.";
        return;
    }

    executionActive = false;

    updateCommandQueue(value);

    std::lock_guard<std::mutex> lock{carSimulationMutex};
    if (!commandQueue.empty()) {
        executionActive = true;
    }
}

void car_simulatorImpl::handleSimulationLoop() {
    while (enabled) {
        if (carSimulation != nullptr && executionActive) {
            runSimulationLoop();
            std::this_thread::sleep_for(std::chrono::milliseconds(loopIntervalMs));
        }
    }
    EVLOG_debug << "Finished simulation.";

    resetCarSimulationDefaults();

    // If we have auto_exec_infinite configured, restart simulation when it is done
    if (mod->config.auto_exec && mod->config.auto_exec_infinite) {
        auto valueCopy = mod->config.auto_exec_commands;
        handle_executeChargingSession(valueCopy);
    }
}

void car_simulatorImpl::registerAllCommands() {
    commandRegistry = std::make_unique<CommandRegistry>();

    commandRegistry->registerCommand(carSimulation.get(), 1, carSimulation->sleep, "sleep");
        commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->iec_wait_pwr_ready,
        "iec_wait_pwr_ready"); commandRegistry->registerCommand(carSimulation.get(), 0,
        carSimulation->iso_wait_pwm_is_running,
                                         "iso_wait_pwm_is_running");
        commandRegistry->registerCommand(carSimulation.get(), 2, carSimulation->draw_power_regulated,
                                         "draw_power_regulated");
        commandRegistry->registerCommand(carSimulation.get(), 2, carSimulation->draw_power_fixed, "draw_power_fixed");
        commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->pause, "pause");
        commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->unplug, "unplug");
        commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->error_e, "error_e");
        commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->diode_fail, "diode_fail");
        commandRegistry->registerCommand(carSimulation.get(), 1, carSimulation->rcd_current, "rcd_current");
        commandRegistry->registerCommand(carSimulation.get(), 2, carSimulation->iso_draw_power_regulated,
                                         "iso_draw_power_regulated");
        commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->wait_for_real_plugin,
                                         "wait_for_real_plugin");

        if (!mod->r_slac.empty()) {
            EVLOG_debug << "Slac undefined";
            commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->iso_wait_slac_matched,
                                             "iso_wait_slac_matched");
        }

        if (!mod->r_ev.empty()) {
            commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->iso_wait_pwr_ready,
                                             "iso_wait_pwr_ready");
            commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->iso_dc_power_on,
            "iso_dc_power_on"); commandRegistry->registerCommand(carSimulation.get(), 1,
            carSimulation->iso_start_v2g_session,
                                             "iso_start_v2g_session");
            commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->iso_stop_charging,
            "iso_stop_charging"); commandRegistry->registerCommand(carSimulation.get(), 1,
            carSimulation->iso_wait_for_stop, "iso_wait_for_stop");
            commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->iso_wait_v2g_session_stopped,
                                             "iso_wait_v2g_session_stopped");
            commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->iso_pause_charging,
                                             "iso_pause_charging");
            commandRegistry->registerCommand(carSimulation.get(), 0, carSimulation->iso_wait_for_resume,
                                             "iso_wait_for_resume");
            commandRegistry->registerCommand(carSimulation.get(), 1, carSimulation->iso_start_bcb_toggle,
                                             "iso_start_bcb_toggle");
        }
}

void car_simulatorImpl::runSimulationLoop() {
    // Execute sim commands until a command blocks, or we are finished
    std::lock_guard<std::mutex> lock{carSimulationMutex};
    while (executionActive && !commandQueue.empty()) {
        auto& currentCommand = commandQueue.front();

        auto commandBlocked = false;

        try {
            commandBlocked = !currentCommand.execute();
        } catch (const std::exception& e) {
            EVLOG_error << e.what();
        }

        if (!commandBlocked) {
            commandQueue.pop();
        } else {
            break; // command blocked, wait for timer to run this function again
        }
    }

    carSimulation->stateMachine();
}

bool car_simulatorImpl::checkCanExecute() {
    if (!enabled) {
        EVLOG_warning << "Simulation disabled, cannot execute charging simulation.";
        return false;
    }
    if (executionActive) {
        EVLOG_warning << "Execution of charging session simulation already running, cannot start new one.";
        return false;
    }

    return true;
}

void car_simulatorImpl::subscribeToVariablesOnInit() {
    // subscribe bsp_event
    std::lock_guard<std::mutex> lock{carSimulationMutex};
    using types::board_support_common::BspEvent;
    mod->r_ev_board_support->subscribe_bsp_event([this](const auto& bsp_event) {
        carSimulation->setBspEvent(bsp_event.event);
        if (bsp_event.event == types::board_support_common::Event::Disconnected &&
            carSimulation->getState() == main::SimState::UNPLUGGED) {
            executionActive = false;
        }
    });

    // subscribe bsp_measurement
    using types::board_support_common::BspMeasurement;
    mod->r_ev_board_support->subscribe_bsp_measurement([this](const auto& measurement) {
        carSimulation->setPp(measurement.proximity_pilot.ampacity);
        carSimulation->setRcdCurrent(measurement.rcd_current_mA.value());
        carSimulation->setPwmDutyCycle(measurement.cp_pwm_duty_cycle);
    });

    // subscribe slac_state
    if (!mod->r_slac.empty()) {
        const auto& slac = mod->r_slac.at(0);
        slac->subscribe_state([this](const auto& state) { carSimulation->setSlacState(state); });
    }

    // subscribe ev events
    if (!mod->r_ev.empty()) {
        const auto& _ev = mod->r_ev.at(0);
        _ev->subscribe_AC_EVPowerReady([this](auto value) { carSimulation->setIsoPwrReady(value); });
        _ev->subscribe_AC_EVSEMaxCurrent([this](auto value) { carSimulation->setEVSEMaxCurrent(value); });
        _ev->subscribe_AC_StopFromCharger([this]() { carSimulation->setIsoStopped(true); });
        _ev->subscribe_V2G_Session_Finished([this]() { carSimulation->setV2gFinished(true); });
        _ev->subscribe_DC_PowerOn([this]() { carSimulation->setDcPowerOn(true); });
    }
}

void car_simulatorImpl::setupEVParameters() {
    if (!mod->r_ev.empty()) {
        mod->r_ev[0]->call_set_dc_params({mod->config.dc_max_current_limit, mod->config.dc_max_power_limit,
                                          mod->config.dc_max_voltage_limit, mod->config.dc_energy_capacity,
                                          mod->config.dc_target_current, mod->config.dc_target_voltage});
        if (mod->config.support_sae_j2847) {
            mod->r_ev[0]->call_enable_sae_j2847_v2g_v2h();
            mod->r_ev[0]->call_set_bpt_dc_params(
                {mod->config.dc_discharge_max_current_limit, mod->config.dc_discharge_max_power_limit,
                 mod->config.dc_discharge_target_current, mod->config.dc_discharge_v2g_minimal_soc});
        }
    }
}

void car_simulatorImpl::callEVBoardSupportFunctions() {
    mod->r_ev_board_support->call_allow_power_on(false);

    mod->r_ev_board_support->call_set_ac_max_current(mod->config.max_current);
    mod->r_ev_board_support->call_set_three_phases(mod->config.three_phases);
}

void car_simulatorImpl::subscribeToExternalMQTT() {
    const auto& mqtt = mod->mqtt;
    mqtt.subscribe("everest_external/nodered/" + std::to_string(mod->config.connector_id) + "/carsim/cmd/enable",
                   [this](const std::string& message) {
                       if (message == "true") {
                           auto enable = true;
                           handle_enable(enable);
                       } else {
                           auto enable = false;
                           handle_enable(enable);
                       }
                   });
    mqtt.subscribe("everest_external/nodered/" + std::to_string(mod->config.connector_id) +
                       "/carsim/cmd/execute_charging_session",
                   [this](const auto data) {
                       auto dataCopy{data};
                       handle_executeChargingSession(dataCopy);
                   });
    mqtt.subscribe("everest_external/nodered/" + std::to_string(mod->config.connector_id) +
                       "/carsim/cmd/modify_charging_session",
                   [this](auto data) {
                       auto dataCopy = data;
                       handle_modifyChargingSession(dataCopy);
                   });
}
void car_simulatorImpl::resetCarSimulationDefaults() {
    std::lock_guard<std::mutex> lock{carSimulationMutex};
    carSimulation = std::make_unique<CarSimulation<car_simulatorImpl>>(this);
}

void car_simulatorImpl::updateCommandQueue(std::string& value) {
    std::lock_guard<std::mutex> lock{carSimulationMutex};
    commandQueue = SimulationCommand::parseSimCommands(value, *commandRegistry);
}
} // namespace module::main
