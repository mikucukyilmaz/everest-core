// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "car_simulatorImpl.hpp"
#include "main/SimCommand.hpp"
#include <everest/logging.hpp>
#include <queue>

namespace module::main {

void car_simulatorImpl::init() {
    // TODO: simulation loop time from manifest.yaml?
    simData = std::make_unique<main::SimData>();
    subscribeToExternalMQTT();
    subscribeToVariablesOnInit();
}

void car_simulatorImpl::ready() {
    registerAllCommands();
    enabled = false;

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

void car_simulatorImpl::handle_enable(bool& value) {
    if (!enabled.has_value()) {
        EVLOG_warning << "Already received data, but framework is not ready yet.";
        return;
    }

    if (enabled == value) {
        // ignore if value is the same
        return;
    }

    simData = std::make_unique<main::SimData>();

    callEVBoardSupportFunctions();

    // set loop interval
    if (value) {
        enabled = true;
        loopIntervalMs = defaultLoopIntervalMs;
        simulationThread = std::thread{[this] { handleSimulationLoop(); }};
    } else {
        enabled = false;
        loopIntervalMs.reset();
        simulationThread.join();
    }

    mod->r_ev_board_support->call_enable(value);
    publish_enabled(value);
}

void car_simulatorImpl::handle_executeChargingSession(std::string& value) {
    // Check enabled
    if (!checkCanExecute()) {
        return;
    }

    simData = std::make_unique<main::SimData>();

    auto valueCopy = value;
    auto& commandQueue = simData->commandQueue;
    commandQueue = parseSimCommands(valueCopy);

    // Start execution
    if (!commandQueue.empty()) {
        simData->executionActive = true;
    }
}

void car_simulatorImpl::handle_modifyChargingSession(std::string& value) {
    if (!enabled) {
        EVLOG_warning << "Simulation disabled, cannot execute charging simulation.";
        return;
    }

    auto valueCopy = value;
    simData->commandQueue = parseSimCommands(valueCopy);

    if (!simData->commandQueue.empty()) {
        simData->executionActive = true;
    }
}

void car_simulatorImpl::handleSimulationLoop() {
    while (enabled) {
        if (simData && simData->executionActive) {
            if (loopIntervalMs.has_value()) {
                runSimulationLoop();
                std::this_thread::sleep_for(std::chrono::milliseconds(loopIntervalMs.value()));
            } else {
                break;
            }
        }
    }
}

void car_simulatorImpl::registerAllCommands() {
    const auto sleepFuntion = [this](const std::vector<std::string>& arguments) {
        static std::optional<size_t> sleepTimeLeftMs;
        if (!sleepTimeLeftMs.has_value()) {
            sleepTimeLeftMs =
                std::stoll(arguments[0]) * static_cast<long>((1 / static_cast<float>(loopIntervalMs.value()))) + 1;
        }
        --sleepTimeLeftMs.value();
        return (!(sleepTimeLeftMs > 0));
    };
    RegisteredCommandBase::registerCommand("sleep", sleepFuntion, 1);

    const auto iec_wait_pwr_readyFunction = [this](const std::vector<std::string>& arguments) {
        return (simData->pwm_duty_cycle > 7.0f && simData->pwm_duty_cycle < 97.0f);
    };
    RegisteredCommandBase::registerCommand("iec_wait_pwr_ready", iec_wait_pwr_readyFunction, 0);

    const auto iso_wait_pwm_is_runningFunction = [this](const std::vector<std::string>& arguments) {
        simData->state = SimState::PLUGGED_IN;
        return (simData->pwm_duty_cycle > 4.0f && simData->pwm_duty_cycle < 97.0f);
    };
    RegisteredCommandBase::registerCommand("iso_wait_pwm_is_running", iso_wait_pwm_is_runningFunction, 0);

    const auto draw_power_regulatedFunction = [this](const std::vector<std::string>& arguments) {
        mod->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == "3") {
            mod->r_ev_board_support->call_set_three_phases(true);
        } else {
            mod->r_ev_board_support->call_set_three_phases(false);
        }
        simData->state = SimState::CHARGING_REGULATED;
        return true;
    };
    RegisteredCommandBase::registerCommand("draw_power_regulated", draw_power_regulatedFunction, 2);

    const auto draw_power_fixedFunction = [this](const std::vector<std::string>& arguments) {
        mod->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == "3") {
            mod->r_ev_board_support->call_set_three_phases(true);
        } else {
            mod->r_ev_board_support->call_set_three_phases(false);
        }
        simData->state = SimState::CHARGING_FIXED;
        return true;
    };
    RegisteredCommandBase::registerCommand("draw_power_fixed", draw_power_fixedFunction, 2);

    const auto pauseFunction = [this](const std::vector<std::string>& arguments) {
        simData->state = SimState::PLUGGED_IN;
        return true;
    };
    RegisteredCommandBase::registerCommand("pause", pauseFunction, 0);

    const auto unplugFunction = [this](const std::vector<std::string>& arguments) {
        simData->state = SimState::UNPLUGGED;
        return true;
    };
    RegisteredCommandBase::registerCommand("unplug", unplugFunction, 0);

    const auto error_eFunction = [this](const std::vector<std::string>& arguments) {
        simData->state = SimState::ERROR_E;
        return true;
    };
    RegisteredCommandBase::registerCommand("error_e", error_eFunction, 0);

    const auto diode_failFunction = [this](const std::vector<std::string>& arguments) {
        simData->state = SimState::DIODE_FAIL;
        return true;
    };
    RegisteredCommandBase::registerCommand("diode_fail", diode_failFunction, 0);

    const auto rcd_currentFunction = [this](const std::vector<std::string>& arguments) {
        simData->rcd_current_mA = std::stof(arguments[0]);
        return true;
    };
    RegisteredCommandBase::registerCommand("rcd_current", rcd_currentFunction, 1);

    const auto iso_wait_slac_matchedFunction = [this](const std::vector<std::string>& arguments) {
        simData->state = SimState::PLUGGED_IN;

        if (mod->r_slac.empty()) {
            EVLOG_debug << "Slac undefined";
        }
        if (simData->slacState == "UNMATCHED") {
            EVLOG_debug << "Slac UNMATCHED";
            if (!mod->r_slac.empty()) {
                EVLOG_debug << "Slac trigger matching";
                mod->r_slac[0]->call_reset();
                mod->r_slac[0]->call_trigger_matching();
                simData->slacState = "TRIGGERED";
            }
        }
        if (simData->slacState == "MATCHED") {
            EVLOG_debug << "Slac Matched";
            return true;
        }
        return false;
    };
    RegisteredCommandBase::registerCommand("iso_wait_slac_matched", iso_wait_slac_matchedFunction, 0);

    if (!mod->r_ev.empty()) {
        const auto iso_start_v2g_sessionFunction = [this](const std::vector<std::string>& arguments) {
            const auto& argument = arguments[0];
            if (argument == "ac_single_phase_core") {
                mod->r_ev[0]->call_start_charging("AC_single_phase_core");
            }
            if (argument == "ac_three_phase_core") {
                mod->r_ev[0]->call_start_charging("AC_three_phase_core");
            }
            if (argument == "dc_core") {
                mod->r_ev[0]->call_start_charging("DC_core");
            }
            if (argument == "dc_extended") {
                mod->r_ev[0]->call_start_charging("DC_extended");
            }
            if (argument == "dc_combo_core") {
                mod->r_ev[0]->call_start_charging("DC_combo_core");
            }
            if (argument == "dc_unique") {
                mod->r_ev[0]->call_start_charging("DC_unique");
            } else {
                return false;
            }
            return true;
        };
        RegisteredCommandBase::registerCommand("iso_start_v2g_sessionFunction", iso_start_v2g_sessionFunction, 1);
    }

    const auto iso_wait_pwr_readyFunction = [this](const std::vector<std::string>& arguments) {
        if (simData->iso_pwr_ready) {
            simData->state = SimState::ISO_POWER_READY;
            return true;
        }
        return false;
    };
    RegisteredCommandBase::registerCommand("iso_wait_pwr_ready", iso_wait_pwr_readyFunction, 0);

    const auto iso_dc_power_onFunction = [this](const std::vector<std::string>& arguments) {
        simData->state = SimState::ISO_POWER_READY;
        if (simData->dc_power_on) {
            simData->state = SimState::ISO_CHARGING_REGULATED;
            mod->r_ev_board_support->call_allow_power_on(true);
            return true;
        }
        return false;
    };
    RegisteredCommandBase::registerCommand("iso_dc_power_on", iso_dc_power_onFunction, 0);

    const auto iso_draw_power_regulatedFuncdtion = [this](const std::vector<std::string>& arguments) {
        mod->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == "3") {
            mod->r_ev_board_support->call_set_three_phases(true);
        } else {
            mod->r_ev_board_support->call_set_three_phases(false);
        }
        simData->state = SimState::ISO_CHARGING_REGULATED;
        return true;
    };
    RegisteredCommandBase::registerCommand("iso_draw_power_regulated", iso_draw_power_regulatedFuncdtion, 2);

    if (!mod->r_ev.empty()) {
        const auto iso_stop_chargingFunction = [this](const std::vector<std::string>& arguments) {
            mod->r_ev[0]->call_stop_charging();
            mod->r_ev_board_support->call_allow_power_on(false);
            simData->state = SimState::PLUGGED_IN;
            return true;
        };
        RegisteredCommandBase::registerCommand("iso_stop_charging", iso_stop_chargingFunction, 0);

        const auto iso_wait_for_stop = [this](const std::vector<std::string>& arguments) {
            static auto sleepTimeLeftMs = std::optional<size_t>{};
            if (!sleepTimeLeftMs.has_value()) {
                sleepTimeLeftMs =
                    std::stoll(arguments[0]) * static_cast<long>(1 / static_cast<float>(loopIntervalMs.value())) + 1;
            }
            --sleepTimeLeftMs.value();
            if (!sleepTimeLeftMs > 0) {
                mod->r_ev[0]->call_stop_charging();
                mod->r_ev_board_support->call_allow_power_on(false);
                simData->state = SimState::PLUGGED_IN;
                return true;
            }
            if (simData->iso_stopped) {
                EVLOG_info << "POWER OFF iso stopped";
                mod->r_ev_board_support->call_allow_power_on(false);
                simData->state = SimState::PLUGGED_IN;
                return true;
            }
            return false;
        };
        RegisteredCommandBase::registerCommand("iso_wait_for_stop", iso_wait_for_stop, 1);

        const auto iso_wait_v2g_session_stoppedFunction = [this](const std::vector<std::string>& arguments) {
            if (simData->v2g_finished) {
                return true;
            }
            return false;
        };
        RegisteredCommandBase::registerCommand("iso_wait_v2g_session_stopped", iso_wait_v2g_session_stoppedFunction, 0);

        const auto iso_pause_chargingFunction = [this](const std::vector<std::string>& arguments) {
            mod->r_ev[0]->call_pause_charging();
            simData->state = SimState::PLUGGED_IN;
            simData->iso_pwr_ready = false;
            return true;
        };
        RegisteredCommandBase::registerCommand("iso_pause_charging", iso_pause_chargingFunction, 0);

        const auto iso_wait_for_resumeFunction = [this](const std::vector<std::string>& arguments) { return false; };
        RegisteredCommandBase::registerCommand("iso_wait_for_resume", iso_wait_for_resumeFunction, 0);

        const auto iso_start_bcb_toggleFunction = [this](const std::vector<std::string>& arguments) {
            simData->state = SimState::BCB_TOGGLE;
            if (simData->bcb_toggles >= std::stoul(arguments[0]) || simData->bcb_toggles == 3) {
                simData->bcb_toggles = 0;
                simData->state = SimState::PLUGGED_IN;
                return true;
            }
            return false;
        };
        RegisteredCommandBase::registerCommand("iso_start_bcb_toggle", iso_start_bcb_toggleFunction, 1);
    }

    const auto wait_for_real_pluginFunction = [this](const std::vector<std::string>& arguments) {
        using types::board_support_common::Event;
        if (simData->actualBspEvent == Event::A) {
            EVLOG_info << "Real plugin detected";
            simData->state = SimState::PLUGGED_IN;
            return true;
        }
        return false;
    };
    RegisteredCommandBase::registerCommand("wait_for_real_plugin", wait_for_real_pluginFunction, 0);
}

void car_simulatorImpl::carStateMachine() {
    using types::ev_board_support::EvCpState;

    const auto stateHasChanged = simData->state == simData->lastState;
    simData->lastState = simData->state;

    switch (simData->state) {
    case SimState::UNPLUGGED:
        if (stateHasChanged) {

            mod->r_ev_board_support->call_set_cp_state(EvCpState::A);
            mod->r_ev_board_support->call_allow_power_on(false);
            // Wait for physical plugin (ev BSP sees state A on CP and not Disconnected)

            // If we have auto_exec configured, restart simulation when it was unplugged
            EVLOG_info << "Unplug detected, restarting simulation.";
            simData->slacState = "UNMATCHED";
            mod->r_ev[0]->call_stop_charging();
            if (mod->config.auto_exec) {
                auto valueCopy = mod->config.auto_exec_commands;
                handle_executeChargingSession(valueCopy);
            }
        }
        break;
    case SimState::PLUGGED_IN:
        if (stateHasChanged) {
            mod->r_ev_board_support->call_set_cp_state(EvCpState::B);
            mod->r_ev_board_support->call_allow_power_on(false);
        }
        break;
    case SimState::CHARGING_REGULATED:
        if (stateHasChanged || simData->pwm_duty_cycle != simData->last_pwm_duty_cycle) {
            simData->last_pwm_duty_cycle = simData->pwm_duty_cycle;
            // do not draw power if EVSE paused by stopping PWM
            if (simData->pwm_duty_cycle > 7.0 && simData->pwm_duty_cycle < 97.0) {
                mod->r_ev_board_support->call_set_cp_state(EvCpState::C);
            } else {
                mod->r_ev_board_support->call_set_cp_state(EvCpState::B);
            }
        }
        break;
    case SimState::CHARGING_FIXED:
        // Todo(sl): What to do here
        if (stateHasChanged) {
            // Also draw power if EVSE stopped PWM - this is a break the rules mode to test the charging implementation!
            mod->r_ev_board_support->call_set_cp_state(EvCpState::C);
        }
        break;

    case SimState::ERROR_E:
        if (stateHasChanged) {
            mod->r_ev_board_support->call_set_cp_state(EvCpState::E);
            mod->r_ev_board_support->call_allow_power_on(false);
        }
        break;
    case SimState::DIODE_FAIL:
        if (stateHasChanged) {
            mod->r_ev_board_support->call_diode_fail(true);
            mod->r_ev_board_support->call_allow_power_on(false);
        }
        break;
    case SimState::ISO_POWER_READY:
        if (stateHasChanged) {
            mod->r_ev_board_support->call_set_cp_state(EvCpState::C);
        }
        break;
    case SimState::ISO_CHARGING_REGULATED:
        if (stateHasChanged) {
            mod->r_ev_board_support->call_set_cp_state(EvCpState::C);
        }
        break;
    case SimState::BCB_TOGGLE:
        if (simData->bcb_toggle_C) {
            mod->r_ev_board_support->call_set_cp_state(EvCpState::C);
            simData->bcb_toggle_C = false;
        } else {
            mod->r_ev_board_support->call_set_cp_state(EvCpState::B);
            simData->bcb_toggle_C = true;
            ++simData->bcb_toggles;
        }
        break;
    default:
        simData->state = SimState::UNPLUGGED;
        break;
    }
}

void car_simulatorImpl::runSimulationLoop() {
    // Execute sim commands until a command blocks or we are finished
    auto& commandQueue = simData->commandQueue;
    while (simData->executionActive && !commandQueue.empty()) {
        auto& currentCommand = commandQueue.front();

        if (currentCommand.execute()) {
        } else
            break; // command blocked, wait for timer to run this function again
        commandQueue.pop();
    }

    EVLOG_debug << "Finished simulation.";

    // reset simData
    simData = std::make_unique<main::SimData>();

    // If we have auto_exec configured, restart simulation when it is done
    if (mod->config.auto_exec) {
        auto valueCopy = mod->config.auto_exec_commands;
        handle_executeChargingSession(valueCopy);
    }
    carStateMachine();
}

bool car_simulatorImpl::checkCanExecute() {
    if (!enabled.has_value()) {
        EVLOG_warning << "Already received data, but framework is not ready yet.";
        return false;
    }

    const auto enabledValue = enabled.value();
    if (!enabledValue) {
        EVLOG_warning << "Simulation disabled, cannot execute charging simulation.";
        return false;
    }
    if (simData->executionActive) {
        EVLOG_warning << "Execution of charging session simulation already running, cannot start new one.";
        return false;
    }

    return true;
}

void car_simulatorImpl::subscribeToVariablesOnInit() {
    // subscribe bsp_event
    using types::board_support_common::BspEvent;
    mod->r_ev_board_support->subscribe_bsp_event([this](auto bsp_event) {
        simData->actualBspEvent = bsp_event.event;
        if (bsp_event.event == types::board_support_common::Event::Disconnected &&
            simData->state == main::SimState::UNPLUGGED) {
            simData->executionActive = false;
            simData->state = main::SimState::UNPLUGGED;
        }
    });

    // subscribe bsp_measurement
    using types::board_support_common::BspMeasurement;
    mod->r_ev_board_support->subscribe_bsp_measurement([this](auto measurement) {
        simData->pp = measurement.proximity_pilot.ampacity;
        simData->rcd_current_mA = measurement.rcd_current_mA.value();
        simData->pwm_duty_cycle = measurement.cp_pwm_duty_cycle;
    });

    // subscribe slac_state
    if (!mod->r_slac.empty()) {
        const auto& slac = mod->r_slac.at(0);
        slac->subscribe_state([this](const auto& state) { simData->slacState = state; });
    }

    // subscribe ev events
    if (!mod->r_ev.empty()) {
        const auto& _ev = mod->r_ev.at(0);
        _ev->subscribe_AC_EVPowerReady([this](auto value) { simData->iso_pwr_ready = value; });
        _ev->subscribe_AC_EVSEMaxCurrent([this](auto value) { simData->iso_pwr_ready = value; });
        _ev->subscribe_AC_StopFromCharger([this]() { simData->iso_stopped = true; });
        _ev->subscribe_V2G_Session_Finished([this]() { simData->v2g_finished = true; });
        _ev->subscribe_DC_PowerOn([this]() { simData->dc_power_on = true; });
    }
}
void car_simulatorImpl::setupEVParameters() {
    if (!mod->r_ev.empty()) {
        mod->r_ev[0]->call_set_dc_params({.MaxCurrentLimit = mod->config.dc_max_current_limit,
                                          .MaxPowerLimit = mod->config.dc_max_power_limit,
                                          .MaxVoltageLimit = mod->config.dc_max_voltage_limit,
                                          .EnergyCapacity = mod->config.dc_energy_capacity,
                                          .TargetCurrent = mod->config.dc_target_current,
                                          .TargetVoltage = mod->config.dc_target_voltage});
        if (mod->config.support_sae_j2847) {
            mod->r_ev[0]->call_enable_sae_j2847_v2g_v2h();
            mod->r_ev[0]->call_set_bpt_dc_params(
                {.DischargeMaxCurrentLimit = mod->config.dc_discharge_max_current_limit,
                 .DischargeMaxPowerLimit = mod->config.dc_discharge_max_power_limit,
                 .DischargeTargetCurrent = mod->config.dc_discharge_target_current,
                 .DischargeMinimalSoC = mod->config.dc_discharge_v2g_minimal_soc});
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
    EVLOG_critical << "TEST MQTT SUBSCRIBE";
    EVLOG_critical << "everest_external/nodered/" + std::to_string(mod->config.connector_id) + "/carsim/cmd/enable";
    mqtt.subscribe("everest_external/nodered/" + std::to_string(mod->config.connector_id) + "/carsim/cmd/enable",
                   [this](const std::string& message) { EVLOG_critical << "Received message: " << message; });
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
    EVLOG_critical << "TEST MQTT DONE";
}

} // namespace module::main
