// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "car_simulatorImpl.hpp"
#include "Constants.hpp"
#include "SimCommand.hpp"
#include <everest/logging.hpp>

namespace module::main {

void car_simulatorImpl::init() {
    enabled = false;
    loopIntervalMs = DEFAULT_LOOP_INTERVAL_MS;
    resetSimDataToDefaults();
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

void car_simulatorImpl::handle_enable(bool& value) {
    if (enabled == value) {
        // ignore if value is the same
        EVLOG_warning << "Enabled value didn't change, ignoring enable!";
        return;
    }

    resetSimDataToDefaults();

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
    resetSimDataToDefaults();

    updateCommandQueue(value);

    std::lock_guard<std::mutex> lock{simDataMutex};
    if (!simData->commandQueue.empty()) {
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

    std::lock_guard<std::mutex> lock{simDataMutex};
    if (!simData->commandQueue.empty()) {
        executionActive = true;
    }
}

void car_simulatorImpl::handleSimulationLoop() {
    while (enabled) {
        if (simData != nullptr && executionActive) {
            runSimulationLoop();
            std::this_thread::sleep_for(std::chrono::milliseconds(loopIntervalMs));
        }
    }
    EVLOG_debug << "Finished simulation.";

    resetSimDataToDefaults();

    // If we have auto_exec_infinite configured, restart simulation when it is done
    if (mod->config.auto_exec && mod->config.auto_exec_infinite) {
        auto valueCopy = mod->config.auto_exec_commands;
        handle_executeChargingSession(valueCopy);
    }
}

void car_simulatorImpl::registerAllCommands() {
    commandRegistry = std::make_unique<CommandRegistry>(this);

    const auto sleepFunction = [](car_simulatorImpl* const simulator, const std::vector<std::string>& arguments) {
        if (!simulator->simData->sleepTicksLeft.has_value()) {
            const auto sleepTime = std::stold(arguments[0]);
            const auto sleepTimeMs = sleepTime * 1000;
            simulator->simData->sleepTicksLeft = static_cast<long long>(sleepTimeMs / simulator->loopIntervalMs) + 1;
        }
        simulator->simData->sleepTicksLeft = simulator->simData->sleepTicksLeft.value() - 1;
        if (!(simulator->simData->sleepTicksLeft > 0)) {
            simulator->simData->sleepTicksLeft.reset();
            return true;
        } else {
            return false;
        }
    };
    commandRegistry->registerCommand("sleep", 1, sleepFunction);

    const auto iec_wait_pwr_readyFunction = [](car_simulatorImpl* const simulator,
                                               const std::vector<std::string>& arguments) {
        simulator->simData->state = SimState::PLUGGED_IN;
        return (simulator->simData->pwm_duty_cycle > 7.0f && simulator->simData->pwm_duty_cycle < 97.0f);
    };
    commandRegistry->registerCommand("iec_wait_pwr_ready", 0, iec_wait_pwr_readyFunction);

    const auto iso_wait_pwm_is_runningFunction = [](car_simulatorImpl* const simulator,
                                                    const std::vector<std::string>& arguments) {
        simulator->simData->state = SimState::PLUGGED_IN;
        return (simulator->simData->pwm_duty_cycle > 4.0f && simulator->simData->pwm_duty_cycle < 97.0f);
    };
    commandRegistry->registerCommand("iso_wait_pwm_is_running", 0, iso_wait_pwm_is_runningFunction);

    const auto draw_power_regulatedFunction = [](car_simulatorImpl* const simulator,
                                                 const std::vector<std::string>& arguments) {
        simulator->mod->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == THREE_PHASES) {
            simulator->mod->r_ev_board_support->call_set_three_phases(true);
        } else {
            simulator->mod->r_ev_board_support->call_set_three_phases(false);
        }
        simulator->simData->state = SimState::CHARGING_REGULATED;
        return true;
    };
    commandRegistry->registerCommand("draw_power_regulated", 2, draw_power_regulatedFunction);

    const auto draw_power_fixedFunction = [](car_simulatorImpl* const simulator,
                                             const std::vector<std::string>& arguments) {
        simulator->mod->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == THREE_PHASES) {
            simulator->mod->r_ev_board_support->call_set_three_phases(true);
        } else {
            simulator->mod->r_ev_board_support->call_set_three_phases(false);
        }
        simulator->simData->state = SimState::CHARGING_FIXED;
        return true;
    };
    commandRegistry->registerCommand("draw_power_fixed", 2, draw_power_fixedFunction);

    const auto pauseFunction = [](car_simulatorImpl* const simulator, const std::vector<std::string>& arguments) {
        simulator->simData->state = SimState::PLUGGED_IN;
        return true;
    };
    commandRegistry->registerCommand("pause", 0, pauseFunction);

    const auto unplugFunction = [](car_simulatorImpl* const simulator, const std::vector<std::string>& arguments) {
        simulator->simData->state = SimState::UNPLUGGED;
        return true;
    };
    commandRegistry->registerCommand("unplug", 0, unplugFunction);

    const auto error_eFunction = [](car_simulatorImpl* const simulator, const std::vector<std::string>& arguments) {
        simulator->simData->state = SimState::ERROR_E;
        return true;
    };
    commandRegistry->registerCommand("error_e", 0, error_eFunction);

    const auto diode_failFunction = [](car_simulatorImpl* const simulator, const std::vector<std::string>& arguments) {
        simulator->simData->state = SimState::DIODE_FAIL;
        return true;
    };
    commandRegistry->registerCommand("diode_fail", 0, diode_failFunction);

    const auto rcd_currentFunction = [](car_simulatorImpl* const simulator, const std::vector<std::string>& arguments) {
        simulator->simData->rcd_current_mA = std::stof(arguments[0]);
        return true;
    };
    commandRegistry->registerCommand("rcd_current", 1, rcd_currentFunction);

    if (!mod->r_slac.empty()) {
        EVLOG_debug << "Slac undefined";
        const auto iso_wait_slac_matchedFunction = [](car_simulatorImpl* const simulator,
                                                      const std::vector<std::string>& arguments) {
            simulator->simData->state = SimState::PLUGGED_IN;

            if (simulator->simData->slacState == "UNMATCHED") {
                EVLOG_debug << "Slac UNMATCHED";
                if (!simulator->mod->r_slac.empty()) {
                    EVLOG_debug << "Slac trigger matching";
                    simulator->mod->r_slac[0]->call_reset();
                    simulator->mod->r_slac[0]->call_trigger_matching();
                    simulator->simData->slacState = "TRIGGERED";
                }
            }
            if (simulator->simData->slacState == "MATCHED") {
                EVLOG_debug << "Slac Matched";
                return true;
            }
            return false;
        };
        commandRegistry->registerCommand("iso_wait_slac_matched", 0, iso_wait_slac_matchedFunction);
    }

    if (!mod->r_ev.empty()) {
        const auto iso_wait_pwr_readyFunction = [](car_simulatorImpl* const simulator,
                                                   const std::vector<std::string>& arguments) {
            if (simulator->simData->iso_pwr_ready) {
                simulator->simData->state = SimState::ISO_POWER_READY;
                return true;
            }
            return false;
        };
        commandRegistry->registerCommand("iso_wait_pwr_ready", 0, iso_wait_pwr_readyFunction);

        const auto iso_dc_power_onFunction = [](car_simulatorImpl* const simulator,
                                                const std::vector<std::string>& arguments) {
            simulator->simData->state = SimState::ISO_POWER_READY;
            if (simulator->simData->dc_power_on) {
                simulator->simData->state = SimState::ISO_CHARGING_REGULATED;
                simulator->mod->r_ev_board_support->call_allow_power_on(true);
                return true;
            }
            return false;
        };
        commandRegistry->registerCommand("iso_dc_power_on", 0, iso_dc_power_onFunction);

        const auto iso_start_v2g_sessionFunction = [](car_simulatorImpl* const simulator,
                                                      const std::vector<std::string>& arguments) {
            const auto& argument = arguments[0];
            if (argument == EnergyMode::AC_SINGLE_PHASE_CORE) {
                simulator->simData->energymode = &EnergyMode::AC_SINGLE_PHASE_CORE;
            } else if (argument == EnergyMode::AC_THREE_PHASE_CORE) {
                simulator->simData->energymode = &EnergyMode::AC_THREE_PHASE_CORE;
            } else if (argument == EnergyMode::DC_CORE) {
                simulator->simData->energymode = &EnergyMode::DC_CORE;
            } else if (argument == EnergyMode::DC_EXTENDED) {
                simulator->simData->energymode = &EnergyMode::DC_EXTENDED;
            } else if (argument == EnergyMode::DC_COMBO_CORE) {
                simulator->simData->energymode = &EnergyMode::DC_COMBO_CORE;
            } else if (argument == EnergyMode::DC_UNIQUE) {
                simulator->simData->energymode = &EnergyMode::DC_UNIQUE;
            } else {
                return false;
            }

            simulator->mod->r_ev[0]->call_start_charging(*simulator->simData->energymode);
            return true;
        };
        commandRegistry->registerCommand("iso_start_v2g_session", 1, iso_start_v2g_sessionFunction);
    }

    const auto iso_draw_power_regulatedFunction = [](car_simulatorImpl* const simulator,
                                                     const std::vector<std::string>& arguments) {
        simulator->mod->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == THREE_PHASES) {
            simulator->mod->r_ev_board_support->call_set_three_phases(true);
        } else {
            simulator->mod->r_ev_board_support->call_set_three_phases(false);
        }
        simulator->simData->state = SimState::ISO_CHARGING_REGULATED;
        return true;
    };
    commandRegistry->registerCommand("iso_draw_power_regulated", 2, iso_draw_power_regulatedFunction);

    if (!mod->r_ev.empty()) {
        const auto iso_stop_chargingFunction = [](car_simulatorImpl* const simulator,
                                                  const std::vector<std::string>& arguments) {
            simulator->mod->r_ev[0]->call_stop_charging();
            simulator->mod->r_ev_board_support->call_allow_power_on(false);
            simulator->simData->state = SimState::PLUGGED_IN;
            return true;
        };
        commandRegistry->registerCommand("iso_stop_charging", 0, iso_stop_chargingFunction);

        const auto iso_wait_for_stop = [](car_simulatorImpl* const simulator,
                                          const std::vector<std::string>& arguments) {
            if (!simulator->simData->sleepTicksLeft.has_value()) {
                simulator->simData->sleepTicksLeft =
                    std::stoll(arguments[0]) * static_cast<long>(1 / static_cast<float>(simulator->loopIntervalMs)) + 1;
            }
            simulator->simData->sleepTicksLeft = simulator->simData->sleepTicksLeft.value() - 1;
            if (!simulator->simData->sleepTicksLeft > 0) {
                simulator->mod->r_ev[0]->call_stop_charging();
                simulator->mod->r_ev_board_support->call_allow_power_on(false);
                simulator->simData->state = SimState::PLUGGED_IN;
                return true;
            }
            if (simulator->simData->iso_stopped) {
                EVLOG_info << "POWER OFF iso stopped";
                simulator->mod->r_ev_board_support->call_allow_power_on(false);
                simulator->simData->state = SimState::PLUGGED_IN;
                return true;
            }
            return false;
        };
        commandRegistry->registerCommand("iso_wait_for_stop", 1, iso_wait_for_stop);

        const auto iso_wait_v2g_session_stoppedFunction = [](car_simulatorImpl* const simulator,
                                                             const std::vector<std::string>& arguments) {
            if (simulator->simData->v2g_finished) {
                return true;
            }
            return false;
        };
        commandRegistry->registerCommand("iso_wait_v2g_session_stopped", 0, iso_wait_v2g_session_stoppedFunction);

        const auto iso_pause_chargingFunction = [](car_simulatorImpl* const simulator,
                                                   const std::vector<std::string>& arguments) {
            simulator->mod->r_ev[0]->call_pause_charging();
            simulator->simData->state = SimState::PLUGGED_IN;
            simulator->simData->iso_pwr_ready = false;
            return true;
        };
        commandRegistry->registerCommand("iso_pause_charging", 0, iso_pause_chargingFunction);

        const auto iso_wait_for_resumeFunction = [](car_simulatorImpl* const simulator,
                                                    const std::vector<std::string>& arguments) { return false; };
        commandRegistry->registerCommand("iso_wait_for_resume", 0, iso_wait_for_resumeFunction);

        const auto iso_start_bcb_toggleFunction = [](car_simulatorImpl* const simulator,
                                                     const std::vector<std::string>& arguments) {
            simulator->simData->state = SimState::BCB_TOGGLE;
            if (simulator->simData->bcb_toggles >= std::stoul(arguments[0]) || simulator->simData->bcb_toggles == 3) {
                simulator->simData->bcb_toggles = 0;
                simulator->simData->state = SimState::PLUGGED_IN;
                return true;
            }
            return false;
        };
        commandRegistry->registerCommand("iso_start_bcb_toggle", 1, iso_start_bcb_toggleFunction);
    }

    const auto wait_for_real_pluginFunction = [](car_simulatorImpl* const simulator,
                                                 const std::vector<std::string>& arguments) {
        using types::board_support_common::Event;
        if (simulator->simData->actualBspEvent == Event::A) {
            EVLOG_info << "Real plugin detected";
            simulator->simData->state = SimState::PLUGGED_IN;
            return true;
        }
        return false;
    };
    commandRegistry->registerCommand("wait_for_real_plugin", 0, wait_for_real_pluginFunction);
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
    // Execute sim commands until a command blocks, or we are finished
    std::lock_guard<std::mutex> lock{simDataMutex};
    auto& commandQueue = simData->commandQueue;
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

    carStateMachine();
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
    std::lock_guard<std::mutex> lock{simDataMutex};
    using types::board_support_common::BspEvent;
    mod->r_ev_board_support->subscribe_bsp_event([this](const auto& bsp_event) {
        simData->actualBspEvent = bsp_event.event;
        if (bsp_event.event == types::board_support_common::Event::Disconnected &&
            simData->state == main::SimState::UNPLUGGED) {
            executionActive = false;
        }
    });

    // subscribe bsp_measurement
    using types::board_support_common::BspMeasurement;
    mod->r_ev_board_support->subscribe_bsp_measurement([this](const auto& measurement) {
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
void car_simulatorImpl::resetSimDataToDefaults() {
    std::lock_guard<std::mutex> lock{simDataMutex};
    simData = std::make_unique<SimData>();
}

void car_simulatorImpl::updateCommandQueue(std::string& value) {
    std::lock_guard<std::mutex> lock{simDataMutex};
    auto& commandQueue = simData->commandQueue;
    commandQueue = SimData::parseSimCommands(value, *commandRegistry);
}

} // namespace module::main
