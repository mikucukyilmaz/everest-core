// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#pragma once

#include "Constants.hpp"
#include "EnergyMode.hpp"
#include "SimulationCommand.hpp"
#include "SimulationData.hpp"
#include "generated/types/ev_board_support.hpp"

namespace module::main {

template <typename car_simulatorImplT> class CarSimulation {
public:
    CarSimulation(car_simulatorImplT* simulatorIn) : simulator{simulatorIn}, simData{} {};

    const SimState& getState() const {
        return simData.state;
    }

    void setBspEvent(types::board_support_common::Event event) {
        simData.actualBspEvent = event;
    }

    void setPp(types::board_support_common::Ampacity pp) {
        simData.pp = pp;
    }

    void setRcdCurrent(float rcdCurrent) {
        simData.rcd_current_mA = rcdCurrent;
    }

    void setPwmDutyCycle(float pwmDutyCycle) {
        simData.pwm_duty_cycle = pwmDutyCycle;
    }

    void setSlacState(std::string slacState) {
        simData.slacState = std::move(slacState);
    }

    void setIsoPwrReady(bool isoPwrReady) {
        simData.iso_pwr_ready = isoPwrReady;
    }

    void setEVSEMaxCurrent(size_t evseMaxCurrent) {
        simData.evse_maxcurrent = evseMaxCurrent;
    }

    void setIsoStopped(bool isoStopped) {
        simData.iso_stopped = isoStopped;
    }

    void setV2gFinished(bool v2gFinished) {
        simData.v2g_finished = v2gFinished;
    }

    void setDcPowerOn(bool dcPowerOn) {
        simData.dc_power_on = dcPowerOn;
    }

    void stateMachine() {
        using types::ev_board_support::EvCpState;

        const auto stateHasChanged = simData.state != simData.lastState;
        simData.lastState = simData.state;

        switch (simData.state) {
        case SimState::UNPLUGGED:
            if (stateHasChanged) {

                simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::A);
                simulator->getMod()->r_ev_board_support->call_allow_power_on(false);
                // Wait for physical plugin (ev BSP sees state A on CP and not Disconnected)

                // If we have auto_exec configured, restart simulation when it was unplugged
                EVLOG_info << "Unplug detected, restarting simulation.";
                simData.slacState = "UNMATCHED";
                simulator->getMod()->r_ev[0]->call_stop_charging();
            }
            break;
        case SimState::PLUGGED_IN:
            if (stateHasChanged) {
                simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::B);
                simulator->getMod()->r_ev_board_support->call_allow_power_on(false);
            }
            break;
        case SimState::CHARGING_REGULATED:
            if (stateHasChanged || simData.pwm_duty_cycle != simData.last_pwm_duty_cycle) {
                simData.last_pwm_duty_cycle = simData.pwm_duty_cycle;
                // do not draw power if EVSE paused by stopping PWM
                if (simData.pwm_duty_cycle > 7.0 && simData.pwm_duty_cycle < 97.0) {
                    simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::C);
                } else {
                    simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::B);
                }
            }
            break;
        case SimState::CHARGING_FIXED:
            // Todo(sl): What to do here
            if (stateHasChanged) {
                // Also draw power if EVSE stopped PWM - this is a break the rules simulator->mode to test the charging
                // implementation!
                simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::C);
            }
            break;

        case SimState::ERROR_E:
            if (stateHasChanged) {
                simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::E);
                simulator->getMod()->r_ev_board_support->call_allow_power_on(false);
            }
            break;
        case SimState::DIODE_FAIL:
            if (stateHasChanged) {
                simulator->getMod()->r_ev_board_support->call_diode_fail(true);
                simulator->getMod()->r_ev_board_support->call_allow_power_on(false);
            }
            break;
        case SimState::ISO_POWER_READY:
            if (stateHasChanged) {
                simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::C);
            }
            break;
        case SimState::ISO_CHARGING_REGULATED:
            if (stateHasChanged) {
                simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::C);
            }
            break;
        case SimState::BCB_TOGGLE:
            if (simData.bcb_toggle_C) {
                simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::C);
                simData.bcb_toggle_C = false;
            } else {
                simulator->getMod()->r_ev_board_support->call_set_cp_state(EvCpState::B);
                simData.bcb_toggle_C = true;
                ++simData.bcb_toggles;
            }
            break;
        default:
            simData.state = SimState::UNPLUGGED;
            break;
        }
    };

    static bool sleep(CarSimulation* const carSimulation, const Arguments& arguments) {
        if (!carSimulation->simData.sleepTicksLeft.has_value()) {
            const auto sleepTime = std::stold(arguments[0]);
            const auto sleepTimeMs = sleepTime * 1000;
            carSimulation->simData.sleepTicksLeft =
                static_cast<long long>(sleepTimeMs / carSimulation->simulator->getLoopIntervalMs()) + 1;
        }
        carSimulation->simData.sleepTicksLeft = carSimulation->simData.sleepTicksLeft.value() - 1;
        if (!(carSimulation->simData.sleepTicksLeft > 0)) {
            carSimulation->simData.sleepTicksLeft.reset();
            return true;
        } else {
            return false;
        }
    }
    static bool iec_wait_pwr_ready(CarSimulation* const carSimulation, const module::main::Arguments& arguments) {
        carSimulation->simData.state = SimState::PLUGGED_IN;
        return (carSimulation->simData.pwm_duty_cycle > 7.0f && carSimulation->simData.pwm_duty_cycle < 97.0f);
    }
    static bool iso_wait_pwm_is_running(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.state = SimState::PLUGGED_IN;
        return (carSimulation->simData.pwm_duty_cycle > 4.0f && carSimulation->simData.pwm_duty_cycle < 97.0f);
    }
    static bool draw_power_regulated(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simulator->getMod()->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == THREE_PHASES) {
            carSimulation->simulator->getMod()->r_ev_board_support->call_set_three_phases(true);
        } else {
            carSimulation->simulator->getMod()->r_ev_board_support->call_set_three_phases(false);
        }
        carSimulation->simData.state = SimState::CHARGING_REGULATED;
        return true;
    }
    static bool draw_power_fixed(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simulator->getMod()->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == THREE_PHASES) {
            carSimulation->simulator->getMod()->r_ev_board_support->call_set_three_phases(true);
        } else {
            carSimulation->simulator->getMod()->r_ev_board_support->call_set_three_phases(false);
        }
        carSimulation->simData.state = SimState::CHARGING_FIXED;
        return true;
    }
    static bool pause(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.state = SimState::PLUGGED_IN;
        return true;
    }
    static bool unplug(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.state = SimState::UNPLUGGED;
        return true;
    }
    static bool error_e(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.state = SimState::ERROR_E;
        return true;
    }
    static bool diode_fail(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.state = SimState::DIODE_FAIL;
        return true;
    }
    static bool rcd_current(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.rcd_current_mA = std::stof(arguments[0]);
        return true;
    }
    static bool iso_wait_slac_matched(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.state = SimState::PLUGGED_IN;

        if (carSimulation->simData.slacState == "UNMATCHED") {
            EVLOG_debug << "Slac UNMATCHED";
            if (!carSimulation->simulator->getMod()->r_slac.empty()) {
                EVLOG_debug << "Slac trigger matching";
                carSimulation->simulator->getMod()->r_slac[0]->call_reset();
                carSimulation->simulator->getMod()->r_slac[0]->call_trigger_matching();
                carSimulation->simData.slacState = "TRIGGERED";
            }
        }
        if (carSimulation->simData.slacState == "MATCHED") {
            EVLOG_debug << "Slac Matched";
            return true;
        }
        return false;
    }
    static bool iso_wait_pwr_ready(CarSimulation* const carSimulation, const Arguments& arguments) {
        if (carSimulation->simData.iso_pwr_ready) {
            carSimulation->simData.state = SimState::ISO_POWER_READY;
            return true;
        }
        return false;
    }
    static bool iso_dc_power_on(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.state = SimState::ISO_POWER_READY;
        if (carSimulation->simData.dc_power_on) {
            carSimulation->simData.state = SimState::ISO_CHARGING_REGULATED;
            carSimulation->simulator->getMod()->r_ev_board_support->call_allow_power_on(true);
            return true;
        }
        return false;
    }
    static bool iso_start_v2g_session(CarSimulation* const carSimulation, const Arguments& arguments) {
        const auto& argument = arguments[0];
        if (argument == EnergyMode::AC_SINGLE_PHASE_CORE) {
            carSimulation->simData.energymode = &EnergyMode::AC_SINGLE_PHASE_CORE;
        } else if (argument == EnergyMode::AC_THREE_PHASE_CORE) {
            carSimulation->simData.energymode = &EnergyMode::AC_THREE_PHASE_CORE;
        } else if (argument == EnergyMode::DC_CORE) {
            carSimulation->simData.energymode = &EnergyMode::DC_CORE;
        } else if (argument == EnergyMode::DC_EXTENDED) {
            carSimulation->simData.energymode = &EnergyMode::DC_EXTENDED;
        } else if (argument == EnergyMode::DC_COMBO_CORE) {
            carSimulation->simData.energymode = &EnergyMode::DC_COMBO_CORE;
        } else if (argument == EnergyMode::DC_UNIQUE) {
            carSimulation->simData.energymode = &EnergyMode::DC_UNIQUE;
        } else {
            return false;
        }

        carSimulation->simulator->getMod()->r_ev[0]->call_start_charging(*carSimulation->simData.energymode);
        return true;
    }
    static bool iso_draw_power_regulated(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simulator->getMod()->r_ev_board_support->call_set_ac_max_current(std::stod(arguments[0]));
        if (arguments[1] == THREE_PHASES) {
            carSimulation->simulator->getMod()->r_ev_board_support->call_set_three_phases(true);
        } else {
            carSimulation->simulator->getMod()->r_ev_board_support->call_set_three_phases(false);
        }
        carSimulation->simData.state = SimState::ISO_CHARGING_REGULATED;
        return true;
    }
    static bool iso_stop_charging(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simulator->getMod()->r_ev[0]->call_stop_charging();
        carSimulation->simulator->getMod()->r_ev_board_support->call_allow_power_on(false);
        carSimulation->simData.state = SimState::PLUGGED_IN;
        return true;
    }
    static bool iso_wait_for_stop(CarSimulation* const carSimulation, const Arguments& arguments) {
        if (!carSimulation->simData.sleepTicksLeft.has_value()) {
            carSimulation->simData.sleepTicksLeft =
                std::stoll(arguments[0]) *
                    static_cast<long>(1 / static_cast<float>(carSimulation->simulator->getLoopIntervalMs())) +
                1;
        }
        carSimulation->simData.sleepTicksLeft = carSimulation->simData.sleepTicksLeft.value() - 1;
        if (!carSimulation->simData.sleepTicksLeft > 0) {
            carSimulation->simulator->getMod()->r_ev[0]->call_stop_charging();
            carSimulation->simulator->getMod()->r_ev_board_support->call_allow_power_on(false);
            carSimulation->simData.state = SimState::PLUGGED_IN;
            return true;
        }
        if (carSimulation->simData.iso_stopped) {
            EVLOG_info << "POWER OFF iso stopped";
            carSimulation->simulator->getMod()->r_ev_board_support->call_allow_power_on(false);
            carSimulation->simData.state = SimState::PLUGGED_IN;
            return true;
        }
        return false;
    }
    static bool iso_wait_v2g_session_stopped(CarSimulation* const carSimulation, const Arguments& arguments) {
        if (carSimulation->simData.v2g_finished) {
            return true;
        }
        return false;
    }
    static bool iso_pause_charging(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simulator->getMod()->r_ev[0]->call_pause_charging();
        carSimulation->simData.state = SimState::PLUGGED_IN;
        carSimulation->simData.iso_pwr_ready = false;
        return true;
    }
    static bool iso_wait_for_resume(CarSimulation* const carSimulation, const Arguments& arguments) {
        return false;
    }
    static bool iso_start_bcb_toggle(CarSimulation* const carSimulation, const Arguments& arguments) {
        carSimulation->simData.state = SimState::BCB_TOGGLE;
        if (carSimulation->simData.bcb_toggles >= std::stoul(arguments[0]) || carSimulation->simData.bcb_toggles == 3) {
            carSimulation->simData.bcb_toggles = 0;
            carSimulation->simData.state = SimState::PLUGGED_IN;
            return true;
        }
        return false;
    }
    static bool wait_for_real_plugin(CarSimulation* const carSimulation, const Arguments& arguments) {
        using types::board_support_common::Event;
        if (carSimulation->simData.actualBspEvent == Event::A) {
            EVLOG_info << "Real plugin detected";
            carSimulation->simData.state = SimState::PLUGGED_IN;
            return true;
        }
        return false;
    }

private:
    SimulationData simData;
    car_simulatorImplT* simulator;
};

} // namespace module::main