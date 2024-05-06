// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#pragma once

#include "CommandRegistry.hpp"
#include "EnergyMode.hpp"
#include "SimulationCommand.hpp"
#include <generated/types/board_support_common.hpp>
#include <optional>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace module::main {

enum class SimState {
    UNPLUGGED,
    PLUGGED_IN,
    CHARGING_REGULATED,
    CHARGING_FIXED,
    ERROR_E,
    DIODE_FAIL,
    ISO_POWER_READY,
    ISO_CHARGING_REGULATED,
    BCB_TOGGLE,
};

struct SimulationData {
    SimState state{SimState::UNPLUGGED};
    SimState lastState{SimState::UNPLUGGED};
    std::string slacState;
    std::optional<size_t> sleepTicksLeft{};

    bool v2g_finished{false};
    bool iso_stopped{false};
    size_t evse_maxcurrent{0};
    size_t maxCurrent{0};
    std::string payment{"ExternalPayment"};
    // pointer to const EnergyMode
    EnergyMode const* energymode = &EnergyMode::AC_SINGLE_PHASE_CORE;
    bool iso_pwr_ready{false};

    size_t bcb_toggles{0};
    bool bcb_toggle_C{true};

    types::board_support_common::Ampacity pp;
    float rcd_current_mA{0.0f};
    float pwm_duty_cycle{0.0f};
    float last_pwm_duty_cycle{0.0f};

    bool dc_power_on{false};

    types::board_support_common::Event actualBspEvent{};
};

} // namespace module::main
