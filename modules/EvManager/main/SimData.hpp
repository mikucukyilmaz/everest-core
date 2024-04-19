// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#ifndef EVEREST_CORE_SIMDATA_HPP
#define EVEREST_CORE_SIMDATA_HPP

#include "SimCommand.hpp"
#include "generated/types/board_support_common.hpp"
#include <optional>
#include <queue>
#include <string>
#include <string_view>
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

struct SimData {
    bool executionActive{false};
    size_t loopCurrentCommandIndex{0};

    SimState state{SimState::UNPLUGGED};
    SimState lastState{SimState::UNPLUGGED};
    std::string slacState;

    bool v2g_finished{false};
    bool iso_stopped{false};
    size_t evse_maxcurrent{0};
    size_t maxCurrent{0};
    std::string payment{"ExternalPayment"};
    std::string energymode{"AC_single_phase_core"};
    bool iso_pwr_ready{false};

    size_t bcb_toggles{0};
    bool bcb_toggle_C{true};

    types::board_support_common::Ampacity pp;
    float rcd_current_mA{0.0f};
    float pwm_duty_cycle{0.0f};

    bool dc_power_on{false};
    size_t last_pwm_duty_cycle{0};

    types::board_support_common::Event actualBspEvent{};



    std::queue<SimCommand> commandQueue;
};

std::queue<SimCommand> parseSimCommands(const std::string& value);

std::vector<std::string> convertCommandsStringToVector(const std::string_view& commands);

std::vector<std::pair<std::string, std::vector<std::string>>>
splitIntoCommandsWithArguments(std::vector<std::string>& commandsVector);

std::pair<std::string, std::vector<std::string>> splitIntoCommandWithArguments(std::string& command);

std::queue<SimCommand>
compileCommands(std::vector<std::pair<std::string, std::vector<std::string>>>& commandsWithArguments);

} // namespace module::main

#endif // EVEREST_CORE_SIMDATA_HPP
