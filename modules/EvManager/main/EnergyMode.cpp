// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "EnergyMode.hpp"
#include <algorithm>
#include <stdexcept>
#include <string>
#include <string_view>

namespace module::main {

const EnergyMode EnergyMode::AC_SINGLE_PHASE_CORE{"AC_single_phase_core"};
const EnergyMode EnergyMode::AC_THREE_PHASE_CORE{"AC_three_phase_core"};
const EnergyMode EnergyMode::DC_CORE{"DC_core"};
const EnergyMode EnergyMode::DC_EXTENDED{"DC_extended"};
const EnergyMode EnergyMode::DC_COMBO_CORE{"DC_combo_core"};
const EnergyMode EnergyMode::DC_UNIQUE{"DC_unique"};

EnergyMode::EnergyMode(std::string valueIn) : value(valueIn) {
}

EnergyMode::operator std::string() const {
    return value;
}

std::string EnergyMode::fromString(std::string_view value) {
    if (value == "AC_single_phase_core") {
        return "ac_single_phase_core";
    }
    if (value == "AC_three_phase_core") {
        return "ac_three_phase_core";
    }
    if (value == "DC_core") {
        return "dc_core";
    }
    if (value == "DC_extended") {
        return "dc_extended";
    }
    if (value == "DC_combo_core") {
        return "dc_combo_core";
    }
    if (value == "DC_unique") {
        return "dc_unique";
    }
    throw std::invalid_argument("Invalid EnergyMode value");
}

bool operator==(std::string_view lhs, const EnergyMode& rhs) {
    const auto toLower = [](const auto& character) { return std::tolower(character); };
    std::string lowerValue = rhs.value;
    std::transform(lowerValue.begin(), lowerValue.end(), lowerValue.begin(), toLower);
    return lowerValue == lhs;
}

bool operator==(const EnergyMode& lhs, const EnergyMode& rhs) {
    return &lhs == &rhs;
}
} // namespace module::main