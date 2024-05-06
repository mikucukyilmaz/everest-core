// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#pragma once

#include <string>
#include <string_view>

namespace module::main {

class EnergyMode {
public:
    operator std::string() const;

    const static EnergyMode AC_SINGLE_PHASE_CORE;
    const static EnergyMode AC_THREE_PHASE_CORE;
    const static EnergyMode DC_CORE;
    const static EnergyMode DC_EXTENDED;
    const static EnergyMode DC_COMBO_CORE;
    const static EnergyMode DC_UNIQUE;

private:
    EnergyMode() = default;
    explicit EnergyMode(std::string valueIn);
    friend bool operator==(std::string_view lhs, const EnergyMode& rhs);
    friend bool operator==(const EnergyMode& lhs, const EnergyMode& rhs);

    static std::string fromString(std::string_view value);
    const std::string value;
};

} // namespace module::main