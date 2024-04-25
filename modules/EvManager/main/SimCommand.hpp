// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#pragma once

#include "CommandRegistry.hpp"
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace module::main {

class SimCommand {
public:
    SimCommand(const RegisteredCommandBase* registeredCommandIn, const std::vector<std::string>& arguments);

    bool execute() const;

private:
    std::vector<std::string> arguments;

    const RegisteredCommandBase* registeredCommand;
};

} // namespace module::main
