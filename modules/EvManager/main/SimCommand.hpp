// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#pragma once

#include "RegisteredCommand.hpp"
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
    SimCommand(std::string&& commandName, std::vector<std::string>&& arguments);
    SimCommand(const std::string& commandName, const std::vector<std::string>& arguments);

    bool execute() const;

private:
    std::vector<std::string> arguments;

    const RegisteredCommandBase* registeredCommand;
};

} // namespace module::main
