// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#pragma once

#include "CommandRegistry.hpp"
#include <cstddef>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace module::main {

using Arguments = std::vector<std::string>;

class SimulationCommand {
public:
    SimulationCommand(const RegisteredCommandBase* registeredCommandIn, const std::vector<std::string>& argumentsIn);

    bool execute() const;

    static std::queue<SimulationCommand> parseSimCommands(const std::string& value,
                                                          const CommandRegistry& registeredCommands);

private:
    using RawCommands = std::vector<std::string>;
    using CommandWithArguments = std::pair<std::string, Arguments>;
    using CommandsWithArguments = std::vector<CommandWithArguments>;

    static RawCommands convertCommandsStringToVector(const std::string& commands);
    static CommandsWithArguments splitIntoCommandsWithArguments(std::vector<std::string>& commandsVector);
    static CommandWithArguments splitIntoCommandWithArguments(std::string& command);
    static std::queue<SimulationCommand> compileCommands(CommandsWithArguments& commandsWithArguments,
                                                         const CommandRegistry& commandRegistry);
    std::vector<std::string> arguments;

    const RegisteredCommandBase* registeredCommand;
};

} // namespace module::main
