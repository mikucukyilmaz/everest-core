// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "SimCommand.hpp"
#include "RegisteredCommand.hpp"
#include <string>
#include <utility>
#include <vector>
namespace module::main {

SimCommand::SimCommand(std::string&& commandName, std::vector<std::string>&& arguments) :
    arguments{std::move(arguments)}, registeredCommand{RegisteredCommandBase::getRegisteredCommand(commandName)} {
}

SimCommand::SimCommand(const std::string& commandName, const std::vector<std::string>& arguments) :
    arguments{arguments}, registeredCommand{RegisteredCommandBase::getRegisteredCommand(commandName)} {
}

bool SimCommand::execute() const {
    return (*registeredCommand)(arguments);
}

} // namespace module::main
