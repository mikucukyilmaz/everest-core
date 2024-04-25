// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "SimCommand.hpp"
#include "CommandRegistry.hpp"
#include <string>
#include <vector>
namespace module::main {

SimCommand::SimCommand(const RegisteredCommandBase* registeredCommandIn, const std::vector<std::string>& arguments) :
    arguments{arguments}, registeredCommand{registeredCommandIn} {
}

bool SimCommand::execute() const {
    return (*registeredCommand)(arguments);
}
} // namespace module::main
