// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "SimulationCommand.hpp"
#include "CommandRegistry.hpp"
#include <queue>
#include <string>
#include <vector>
namespace module::main {

SimulationCommand::SimulationCommand(const RegisteredCommandBase* registeredCommandIn, const std::vector<std::string>& arguments) :
    arguments{arguments}, registeredCommand{registeredCommandIn} {
}

bool SimulationCommand::execute() const {
    return (*registeredCommand)(arguments);
}

std::queue<SimulationCommand> SimulationCommand::parseSimCommands(const std::string& value, const CommandRegistry& commandRegistry) {
    auto commandsVector{convertCommandsStringToVector(value)};

    auto commandsWithArguments{splitIntoCommandsWithArguments(commandsVector)};

    return compileCommands(commandsWithArguments, commandRegistry);
}

SimulationCommand::RawCommands SimulationCommand::convertCommandsStringToVector(const std::string& commandsView) {

    auto commands = std::string{commandsView};

    // convert to lower case inplace
    std::transform(commands.begin(), commands.end(), commands.begin(),
                   [](const auto& character) { return std::tolower(character); });

    // replace newlines with semicolons
    std::replace(commands.begin(), commands.end(), '\n', ';');

    // split by semicolons
    std::stringstream commandsStream{commands};
    auto command = std::string{};
    auto commandsVector = std::vector<std::string>{};

    while (std::getline(commandsStream, command, ';')) {
        commandsVector.push_back(command);
    }
    return commandsVector;
}
SimulationCommand::CommandsWithArguments
SimulationCommand::splitIntoCommandsWithArguments(std::vector<std::string>& commandsVector) {
    auto commandsWithArguments = std::vector<std::pair<std::string, std::vector<std::string>>>{};

    for (auto& command : commandsVector) {
        commandsWithArguments.push_back(splitIntoCommandWithArguments(command));
    }
    return commandsWithArguments;
}

SimulationCommand::CommandWithArguments SimulationCommand::splitIntoCommandWithArguments(std::string& command) {
    // replace commas with spaces
    std::replace(command.begin(), command.end(), ',', ' ');

    // get command name and arguments
    auto commandStream = std::stringstream{command};
    auto commandName = std::string{};
    auto argument = std::string{};
    auto arguments = std::vector<std::string>{};

    // get command name
    std::getline(commandStream, commandName, ' ');

    // get arguments
    while (std::getline(commandStream, argument, ' ')) {
        arguments.push_back(argument);
    }

    return {commandName, arguments};
}
std::queue<SimulationCommand> SimulationCommand::compileCommands(CommandsWithArguments& commandsWithArguments,
                                                const CommandRegistry& commandRegistry) {
    auto compiledCommands = std::queue<SimulationCommand>{};

    for (auto& [command, arguments] : commandsWithArguments) {
        compiledCommands.emplace(&commandRegistry.getRegisteredCommand(command), arguments);
    }

    return compiledCommands;
}

} // namespace module::main
