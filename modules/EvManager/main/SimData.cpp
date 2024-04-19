// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "SimData.hpp"
#include <algorithm>
#include <sstream>

namespace module::main {

std::queue<SimCommand> parseSimCommands(const std::string& commands) {
    auto commandsVector{convertCommandsStringToVector(commands)};

    auto commandsWithArguments{splitIntoCommandsWithArguments(commandsVector)};

    return compileCommands(commandsWithArguments);
}

std::vector<std::string> convertCommandsStringToVector(const std::string_view& commandsView) {

    auto commands = std::string{commandsView};

    // convert to lower case inplace
    std::transform(commands.begin(), commands.end(), commands.begin(),
                   [](unsigned char character) { return std::tolower(character); });

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
std::vector<std::pair<std::string, std::vector<std::string>>>
splitIntoCommandsWithArguments(std::vector<std::string>& commandsVector) {
    auto commandsWithArguments = std::vector<std::pair<std::string, std::vector<std::string>>>{};

    for (auto& command : commandsVector) {
        commandsWithArguments.push_back(splitIntoCommandWithArguments(command));
    }
    return commandsWithArguments;
}

std::pair<std::string, std::vector<std::string>> splitIntoCommandWithArguments(std::string& command) {
    // replace commas with spaces
    std::replace(command.begin(), command.end(), ',', ' ');

    // get commandName name and arguments
    auto commandStream = std::stringstream{command};
    auto commandName = std::string{};
    auto argument = std::string{};
    auto arguments = std::vector<std::string>{};

    // get commandName name
    std::getline(commandStream, commandName, ' ');

    // get arguments
    while (std::getline(commandStream, argument, ' ')) {
        arguments.push_back(argument);
    }

    return {commandName, arguments};
}
std::queue<SimCommand>
compileCommands(std::vector<std::pair<std::string, std::vector<std::string>>>& commandsWithArguments) {
    auto compiledCommands = std::queue<SimCommand>{};
    for (auto& [command, arguments] : commandsWithArguments) {
        compiledCommands.emplace(command, arguments);
    }
    return compiledCommands;
}
} // namespace module::main
