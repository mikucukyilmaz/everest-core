// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#ifndef TMP_CLION_CLANG_TIDY_SIMCOMMAND_HPP
#define TMP_CLION_CLANG_TIDY_SIMCOMMAND_HPP

#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace module::main {

class RegisteredCommandBase;

class SimCommand {
public:
    SimCommand(std::string commandName, std::vector<std::string> arguments);

    bool execute();

private:
    std::vector<std::string> arguments;

    const RegisteredCommandBase& registeredCommand;
};

} // namespace module::main

#endif // TMP_CLION_CLANG_TIDY_SIMCOMMAND_HPP
