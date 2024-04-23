// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
namespace module::main {

template <typename Func> class RegisteredCommand;

class RegisteredCommandBase {
public:
    RegisteredCommandBase() = default;
    virtual ~RegisteredCommandBase() = default;
    RegisteredCommandBase(const RegisteredCommandBase&) = default;
    RegisteredCommandBase& operator=(const RegisteredCommandBase&) = default;
    RegisteredCommandBase(RegisteredCommandBase&&) = default;
    RegisteredCommandBase& operator=(RegisteredCommandBase&&) = default;

    virtual bool operator()(const std::vector<std::string>& arguments) const = 0;

    // TODO: maybe we could deduce the argument count from the function signature somehow
    template <typename FunctionT>
    static void registerCommand(std::string commandName, FunctionT&& function, size_t argumentCount) {
        registeredCommands.try_emplace(commandName, std::make_unique<RegisteredCommand<FunctionT>>(
                                                        commandName, argumentCount, std::forward<FunctionT>(function)));
    }

    static const RegisteredCommandBase* getRegisteredCommand(const std::string& commandName) {

        try {
            const auto& registeredCommand = registeredCommands.at(commandName);
            return registeredCommand.get();
        } catch (const std::out_of_range&) {
            throw std::invalid_argument{"Command not found"};
        }
    }

private:
    inline static std::unordered_map<std::string, std::unique_ptr<RegisteredCommandBase>> registeredCommands;
};

template <typename FunctionT> class RegisteredCommand : public RegisteredCommandBase {
public:
    RegisteredCommand(std::string commandName, std::size_t argumentCount, FunctionT function) :
        commandName{std::move(commandName)}, argumentCount{argumentCount}, function{std::move(function)} {
    }

    ~RegisteredCommand() override = default;

    bool operator()(const std::vector<std::string>& arguments) const override {
        if (arguments.size() != argumentCount) {
            throw std::invalid_argument{"Invalid number of arguments"};
        }
        return function(arguments);
    }

private:
    std::string commandName;
    std::size_t argumentCount;
    FunctionT function;
};
} // namespace module::main