// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#pragma once

#include <cstddef>
#include <everest/logging.hpp>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace module::main {

class car_simulatorImpl;
template <typename Func> class RegisteredCommand;

class RegisteredCommandBase;

class CommandRegistry {
public:
    CommandRegistry(car_simulatorImpl* const simulatorIn) : simulator{simulatorIn} {
    }

    // TODO(MarzellT): maybe we could deduce the argument count from the function signature somehow
    template <typename FunctionT>
    void registerCommand(std::string commandName, size_t argumentCount, FunctionT&& function) {
        registeredCommands.try_emplace(
            commandName, std::make_unique<RegisteredCommand<FunctionT>>(simulator, commandName, argumentCount,
                                                                        std::forward<FunctionT>(function)));
    }

    const RegisteredCommandBase& getRegisteredCommand(const std::string& commandName) const {
        try {
            const auto& registeredCommand = registeredCommands.at(commandName);
            return *registeredCommand.get();
        } catch (const std::out_of_range&) {
            throw std::invalid_argument{"Command not found: " + commandName};
        }
    }

    car_simulatorImpl* getSimulator() const {
        return simulator;
    }

private:
    using RegisteredCommands = std::unordered_map<std::string, std::unique_ptr<RegisteredCommandBase>>;

    car_simulatorImpl* simulator{nullptr};
    RegisteredCommands registeredCommands;
};

class RegisteredCommandBase {
public:
    virtual ~RegisteredCommandBase() = default;
    virtual bool operator()(const std::vector<std::string>& /*arguments*/) const = 0;
};

template <typename FunctionT> class RegisteredCommand : public RegisteredCommandBase {
public:
    RegisteredCommand(car_simulatorImpl* simulatorIn, std::string commandNameIn, std::size_t argumentCountIn,
                      FunctionT functionIn) :
        simulator{simulatorIn},
        commandName{std::move(commandNameIn)},
        argumentCount{argumentCountIn},
        function{std::move(functionIn)} {
    }

    ~RegisteredCommand() override = default;

    bool operator()(const std::vector<std::string>& arguments) const override {
        if (arguments.size() != argumentCount) {
            throw std::invalid_argument{"Invalid number of arguments for: " + commandName};
        }
        return function(simulator, arguments);
    }

private:
    car_simulatorImpl* simulator;
    std::string commandName;
    std::size_t argumentCount;
    FunctionT function;
};
} // namespace module::main