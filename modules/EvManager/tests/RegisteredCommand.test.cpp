// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "../main/RegisteredCommand.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <string>
#include <vector>

using namespace module::main;

SCENARIO("Commands can be registered", "[RegisteredCommand]") {
    GIVEN("A command with 0 arguments") {
        const auto commandName = std::string{"testCommand0"};
        const auto argumentCount = 0;
        const auto testCommand0Function = [](const std::vector<std::string>& arguments) { return arguments.empty(); };

        WHEN("The command is registered") {
            RegisteredCommandBase::registerCommand(commandName, testCommand0Function, argumentCount);

            THEN("The command can be retrieved") {
                const auto* registeredCommand = RegisteredCommandBase::getRegisteredCommand(commandName);
                REQUIRE(registeredCommand != nullptr);
                THEN("The command can be executed") {
                    REQUIRE((*registeredCommand)({}) == true);
                }
                THEN("The command throws when the number of arguments is invalid") {
                    REQUIRE((*registeredCommand)({}) == true);
                    REQUIRE_THROWS_WITH((*registeredCommand)({"arg1"}), "Invalid number of arguments");
                    REQUIRE_THROWS_WITH((*registeredCommand)({"arg1", "arg2"}), "Invalid number of arguments");
                    REQUIRE_THROWS_WITH((*registeredCommand)({"arg1", "arg2", "arg3"}), "Invalid number of arguments");
                }
            }
        }
    }

    GIVEN("A command with 1 argument") {
        const auto commandName = std::string{"testCommand1"};
        const auto argumentCount = 1;
        const auto testCommand1Function = [](const std::vector<std::string>& arguments) {
            return arguments.size() == 1;
        };

        WHEN("The command is registered") {
            RegisteredCommandBase::registerCommand(commandName, testCommand1Function, argumentCount);

            THEN("The command can be retrieved") {
                const auto* registeredCommand = RegisteredCommandBase::getRegisteredCommand(commandName);
                REQUIRE(registeredCommand != nullptr);
                THEN("The command can be executed") {
                    REQUIRE((*registeredCommand)({"arg1"}) == true);
                }
                THEN("The command throws when the number of arguments is invalid") {
                    REQUIRE_THROWS_WITH((*registeredCommand)({}), "Invalid number of arguments");
                    REQUIRE_THROWS_WITH((*registeredCommand)({"arg1", "arg2"}), "Invalid number of arguments");
                }
            }
        }
    }

    GIVEN("A command with 2 arguments") {
        const auto commandName = std::string{"testCommand2"};
        const auto argumentCount = 2;
        const auto testCommand2Function = [](const std::vector<std::string>& arguments) {
            return arguments.size() == 2;
        };

        WHEN("The command is registered") {
            RegisteredCommandBase::registerCommand(commandName, testCommand2Function, argumentCount);

            THEN("The command can be retrieved") {
                const auto* registeredCommand = RegisteredCommandBase::getRegisteredCommand(commandName);
                REQUIRE(registeredCommand != nullptr);
                THEN("The command can be executed") {
                    REQUIRE((*registeredCommand)({"arg1", "arg2"}) == true);
                }
                THEN("The command throws when the number of arguments is invalid") {
                    REQUIRE_THROWS_WITH((*registeredCommand)({}), "Invalid number of arguments");
                    REQUIRE_THROWS_WITH((*registeredCommand)({"arg1"}), "Invalid number of arguments");
                    REQUIRE_THROWS_WITH((*registeredCommand)({"arg1", "arg2", "arg3"}), "Invalid number of arguments");
                }
            }
        }
    }
}
