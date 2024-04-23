// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "../main/SimCommand.hpp"
#include "../main/RegisteredCommand.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <string>
#include <vector>

using namespace module::main;

SCENARIO("SimCommands can be created", "[SimCommand]") {
    GIVEN("A command with 0 arguments called testCommand is registered") {
        const auto commandName = std::string{"testCommand"};
        const auto argumentCount = 0;
        const auto testCommandFunction = [](const std::vector<std::string>& arguments) { return arguments.empty(); };
        RegisteredCommandBase::registerCommand(commandName, testCommandFunction, argumentCount);

        WHEN("The SimCommand is created") {
            const auto simCommand = SimCommand{commandName, {}};

            THEN("The command can be executed") {
                REQUIRE(simCommand.execute() == true);
            }
        }

        WHEN("The SimCommand is created with the wrong number of arguments") {
            const auto simCommand = SimCommand{commandName, {"arg1"}};

            THEN("The command throws") {
                REQUIRE_THROWS_WITH(simCommand.execute(), "Invalid number of arguments");
            }
        }
    }

    WHEN("The SimCommand is created with the wrong commandName") {
        THEN("The command throws") {
            REQUIRE_THROWS_WITH(SimCommand("wrongCommand", {}), "Command not found");
        }
    }
}