// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "../main/SimData.hpp"
#include "../main/CommandRegistry.hpp"
#include "../main/SimCommand.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include <catch2/catch_test_macros.hpp>
#include <vector>

using namespace module::main;

namespace module::main {
class car_simulatorImpl { /*mock class*/
};

SCENARIO("SimCommands can be parsed", "[SimData]") {
    GIVEN("A few commands registered and a command string") {
        auto carSimulator = car_simulatorImpl();
        auto commandRegistry = CommandRegistry(&carSimulator);
        const auto commandNameA = std::string{"commanda"};
        const auto argumentCountA = 0;
        const auto commandFunctionA = [](car_simulatorImpl* /*simulator*/, const std::vector<std::string>& arguments) {
            return arguments.empty();
        };
        commandRegistry.registerCommand(commandNameA, argumentCountA, commandFunctionA);

        const auto commandNameB = std::string{"commandb"};
        const auto argumentCountB = 1;
        const auto commandFunctionB = [](car_simulatorImpl* /*simulator*/, const std::vector<std::string>& arguments) {
            return arguments.size() == 1;
        };
        commandRegistry.registerCommand(commandNameB, argumentCountB, commandFunctionB);

        const auto commandNameC = std::string{"commandc"};
        const auto argumentCountC = 2;
        const auto commandFunctionC = [](car_simulatorImpl* /*simulator*/, const std::vector<std::string>& arguments) {
            return arguments.size() == 2;
        };
        commandRegistry.registerCommand(commandNameC, argumentCountC, commandFunctionC);

        WHEN("A correct command string is to be parsed") {
            const auto commandString = "commandA;commandB 0;commandC abc 0.0";
            auto parsedCommands = SimData::parseSimCommands(commandString, commandRegistry);

            THEN("A queue of executable SimCommands exists.") {
                CHECK(parsedCommands.front().execute());
                parsedCommands.pop();
                CHECK(parsedCommands.front().execute());
                parsedCommands.pop();
                CHECK(parsedCommands.front().execute());
                parsedCommands.pop();
                CHECK(parsedCommands.empty());
            }

            THEN("A queue of executable SimCommands exists.") {
                CHECK(parsedCommands.front().execute());
                parsedCommands.pop();
                CHECK(parsedCommands.front().execute());
                parsedCommands.pop();
                CHECK(parsedCommands.front().execute());
                parsedCommands.pop();
                CHECK(parsedCommands.empty());
            }
        }

        WHEN("A command string with wrong arguments is to be parsed") {
            const auto commandString = "commandA 1;commandB;commandC abc 0.0 def";
            auto parsedCommands = SimData::parseSimCommands(commandString, commandRegistry);

            THEN("The execution of the commands should fail.") {
                CHECK_THROWS_WITH(parsedCommands.front().execute(), "Invalid number of arguments for: commanda");
                parsedCommands.pop();
                CHECK_THROWS_WITH(parsedCommands.front().execute(), "Invalid number of arguments for: commandb");
                parsedCommands.pop();
                CHECK_THROWS_WITH(parsedCommands.front().execute(), "Invalid number of arguments for: commandc");
                parsedCommands.pop();
                CHECK(parsedCommands.empty());
            }
        }

        WHEN("A command string with unregistered arguments is to be parsed") {
            const auto commandString = "commandd;commande;commandf";

            THEN("Parsing should fail") {
                CHECK_THROWS_WITH(SimData::parseSimCommands(commandString, commandRegistry),
                                  "Command not found: commandd");
            }
        }

        WHEN("An empty string is to be parsed") {
            const auto commandString = "";
            const auto parsedCommands = SimData::parseSimCommands(commandString, commandRegistry);

            THEN("It should create an empty queue") {
                CHECK(parsedCommands.empty());
            }
        }
    }
}
} // namespace module::main
