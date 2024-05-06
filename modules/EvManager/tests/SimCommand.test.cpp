// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "../main/CarSimulation.hpp"
#include "../main/CommandRegistry.hpp"
#include "../main/SimulationCommand.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <string>
#include <vector>

using namespace module::main;

namespace module::main {
class car_simulatorImpl { /*mock class*/
};

SCENARIO("SimCommands can be created", "[SimCommand]") {
    GIVEN("A command with 0 arguments called testCommand is registered") {
        auto carSimulator = car_simulatorImpl();
        auto carSimulation = CarSimulation<car_simulatorImpl>(&carSimulator);
        const auto commandName = std::string{"testCommand"};
        const auto argumentCount = 0;
        const auto testCommandFunction = [](CarSimulation<car_simulatorImpl>* const /*simulator*/,
                                            const std::vector<std::string>& arguments) { return arguments.empty(); };
        auto commandRegistry = CommandRegistry();
        commandRegistry.registerCommand(&carSimulation, argumentCount, testCommandFunction, commandName);

        WHEN("The SimCommand is created") {
            const auto simCommand = SimulationCommand{&commandRegistry.getRegisteredCommand(commandName), {}};

            THEN("The command can be executed") {
                CHECK(simCommand.execute() == true);
            }
        }

        WHEN("The SimCommand is created with the wrong number of arguments") {
            const auto simCommand = SimulationCommand{&commandRegistry.getRegisteredCommand(commandName), {"arg1"}};

            THEN("The command throws") {
                CHECK_THROWS_WITH(simCommand.execute(), "Invalid number of arguments for: testCommand");
            }
        }
    }
}

SCENARIO("SimCommands can be parsed", "[SimCommand]") {
    GIVEN("A few commands registered and a command string") {
        auto carSimulator = car_simulatorImpl();
        auto carSimulation = CarSimulation<car_simulatorImpl>(&carSimulator);
        auto commandRegistry = CommandRegistry();
        const auto commandNameA = std::string{"commanda"};
        const auto argumentCountA = 0;
        const auto commandFunctionA = [](CarSimulation<car_simulatorImpl>* /*simulator*/, const std::vector<std::string>& arguments) {
            return arguments.empty();
        };
        commandRegistry.registerCommand(&carSimulation, argumentCountA, commandFunctionA, commandNameA);

        const auto commandNameB = std::string{"commandb"};
        const auto argumentCountB = 1;
        const auto commandFunctionB = [](CarSimulation<car_simulatorImpl>* /*simulator*/, const std::vector<std::string>& arguments) {
            return arguments.size() == 1;
        };
        commandRegistry.registerCommand(&carSimulation, argumentCountB, commandFunctionB, commandNameB);

        const auto commandNameC = std::string{"commandc"};
        const auto argumentCountC = 2;
        const auto commandFunctionC = [](CarSimulation<car_simulatorImpl>* /*simulator*/, const std::vector<std::string>& arguments) {
            return arguments.size() == 2;
        };
        commandRegistry.registerCommand(&carSimulation, argumentCountC, commandFunctionC, commandNameC);

        WHEN("A correct command string is to be parsed") {
            const auto commandString = "commandA;commandB 0;commandC abc 0.0";
            auto parsedCommands = SimulationCommand::parseSimCommands(commandString, commandRegistry);

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
            auto parsedCommands = SimulationCommand::parseSimCommands(commandString, commandRegistry);

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
                CHECK_THROWS_WITH(SimulationCommand::parseSimCommands(commandString, commandRegistry),
                                  "Command not found: commandd");
            }
        }

        WHEN("An empty string is to be parsed") {
            const auto commandString = "";
            const auto parsedCommands = SimulationCommand::parseSimCommands(commandString, commandRegistry);

            THEN("It should create an empty queue") {
                CHECK(parsedCommands.empty());
            }
        }
    }
}
} // namespace module::main
