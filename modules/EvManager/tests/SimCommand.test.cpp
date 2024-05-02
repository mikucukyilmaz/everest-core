// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "../main/SimCommand.hpp"
#include "../main/CommandRegistry.hpp"
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
        const auto commandName = std::string{"testCommand"};
        const auto argumentCount = 0;
        const auto testCommandFunction = [](car_simulatorImpl* const /*simulator*/,
                                            const std::vector<std::string>& arguments) { return arguments.empty(); };
        auto commandRegistry = CommandRegistry(&carSimulator);
        commandRegistry.registerCommand(commandName, argumentCount, testCommandFunction);

        WHEN("The SimCommand is created") {
            const auto simCommand = SimCommand{&commandRegistry.getRegisteredCommand(commandName), {}};

            THEN("The command can be executed") {
                CHECK(simCommand.execute() == true);
            }
        }

        WHEN("The SimCommand is created with the wrong number of arguments") {
            const auto simCommand = SimCommand{&commandRegistry.getRegisteredCommand(commandName), {"arg1"}};

            THEN("The command throws") {
                CHECK_THROWS_WITH(simCommand.execute(), "Invalid number of arguments for: testCommand");
            }
        }
    }
}
} // namespace module::main
