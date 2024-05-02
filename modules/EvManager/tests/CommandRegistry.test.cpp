// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest

#include "../main/CommandRegistry.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <string>
#include <vector>

namespace module::main {
class car_simulatorImpl { /*mock class*/
};
SCENARIO("Commands can be registered", "[RegisteredCommand]") {
    GIVEN("A command with 0 arguments") {
        auto simulator = car_simulatorImpl();
        auto commandRegistry = CommandRegistry(&simulator);
        const auto commandName = std::string{"testCommand0"};
        const auto argumentCount = 0;
        const auto testCommand0Function = [](car_simulatorImpl* const /*simulator*/,
                                             const std::vector<std::string>& arguments) { return arguments.empty(); };

        WHEN("The command is registered") {
            commandRegistry.registerCommand(commandName, argumentCount, testCommand0Function);

            THEN("The command can be retrieved") {
                const auto& registeredCommand = commandRegistry.getRegisteredCommand(commandName);
                THEN("The command can be executed") {
                    CHECK(registeredCommand({}) == true);
                }
                THEN("The command throws when the number of arguments is invalid") {
                    CHECK(registeredCommand({}) == true);
                    CHECK_THROWS_WITH(registeredCommand({"arg1"}), "Invalid number of arguments for: testCommand0");
                    CHECK_THROWS_WITH(registeredCommand({"arg1", "arg2"}),
                                      "Invalid number of arguments for: testCommand0");
                    CHECK_THROWS_WITH(registeredCommand({"arg1", "arg2", "arg3"}),
                                      "Invalid number of arguments for: testCommand0");
                }
            }
        }
    }

    GIVEN("A command with 1 argument") {
        auto simulator = car_simulatorImpl();
        auto commandRegistry = CommandRegistry(&simulator);
        const auto commandName = std::string{"testCommand1"};
        const auto argumentCount = 1;
        const auto testCommand1Function = [](car_simulatorImpl* const /*simulator*/,
                                             const std::vector<std::string>& arguments) {
            return arguments.size() == 1;
        };

        WHEN("The command is registered") {
            commandRegistry.registerCommand(commandName, argumentCount, testCommand1Function);

            THEN("The command can be retrieved") {
                const auto& registeredCommand = commandRegistry.getRegisteredCommand(commandName);
                THEN("The command can be executed") {
                    CHECK(registeredCommand({"arg1"}) == true);
                }
                THEN("The command throws when the number of arguments is invalid") {
                    CHECK_THROWS_WITH(registeredCommand({}), "Invalid number of arguments for: testCommand1");
                    CHECK_THROWS_WITH(registeredCommand({"arg1", "arg2"}),
                                      "Invalid number of arguments for: testCommand1");
                }
            }
        }
    }

    GIVEN("A command with 2 arguments") {
        auto simulator = car_simulatorImpl();
        auto commandRegistry = CommandRegistry(&simulator);
        const auto commandName = std::string{"testCommand2"};
        const auto argumentCount = 2;
        const auto testCommand2Function = [](car_simulatorImpl* const /*simulator*/,
                                             const std::vector<std::string>& arguments) {
            return arguments.size() == 2;
        };

        WHEN("The command is registered") {
            commandRegistry.registerCommand(commandName, argumentCount, testCommand2Function);

            THEN("The command can be retrieved") {
                const auto& registeredCommand = commandRegistry.getRegisteredCommand(commandName);
                THEN("The command can be executed") {
                    CHECK(registeredCommand({"arg1", "arg2"}) == true);
                }
                THEN("The command throws when the number of arguments is invalid") {
                    CHECK_THROWS_WITH(registeredCommand({}), "Invalid number of arguments for: testCommand2");
                    CHECK_THROWS_WITH(registeredCommand({"arg1"}), "Invalid number of arguments for: testCommand2");
                    CHECK_THROWS_WITH(registeredCommand({"arg1", "arg2", "arg3"}),
                                      "Invalid number of arguments for: testCommand2");
                }
            }
        }
    }
}
} // namespace module::main
