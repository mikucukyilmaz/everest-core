// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef MAIN_CAR_SIMULATOR_IMPL_HPP
#define MAIN_CAR_SIMULATOR_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/car_simulator/Implementation.hpp>
#include <queue>

#include "../EvManager.hpp"
#include "SimData.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module::main {

struct Conf {};

class car_simulatorImpl : public car_simulatorImplBase {
public:
    car_simulatorImpl() = delete;
    car_simulatorImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<EvManager>& mod, Conf& config) :
        car_simulatorImplBase(ev, "main"), mod(mod), config(config){};

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual void handle_enable(bool& value) override;
    virtual void handle_executeChargingSession(std::string& value) override;

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<EvManager>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    // insert your private definitions here

    void handle_modifyChargingSession(std::string& value);
    bool checkCanExecute();
    void handleSimulationLoop();
    void runSimulationLoop();
    void registerAllCommands();
    void carStateMachine();
    void subscribeToVariablesOnInit();
    void setupEVParameters();
    void callEVBoardSupportFunctions();
    void subscribeToExternalMQTT();
    void resetSimDataToDefaults();
    void updateCommandQueue(std::string& value);

    std::unique_ptr<CommandRegistry> commandRegistry;

    std::mutex simDataMutex;
    std::unique_ptr<SimData> simData;

    bool enabled;
    std::atomic<bool> executionActive{false};
    size_t loopIntervalMs{};

    std::thread simulationThread;
    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace module::main

#endif // MAIN_CAR_SIMULATOR_IMPL_HPP
