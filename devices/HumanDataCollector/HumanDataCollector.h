/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HDE_DEVICES_HUMANDATACOLLECTOR
#define HDE_DEVICES_HUMANDATACOLLECTOR

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/PeriodicThread.h>

#include <memory>

namespace hde {
    namespace devices {
        class HumanDataCollector;
    } // namespace devices
} // namespace hde

class hde::devices::HumanDataCollector final
        : public yarp::dev::DeviceDriver
        , public yarp::dev::IWrapper
        , public yarp::dev::IMultipleWrapper
        , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanDataCollector();
    ~HumanDataCollector() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;

    // IWrapper interface
    bool attach(yarp::dev::PolyDriver* poly) override;
    bool detach() override;

    // IMultipleWrapper interface
    bool attachAll(const yarp::dev::PolyDriverList& driverList) override;
    bool detachAll() override;
};

#endif // HDE_DEVICES_HUMANDATACOLLECTOR
