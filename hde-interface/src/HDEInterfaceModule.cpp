/*
 * Copyright (c) 2018, <copyright holder> <email>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "HDEInterfaceModule.h"

double HDEInterfaceModule::getPeriod()
{
    return 0.01;
}

HDEInterfaceModule::HDEInterfaceModule(): iWrapper(0) {}

HDEInterfaceModule::~HDEInterfaceModule()
{
    if(iWrapper)
    {
        iWrapper->detachAll();
        iWrapper = 0;
    }
    if(wrapper.isValid())
    {
        wrapper.close();
    }
    
    if(hde_controlboard_driver.isValid())
    {
        hde_controlboard_driver.close();
    }
    
    yarp::os::Network::fini();
}
    
bool HDEInterfaceModule::configure(yarp::os::ResourceFinder& rf)
{
    if(!yarp::os::Network::initialized())
    {
        yarp::os::Network::init();
    }
    
    std::string rpc_port_name = rf.find("name").asString();
    
    if(hde_interface_rpc_port.open(rpc_port_name+"/rpc:i"))
    {
        attach(hde_interface_rpc_port);
    }
    
    if(state_port.open(rpc_port_name+"/state:i"))
    {
        if(!yarp::os::Network::connect("/human-state-provider/state:o",state_port.getName().c_str()))
        {
            yError() << "HDEInterfaceModule: Failed to connect /human-state-provider/state:o and /hde-interface/state:i ports";
            return false;
        }
    }
    else
    {
        yError() << "HDEInterfaceModule: Failed to open /hde-interface/state:i port";
        return false;
    }
    
    if(forces_port.open(rpc_port_name+"/forces:i"))
    {
        if(!yarp::os::Network::connect("/human-forces-provider/forces:o",forces_port.getName().c_str()))
        {
            yError() << "HDEInterfaceModule: Failed to connect /human-forces-provider/forces:o and /hde-interface/forces:i ports";
            return false;
        }
    }
    else
    {
        yError() << "HDEInterfaceModule: Failed to open /hde-interface/forces:i port";
        return false;
    }
    
    if(dynamics_port.open(rpc_port_name+"/dynamicsEstimation:i"))
    {
        if(!yarp::os::Network::connect("/human-dynamics-estimator/dynamicsEstimation:o",dynamics_port.getName().c_str()))
        {
            yError() << "HDEInterfaceModule: Failed to connect /human-dynamics-estimator/dynamicsEstimation:o and /hde-interface/dynamicsEstimation:i ports";
            return false;
        }
    }
    else
    {
        yError() << "HDEInterfaceModule: Failed to open /hde-interface/dynamicsEstimation:i port";
        return false;
    }
    
    yarp::dev::Drivers::factory().add(new yarp::dev::DriverCreatorOf<yarp::dev::HDEControlBoardDriver>("hde_controlboard", "controlboardwrapper2", "HDEControlBoardDriver"));
    
    std::string device_name = rf.find("device").asString();
    driver_parameters.put("device",device_name);
    hde_controlboard_driver.open(driver_parameters);
    
    yarp::dev::HDEControlBoardDriver* hde_controlboard_driver_ptr = dynamic_cast<yarp::dev::HDEControlBoardDriver*>(hde_controlboard_driver.getImplementation());
    
    hde_controlboard_driver_ptr->number_of_dofs = rf.find("joints").asInt();

    hde_controlboard_driver_ptr->joint_positions_rad.resize(hde_controlboard_driver_ptr->number_of_dofs);
    hde_controlboard_driver_ptr->joint_velocities_rad.resize(hde_controlboard_driver_ptr->number_of_dofs);
    hde_controlboard_driver_ptr->joint_accelerations_rad.resize(hde_controlboard_driver_ptr->number_of_dofs);

    hde_controlboard_driver_ptr->joint_positions.resize(hde_controlboard_driver_ptr->number_of_dofs);
    hde_controlboard_driver_ptr->joint_velocities.resize(hde_controlboard_driver_ptr->number_of_dofs);
    hde_controlboard_driver_ptr->joint_accelerations.resize(hde_controlboard_driver_ptr->number_of_dofs);
    hde_controlboard_driver_ptr->joint_torques.resize(hde_controlboard_driver_ptr->number_of_dofs);
    
    
    yarp::os::Bottle joints = rf.findGroup("joint_name_list");
    
    if(!joints.check("joint_name_list"))
    {
        yError() << "HDEInterfaceModule: Failed to read joints name list";
        return false;
    }
    else
    {
        if(joints.size()-1 != hde_controlboard_driver_ptr->number_of_dofs)
        {
            yError() << "HDEInterfaceModule: mismatch between the joints number and joint name list size from the config file";
            return false;
            
        }
        else
        {
            hde_controlboard_driver_ptr->joint_name_list.resize(hde_controlboard_driver_ptr->number_of_dofs);
            for(int i=0; i < hde_controlboard_driver_ptr->number_of_dofs; i++)
            {
                hde_controlboard_driver_ptr->joint_name_list.at(i) = joints.get(i+1).asString();
            }
        }
    }
    
    wrapper_properties = rf.findGroup("WRAPPER");    
    wrapper.open(wrapper_properties);

    if(!wrapper.view(iWrapper))
    {
        yError() << "HDEInterfaceModule: Error while loading the wrapper";
        return false;
    }
    
    driver_list.push(&hde_controlboard_driver,"HDE");
    
    if(!iWrapper->attachAll(driver_list))
    {
        yError() << "HDEInterfaceModule: Error while attaching the device to the wrapper interface";
        return false;
    }
    
    return true;
}
    
bool HDEInterfaceModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    if (command.get(0).asString()=="quit")
        return false;
    else
        reply=command;
    
    return true;
}

bool HDEInterfaceModule::updateModule()
{    
    yarp::dev::HDEControlBoardDriver* hde_controlboard_driver_ptr = dynamic_cast<yarp::dev::HDEControlBoardDriver*>(hde_controlboard_driver.getImplementation());
    
    //Human-state-provider
    human::HumanState *input_state = state_port.read();
    
    if(input_state->positions.size() == hde_controlboard_driver_ptr->number_of_dofs)
    {
        hde_controlboard_driver_ptr->joint_positions_rad = input_state->positions;
        //yInfo() << "Joint Positions: " << hde_controlboard_driver_ptr->joint_positions.toString();
        
        hde_controlboard_driver_ptr->joint_velocities_rad = input_state->velocities;
        //yInfo() << "Joint Velocities:: " << hde_controlboard_driver_ptr->joint_velocities.toString();
    }
    else
    {
        yError() << "HDEInterfaceModule: DoFs mismatch between the config file and human state port";
        return false;
    }
    
    //Human-dynamics-estimation
    human::HumanDynamics *input_dynamics = dynamics_port.read();
    
    std::vector<human::JointDynamicsEstimation> input_joint_dynamics = input_dynamics->jointVariables;
    
    if(input_joint_dynamics.size() == hde_controlboard_driver_ptr->number_of_dofs)
    {
        for(int j = 0; j < input_joint_dynamics.size(); j++)
        {
            if(input_joint_dynamics.at(j).jointName == hde_controlboard_driver_ptr->joint_name_list.at(j))
            {
                double joint_acceleration = input_joint_dynamics.at(j).acceleration[0];
                hde_controlboard_driver_ptr->joint_accelerations_rad[j] = joint_acceleration;
                
                
                double joint_torque = input_joint_dynamics.at(j).torque[0];
                hde_controlboard_driver_ptr->joint_torques[j] = joint_torque;
                
            }
            else
            {
                yError() << "HDEInterfaceModule: Joint name mismatch while getting jonit torques";
                return false;
            }
        }
    }
    else
    {
        yError() << "HDEInterfaceModule: DoFs mismatch between config file and human joint dynamics";
        return false;
    }

    for(int j=1; j <= hde_controlboard_driver_ptr->number_of_dofs; j++)
    {
        hde_controlboard_driver_ptr->joint_positions[j] = hde_controlboard_driver_ptr->joint_positions_rad[j]*(180/M_PI);
        hde_controlboard_driver_ptr->joint_velocities[j] = hde_controlboard_driver_ptr->joint_velocities_rad[j]*(180/M_PI);
        hde_controlboard_driver_ptr->joint_accelerations[j] = hde_controlboard_driver_ptr->joint_accelerations_rad[j]*(180/M_PI);
    }
    
    //yInfo() << "Joint Accelerations: " << hde_controlboard_driver_ptr->joint_accelerations.toString();
    //yInfo() << "Joint Torques: " << hde_controlboard_driver_ptr->joint_torques.toString();
    
    return true;
}

 bool HDEInterfaceModule::interruptModule()
 {
     yInfo() << "HDEInterfaceModule: Interrupting module for port cleanup";
     return true;
}

bool HDEInterfaceModule::close()
{
    yInfo() << "HDEInterfaceModule: Calling close function";
    
    yarp::os::Network::disconnect("/human-state-provider/state:o",state_port.getName().c_str());
    yarp::os::Network::disconnect("/human-forces-provider/forces:o",forces_port.getName().c_str());
    yarp::os::Network::disconnect("/human-dynamics-estimator/dynamicsEstimation:o",dynamics_port.getName().c_str());
    
    hde_interface_rpc_port.close();
    return true;
}
