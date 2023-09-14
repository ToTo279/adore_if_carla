/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/

#include <iostream>
#include <thread>
#include <cstdlib>
#include <ros/ros.h>
//#include <std_msgs/Float64.h>
//#include <rosgraph_msgs/Clock.h>

#include <dsrc_v2_mapem_pdu_descriptions/MAPEM.h>
#include <dsrc_v2_spatem_pdu_descriptions/SPATEM.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include <carla_msgs/CarlaTrafficLightStatus.h>
#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_msgs/CarlaTrafficLightInfo.h>
#include <carla_msgs/CarlaBoundingBox.h>
#include <geometry_msgs/Pose.h>
#include <adore_v2x_sim/SimMAPEM.h>
#include <adore_v2x_sim/SimSPATEM.h>
//#include <geometry_msgs/Twist.h>
//#include <dsrc_v2_dsrc/MapData.h>
#include <adore_if_ros_msg/TCDConnectionStateTrace.h>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>

/**
 * This nodes ...
 */

namespace adore
{
    namespace adore_if_carla
    {

        enum Betriebsmodus
        {
            V2XSIM,
            V2XDIRECT,
            DIRECT
        };

        class Trafficlights2Adore
        {
          public:
            Trafficlights2Adore()
            {
            }

            void init(int argc, char** argv, double rate, std::string nodename)
            {
                // Although the application has no periodically called functions, the rate is required for scheduling
                ros::init(argc, argv, nodename);
                ros::NodeHandle* n = new ros::NodeHandle();
                n_ = n;
                initSim();
                bool carla_namespace_specified = n_->getParam("PARAMS/adore_if_carla/carla_namespace", namespace_carla_);
                std::cout << "Objects2Adore: namespace of the carla vehicle is: "
                          << (carla_namespace_specified ? namespace_carla_ : "NOT SPECIFIED") << std::endl;
                initROSConnections();
                getConnections();
            }

            void run()
            {
                while (n_->ok())
                {
                    ros::spin();
                }
            }

          private:
            ros::Publisher publisher_spatem_sim_;
            ros::Publisher publisher_mapem_sim_;
            ros::Publisher publisher_spatem_;
            ros::Publisher publisher_mapem_;
            ros::Publisher publisher_direct_;
            ros::Subscriber subscriber_traffic_lights_status_;
            //ros::Subscriber subscriber_traffic_lights_info_;

            std::string namespace_carla_;
            ros::NodeHandle* n_;

            std::multimap<uint32_t, adore_if_ros_msg::Connection_<std::allocator<void>>> Connections;
            Betriebsmodus betriebsmodus = DIRECT;
            int discard_age = 1;    //in [sec]
            //double t_;

            struct CarlaTrafficLightStatus
            {
                uint32_t id;
                uint8_t state;

                    /*
                    uint8 RED=0
                    uint8 YELLOW=1
                    uint8 GREEN=2
                    uint8 OFF=3
                    uint8 UNKNOWN=4
                    */
            };
            /*struct CarlaTrafficLightInfo
            {
                uint32_t id;
                geometry_msgs::Pose transform;
                carla_msgs::CarlaBoundingBox trigger_volume;    
            };*/
            
            std::vector<CarlaTrafficLightStatus> traffic_lights_status;
            std::vector<CarlaTrafficLightStatus> traffic_lights_status_old;
            //std::vector<CarlaTrafficLightInfo> traffic_lights_info;
            //std::vector<CarlaTrafficLightInfo> traffic_lights_info_old;

            void initSim()
            {
            }

            ros::NodeHandle* getRosNodeHandle()
            {
                return n_;
            }

            void initROSConnections()
            {
                subscriber_traffic_lights_status_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaTrafficLightStatusList>(
                    "/carla/traffic_lights/status", 1, &Trafficlights2Adore::receiveTrafficLightsStatusList, this);
                /*subscriber_traffic_lights_info_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaTrafficLightInfoList>(
                    "/carla/traffic_lights/info", 1, &Trafficlights2Adore::receiveTrafficLightsInfoList, this);*/
                publisher_spatem_sim_ = getRosNodeHandle()->advertise<adore_v2x_sim::SimSPATEM>("/SIM/v2x/SPATEM", 1);
                publisher_mapem_sim_ = getRosNodeHandle()->advertise<adore_v2x_sim::SimMAPEM>("/SIM/v2x/MAPEM", 1);
                publisher_spatem_ = getRosNodeHandle()->advertise<dsrc_v2_spatem_pdu_descriptions::SPATEM>("v2x/incoming/SPATEM", 1);
                publisher_mapem_ = getRosNodeHandle()->advertise<dsrc_v2_mapem_pdu_descriptions::MAPEM>("v2x/incoming/MAPEM", 1);
                publisher_direct_ = getRosNodeHandle()->advertise<adore_if_ros_msg::TCDConnectionStateTrace>("ENV/tcd", 500, true);
            }

            /*double getTime()
            {
                double time = 0;

                using namespace std::chrono;
                uint64_t ms = system_clock::now().time_since_epoch().count();
                time = ((double)ms) / 1000;//000000;

                return time;   
            }*/

            void getConnections()
            {
                std::ifstream inputFile("trafficlightsConnections.txt");

                std::string line;
                while (std::getline(inputFile, line)) {
                    std::istringstream iss(line);
                    std::string ampelIDStr;
                    adore_if_ros_msg::Connection_<std::allocator<void>> connection;
                    
                    if (iss >> ampelIDStr >> connection.first.x >> connection.first.y >> connection.first.z >> connection.last.x >> connection.last.y >> connection.last.z)
                    {
                        uint32_t ampelID = std::stoul(ampelIDStr);
                        Connections.insert(std::make_pair(ampelID, connection));
                    }
                }
                
                inputFile.close(); 
            }

            void receiveTrafficLightsStatusList(carla_msgs::CarlaTrafficLightStatusList carla_traffic_light_status_list_)
            {
                traffic_lights_status_old.clear();
                traffic_lights_status_old = traffic_lights_status;
                traffic_lights_status.clear();

                for (const auto& carla_traffic_light_status : carla_traffic_light_status_list_.traffic_lights)
                {
                    adore::adore_if_carla::Trafficlights2Adore::CarlaTrafficLightStatus status;
                    status.id = carla_traffic_light_status.id;
                    status.state = carla_traffic_light_status.state;

                    traffic_lights_status.push_back(status);
                }
                if (betriebsmodus == V2XSIM)
                {
                    sendSPATEMSim();
                    sendMAPEMSim();
                }
                else if (betriebsmodus == V2XDIRECT)
                {
                    sendSPATEM();
                    sendMAPEM();
                }
                else if (betriebsmodus == DIRECT)
                {
                    sendDirect();
                }
            }

            /*void receiveTrafficLightsStatus(carla_msgs::CarlaTrafficLightStatus carla_traffic_light_status_)
            {
                for (int i = 0; i < traffic_lights_status.size(); ++i)
                {
                    if (traffic_lights_status[i].id == carla_traffic_light_status_.id)
                    {
                        traffic_lights_status[i].state = carla_traffic_light_status_.state;
                        break;
                    }
                }
            }

            void receiveTrafficLightsInfoList(carla_msgs::CarlaTrafficLightInfoList carla_traffic_light_info_list_)
            {
                traffic_lights_info_old.clear();
                traffic_lights_info_old = traffic_lights_info;
                traffic_lights_info.clear();

                for (const auto& carla_traffic_light_info : carla_traffic_light_info_list_.traffic_lights)
                {
                    adore::adore_if_carla::Trafficlights2Adore::CarlaTrafficLightInfo info;
                    info.id = carla_traffic_light_info.id;
                    info.transform = carla_traffic_light_info.transform;
                    info.trigger_volume = carla_traffic_light_info.trigger_volume;

                    traffic_lights_info.push_back(info);
                }
            }

            void receiveTrafficLightsInfo(carla_msgs::CarlaTrafficLightInfo carla_traffic_light_info_)
            {
                for (int i = 0; i < traffic_lights_info.size(); ++i)
                {
                    if (traffic_lights_info[i].id == carla_traffic_light_info_.id)
                    {
                        traffic_lights_info[i].transform = carla_traffic_light_info_.transform;
                        traffic_lights_info[i].trigger_volume = carla_traffic_light_info_.trigger_volume;
                        break;
                    }
                }
            }*/

            void sendDirect()
            {
                adore_if_ros_msg::TCDConnectionStateTrace out_msg;

                for (const auto& entry : Connections)
                {
                    uint32_t targetAmpelID = entry.first;
                    const adore_if_ros_msg::Connection_<std::allocator<void>>& connection = entry.second;
                    adore_if_ros_msg::TCDConnectionState_<std::allocator<void>> newState;

                    

                    out_msg.connection.first = connection.first;
                    out_msg.connection.last = connection.last;

                    /*out_msg.connection.first = entry.second.first;
                    out_msg.connection.last = entry.second.last;*/

                    uint8_t state = 0;  
                    for (const auto& status : traffic_lights_status) {
                        if (status.id == targetAmpelID) {
                            newState.state = static_cast<int8_t>(state); 
                            break;  
                        }
                    }

                    /*out_msg.data.minEndTime = 
                    out_msg.data.maxEndTime = 
                    out_msg.data.likelyTime =*/

                    out_msg.data.push_back(newState);

                    publisher_direct_.publish(out_msg);
                }
            }


            void sendSPATEMSim()
            {
                adore_v2x_sim::SimSPATEM out_msg;
                /*  TO DO
                out_msg.meta =
                out_msg.data ENTSPRICHT dsrc_v2_spatem_pdu_descriptions::SPATEM
                */

                publisher_spatem_sim_.publish(out_msg);
            }
            void sendSPATEM()
            {
                dsrc_v2_spatem_pdu_descriptions::SPATEM out_msg;

                /*  TO DO
                out_msg.header = 
                out_msg.spat.timeStamp = 
                out_msg.spat.name = 
                out_msg.spat.intersections = 
                */

                publisher_spatem_.publish(out_msg);
            }
            void sendMAPEMSim()
            {
                adore_v2x_sim::SimMAPEM out_msg;
                /*  TO DO
                out_msg.meta =
                out_msg.data ENTSPRICHT dsrc_v2_mapem_pdu_descriptions::MAPEM
                */

                publisher_mapem_sim_.publish(out_msg);
            }
            void sendMAPEM()
            {
                dsrc_v2_mapem_pdu_descriptions::MAPEM out_msg;
                /*  TO DO
                out_msg.header = 
                out_msg.map.timeStamp =
                out_msg.map.msgIssueRevision =
                out_msg.map.layerType =
                out_msg.map.layerID =
                out_msg.map.intersections =
                out_msg.map.roadSegments =
                out_msg.map.dataParameters =
                out_msg.map.restrictionList =
                */

                publisher_mapem_.publish(out_msg);
            }
        };
    }  // namespace adore_if_carla
}  // namespace adore

int main(int argc, char** argv)
{
    adore::adore_if_carla::Trafficlights2Adore trafficlights2adore;
    trafficlights2adore.init(argc, argv, 10.0, "trafficlights2adore");
    trafficlights2adore.run();
    return 0;
}
