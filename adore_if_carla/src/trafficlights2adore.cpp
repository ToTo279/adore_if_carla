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

/**
 * This nodes ...
 */

namespace adore
{
    namespace adore_if_carla
    {
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
                initROSConnections();
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
            ros::Publisher publisher_spatem_;
            ros::Publisher publisher_mapem_sim_;
            ros::Publisher publisher_mapem_;
            
            ros::NodeHandle* n_;

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
            struct CarlaTrafficLightInfo
            {
                uint32_t id;
                geometry_msgs::Pose transform;
                carla_msgs::CarlaBoundingBox trigger_volume;    
            };
            
            std::vector<CarlaTrafficLightStatus> traffic_lights_status;
            std::vector<CarlaTrafficLightInfo> traffic_lights_info;

            

            void initSim()
            {
            }
            ros::NodeHandle* getRosNodeHandle()
            {
                return n_;
            }
            void initROSConnections()
            {
                publisher_spatem_sim_ = getRosNodeHandle()->advertise<adore_v2x_sim::SimSPATEM>("/SIM/v2x/SPATEM", 1);
                publisher_spatem_ = getRosNodeHandle()->advertise<dsrc_v2_dsrc::SPAT>("v2x/incoming/SPATEM", 1);
                publisher_mapem_sim_ = getRosNodeHandle()->advertise<adore_v2x_sim::SimMAPEM>("/SIM/v2x/MAPEM", 1);
                publisher_mapem_ = getRosNodeHandle()->advertise<dsrc_v2_mapem_pdu_descriptions::MAPEM>("v2x/incoming/MAPEM", 1);

            }


            void receiveTrafficLightsStatusList(carla_msgs::CarlaTrafficLightStatusList carla_traffic_light_status_list_)
            {
                //traffic_lights_status.clear();

                for (const auto& carla_traffic_light_status : carla_traffic_light_status_list_.traffic_lights)
                {
                    adore::adore_if_carla::Trafficlights2Adore::CarlaTrafficLightStatus status;
                    status.id = carla_traffic_light_status.id;
                    status.state = carla_traffic_light_status.state;

                    traffic_lights_status.push_back(status);
                }
            }

            void receiveTrafficLightsStatus(carla_msgs::CarlaTrafficLightStatus carla_traffic_light_status_)
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
            }

            void sendSPATEMSim()
            {
                dsrc_v2_spatem_pdu_descriptions::SPATEM out_msg;

            }
            void sendSPATEM()
            {
                dsrc_v2_dsrc::SPAT out_msg;

            }
            void sendMAPEMSim()
            {
                dsrc_v2_mapem_pdu_descriptions::MAPEM out_msg;

            }
            void sendMAPEM()
            {
                dsrc_v2_mapem_pdu_descriptions::MAPEM out_msg;
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
