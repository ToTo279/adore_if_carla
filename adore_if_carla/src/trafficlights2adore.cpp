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

#include <dsrc_v2_mapem_pdu_descriptions/MAPEM.h>   //  Noch CMake anpassen!!!
#include <dsrc_v2_spatem_pdu_descriptions/SPATEM.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
                CarlaBoundingBox trigger_volume;    // !!!
            };
            
            CarlaTrafficLightStatus traffic_lights_status[];
            CarlaTrafficLightInfo traffic_lights_info[];
            

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
                publisher_spatem_ = getRosNodeHandle()->advertise<dsrc_v2_dsrc::SPATEM>("v2x/incoming/SPATEM", 1);
                publisher_mapem_sim_ = getRosNodeHandle()->advertise<adore_v2x_sim::SimMAPEM>("/SIM/v2x/MAPEM", 1);
                publisher_mapem_ = getRosNodeHandle()->advertise<dsrc_v2_dsrc::MAPEM>("v2x/incoming/MAPEM", 1);

            }

            void receiveTrafficLightsStatusList(carla_msgs::CarlaTrafficLightStatusList carla_traffic_light_status_list_)
            {
                traffic_lights_status = carla_traffic_light_status_list_.traffic_lights;
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
                traffic_lights_info = carla_traffic_light_info_list_.traffic_lights;
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
                dsrc_v2_dsrc::SPATEM out_msg;

            }
            void sendMAPEMSim()
            {
                dsrc_v2_mapem_pdu_descriptions::MAPEM out_msg;

            }
            void sendMAPEM()
            {
                dsrc_v2_dsrc::MAPEM out_msg;
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
