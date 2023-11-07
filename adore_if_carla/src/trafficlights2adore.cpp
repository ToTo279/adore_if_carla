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
#include <nav_msgs/Odometry.h>
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

//#include <plotlablib/plcommands.h> 
//#include <plotlablib/afigurestub.h>
#include <adore_if_carla/plot_trafficlighttriggervolumes.h>
#include <adore/env/threelaneviewdecoupled.h>

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
            Trafficlights2Adore() : lv_(false)
            {
                //lv_ = adore::env::ThreeLaneViewDecoupled();
            }

            /*TO DO:
            3 lanes view im konstruktor instanzieren (threelaneview)
            3 lanes view in periodischer Funktion updaten (.update)
            4 (parametrierbar mit Varble) Punkte aus TriggerVolume extrahieren
            mit 3 lanes View die realtiven koordinaten zu jedem Punkt bestimmen (.torelativecoordinates)
            überprrüfen, ob Punkt in Reichweite des Fahrzeugs und innerhalb Lane ist
                Reichweite: Abstand s-Koordinate TriggerVolume - s-Koordinate Fahrzeug 
                Innerhlab Lane: n-Koordinate innerhalb von .currentlane().getleftborder(s_triggerVolume) & ...geetrightborder*/
            void init(int argc, char** argv, double rate, std::string nodename)
            {
                // Although the application has no periodically called functions, the rate is required for scheduling
                ros::init(argc, argv, nodename);
                ros::NodeHandle* n = new ros::NodeHandle();
                n_ = n;
                //initSim();
                /* NOTWENDIG???
                bool carla_namespace_specified = n_->getParam("PARAMS/adore_if_carla/carla_namespace", namespace_carla_);
                std::cout << "Objects2Adore: namespace of the carla vehicle is: "
                          << (carla_namespace_specified ? namespace_carla_ : "NOT SPECIFIED") << std::endl;*/
                std::cout<<"in init"<<std::endl;
                timer_ = n_->createTimer(ros::Duration(1 / rate), std::bind(&Trafficlights2Adore::periodic_run, this, std::placeholders::_1));
                std::cout<<"timer created"<<std::endl;
                initROSConnections();
                std::cout<<"init ros connection aufgerufen"<<std::endl;
                //getConnections();
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
            ros::Subscriber subscriber_vehicle_localization_;

            std::string namespace_carla_;
            ros::NodeHandle* n_;

            std::multimap<uint32_t, adore_if_ros_msg::Connection_<std::allocator<void>>> Connections;
            Betriebsmodus betriebsmodus = DIRECT;
            int discard_age = 1;    //in [sec]
            //double t_;
            //nav_msgs::Odometry vehicle_position;
            double vehicle_alpha, vehicle_x, vehicle_y;
            adore::env::ThreeLaneViewDecoupled lv_;/**<lane-based representation of environment*/

            ros::Timer timer_;

            PlotTrafficLightTriggerVolumes plt_;

            /*struct CarlaTrafficLightStatus
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
            //};
            /*struct CarlaTrafficLightInfo
            {
                uint32_t id;
                geometry_msgs::Pose transform;
                carla_msgs::CarlaBoundingBox trigger_volume;    
            };*/
            
            std::vector<carla_msgs::CarlaTrafficLightStatus> traffic_lights_status;
            std::vector<carla_msgs::CarlaTrafficLightStatus> traffic_lights_status_old;
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
                std::cout<<"subscriber_traffic_lights_status_ init"<<std::endl;
                /*subscriber_traffic_lights_info_ = getRosNodeHandle()->subscribe<carla_msgs::CarlaTrafficLightInfoList>(
                    "/carla/traffic_lights/info", 1, &Trafficlights2Adore::receiveTrafficLightsInfoList, this);*/
                subscriber_vehicle_localization_ = getRosNodeHandle()->subscribe<nav_msgs::Odometry>(
                    "/vehicle0/localization", 1, &Trafficlights2Adore::receiveVehicleLocalization, this);
                std::cout<<"subscriber_vehicle_localization_ init"<<std::endl;
                publisher_spatem_sim_ = getRosNodeHandle()->advertise<adore_v2x_sim::SimSPATEM>("/SIM/v2x/SPATEM", 1);
                publisher_mapem_sim_ = getRosNodeHandle()->advertise<adore_v2x_sim::SimMAPEM>("/SIM/v2x/MAPEM", 1);
                publisher_spatem_ = getRosNodeHandle()->advertise<dsrc_v2_spatem_pdu_descriptions::SPATEM>("v2x/incoming/SPATEM", 1);
                publisher_mapem_ = getRosNodeHandle()->advertise<dsrc_v2_mapem_pdu_descriptions::MAPEM>("v2x/incoming/MAPEM", 1);
                publisher_direct_ = getRosNodeHandle()->advertise<adore_if_ros_msg::TCDConnectionStateTrace>("ENV/tcd", 500, true);
                std::cout<<"publisher init"<<std::endl;
            }
            void periodic_run(const ros::TimerEvent &te)
            {
                std::cout<<"periodic_run aufgerufen"<<std::endl;
                lv_.update();
                isVehicleInTriggerVolume();
            }
            /*double getTime()
            {
                double time = 0;

                using namespace std::chrono;
                uint64_t ms = system_clock::now().time_since_epoch().count();
                time = ((double)ms) / 1000;//000000;

                return time;   
            }*/

            /*void getConnections()
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
            }*/

            void receiveTrafficLightsStatusList(carla_msgs::CarlaTrafficLightStatusList carla_traffic_light_status_list_)
            {
                traffic_lights_status_old.clear();
                traffic_lights_status_old = traffic_lights_status;
                traffic_lights_status.clear();

                for (const auto& carla_traffic_light_status : carla_traffic_light_status_list_.traffic_lights)
                {
                    //adore::adore_if_carla::Trafficlights2Adore::CarlaTrafficLightStatus status;
                    carla_msgs::CarlaTrafficLightStatus status;
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

            void receiveVehicleLocalization(const nav_msgs::OdometryConstPtr &in_msg)
            {
                vehicle_x = in_msg->pose.pose.position.x;
                vehicle_y = in_msg->pose.pose.position.y;
                /*vehicle_position.pose.pose.position.x = in_msg->pose.pose.position.x;
                vehicle_position.pose.pose.position.y = in_msg->pose.pose.position.y;
                vehicle_position.pose.pose.orientation.x = in_msg->pose.pose.orientation.x;
                vehicle_position.pose.pose.orientation.y = in_msg->pose.pose.orientation.y;
                vehicle_position.pose.pose.orientation.z = in_msg->pose.pose.orientation.z;
                vehicle_position.pose.pose.orientation.w = in_msg->pose.pose.orientation.w;*/
                double siny_cosp = 2 * (in_msg->pose.pose.orientation.w * in_msg->pose.pose.orientation.z + in_msg->pose.pose.orientation.x * in_msg->pose.pose.orientation.y);
                double cosy_cosp = 1 - 2 * (in_msg->pose.pose.orientation.y * in_msg->pose.pose.orientation.y + in_msg->pose.pose.orientation.z * in_msg->pose.pose.orientation.z);
                vehicle_alpha = std::atan2(siny_cosp, cosy_cosp);
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

                    uint8_t state = 0;  
                    for (const auto& status : traffic_lights_status) {
                        if (status.id == targetAmpelID) {
                            newState.state = static_cast<int8_t>(state); 
                            break;  
                        }
                    }
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

    
            /*bool isVehicleInTriggerVolume()
            {
                for (const auto& entry : plt_.id_to_triggervolume_)
                {
                    double min_x = entry.second.center_x - (entry.second.width * std::abs(std::cos(entry.second.alpha)) + entry.second.length * std::abs(std::sin(entry.second.alpha))) / 2.0;
                    double max_x = entry.second.center_x + (entry.second.width * std::abs(std::cos(entry.second.alpha)) + entry.second.length * std::abs(std::sin(entry.second.alpha))) / 2.0;
                    double min_y = entry.second.center_y - (entry.second.length * std::abs(std::cos(entry.second.alpha)) + entry.second.width * std::abs(std::sin(entry.second.alpha))) / 2.0;
                    double max_y = entry.second.center_y + (entry.second.length * std::abs(std::cos(entry.second.alpha)) + entry.second.width * std::abs(std::sin(entry.second.alpha))) / 2.0;

                    if (vehicle_x >= min_x && vehicle_x <= max_x && vehicle_y >= min_y && vehicle_y <= max_y)
                    {
                        std::cout << "true" << std::endl;
                        return true;
                    }
                }
                std::cout << "false" << std::endl;
                return false;
            }*/
            /*bool isVehicleInTriggerVolume()
            {
                //std::vector<double> s,n;
                std::unordered_map<unsigned int,std::unordered_map<unsigned int, double>> s,n;
                for (const auto& entry : plt_.points)
                {
                    const std::unordered_map<unsigned int, PlotTrafficLightTriggerVolumes::pair>& innerMap = entry.second;
                    //const unsigned int id = entry.first
                    for (const auto& entry_ : innerMap)
                    {
                        lv_.getCurrentLane()->toRelativeCoordinates(entry_.second.x, entry_.second.y, s[entry.first][entry_.first], n[entry.first][entry_.first]);
                    }
                    //lv_.toRelativeCoordinates(entry.second, x_replan.getY(), s.at(entry.first), n.at(entry.first));
                }
                //lv_.toRelativeCoordinates(x_replan.getX(), x_replan.getY(), s, n);
            }*/
            bool isVehicleInTriggerVolume()
            {
                for (const auto& entry : plt_.points)
                {
                    const std::unordered_map<unsigned int, PlotTrafficLightTriggerVolumes::pair>& innerMap = entry.second;
                    for (const auto& entry_ : innerMap)
                    {
                        double triggerVolume_s, triggerVolume_n, vehicle_s, vehicle_n;

                        lv_.getCurrentLane()->toRelativeCoordinates(entry_.second.x, entry_.second.y, triggerVolume_s, triggerVolume_n);
                        lv_.getCurrentLane()->toRelativeCoordinates(vehicle_x, vehicle_y, vehicle_s, vehicle_n);

                        double range = std::abs(triggerVolume_s - vehicle_s);

                        //if (/*???*/ <= range)
                        //{
                            //if (n >= lv_.getCurrentLane().getLeftBorders()->getBorders(s) && n <= lv_.getCurrentLane().getRightBorders()->getBorders(s))
                            if (vehicle_n >= lv_.getCurrentLane()->getOffsetOfLeftBorder(triggerVolume_s) && vehicle_n <= lv_.getCurrentLane()->getOffsetOfRightBorder(triggerVolume_s))
                            {
                                std::cout << "Punkt in Reichweite und innerhalb der Lane" << std::endl;
                                return true;
                            }//.currentlane().getleftborder(s_triggerVolume)
                        //}
                    }
                }

                std::cout << "Punkt nicht in Reichweite oder außerhalb der Lane" << std::endl;
                return false;
            }



        };
    }  // namespace adore_if_carla
}  // namespace adore

int main(int argc, char** argv)
{
    std::cout<<"main: "<<std::endl;
    ros::init(argc, argv, "trafficlights2adore");
    std::cout<<"ros init aufgerufen"<<std::endl;
    /*PlotTrafficLightTriggerVolumes ptltv;
    ptltv.run();*/
    adore::adore_if_carla::Trafficlights2Adore trafficlights2adore;
    trafficlights2adore.init(argc, argv, 10.0, "trafficlights2adore");
    std::cout<<"init aufgerufen"<<std::endl;
    trafficlights2adore.run();
    std::cout<<"run aufgerufen"<<std::endl;
    std::cout<<""<<std::endl;
    //return 0;
}