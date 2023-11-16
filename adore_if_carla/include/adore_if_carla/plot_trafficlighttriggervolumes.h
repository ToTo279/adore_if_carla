#pragma once
#include <adore/apps/if_plotlab/plot_shape.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h> 

#include <carla_msgs/CarlaTrafficLightStatusList.h>
#include <carla_msgs/CarlaTrafficLightStatus.h>
#include <carla_msgs/CarlaTrafficLightInfoList.h>
#include <carla_msgs/CarlaTrafficLightInfo.h>
//#include <carla_msgs/CarlaBoundingBox.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

class PlotTrafficLightTriggerVolumes
{
private:
    struct triggervolume
    {
        double center_x;
        double center_y;
        double width;
        double length;
        double alpha;
    };
    
    
    std::unordered_map<unsigned int,std::string> status_to_style_;
    //std::unordered_map<unsigned int,triggervolume> id_to_triggervolume_;
    std::unordered_map<unsigned int,int> id_to_status_;

	std::string prefix_;

    ros::Subscriber subscriber_tl_info_list;
    ros::Subscriber subscriber_tl_status_list;

    std::vector<carla_msgs::CarlaTrafficLightStatus> traffic_lights_status;
    std::vector<carla_msgs::CarlaTrafficLightStatus> traffic_lights_status_old;

    DLR_TS::PlotLab::AFigureStub *figure_;

    ros::NodeHandle* n_ = new ros::NodeHandle();

    ros::Timer timer_;

    void receive_tl_info_list(carla_msgs::CarlaTrafficLightInfoList carla_traffic_light_info_list_)
    {
        id_to_triggervolume_.clear();
        std::cout<<"receive_tl_info_list cleared"<<std::endl;


        for (const auto& carla_traffic_light_info : carla_traffic_light_info_list_.traffic_lights)
        {
            triggervolume volume;

            double siny_cosp = 2 * (carla_traffic_light_info.transform.orientation.w * carla_traffic_light_info.transform.orientation.z + carla_traffic_light_info.transform.orientation.x * carla_traffic_light_info.transform.orientation.y);
            double cosy_cosp = 1 - 2 * (carla_traffic_light_info.transform.orientation.y * carla_traffic_light_info.transform.orientation.y + carla_traffic_light_info.transform.orientation.z * carla_traffic_light_info.transform.orientation.z);
            volume.alpha = std::atan2(siny_cosp, cosy_cosp);
            tf::Quaternion q(carla_traffic_light_info.transform.orientation.x, carla_traffic_light_info.transform.orientation.y, carla_traffic_light_info.transform.orientation.z, carla_traffic_light_info.transform.orientation.w);
            tf::Vector3 v(carla_traffic_light_info.transform.position.x, carla_traffic_light_info.transform.position.y, carla_traffic_light_info.transform.position.z);
            tf::Vector3 v_center(carla_traffic_light_info.trigger_volume.center.x, carla_traffic_light_info.trigger_volume.center.y, carla_traffic_light_info.trigger_volume.center.z);
            tf::Transform t(q,v);
            tf::Vector3 v_operator;
            v_operator = t(v_center);

            volume.center_x = v_operator.x();
            volume.center_y = v_operator.y();
            volume.width = carla_traffic_light_info.trigger_volume.size.y;
            volume.length = carla_traffic_light_info.trigger_volume.size.x;

            /*double half_width = 0.5 * carla_traffic_light_info.trigger_volume.size.x;
            double half_length = 0.5 * carla_traffic_light_info.trigger_volume.size.y;

            volume.width = std::abs(carla_traffic_light_info.trigger_volume.size.x * std::cos(volume.alpha)) + std::abs(carla_traffic_light_info.trigger_volume.size.y * std::sin(volume.alpha));
            volume.length = std::abs(carla_traffic_light_info.trigger_volume.size.x * std::sin(volume.alpha)) + std::abs(carla_traffic_light_info.trigger_volume.size.y * std::cos(volume.alpha));*/
            
            //std::cout<<"volume befüllt"<<std::endl;
            id_to_triggervolume_[carla_traffic_light_info.id] = volume;
            //std::cout<<"id_to_triggervolume_ befüllt"<<std::endl;

            //std::cout<<"receive_tl_info_list fuer"<< carla_traffic_light_info.id<<"wurde mit Winkel: "<<volume.alpha<<" aufgerufen"<<std::endl;
        }
    }

    void receive_tl_status_list(carla_msgs::CarlaTrafficLightStatusList carla_traffic_light_status_list_)
    {
        traffic_lights_status_old.clear();
        traffic_lights_status_old = traffic_lights_status;
        traffic_lights_status.clear();
        //std::cout<<"traffic_lights_status cleared"<<std::endl;
        for (const auto& carla_traffic_light_status : carla_traffic_light_status_list_.traffic_lights)
        {
            id_to_status_[carla_traffic_light_status.id] = carla_traffic_light_status.state;
            //std::cout<<"receive_tl_status_list fuer"<< carla_traffic_light_status.id<<"wurde aufgerufen"<<std::endl;
        }
    }

    void plot_triggger_volumes()
    {
        for (const auto& entry : id_to_triggervolume_)
        {
            //unsigned int id = entry.first;
            //triggervolume volume = entry.second;
            adore::PLOT::plotRectangle(prefix_+std::to_string(entry.first), entry.second.center_x, entry.second.center_y, entry.second.length, entry.second.width, figure_, status_to_style_.at(id_to_status_[entry.first]), entry.second.alpha);
            //adore::PLOT::plotArrow(prefix_+std::to_string(entry.first), entry.second.center_x,entry.second.center_y,1,entry.second.alpha,5, 2,status_to_style_.at(id_to_status_[entry.first]), figure_);
        }
        //std::cout<<"plot trigger volumes aufgerufen"<<std::endl;
    }
    void PointsInTriggerVolume()
    {
        for (const auto& entry : id_to_triggervolume_)
        {
            pair p1 , p2 , p3 , p4;

            p1.x = entry.second.center_x + (entry.second.length / 2) * std::cos(entry.second.alpha);
            p1.y = entry.second.center_y + (entry.second.length / 2) * std::sin(entry.second.alpha);
            p2.x = entry.second.center_x + (entry.second.length * 1/6) * std::cos(entry.second.alpha);
            p2.y = entry.second.center_y + (entry.second.length * 1/6) * std::sin(entry.second.alpha);
            p3.x = entry.second.center_x - (entry.second.length / 2) * std::cos(entry.second.alpha);
            p3.y = entry.second.center_y - (entry.second.length / 2) * std::sin(entry.second.alpha);
            p4.x = entry.second.center_x - (entry.second.length * 1/6) * std::cos(entry.second.alpha);
            p4.y = entry.second.center_y - (entry.second.length * 1/6) * std::sin(entry.second.alpha);

            points[entry.first][0] = p1;
            points[entry.first][1] = p2;
            points[entry.first][2] = p3;
            points[entry.first][3] = p4;

            /*points[entry.first][0].x = entry.second.center_x + (entry.second.length / 2) * std::cos(entry.second.alpha);
            points[entry.first][0].y = entry.second.center_y + (entry.second.length / 2) * std::sin(entry.second.alpha);
            points[entry.first][1].x = entry.second.center_x + (entry.second.length * 1/6) * std::cos(entry.second.alpha);
            points[entry.first][1].y = entry.second.center_y + (entry.second.length * 1/6) * std::sin(entry.second.alpha);
            points[entry.first][2].x = entry.second.center_x - (entry.second.length / 2) * std::cos(entry.second.alpha);
            points[entry.first][2].y = entry.second.center_y - (entry.second.length / 2) * std::sin(entry.second.alpha);
            points[entry.first][3].x = entry.second.center_x - (entry.second.length * 1/6) * std::cos(entry.second.alpha);
            points[entry.first][3].y = entry.second.center_y - (entry.second.length * 1/6) * std::sin(entry.second.alpha);*/

            
            adore::PLOT::plotPosition(prefix_+std::to_string(entry.first)+"1", p1.x, p1.y, figure_, status_to_style_.at(id_to_status_[entry.first]), 0.5);
            adore::PLOT::plotPosition(prefix_+std::to_string(entry.first)+"2", p3.x, p3.y, figure_, status_to_style_.at(id_to_status_[entry.first]), 0.5);
            adore::PLOT::plotPosition(prefix_+std::to_string(entry.first)+"3", p2.x, p2.y, figure_, status_to_style_.at(id_to_status_[entry.first]), 0.5);
            adore::PLOT::plotPosition(prefix_+std::to_string(entry.first)+"4", p4.x, p4.y, figure_, status_to_style_.at(id_to_status_[entry.first]), 0.5);
        }

    }


    void periodic_run(const ros::TimerEvent &te)
    {
        plot_triggger_volumes();
        PointsInTriggerVolume();
    }

public:
    struct pair
        {
            double x;
            double y;
        };
    std::unordered_map<unsigned int,triggervolume> id_to_triggervolume_;
    //std::unordered_map<unsigned int,std::vector<pair>[4]> points;
    std::unordered_map<unsigned int,std::unordered_map<unsigned int,pair>> points;
    PlotTrafficLightTriggerVolumes()
    {
        status_to_style_.emplace(0, "LineColor=1,0,0;LineWidth=2"); // RED
        status_to_style_.emplace(1, "LineColor=1,1,0;LineWidth=2"); // YELLOW
        status_to_style_.emplace(2, "LineColor=0,1,0;LineWidth=2"); // GREEN
        status_to_style_.emplace(3, "LineColor=0,0,0;LineWidth=2"); // OFF
        status_to_style_.emplace(4, "LineColor=0,0,1;LineWidth=2"); // UNKNOWN

		DLR_TS::PlotLab::FigureStubFactory fig_factory;
        figure_ = fig_factory.createFigureStub(2);
        figure_->show();

        subscriber_tl_info_list = n_->subscribe<carla_msgs::CarlaTrafficLightInfoList>("/carla/traffic_lights/info", 1, &PlotTrafficLightTriggerVolumes::receive_tl_info_list, this);
        subscriber_tl_status_list = n_->subscribe<carla_msgs::CarlaTrafficLightStatusList>("/carla/traffic_lights/status", 1, &PlotTrafficLightTriggerVolumes::receive_tl_status_list, this);
        prefix_ = "trafficlighttrigger";
        //std::cout<<"Constructor wurde aufgerufen"<<std::endl;
    }
    void init(int argc, char** argv, double rate, std::string nodename)
    {
        // Although the application has no periodically called functions, the rate is required for scheduling
        //ros::init(argc, argv, "plot_traffic_light_trigger_volumes");


        //ros::init(argc, argv, nodename);
        //ros::NodeHandle* n_ = new ros::NodeHandle();

        //n_ = n;
        //initSim();
        //bool carla_namespace_specified = n_->getParam("PARAMS/adore_if_carla/carla_namespace", namespace_carla_);
        //std::cout << "Objects2Adore: namespace of the carla vehicle is: "
        //          << (carla_namespace_specified ? namespace_carla_ : "NOT SPECIFIED") << std::endl;
        //initROSConnections();
        //getConnections();
        
        timer_ = n_->createTimer(ros::Duration(1 / rate), std::bind(&PlotTrafficLightTriggerVolumes::periodic_run, this, std::placeholders::_1));
        std::cout<<"init aufgerufen"<<std::endl;
    }
    void run()
    {
        while (n_->ok())
        {
            ros::spin();
            std::cout<<"run"<<std::endl;
        }
    }
    std::unordered_map<unsigned int,std::unordered_map<unsigned int,pair>> getPoints()
    {
        PointsInTriggerVolume();
        return points;
    }
};