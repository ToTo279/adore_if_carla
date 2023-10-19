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
    
    std::unordered_map<int,std::string> status_to_style_;
    std::unordered_map<unsigned int,triggervolume> id_to_triggervolume_;
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

            /*double siny_cosp = 2 * (carla_traffic_light_info.transform.orientation.w * carla_traffic_light_info.transform.orientation.z + carla_traffic_light_info.transform.orientation.x * carla_traffic_light_info.transform.orientation.y);
            double cosy_cosp = 1 - 2 * (carla_traffic_light_info.transform.orientation.y * carla_traffic_light_info.transform.orientation.y + carla_traffic_light_info.transform.orientation.z * carla_traffic_light_info.transform.orientation.z);
            volume.alpha = std::atan2(siny_cosp, cosy_cosp);*/
            tf::Quaternion q(carla_traffic_light_info.transform.orientation.x, carla_traffic_light_info.transform.orientation.y, carla_traffic_light_info.transform.orientation.z, carla_traffic_light_info.transform.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            volume.alpha = yaw;
            volume.center_x = carla_traffic_light_info.transform.position.x + std::cos(volume.alpha)*carla_traffic_light_info.trigger_volume.center.x;
            volume.center_y = carla_traffic_light_info.transform.position.y + std::sin(volume.alpha)*carla_traffic_light_info.trigger_volume.center.y;
            //volume.center_x = carla_traffic_light_info.transform.position.x + carla_traffic_light_info.trigger_volume.center.x/std::cos(volume.alpha);
            //volume.center_y = carla_traffic_light_info.transform.position.y + carla_traffic_light_info.trigger_volume.center.y/std::sin(volume.alpha);
            //volume.center_x = carla_traffic_light_info.transform.position.x + carla_traffic_light_info.trigger_volume.center.x;
            //volume.center_y = carla_traffic_light_info.transform.position.y + carla_traffic_light_info.trigger_volume.center.y;
            volume.width = carla_traffic_light_info.trigger_volume.size.x;
            volume.length = carla_traffic_light_info.trigger_volume.size.y;
            
            //std::cout<<"volume befüllt"<<std::endl;
            id_to_triggervolume_[carla_traffic_light_info.id] = volume;
            //std::cout<<"id_to_triggervolume_ befüllt"<<std::endl;

            std::cout<<"receive_tl_info_list fuer"<< carla_traffic_light_info.id<<"wurde mit Winkel: "<<volume.alpha<<" aufgerufen"<<std::endl;
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
        for (const auto& entry : id_to_triggervolume_) {
            unsigned int id = entry.first;
            triggervolume volume = entry.second;
            adore::PLOT::plotRectangle(prefix_+std::to_string(id), volume.center_x, volume.center_y, volume.length, volume.width, figure_, status_to_style_.at(id_to_status_[id]), volume.alpha);
            //std::cout<<id<<": "<<status_to_style_.at(id_to_status_[id])<<std::endl;
        }
        std::cout<<"plot trigger volumes aufgerufen"<<std::endl;
    }

    void periodic_run(const ros::TimerEvent &te)
    {
        plot_triggger_volumes();
    }

public:
    PlotTrafficLightTriggerVolumes()
    {
		/*status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::RED,"LineColor=0,0,1;LineWidth=2");
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::YELLOW,"LineColor=0,0,1;LineWidth=2");
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::GREEN,"LineColor=0,0,1;LineWidth=2");
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::OFF,"LineColor=0,0,1;LineWidth=2");
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::UNKNOWN,"LineColor=0,0,1;LineWidth=2");*/

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
};