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

    void receive_tl_info_list(carla_msgs::CarlaTrafficLightInfoList carla_traffic_light_info_list_)
    {
        id_to_triggervolume_.clear();

        for (const auto& carla_traffic_light_info : carla_traffic_light_info_list_.traffic_lights)
        {
            triggervolume volume;

            volume.center_x = carla_traffic_light_info.transform.position.x + carla_traffic_light_info.trigger_volume.center.x;
            volume.center_y = carla_traffic_light_info.transform.position.y + carla_traffic_light_info.trigger_volume.center.y;
            
            volume.width = carla_traffic_light_info.trigger_volume.size.x;
            volume.length = carla_traffic_light_info.trigger_volume.size.y;
            volume.alpha = 0.0; 

            id_to_triggervolume_[carla_traffic_light_info.id] = volume;

            adore::PLOT::plotRectangle(prefix_+std::to_string(carla_traffic_light_info.id), volume.center_x, volume.center_y, volume.length, volume.width, figure_, status_to_style_.at(carla_traffic_light_info.id), volume.alpha=0.0);
        }
    }

    void receive_tl_status_list(carla_msgs::CarlaTrafficLightStatusList carla_traffic_light_status_list_)
    {
        traffic_lights_status_old.clear();
        traffic_lights_status_old = traffic_lights_status;
        traffic_lights_status.clear();
        for (const auto& carla_traffic_light_status : carla_traffic_light_status_list_.traffic_lights)
        {
            id_to_status_[carla_traffic_light_status.id] = carla_traffic_light_status.state;
        }

    }

public:
    PlotTrafficLightTriggerVolumes()
    {
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::RED,"LineColor=0,0,1;LineWidth=2");
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::YELLOW,"LineColor=0,0,1;LineWidth=2");
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::GREEN,"LineColor=0,0,1;LineWidth=2");
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::OFF,"LineColor=0,0,1;LineWidth=2");
		status_to_style_.emplace(carla_msgs::CarlaTrafficLightStatus::UNKNOWN,"LineColor=0,0,1;LineWidth=2");

        /*
        status_to_style_.emplace(0, "LineColor=1,0,0;LineWidth=2"); // RED
        status_to_style_.emplace(1, "LineColor=1,1,0;LineWidth=2"); // YELLOW
        status_to_style_.emplace(2, "LineColor=0,1,0;LineWidth=2"); // GREEN
        status_to_style_.emplace(3, "LineColor=0,0,0;LineWidth=2"); // OFF
        status_to_style_.emplace(4, "LineColor=0,0,1;LineWidth=2"); // UNKNOWN
        */

		DLR_TS::PlotLab::FigureStubFactory fig_factory;
        figure_ = fig_factory.createFigureStub(2);
        figure_->show();

        subscriber_tl_info_list = n_->subscribe<carla_msgs::CarlaTrafficLightInfoList>("/carla/traffic_lights/info", 1, &PlotTrafficLightTriggerVolumes::receive_tl_info_list, this);
        subscriber_tl_status_list = n_->subscribe<carla_msgs::CarlaTrafficLightStatusList>("/carla/traffic_lights/status", 1, &PlotTrafficLightTriggerVolumes::receive_tl_status_list, this);
        prefix_ = "trafficlighttrigger";
    }
    void run()
    {
        while (n_->ok())
        {
            ros::spin();
        }
    }
};