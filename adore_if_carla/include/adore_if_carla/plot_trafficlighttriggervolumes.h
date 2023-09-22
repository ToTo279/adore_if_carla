#pragma once
#include <adore/apps/if_plotlab/plot_shape.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <plotlablib/afigurestub.h>
#include <plotlablib/plcommands.h> 



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



    DLR_TS::PlotLab::AFigureStub *figure_;



    void receive_tl_info_list(todo)

    {

        //TODO

        void plotRectangle(prefix_+std::to_string(info.id), double center_x, double center_y, double length, double width, figure_, styles_.at(info.status), double alpha=0.0) 

    }

    void receive_tl_status_list(todo)

    {


        //todo geometries befüllen

    }

	




public:

    PlotTrafficLightTriggerVolumes()

    {

		styles_.emplace(CarlaTrafficLightStatus::RED,"LineColor=0,0,1;LineWidth=2");

		styles_.emplace(CarlaTrafficLightStatus::YELLOW,"LineColor=0,0,1;LineWidth=2");

		styles_.emplace(CarlaTrafficLightStatus::GREEN,"LineColor=0,0,1;LineWidth=2");

		styles_.emplace(CarlaTrafficLightStatus::OFF,"LineColor=0,0,1;LineWidth=2");

		styles_.emplace(CarlaTrafficLightStatus::UNKNOWN,"LineColor=0,0,1;LineWidth=2");

		

		DLR_TS::PlotLab::FigureStubFactory fig_factory;

        figure_ = fig_factory.createFigureStub(2);

        figure_->show();



        subscriber_tl_info_list = n_->subscribe("todo", 1, &PlotTrafficLightTriggerVolumes::receive_tl_info_list, this);

        subscriber_tl_status_list = n_->subscribe("todo", 1, &PlotTrafficLightTriggerVolumes::receive_tl_status_list, this);
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



int main(int argc, char **argv)

{

    ros::init(argc, argv, "plot_traffic_ligtht_trigger_volumes");

    PlotLongControlInfo plci;

    plci.run();

}