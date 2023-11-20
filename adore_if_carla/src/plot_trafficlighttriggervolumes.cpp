
#include <adore_if_carla/plot_trafficlighttriggervolumes.h>

#include <adore_if_ros/factorycollection.h>
#include <adore_if_ros_scheduling/baseapp.h>

namespace adore
{
  namespace adore_if_carla
  {  
    class PlotTrafficLightTriggerVolumesNode : public adore::if_ROS::FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      //adore::apps::PlotEgo* app_;
      PlotTrafficLightTriggerVolumesNode(){}
      ~PlotTrafficLightTriggerVolumesNode()
      {
        delete ptl_;
      }

      PlotTrafficLightTriggerVolumes* ptl_;

      void init(int argc, char **argv, double rate, std::string nodename)
      {
        Baseapp::init(argc, argv, rate, nodename);
        Baseapp::initSim();
        FactoryCollection::init(getRosNodeHandle());
        ptl_ = new PlotTrafficLightTriggerVolumes();
        // timer callbacks
        std::function<void()> run_fcn(std::bind(&PlotTrafficLightTriggerVolumes::periodic_run, ptl_));
        Baseapp::addTimerCallback(run_fcn);
      }
    };
  }
}

adore::adore_if_carla::PlotTrafficLightTriggerVolumesNode ptltvn_;

/*int main(int argc,char **argv)
{
    adore::if_ROS::PlotTrafficLightTriggerVolumesNode fbn;    
    fbn.init(argc, argv, 10.0, "plot_ego_node");

    std::string picture = "../images/fascare_hq.png";
    ros::NodeHandle pnh("~");
    pnh.getParam("vehicle_picture",picture);
    std::cout<<"vehicle_picture="<<picture<<std::endl;
    fbn.setPicture(picture);

    fbn.run();
    return 0;
}*/

int main(int argc, char **argv)
{
    /*ros::init(argc, argv, "plot_traffic_light_trigger_volumes");
    PlotTrafficLightTriggerVolumes ptltv;
    std::cout<<"Instz erzeugt"<<std::endl;
    ptltv.init(argc, argv, 10, "plot_traffic_light_trigger_volumes");
    //std::cout<<"init erzeugt"<<std::endl;
    //ptltv.init();
    //timer_ = n_->createTimer(ros::Duration(1 / rate), std::bind(&PlotTrafficLightTriggerVolumes::periodic_run, this, std::placeholders::_1));
    ptltv.run();*/
    ptltvn_.init(argc, argv, 10.0, "adore_plot_planning_details_node");
    ptltvn_.run();
    return 0;
}