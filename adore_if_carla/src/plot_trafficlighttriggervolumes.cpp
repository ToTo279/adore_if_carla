
#include <adore_if_carla/plot_trafficlighttriggervolumes.h>



namespace adore
{
  namespace adore_if_carla
  {  
    class PlotTrafficLightTriggerVolumesNode //: public FactoryCollection, public adore_if_ros_scheduling::Baseapp
    {
      public:
      //adore::apps::PlotEgo* app_;
      PlotTrafficLightTriggerVolumesNode(){}
      /*void setPicture(std::string value)
      {
        app_->setPicture(value);
      }*/
      /*void init(int argc, char **argv, double rate, std::string nodename)
      {
        //Baseapp::init(argc, argv, rate, nodename);
        //Baseapp::initSim();
        //FactoryCollection::init(getRosNodeHandle());
        ros::init(argc, argv, "plot_traffic_ligtht_trigger_volumes");

        DLR_TS::PlotLab::FigureStubFactory fig_factory;
        auto figure = fig_factory.createFigureStub(2);
        figure->show();
        //int simulationID = 0;
        //getParam("simulationID",simulationID);

        /*int followMode = 0;
        getParam("plotoptions/followMode",followMode);

 
        std::stringstream ss;
        ss<<"v"<<simulationID<<"/";
        app_ = new adore::apps::PlotEgo(figure,ss.str(),followMode);
        app_->setMapFigure(fig_factory.createFigureStub(1));
        std::function<void()> run_fcn(std::bind(&adore::apps::PlotEgo::run,app_));
        Baseapp::addTimerCallback(run_fcn);*/
      //}
    };
  }
}

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
    ros::init(argc, argv, "plot_traffic_ligtht_trigger_volumes");

    PlotTrafficLightTriggerVolumes ptltv;
    //ptltv.init();
    ptltv.run();
}