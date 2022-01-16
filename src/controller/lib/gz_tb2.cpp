#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <ignition/math.hh>
#include <brl_msgs/brl_msgs.h>
using namespace std;

#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>
#include <sstream>
#define READ_TIME   0
#define WAITE_TIME  1
#define SET_POSE    2

//Global Variables
brl_msgs::brl_msgs Turtle_odom2;
geometry_msgs::Twist Turtle_twist[3];
static double des_position = 0.2;
static double err_position = 0;
static double preerr_position = 0;

void Robot2PoseSub(const brl_msgs::brl_msgs::ConstPtr &odom){
    Turtle_odom2.posx = odom->posx;
    Turtle_odom2.posy = odom->posy;
    Turtle_odom2.yaw = odom->yaw;
}


namespace gazebo
{
  class ModelPush2 : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      RobotPose = nh.subscribe("/tb_2/odom",100,Robot2PoseSub);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ModelPush2::OnUpdate, this));

      readFile.open("/home/ubuntu/Draw_xy/xy_tt3.txt");
    }


  public: vector<string> split(string str, char delimiter_1, char delimiter_2 , char delimiter_3) {
        istringstream issString(str);
        string buffer;

        vector<string> result;

        getline(issString, buffer, delimiter_1);
        result.push_back(buffer);

        getline(issString, buffer, delimiter_2);
        result.push_back(buffer);

        getline(issString, buffer, delimiter_3);
        result.push_back(buffer);

        return result;
   }

  // Called by the world update start event
  public: void OnUpdate()
  {
    // Apply a small linear velocity to the model.
    ignition::math::Pose3d cur_pose= this->model->WorldPose();
    sec = (ros::Time::now().toSec()/1000)*1000;

    if(phase == READ_TIME)
    {
      getline(readFile, fileData);
      time = std::stod(fileData);

      phase = WAITE_TIME;
    }

    if(phase == WAITE_TIME)
    {
      if(time < sec - default_secs +20)
          phase = SET_POSE;
    }

    if(phase == SET_POSE)
    {
      getline(readFile, fileData);
      getline(readFile, fileData);
      getline(readFile, fileData);

      vector<string> vString = split(fileData, ' ', '\t', '\n');

      double pos_x   = std::stod(vString[0]);
      double pos_y   = std::stod(vString[1]);
      double pos_yaw = std::stod(vString[2]);

      Turtle_pose.Set(pos_x,pos_y,0,0,0,pos_yaw);
      this->model->SetWorldPose(Turtle_pose);

      px = pos_x;
      py = pos_y;
      pyaw = pos_yaw;

      getline(readFile, fileData);

      phase = READ_TIME;

    }

    Turtle_pose.Set(px,py,0,0,0,pyaw);
    this->model->SetWorldPose(Turtle_pose);

    cout<<"POSE2 : "<<px<<"\t"<<py<<"\t"<<pyaw<<endl;

    //Turtle_pose.Set(Turtle_odom0.posx,Turtle_odom0.posy,0,0,0,Turtle_odom0.yaw);
    ignition::math::Vector3d linVel, angVel;
    linVel.X(0);
    linVel.Y(0);
    linVel.Z(0);
    angVel.X(0);
    angVel.Y(0);
    angVel.Z(0);
    //this->model->SetWorldPose(Turtle_pose);
    this->model->SetWorldTwist(linVel, angVel);
  }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private:
      ignition::math::Pose3d Turtle_pose;

    public:
      ros::NodeHandle nh;
      ros::Subscriber RobotPose;
      double sec = 0;
      double default_secs = (ros::Time::now().toSec()/1000)*1000;

      std::ifstream readFile;
      string fileData;
      double time = 0;
      int phase = 0;

      double px = 0;
      double py = 0;
      double pyaw = 0;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush2)
}
