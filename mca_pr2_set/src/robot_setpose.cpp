#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"

class Setpose{
public:
    Setpose(){
        pubCmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

        sub_left_camera = nh.subscribe("/leftcam", 1, &Setpose::subLeftCamera, this);
        sub_right_camera = nh.subscribe("/rightcam", 1, &Setpose::subRightCamera, this);
    }

    void subLeftCamera(const std_msgs::Int16 leftcamera){
        if(leftcamera.data == 1){
            cmd_vel.linear.x = -0.01;
            pubCmdvel.publish(cmd_vel);
        }

        else if (leftcamera.data == 2){
            cmd_vel.linear.x = 0.01;
            pubCmdvel.publish(cmd_vel);
        }
        
        else if (leftcamera.data == 3){
            cmd_vel.linear.x = 0.0;
            pubCmdvel.publish(cmd_vel);
            sleep(1);
        }
    }


    void subRightCamera(const std_msgs::Int16 rightcamera){
        if(rightcamera.data == 1){
            cmd_vel.linear.x = 0.01;
            pubCmdvel.publish(cmd_vel);
        }

        else if (rightcamera.data == 2){
            cmd_vel.linear.x = -0.01;
            pubCmdvel.publish(cmd_vel);
        }


        else if (rightcamera.data == 3){
            cmd_vel.linear.x = 0.0;
            pubCmdvel.publish(cmd_vel);
            sleep(1);
        }
    }

private:
    ros::NodeHandle nh;
    
    //Publisher
    ros::Publisher pubCmdvel;

    //Subscriber
    ros::Subscriber sub_left_camera;
    ros::Subscriber sub_right_camera;

    //msgs
    geometry_msgs::Twist cmd_vel;
};

int main(int argc, char**argv){
    ros::init(argc, argv, "setpose");

    ROS_INFO("Set pose Activated");

    Setpose setpose;

    ros::spin();

    return 0;
}