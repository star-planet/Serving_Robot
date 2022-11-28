#include <string>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <JetsonGPIO.h>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"

#define Trig 19
#define Echo 21

using namespace std;

class Main{
public:
    Main(){
        pubCmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        pubMidCamera = nh.advertise<std_msgs::Int16>("/midcam", 1);
        pubUltraEnd = nh.advertise<std_msgs::Int16>("/ultra", 1);

        sub_turn_ultra = nh.subscribe("/ultra", 1, &Main::subUltra, this);
    }

    void ultra_move(int a){

        clock_t start_time, end_time, distance;

        //go = 1 back = 2
        if (a == 1 || a == 3){
            cmd_vel.linear.x = 0.06;
        }
        else if (a == 2){
            cmd_vel.linear.x = -0.06;
        }

        while (ros::ok())
        {
            GPIO::output(Trig, GPIO::LOW);
            delay(100);
            GPIO::output(Trig, GPIO::HIGH);
            delayMicrosecond(10);
            GPIO::output(Trig, GPIO::LOW);

            while (GPIO::input(Echo) == 0);
            start_time = clock();

            while (GPIO::input(Echo) == 1);
            end_time = clock();

            distance = (end_time - start_time)/29./2.;

            cout << "[INFO] Distance : " << distance << "cm\n" << endl;
            

            if (a == 1){
                if (distance > 4){
                    pubCmdvel.publish(cmd_vel);
                }
                else {
                    cmd_vel.linear.x = 0.0;
                    pubCmdvel.publish(cmd_vel);
                    ROS_INFO("STOP");
                    break;
                }
            }

            else if (a == 3){
                if (distance > 5){
                    pubCmdvel.publish(cmd_vel);
                }
                else {
                    cmd_vel.linear.x = 0.0;
                    pubCmdvel.publish(cmd_vel);
                    ROS_INFO("STOP");
                    break;
                }
            }

            else {
                if (distance < 15){ 
                    pubCmdvel.publish(cmd_vel);
                }
                else {
                    cmd_vel.linear.x = 0.0;
                    pubCmdvel.publish(cmd_vel);
                    ROS_INFO("STOP");
                    break;
                }
            }
        }
    }

    void turn90(int b){
        if (b == 1){
            cmd_vel.angular.z = 0.785;
        }
        
        else if (b == 2){
            cmd_vel.angular.z = -0.785;
        }

        ROS_INFO("Turn 90deg start");
        for (int i = 0; i < 28; i++){
            pubCmdvel.publish(cmd_vel);
            delay(100);
        }
        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 90deg end");
    }

    void turn180(int c){
        if (c == 1){
            cmd_vel.angular.z = 0.785;
        }
        
        else if (c == 2){
            cmd_vel.angular.z = -0.785;
        }

        ROS_INFO("Turn 180deg start");
        for (int i = 0; i < 56; i++){
            pubCmdvel.publish(cmd_vel);
            delay(100);
        }
        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn 180deg end");
    }

    void subUltra(const std_msgs::Int16 ultra){
        end.data = 40;
        // 왼쪽 90도 회전 후 초음파 전진
        if (ultra.data == 1){
            ROS_INFO("Turn");
            turn90(1);
            delay(500);
            ROS_INFO("Ultra");
            ultra_move(1);
            delay(500);
            pubUltraEnd.publish(end);
        }
        // 오른쪽 90도 회전 후 초음파 전진
        else if (ultra.data == 2){
            ROS_INFO("Turn");
            turn90(2);
            delay(500);
            ROS_INFO("Ultra");
            ultra_move(1);
            delay(500);
            pubUltraEnd.publish(end);
        }
        // 초음파 후진 후 왼쪽 90도 회전
        else if (ultra.data == 3){
            ROS_INFO("Ultra");
            ultra_move(2);
            delay(500);
            ROS_INFO("Turn");
            turn90(1);
            delay(500);
            pubUltraEnd.publish(end);
        }
        // 초음파 후진 후 오른쪽 90도 회전
        else if (ultra.data == 4){
            ROS_INFO("Ultra");
            ultra_move(2);
            delay(500);
            ROS_INFO("Turn");
            turn90(2);
            delay(500);
            pubUltraEnd.publish(end);
        }
        // 초음파 후진 후 왼쪽 180도 회전
        else if (ultra.data == 5){
            ROS_INFO("Ultra");
            ultra_move(2);
            delay(500);
            ROS_INFO("Turn");
            turn180(1);
            delay(500);
            pubUltraEnd.publish(end);
        }
        // 초음파 후진 후 오른쪽 180도 회전
        else if (ultra.data == 6){
            ROS_INFO("Ultra");
            ultra_move(2);
            delay(500);
            ROS_INFO("Turn");
            turn180(2);
            delay(500);
            pubUltraEnd.publish(end);
        }
        // 왼쪽 90도 회전 후 가운데 카메라 퍼블리쉬
        else if (ultra.data == 7){
            mid.data = 2;
            ROS_INFO("Turn");
            turn90(1);
            delay(500);
            ultra_move(3);
            ROS_INFO("Camera Publish");
            pubMidCamera.publish(mid);
        }

        else if (ultra.data == 8){
            ROS_INFO("Turn");
            turn90(1);
        }
    }

private:
    ros::NodeHandle nh;
    
    //Publisher
    ros::Publisher pubCmdvel;
    ros::Publisher pubMidCamera;
    ros::Publisher pubUltraEnd;

    //Subscriber
    ros::Subscriber sub_turn_ultra;

    //msgs
    geometry_msgs::Twist cmd_vel;
    std_msgs::Int16 end, mid;

    //delay
    inline void delay(int ms) { this_thread::sleep_for(chrono::milliseconds(ms)); }
    inline void delayMicrosecond(int us) { this_thread::sleep_for(chrono::microseconds(us)); }
};

int main(int argc, char**argv){
    ros::init(argc, argv, "main");

    //Jetson Nano GPIO Set up
    GPIO::setmode(GPIO::BOARD);
    GPIO::setwarnings(false);
    GPIO::setup(Trig, GPIO::OUT);
    GPIO::setup(Echo, GPIO::IN);
    
    ROS_INFO("SET UP ULTRA SENSOR");

    Main main;

    ros::spin();

    return 0;
}