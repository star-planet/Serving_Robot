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

#define Trig 19
#define Echo 21

using namespace std;

class Main{
public:
    Main(){
        pubCmdvel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        pubUltraEnd = nh.advertise<std_msgs::Int16>("/ultra/end", 1);

        sub_turn_ultra = nh.subscribe("/ultra", 1, &Main::subUltra, this);
    }

    void ultra_move(int a){

        clock_t start_time, end_time, distance;

        //go = 1 back = 2
        if (a == 1){
            cmd_vel.linear.x = 0.1;
        }
        else if (a == 2){
            cmd_vel.linear.x = -0.1;
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
                if (distance > 3){
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
    
    void turn(int b){
        if (b == 1){
            cmd_vel.angular.z = 0.52;
        }
        
        else if (b == 2){
            cmd_vel.angular.z = -0.52;
        }

        ROS_INFO("Turn start");
        for (int i = 0; i < 16; i++){
            pubCmdvel.publish(cmd_vel);
            delay(250);
        }
        cmd_vel.angular.z = 0.0;
        pubCmdvel.publish(cmd_vel);
        ROS_INFO("Turn end");
    }

    void subUltra(const std_msgs::Int16 ultra){
        end.data = 1;
        if (ultra.data == 1){
            ultra_move(2);
            delay(500);
            turn(1);
            delay(500);
            pubUltraEnd.publish(end);
        }
        
        else if (ultra.data == 2){
            ultra_move(2);
            delay(500);
            turn(2);
            delay(500);
            pubUltraEnd.publish(end);
        }

        else if (ultra.data == 3){
            ROS_INFO("Ultra start");
            ultra_move(2);
            delay(500);
            turn(1);
            delay(500);
            pubUltraEnd.publish(end);
        }

        else if (ultra.data == 4){
            ROS_INFO("Ultra start");
            ultra_move(2);
            delay(500);
            turn(2);
            delay(500);
            pubUltraEnd.publish(end);
        }
    }

private:
    ros::NodeHandle nh;
    
    //Publisher
    ros::Publisher pubCmdvel;
    ros::Publisher pubUltraEnd;

    //Subscriber
    ros::Subscriber sub_turn_ultra;

    //msgs
    geometry_msgs::Twist cmd_vel;
    std_msgs::Int16 end;

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