#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <thread>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h" 

using namespace std;

class Master
{
public:
Master()
{
    fnInitParam();

    pubPoseStamped = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    pubLeftCamera = nh.advertise<std_msgs::Int16>("/leftcam", 1);
    pubRightCamera = nh.advertise<std_msgs::Int16>("/rightcam", 1);
    pubPlate = nh.advertise<std_msgs::Int16>("/plate", 1);
    pubLift = nh.advertise<std_msgs::Int16>("/lift", 1);
    pubUltra = nh.advertise<std_msgs::Int16>("/ultra", 1);

    sub_arrival_status = nh.subscribe("/move_base/result", 1, &Master::CheckArrival, this);
    sub_lift_status = nh.subscribe("/lift", 1, &Master::CheckLift, this);
    sub_ultra_status = nh.subscribe("/ultra", 1, &Master::CheckUltra, this);
    sub_robot_status = nh.subscribe("/plate", 1, &Master::CheckPlate, this);

    ros::Rate loop_rate(10);

    while(ros::ok()){
        ReceiveAppMsg();

        ros::spinOnce();

        loop_rate.sleep();
    }
}
// 상태 설정
enum State
{
    STATE_WAIT,       // 로봇 대기
    STATE_RCVPLATE,   // 쟁반 받는 곳
    STATE_RCVEND,     // 쟁반 받기 끝
    STATE_TABLE,      // 손님 테이블
    STATE_TABLEEND,   // 손님 테이블 끝
    STATE_WITHDRAW,   // 쟁반 회수대
    STATE_WITHDRAWEND // 쟁반 회수대 끝
};

// 상태 표시
void StateInfo()
{
    ROS_INFO("sv_state: %d", sv_state_);
    ROS_INFO("wd_state: %d\n", wd_state_);
}

//파일 리셋
void reset(){
    ofstream reset("/home/hyun/socket", ios::trunc);
    reset.close();
}

//파일 쓰기
void file(int data, int location){
    ofstream output("/home/hyun/socket" + location);
    output << data;
    output.close();
}

// 초기 좌표 설정
void fnInitParam()
    {
        //1번 테이블
        nh.getParam("table_pose_first/position", target_pose_position);
        nh.getParam("table_pose_first/orientation", target_pose_orientation);

        poseStampedTable[0].header.frame_id = "map";
        poseStampedTable[0].header.stamp = ros::Time::now();

        poseStampedTable[0].pose.position.x = target_pose_position[0];
        poseStampedTable[0].pose.position.y = target_pose_position[1];
        poseStampedTable[0].pose.position.z = target_pose_position[2];

        poseStampedTable[0].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[0].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[0].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[0].pose.orientation.w = target_pose_orientation[3];

        //쟁반 테이블
        nh.getParam("table_order/position", target_pose_position);
        nh.getParam("table_order/orientation", target_pose_orientation);

        poseStampedTable[1].header.frame_id = "map";
        poseStampedTable[1].header.stamp = ros::Time::now();

        poseStampedTable[1].pose.position.x = target_pose_position[0];
        poseStampedTable[1].pose.position.y = target_pose_position[1];
        poseStampedTable[1].pose.position.z = target_pose_position[2];

        poseStampedTable[1].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[1].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[1].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[1].pose.orientation.w = target_pose_orientation[3];

        //로봇 대기 위치
        nh.getParam("init_pose_robot/position", target_pose_position);
        nh.getParam("init_pose_robot/orientation", target_pose_orientation);

        poseStampedTable[2].header.frame_id = "map";
        poseStampedTable[2].header.stamp = ros::Time::now();

        poseStampedTable[2].pose.position.x = target_pose_position[0];
        poseStampedTable[2].pose.position.y = target_pose_position[1];
        poseStampedTable[2].pose.position.z = target_pose_position[2];

        poseStampedTable[2].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[2].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[2].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[2].pose.orientation.w = target_pose_orientation[3];

        //쟁반 반납대
        nh.getParam("table_return/position", target_pose_position);
        nh.getParam("table_return/orientation", target_pose_orientation);

        poseStampedTable[3].header.frame_id = "map";
        poseStampedTable[3].header.stamp = ros::Time::now();

        poseStampedTable[3].pose.position.x = target_pose_position[0];
        poseStampedTable[3].pose.position.y = target_pose_position[1];
        poseStampedTable[3].pose.position.z = target_pose_position[2];

        poseStampedTable[3].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[3].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[3].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[3].pose.orientation.w = target_pose_orientation[3];
    }

// TCP/IP 통신
void ReceiveAppMsg()
{
    int msg = 0;

    ifstream input("/home/hyun/socket");
    input >> msg;
    input.close();

    if (sv_state_ == STATE_WAIT && wd_state_ == STATE_WAIT)
    {
        if (msg == 2)
        {
            ROS_INFO("Get a Plate");
            pubPoseStamped.publish(poseStampedTable[1]);
            sv_state_ = STATE_RCVPLATE;
            StateInfo();
            reset();
        }

        else if (msg == 3)
        {
            ROS_INFO("[Table 1] Withdraw a Plate");
            pubPoseStamped.publish(poseStampedTable[0]);
            wd_state_ = STATE_TABLE;
            StateInfo();
            reset();
        }
    }
}

// 도착 확인
void CheckArrival(const move_base_msgs::MoveBaseActionResult arrival)
{
    left.data = 0;
    right.data = 1;
    init.data = 5;
    if (arrival.status.status == 3)
    {
        // 쟁반 받는 곳 도착
        if (sv_state_ == STATE_RCVPLATE)
        {
            ROS_INFO("[Table R] Reached");
            sleep(0.5);
            ROS_INFO("Left Camera Publish");
            pubLeftCamera.publish(left);
            StateInfo();
        }

        // 손님 테이블 도착
        else if (sv_state_ == STATE_TABLE || wd_state_ == STATE_TABLE)
        {
            ROS_INFO("[Table 1] Reached");
            sleep(0.5);
            ROS_INFO("Right Camera Publish");
            pubRightCamera.publish(right);
            StateInfo();
        }

        // 초기 위치 복귀
        else if (sv_state_ == STATE_WAIT || wd_state_ == STATE_WAIT)
        {
            ROS_INFO("[Init] Reached");
            sleep(0.5);
            ROS_INFO("Lift Down Publish");
            pubLift.publish(init);
            ROS_INFO("Finish!!");
            StateInfo();

            file(1,2);
        }

        //쟁반 회수대 도착
        else if (wd_state_ = STATE_WITHDRAW)
        {
            ROS_INFO("[Table Withdraw] Reached");
            sleep(0.5);
            ROS_INFO("Right Camera Publish");
            pubRightCamera.publish(right);
            StateInfo();
        }
    }
}

// 리프트 확인
void CheckLift(const std_msgs::Int16 lift)
{
    if (lift.data == 30)
    {
        if (sv_state_ == STATE_RCVPLATE)
        {
            ROS_INFO("Turn & Ultra Publish\n");
            turn.data = 1;
            pubUltra.publish(turn);
        }

        else if (sv_state_ == STATE_TABLE || wd_state_ == STATE_TABLE)
        {
            ROS_INFO("Turn & Ultra Publish\n");
            turn.data = 2;
            pubUltra.publish(turn);
        }

        if (wd_state_ == STATE_WITHDRAW)
        {
            ROS_INFO("Turn Publish\n");
            turn.data = 7;
            pubUltra.publish(turn);
        }
    }
}

// 초음파 확인
void CheckUltra(const std_msgs::Int16 ultra)
{
    go.data = 3;
    back.data = 4;
    if (ultra.data == 40)
    {
        // 서빙 루틴
        if (sv_state_ == STATE_RCVPLATE)
        {
            ROS_INFO("Plate Backward Publish\n");
            pubPlate.publish(back);
            sv_state_ = STATE_RCVEND;
            StateInfo();
        }

        else if (sv_state_ == STATE_TABLE)
        {
            ROS_INFO("Plate forward Publish\n");
            pubPlate.publish(go);
            sv_state_ = STATE_TABLEEND;
        }

        else if (sv_state_ == STATE_RCVEND)
        {
            ROS_INFO("[Table 1] Serve\n");
            pubPoseStamped.publish(poseStampedTable[0]);
            sv_state_ = STATE_TABLE;
            StateInfo();
        }

        else if (sv_state_ == STATE_TABLEEND || wd_state_ == STATE_WITHDRAWEND)
        {
            ROS_INFO("Go to init place\n");
            pubPoseStamped.publish(poseStampedTable[2]);
            sv_state_ = STATE_WAIT;
            wd_state_ = STATE_WAIT;
            StateInfo();
        }

        // 회수 루틴
        else if (wd_state_ == STATE_TABLE)
        {
            ROS_INFO("Plate Backward Publish\n");
            pubPlate.publish(back);
            wd_state_ == STATE_TABLEEND;
            StateInfo();
        }

        else if (wd_state_ == STATE_TABLEEND)
        {
            ROS_INFO("Go to Table to return a plate\n");
            pubPoseStamped.publish(poseStampedTable[3]);
            wd_state_ = STATE_WITHDRAW;
            StateInfo();
        }
    }
}

// 쟁반 확인    
void CheckPlate(const std_msgs::Int16 plate)
{
    if (plate.data == 20)
    {
        if (sv_state_ == STATE_RCVEND)
        {
            ROS_INFO("Ultra Publish\n");
            ultra.data = 3;
            pubUltra.publish(ultra);
        }

        else if (sv_state_ == STATE_TABLE)
        {
            ROS_INFO("Ultra Publish\n");
            ultra.data = 3;
            pubUltra.publish(ultra);
        }

        else if (sv_state_ == STATE_TABLEEND)
        {
            ROS_INFO("Ultra Publish\n");
            ultra.data = 4;
            pubUltra.publish(ultra);
        }

        else if (wd_state_ == STATE_TABLEEND)
        {
            ROS_INFO("Ultra Publish\n");
            ultra.data = 5;
            pubUltra.publish(ultra);
        }

        else if (wd_state_ == STATE_WITHDRAW)
        {
            ROS_INFO("Ultra Publish\n");
            ultra.data = 5;
            pubUltra.publish(ultra);
            wd_state_ = STATE_WITHDRAWEND;
            StateInfo();
        }
    }
}

private:
ros::NodeHandle nh;

ros::Publisher pubPoseStamped;
ros::Publisher pubLeftCamera;
ros::Publisher pubRightCamera;
ros::Publisher pubUltra;
ros::Publisher pubPlate;
ros::Publisher pubLift;

ros::Subscriber sub_arrival_status;
ros::Subscriber sub_lift_status;
ros::Subscriber sub_robot_status;
ros::Subscriber sub_ultra_status;
ros::Subscriber sub_camera_status;

geometry_msgs::PoseStamped poseStampedTable[4];

vector<double> target_pose_position;
vector<double> target_pose_orientation;

// State
int sv_state_ = 0;
int wd_state_ = 0;


std_msgs::Int16 left, right, turn, go, back, ultra, init;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "master");

    ROS_INFO("SYSTEM ON");

    ofstream reset("/home/hyun/socket", ios::trunc);
    reset.close();

    Master master;

    return 0; 
}