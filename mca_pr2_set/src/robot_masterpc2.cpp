#include <string.h>
#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"

using namespace std;

class Master{
public:
    Master(){
        fnInitParam();

        pubPoseStamped = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
        pubLeftCamera = nh.advertise<std_msgs::Int16>("/leftcam", 1);
        pubRightCamera = nh.advertise<std_msgs::Int16>("/rightcam", 1);
        pubMidCamera = nh.advertise<std_msgs::Int16>("/midcam", 1);
        pubPlate = nh.advertise<std_msgs::Int16>("/plate", 1);
        pubLift = nh.advertise<std_msgs::Int16>("/lift", 1);
        pubUltra = nh.advertise<std_msgs::Int16>("/ultra", 1);
        pubServer = nh.advertise<std_msgs::Int16>("/server", 1);

        sub_arrival_status = nh.subscribe("/move_base/result", 1, &Master::CheckArrival, this);
        sub_lift_status = nh.subscribe("/lift", 1, &Master::CheckLift, this);
        sub_robot_status = nh.subscribe("/plate", 1, &Master::CheckPlate, this);
        sub_ultra_status = nh.subscribe("/ultra", 1, &Master::CheckUltra, this);

        ros::Rate loop_rate(5);

        while (ros::ok()){
            AppMsgStatus();

            ros::spinOnce();

            loop_rate.sleep();
        }
    }

    void fnInitParam(){
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

        //2번 테이블
        nh.getParam("table_pose_second/position", target_pose_position);
        nh.getParam("table_pose_second/orientation", target_pose_orientation);

        poseStampedTable[1].header.frame_id = "map";
        poseStampedTable[1].header.stamp = ros::Time::now();

        poseStampedTable[1].pose.position.x = target_pose_position[0];
        poseStampedTable[1].pose.position.y = target_pose_position[1];
        poseStampedTable[1].pose.position.z = target_pose_position[2];

        poseStampedTable[1].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[1].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[1].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[1].pose.orientation.w = target_pose_orientation[3];

        //쟁반 테이블
        nh.getParam("table_order/position", target_pose_position);
        nh.getParam("table_order/orientation", target_pose_orientation);

        poseStampedTable[2].header.frame_id = "map";
        poseStampedTable[2].header.stamp = ros::Time::now();

        poseStampedTable[2].pose.position.x = target_pose_position[0];
        poseStampedTable[2].pose.position.y = target_pose_position[1];
        poseStampedTable[2].pose.position.z = target_pose_position[2];

        poseStampedTable[2].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[2].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[2].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[2].pose.orientation.w = target_pose_orientation[3];

        //로봇 대기 위치
        nh.getParam("init_pose_robot/position", target_pose_position);
        nh.getParam("init_pose_robot/orientation", target_pose_orientation);

        poseStampedTable[3].header.frame_id = "map";
        poseStampedTable[3].header.stamp = ros::Time::now();

        poseStampedTable[3].pose.position.x = target_pose_position[0];
        poseStampedTable[3].pose.position.y = target_pose_position[1];
        poseStampedTable[3].pose.position.z = target_pose_position[2];

        poseStampedTable[3].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[3].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[3].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[3].pose.orientation.w = target_pose_orientation[3];

        //쟁반 반납대
        nh.getParam("table_return/position", target_pose_position);
        nh.getParam("table_return/orientation", target_pose_orientation);

        poseStampedTable[4].header.frame_id = "map";
        poseStampedTable[4].header.stamp = ros::Time::now();

        poseStampedTable[4].pose.position.x = target_pose_position[0];
        poseStampedTable[4].pose.position.y = target_pose_position[1];
        poseStampedTable[4].pose.position.z = target_pose_position[2];

        poseStampedTable[4].pose.orientation.x = target_pose_orientation[0];
        poseStampedTable[4].pose.orientation.y = target_pose_orientation[1];
        poseStampedTable[4].pose.orientation.z = target_pose_orientation[2];
        poseStampedTable[4].pose.orientation.w = target_pose_orientation[3];
    }

    void resetFile(){
        ofstream socket("/home/hyun/socket", ios::trunc);
        socket.close();
    }
    
    void info(){
        ROS_INFO("Position = %d Plate to Lift = %d Plate to Table = %d\n",
                                        status[0], status[1], status[2]);
    }

    //앱으로부터 정보 받으면 로봇 출발
    void AppMsgStatus(){
        int num = 0;
        ifstream socket("/home/hyun/socket");
        socket >> num;
        socket.close();

        if (status[0] == 0){
            //1번 테이블 서빙
            if (num == 1){
                ROS_INFO("Go to Table to get a plate");
                pubPoseStamped.publish(poseStampedTable[2]);
                status[0] = 1;
                status[1] = 10;
                info();
                resetFile();
            }

            //2번 테이블 서빙
            else if (num == 2){
                ROS_INFO("Go to Table to get a plate");
                pubPoseStamped.publish(poseStampedTable[2]);
                status[0] = 2;
                status[1] = 10;
                info();
                resetFile();
            }

            //1번 테이블 회수
            else if (num == 3){
                ROS_INFO("Pick up to table 1");
                pubPoseStamped.publish(poseStampedTable[0]);
                status[0] = 3;
                info();
                resetFile();
            }

            //2번 테이블 회수
            else if (num == 4){
                ROS_INFO("Pick up to table 2");
                pubPoseStamped.publish(poseStampedTable[1]);
                status[0] = 4;
                info();
                resetFile();
            }
        }
    }

    void CheckArrival(const move_base_msgs::MoveBaseActionResult arrival){
        left.data = 0;
        right.data = 1;
        mid.data = 2;
        init.data = 6;
        if (arrival.status.status == 3){
            //회수 후 쟁반 테이블 도착
            if (status[0] == 3 && status[1] == 2){
                ROS_INFO("Goal Reached");
                ROS_INFO("Left Camera Publish");
                pubLeftCamera.publish(left);
                status[1] = 3;
                info();
            }
            //회수 후 쟁반 테이블 도착
            else if (status[0] == 4 && status[1] == 2){
                ROS_INFO("Goal Reached");
                ROS_INFO("Left Camera Publish");
                pubMidCamera.publish(left);
                status[1] = 3;
                info();
            }

            else if (status[0] == 1 && status[1] == 1){
                ROS_INFO("Goal Reached");
                ROS_INFO("Left Camera Publish");
                pubLeftCamera.publish(left);
                info(); 
            }
            else if (status[0] == 2 && status[1] == 1){
                ROS_INFO("Goal Reached");
                ROS_INFO("Right Camera Publish");
                pubRightCamera.publish(right);
                info();
            }
            //마무리
            else if (status[0] == 5){
                ROS_INFO("Goal Reached");
                ROS_INFO("Lift Down Publish");
                pubLift.publish(init);
                status[0] = 0;
                status[1] = 0;
                info();
                ROS_INFO("Finish\n");
            }
            //1번 테이블 도착
            else if (status[0] == 1 || status[0] == 3){
                ROS_INFO("Goal Reached");
                ROS_INFO("Left Camera Publish");
                pubLeftCamera.publish(left);
                info();
            }

            //2번 테이블 도착
            else if (status[0] == 2 || status[0] == 4 || status[1] == 2){
                ROS_INFO("Goal Reached");
                ROS_INFO("Right Camera Publish");
                pubRightCamera.publish(right);
                info();
            }
        }
    }
    void CheckLift(const std_msgs::Int16 lift){
        if (lift.data == 30){
            if(status[0] == 3 && status[1] == 3){
                ROS_INFO("Ultra Publish");
                turn.data = 5;
                pubUltra.publish(turn);
                info();
            }

            else if (status[1] == 1 || status[0] == 1 || status[0] == 3){
                ROS_INFO("Ultra Publish");
                turn.data = 1;
                pubUltra.publish(turn);
                info();
            }

            else if (status[0] == 2 || status[0] == 4 || status[1] == 2){
                ROS_INFO("Ultra Publish");
                turn.data = 2;
                pubUltra.publish(turn);
                info();
            }
        }
    }

    void CheckUltra(const std_msgs::Int16 ultra){
        forward.data = 5;
        backward.data = 6;

        if (ultra.data == 40){
            if (status[3] == 1){
                ROS_INFO("Plate Forward Publish");
                pubPlate.publish(forward);
                info();
            }
            //쟁반 테이블 들림
            if (status[1] == 10){
                ROS_INFO("Plate Backward Publish");
                pubPlate.publish(backward);
                status[1] = 1;
                info();
            }

            //1번 테이블 쟁반 서빙
            else if (status[0] == 1 && status[1] == 1){
                ROS_INFO("Serve to Table 1");
                pubPoseStamped.publish(poseStampedTable[0]);
                status[1] = 0;
                info();
            }

            //2번 테이블 쟁반 서빙
            else if (status[0] == 2 && status[1] == 1){
                ROS_INFO("Serve to Table 2");
                pubPoseStamped.publish(poseStampedTable[1]);
                status[1] = 0;
                info();
            }

            //쟁반 회수대로 이동
            else if (status[0] == 3 && status[1] == 2){
                ROS_INFO("Go to Table to return a plate");
                pubPoseStamped.publish(poseStampedTable[4]);
                status[2] = 1;
                info();
            }

            //쟁반 회수대로 이동
            else if (status[0] == 4 && status[1] == 2){
                ROS_INFO("Go to Table to return a plate");
                pubPoseStamped.publish(poseStampedTable[4]);
                status[2] = 1;
                info();
            }

            //쟁반 테이블로 쟁반 이동
            else if (status[0] == 3 && status[1] == 3){
                ROS_INFO("Plate Forward Publish");
                pubPlate.publish(forward);
                info();
            }

            //쟁반 테이블로 쟁반 이동
            else if (status[0] == 4 && status[1] == 3){
                ROS_INFO("Plate Forward Publish");
                pubPlate.publish(forward);
                info();
            }

            //1번 테이블 or 2번 테이블 서빙
            else if (status[0] == 1 || status[0] == 2){
                ROS_INFO("Plate Forward Publish");
                pubPlate.publish(forward);
                status[2] = 1;
                info();
            }
            //1번 테이블 or 2번 테이블 회수
            else if (status[0] == 3 || status[0] == 4){
                ROS_INFO("Plate Backward Publish");
                pubPlate.publish(backward);
                status[2] = 0;
                info();
            }
            //초기 위치 복귀
            else if (status[0] == 5){
                ROS_INFO("Go to init place");
                pubPoseStamped.publish(poseStampedTable[3]);
                info();
            }
        }
    }

    //쟁반 로봇 위치 확인 후 다음 단계 진행
    void CheckPlate(const std_msgs::Int16 plate){
        //쟁반 회수대에 쟁반 넣고 복귀
        if (plate.data == 20){
            if (status[2] == 1){
                ROS_INFO("Ultra Sensor Publish");
                ultra.data = 3;
                pubUltra.publish(ultra);
                status[3] = 0;
                status[0] = 5;
                info();
            }

            //쟁반 테이블 들림 확인
            else if (status[1] == 1){
                //1번 테이블 쟁반 서빙
                if (status[0] == 1){
                    ROS_INFO("Ultra Sensor Publish");
                    ultra.data = 4;
                    pubUltra.publish(ultra);
                    info();
                }
                //2번 테이블 쟁반 서빙
                else if (status[0] == 2){
                    ROS_INFO("Ultra Sensor Publish");
                    ultra.data = 4;
                    pubUltra.publish(ultra);
                    info();
                }
            }

            else if (status[1] == 0){
                //1번 테이블 서빙 마침
                if (status[0] == 1){
                    ROS_INFO("Ultra Sensor Publish");
                    ultra.data = 3;
                    pubUltra.publish(ultra);
                    status[0] = 5;
                    info();
                }
                //2번 테이블 서빙 마침
                else if (status[0] == 2){
                    ROS_INFO("Ultra Sensor Publish");
                    ultra.data = 4;
                    pubUltra.publish(ultra);
                    status[0] = 5;
                    info();
                }
                //각 테이블 회수 단계
                else if (status[0] == 3){
                    ROS_INFO("Ultra Sensor Publish");
                    ultra.data = 3;
                    pubUltra.publish(ultra);
                    status[1] = 2;
                    info();
                }

                else if (status[0] == 4){
                    ROS_INFO("Ultra Sensor Publish");
                    ultra.data = 4;
                    pubUltra.publish(ultra);
                    status[1] = 2;
                    info();
                }
            }
            //쟁반 회수 후 초기 위치 복귀
            else if (status[1] == 3){
                ROS_INFO("Ultra Sensor Publish");
                ultra.data = 3;
                pubUltra.publish(ultra);
                status[0] = 5;
                info();
            }
        }
    }


private:
    ros::NodeHandle nh;

    //Publisher
    ros::Publisher pubPoseStamped;
    ros::Publisher pubLeftCamera;
    ros::Publisher pubRightCamera;
    ros::Publisher pubMidCamera;
    ros::Publisher pubPlate;
    ros::Publisher pubLift;
    ros::Publisher pubUltra;
    ros::Publisher pubServer;

    //Subscriber
    ros::Subscriber sub_arrival_status;
    ros::Subscriber sub_lift_status;
    ros::Subscriber sub_robot_status;
    ros::Subscriber sub_ultra_status;
    ros::Subscriber sub_camera_status;

    //msgs
    geometry_msgs::PoseStamped poseStampedTable[5];

    vector<double> target_pose_position;
    vector<double> target_pose_orientation;

    int status[4] = {0};

    std_msgs::Int16 turn, init, ultra, up, down, left, right, mid, forward, backward;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "master");

    ROS_INFO("System ON");

    Master master;

    ros::spin();

    return 0;
}