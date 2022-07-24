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

        pubPoseStamped = nh.advertise<geometry::PoseStamped>("/move_base_simple/goal", 1);
        pubLeftCamera = nh.advertise<std_msgs::Int16>("/leftcam", 1);
        pubRightCamera = nh.advertise<std_msgs::Int16>("/rightcam", 1);
        pubPlate = nh.advertise<std_msgs::Int16>("/plate", 1);
        pubLift = nh.advertise<std_msgs::Int16>("/lift", 1);
        pubUltra = nh.advertise<std_msgs::Int16>("/ultra", 1);

        sub_arrival_status = nh.subscribe("/move_base/result", 1, &Master::CheckArrival, this);
        sub_lift_status = nh.subscribe("/lift", 1, &Master::CheckLift, this);
        sub_robot_status = nh.subscribe("/plate", 1, &Master::CheckPlate, this);
        sub_ultra_status = nh.subscribe("/ultra/end", 1, &Master::CheckUltra, this);

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
    }

    void resetFile(){
        ofstream socket("/home/hyun/socket", ios::trunc);
        socket.close();
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
                status[1] = 1;
                ROS_INFO("Robot status = %d", status[0]);
                ROS_INFO("Plate table status = %d\n", status[1]);
                resetFile();
            }

            //2번 테이블 서빙
            else if (num == 2){
                ROS_INFO("Go to Table to get a plate");
                pubPoseStamped.publish(poseStampedTable[2]);
                status[0] = 2;
                status[1] = 1;
                ROS_INFO("Robot status = %d", status[0]);
                ROS_INFO("Plate table status = %d\n", status[1]);
                resetFile();
            }

            //1번 테이블 회수
            else if (num == 3){
                ROS_INFO("Pick up to table 1");
                pubPoseStamped.publish(poseStampedTable[0]);
                status[0] = 3;
                ROS_INFO("status[0] = %d\n", status[0]);
                resetFile();
            }

            //2번 테이블 회수
            else if (num == 4){
                ROS_INFO("Pick up to table 2");
                pubPoseStamped.publish(poseStampedTable[1]);
                status[0] = 4;
                ROS_INFO("status[0] = %d\n", status[0]);
                resetFile();
            }
        }
    }

    void CheckArrival(const move_base_msgs::MoveBaseActionResult arrival){
        left.data = 1;
        right.data = 2;
        if (arrival.status.status == 3){
            //쟁반 테이블 도착
            if (status[1] == 1){
                ROS_INFO("Left Camera Publish\n");
                pubLeftCamera.publish(left);
            }

            //1번 테이블 도착
            else if (status[0] == 1 || status[0] == 3){
                ROS_INFO("Left Camera Publish\n");
                pubLeftCamera.publish(left);
            }

            //2번 테이블 도착
            else if (status[0] == 2 || status[0] == 4 || status[1] == 2){
                ROS_INFO("Right Camera Publish\n");
                pubRightCamera.publish(right);
            }
            //회수 후 쟁반 테이블 도착
            else if (status[0] == 3 && status[1] == 2){
                ROS_INFO("Right Camera Publish\n");
                pubRightCamera.publish(right);
                ROS_INFO("[0] = %d [1] = %d [2] = %d\n", status[0], status[1], status[2]);
            }
            //회수 후 쟁반 테이블 도착
            else if (status[0] == 4 && status[1] == 2){
                ROS_INFO("Right Camera Publish\n");
                pubRightCamera.publish(right);
                ROS_INFO("[0] = %d [1] = %d [2] = %d\n", status[0], status[1], status[2]);
            }
        }
    }

    void CheckUltra(const std_msgs::Int16 ultra_end){
        up.data = 3;
        down.data = 4;
        if (status[1] == 1 || status[1] == 2 || status[0] == 3 || status[0] == 4){
            ROS_INFO("Lift Up Publish\n");
            pubLift.publish(up);
        }

        else if (status[0] == 1 || status[0] == 2){
            ROS_INFO("Lift Down Publish\n");
            pubLift.publish(down);
        }
        //서빙 다 마치고 로봇 복귀
        else if (status[0] == 5){
            pubPoseStamped.publish(poseStampedTable[3]);
            status[0] = 0;
            status[1] = 0;
        }
    }


    void CheckLift(const std_msgs::Int16 lift){
        forward.data = 5;
        backward.data = 6;
        if (lift.data == 10){
            //쟁반 테이블 들림
            if (status[1] == 1){
                ROS_INFO("Plate Backward Publish\n");
                pubPlate.publish(backward);
            }
            //1번 테이블 or 2번 테이블 서빙
            else if (status[0] == 1 || status[0] == 2){
                ROS_INFO("Plate Forward Publish\n");
                pubPlate.publish(forward);
                status[2] = 1;
            }
            //1번 테이블 or 2번 테이블 회수
            else if (status[0] == 3 || status[0] == 4){
                ROS_INFO("Plate Forward Publish\n");
                pubPlate.publish(backward);
                status[2] = 0;
            }
            //회수한 쟁반 테이블에 올려두기
            else if (status[1] == 2){
                pubPlate.publish(forward);
                status[1] = 3;
            }
        }
    }

    //쟁반 로봇 위치 확인 후 다음 단계 진행
    void CheckPlate(const std_msgs::Int16 plate){
        if (plate.data == 20){
            //쟁반 테이블 들림 확인
            if (status[1] == 1){
                //1번 테이블 쟁반 서빙
                if (status[0] == 1){
                    ROS_INFO("Serve to Table 1\n");
                    pubPoseStamped.publish(poseStampedTable[0]);
                    status[1] = 0;
                }
                //2번 테이블 쟁반 서빙
                else if (status[0] == 2){
                    ROS_INFO("Serve to Table 2\n");
                    pubPoseStamped.publish(poseStampedTable[1]);
                    status[1] = 0;
                }
            }

            else if (status[1] == 0){
                //1번 테이블 서빙 마침
                if (status[0] == 1){
                    ROS_INFO("Ultra Sensor Publish\n");
                    ultra.data = 3;
                    pubUltra.publish(ultra);
                    status[0] = 5;
                }
                //2번 테이블 서빙 마침
                else if (status[0] == 2){
                    ROS_INFO("Ultra Sensor Publish\n");
                    ultra.data = 4;
                    pubUltra.publish(ultra);
                    status[0] = 5;
                }
                //각 테이블 회수 단계
                else if (status[0] == 3 || status[0] == 4){
                    ROS_INFO("Go to Table to return a plate");
                    pubPoseStamped.publish(poseStampedTable[2]);
                    status[1] = 2;
                }
            }
            //쟁반 회수 후 초기 위치 복귀
            else if (status[1] == 3){
                ROS_INFO("Ultra Sensor Publish\n");
                ultra.data = 3;
                pubUltra.publish(ultra);
                status[0] = 5;
            }
        }
    }


private:
    ros::NodeHandle nh;

    //Publisher
    ros::Publisher pubPoseStamped;
    ros::Publisher pubLeftCamera;
    ros::Publisher pubRightCamera;
    ros::Publisher pubPlate;
    ros::Publisher pubLift;
    ros::Publisher pubUltra;

    //Subscriber
    ros::Subscriber sub_arrival_status;
    ros::Subscriber sub_lift_status;
    ros::Subscriber sub_robot_status;
    ros::Subscriber sub_ultra_status;

    //msgs
    geometry_msgs::PoseStamped poseStampedTable[4];

    vector<double> target_pose_position;
    vector<double> target_pose_orientation;

    int status[3] = {0};

    std_msgs::Int16 ultra, up, down, left, right, forward, backward;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "master");

    ROS_INFO("System ON");

    Master master;

    ros::spin();

    return 0;
}
