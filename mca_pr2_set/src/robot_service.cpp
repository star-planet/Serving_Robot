#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


//Service
#include "mca_pr2_set/setlocation.h"

using namespace std;

string tf_prefix_;
FILE *fp;

mca_pr2_set::setlocation setlocation_cmd;
ros::ServiceServer setlocation_service;

typedef struct TF_POSE
{
    double poseTFx = 0.0;
    double poseTFy = 0.0;
    double poseTFz = 0.0;
    double poseTFqx = 0.0;
    double poseTFqy = 0.0;
    double poseTFqz = 0.0;
    double poseTFqw = 1.0;

}TF_POSE;
TF_POSE _pTF_pose;

bool SaveLocation(string str_location, 
                  float m_fposeAMCLx, float m_fposeAMCLy,
                  float m_fposeAMCLqx, float m_fposeAMCLqy, float m_fposeAMCLqz, float m_fposeAMCLqw)
{
    bool bResult = false;

    string m_strFilePathName;
    m_strFilePathName = "/home/hyun/DATA/" + str_location + ".txt";    
    fp = fopen(m_strFilePathName.c_str(), "w");
    if(fp == NULL)
    { 
        ROS_INFO("file is null");
        bResult = false;
    }
    else
    {
        fprintf(fp, "0,%lf,%lf,%lf,%lf,%lf,%lf \n",
                m_fposeAMCLx, m_fposeAMCLy, m_fposeAMCLqx, m_fposeAMCLqy, m_fposeAMCLqz, m_fposeAMCLqw);
        fclose(fp);
        bResult = true;
    }

    return bResult;
}

bool SetLocation_Command(mca_pr2_set::setlocation::Request  &req, 
					     mca_pr2_set::setlocation::Response &res)
{
	bool bResult = false;

	if(req.Location == "HOME")
	{
	    res.command_Result = false;
	    printf("[ERROR]: HOME location cannot be saved! \n");
	}
	else
	{
        res.command_Result = SaveLocation(req.Location, 
                    _pTF_pose.poseTFx,_pTF_pose.poseTFy,
                    _pTF_pose.poseTFqx,_pTF_pose.poseTFqy,_pTF_pose.poseTFqz,_pTF_pose.poseTFqw);

        res.goal_positionX = _pTF_pose.poseTFx;
        res.goal_positionY = _pTF_pose.poseTFy;
        res.goal_quarterX  = _pTF_pose.poseTFqx;
        res.goal_quarterY  = _pTF_pose.poseTFqy;
        res.goal_quarterZ  = _pTF_pose.poseTFqz;
        res.goal_quarterW  = _pTF_pose.poseTFqw;
	}
    bResult = res.command_Result;

	return true;
}

int main (int argc, char** argv){

    ros::init(argc, argv, "robot_service");
    
    ros::NodeHandle nh;
    
    setlocation_service = nh.advertiseService("setlocation_cmd", SetLocation_Command);

    ROS_INFO("POSE SAVER SERVICE ON");

    tf::TransformListener listener;

    ros::Rate loop_rate(30);

    while(ros::ok())
    {
        ros::spinOnce();
        //map to base_footprint TF Pose////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/map", tf_prefix_ + "/base_footprint", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/map", tf_prefix_ + "/base_footprint", ros::Time(0), transform);

            geometry_msgs::TransformStamped ts_msg;
            tf::transformStampedTFToMsg(transform, ts_msg);

            _pTF_pose.poseTFx = ts_msg.transform.translation.x;
            _pTF_pose.poseTFy = ts_msg.transform.translation.y;
            _pTF_pose.poseTFz = ts_msg.transform.translation.z;
            _pTF_pose.poseTFqx = ts_msg.transform.rotation.x;
            _pTF_pose.poseTFqy = ts_msg.transform.rotation.y;
            _pTF_pose.poseTFqz = ts_msg.transform.rotation.z;
            _pTF_pose.poseTFqw = ts_msg.transform.rotation.w;
                
        }

        catch (tf::TransformException ex)
        {
            ROS_ERROR("[TF_Transform_Error(map to base_footprint)]: %s", ex.what());
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        loop_rate.sleep();
    }

    return 0;
}