//
// Created by huyh on 18-12-28.
// Imprvd by lxb on 19-6

#include <iostream>
#include <time.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
/// MESSAGE FILTER
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace nav_msgs;
using namespace sensor_msgs;
using namespace message_filters;

void robot1SyncCB(const OdometryConstPtr odom, const PointCloud2ConstPtr pointcloud,const CameraInfoConstPtr& cameraInfo , const ImageConstPtr imageRaw, 
ros::Publisher pubO,ros::Publisher pubP,ros::Publisher pubC,ros::Publisher pubI)
{


    	std::cout<<"robot1SyncCB"<<std::endl;
	Odometry O;
	PointCloud2 P;
	CameraInfo c;	
	Image I;

	O= *odom;
	P= *pointcloud;
	c= *cameraInfo;
	I= *imageRaw; 

 	ROS_INFO("first value is: %d", O.header.stamp);
	ROS_INFO("second value is: %d",P.header.stamp);
 	ROS_INFO("third value is: %d", c.header.stamp);
 	ROS_INFO("fourth value is: %d",I.header.stamp);
	
	pubO.publish(O);
	pubP.publish(P);
	pubC.publish(c);
	pubI.publish(I);

}

void robot2SyncCB(const OdometryConstPtr odom2, const PointCloud2ConstPtr pointcloud2,const CameraInfoConstPtr& cameraInfo2 , const ImageConstPtr imageRaw2, 
ros::Publisher pubO2,ros::Publisher pubP2,ros::Publisher pubC2,ros::Publisher pubI2)
{


    	std::cout<<"robot2SyncCB"<<std::endl;
	Odometry O2;
	PointCloud2 P2;
	CameraInfo c2;	
	Image I2;

	O2= *odom2;
	P2= *pointcloud2;
	c2= *cameraInfo2;
	I2= *imageRaw2; 

 	ROS_INFO("first value is: %d", O2.header.stamp);
 	ROS_INFO("second value is: %d",P2.header.stamp);
 	ROS_INFO("third value is: %d", c2.header.stamp);
 	ROS_INFO("fourth value is: %d",I2.header.stamp);
	
	pubO2.publish(O2);
	pubP2.publish(P2);
	pubC2.publish(c2);
	pubI2.publish(I2);

}

void robot3SyncCB(const OdometryConstPtr odom3, const PointCloud2ConstPtr pointcloud3,const CameraInfoConstPtr& cameraInfo3 , const ImageConstPtr imageRaw3, 
ros::Publisher pubO3,ros::Publisher pubP3,ros::Publisher pubC3,ros::Publisher pubI3)
{


    	std::cout<<"robot3SyncCB"<<std::endl;
	Odometry O3;
	PointCloud2 P3;
	CameraInfo c3;	
	Image I3;

	O3= *odom3;
	P3= *pointcloud3;
	c3= *cameraInfo3;
	I3= *imageRaw3; 

 	ROS_INFO("first value is: %d", O3.header.stamp);
 	ROS_INFO("second value is: %d",P3.header.stamp);
 	ROS_INFO("third value is: %d", c3.header.stamp);
 	ROS_INFO("fourth value is: %d",I3.header.stamp);
	
	pubO3.publish(O3);
	pubP3.publish(P3);
	pubC3.publish(c3);
	pubI3.publish(I3);

}

int main(int argc, char** argv){

        ros::init(argc, argv,"data_align");

	ros::NodeHandle nh("~");

        ros::Rate loop_rate(10);

	ros::Publisher pubO = nh.advertise<nav_msgs::Odometry>("/robot0/velodyne/ground_truth/odometry", 1000);
	ros::Publisher pubP = nh.advertise<PointCloud2>("/robot0/velodyne/velodyne_points", 1000);
	ros::Publisher pubC = nh.advertise<CameraInfo>("/robot0/vi_sensor/camera_left/camera_info",1000);
	ros::Publisher pubI = nh.advertise<Image>("/robot0/vi_sensor/camera_left/image_raw", 1000);

	ros::Publisher pubO2 = nh.advertise<nav_msgs::Odometry>("/robot1/velodyne/ground_truth/odometry", 1000);
	ros::Publisher pubP2 = nh.advertise<PointCloud2>("/robot1/velodyne/velodyne_points", 1000);
	ros::Publisher pubC2 = nh.advertise<CameraInfo>("/robot1/vi_sensor/camera_left/camera_info",1000);
	ros::Publisher pubI2 = nh.advertise<Image>("/robot1/vi_sensor/camera_left/image_raw", 1000);

	ros::Publisher pubO3 = nh.advertise<nav_msgs::Odometry>("/robot2/velodyne/ground_truth/odometry", 1000);
	ros::Publisher pubP3 = nh.advertise<PointCloud2>("/robot2/velodyne/velodyne_points", 1000);
	ros::Publisher pubC3 = nh.advertise<CameraInfo>("/robot2/vi_sensor/camera_left/camera_info",1000);
	ros::Publisher pubI3 = nh.advertise<Image>("/robot2/vi_sensor/camera_left/image_raw", 1000);

    message_filters::Subscriber<nav_msgs::Odometry> robot1_velo_gt_odom(nh, "/firefly1/velodyne/ground_truth/odometry", 1);
    message_filters::Subscriber<PointCloud2> robot1_velo_points(nh, "/firefly1/velodyne/velodyne_points", 1);
    message_filters::Subscriber<CameraInfo> robot1_visen_cam_info(nh, "/firefly1/vi_sensor/camera_left/camera_info", 1);
    message_filters::Subscriber<Image> robot1_visen_cam_img(nh, "/firefly1/vi_sensor/camera_left/image_raw", 1);

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry,PointCloud2,CameraInfo, Image> approximateSync;

    Synchronizer<approximateSync> robot1_sync(approximateSync(50),robot1_velo_gt_odom, robot1_velo_points,robot1_visen_cam_info,robot1_visen_cam_img);
    robot1_sync.registerCallback(boost::bind(&robot1SyncCB, _1, _2,_3,_4,pubO,pubP,pubC,pubI));




    message_filters::Subscriber<nav_msgs::Odometry> robot2_velo_gt_odom(nh, "/firefly2/velodyne/ground_truth/odometry", 1);
    message_filters::Subscriber<PointCloud2> robot2_velo_points(nh, "/firefly2/velodyne/velodyne_points", 1);
    message_filters::Subscriber<CameraInfo> robot2_visen_cam_info(nh, "/firefly2/vi_sensor/camera_left/camera_info", 1);
    message_filters::Subscriber<Image> robot2_visen_cam_img(nh, "/firefly2/vi_sensor/camera_left/image_raw", 1);
    Synchronizer<approximateSync> robot2_sync(approximateSync(50),robot2_velo_gt_odom, robot2_velo_points,robot2_visen_cam_info,robot2_visen_cam_img);
    robot2_sync.registerCallback(boost::bind(&robot2SyncCB, _1, _2,_3,_4,pubO2,pubP2,pubC2,pubI2));

    
    message_filters::Subscriber<nav_msgs::Odometry> robot3_velo_gt_odom(nh, "/firefly3/velodyne/ground_truth/odometry", 1);
    message_filters::Subscriber<PointCloud2> robot3_velo_points(nh, "/firefly3/velodyne/velodyne_points", 1);
    message_filters::Subscriber<CameraInfo> robot3_visen_cam_info(nh, "/firefly3/vi_sensor/camera_left/camera_info", 1);
    message_filters::Subscriber<Image> robot3_visen_cam_img(nh, "/firefly3/vi_sensor/camera_left/image_raw", 1);
    Synchronizer<approximateSync> robot3_sync(approximateSync(50),robot3_velo_gt_odom, robot3_velo_points,robot3_visen_cam_info,robot3_visen_cam_img);
    robot3_sync.registerCallback(boost::bind(&robot3SyncCB, _1, _2,_3,_4,pubO3,pubP3,pubC3,pubI3));


   while(ros::ok()){

        ros::spinOnce();
     loop_rate.sleep();	
 	   }

    return 0;
}



/*
    /firefly1/velodyne/ground_truth/odometry

    /firefly1/velodyne/velodyne_points
    /firefly1/vi_sensor/camera_left/camera_info
    /firefly1/vi_sensor/camera_left/image_raw

    /firefly2/velodyne/ground_truth/odometry

    /firefly2/velodyne/velodyne_points
    /firefly2/vi_sensor/camera_left/camera_info
    /firefly2/vi_sensor/camera_left/image_raw


    /firefly3/velodyne/ground_truth/odometry

    /firefly3/velodyne/velodyne_points
    /firefly3/vi_sensor/camera_left/camera_info
    /firefly3/vi_sensor/camera_left/image_raw


hhg@slam-1:~$ rostopic type /firefly1/velodyne/ground_truth/odometry 
nav_msgs/Odometry

hhg@slam-1:~$ rostopic type /firefly1/velodyne/velodyne_points 
sensor_msgs/PointCloud2



hhg@slam-1:~$ rostopic type /firefly1/vi_sensor/camera_left/camera_info 
sensor_msgs/CameraInfo

hhg@slam-1:~$ rostopic type /firefly1/vi_sensor/camera_left/image_raw
sensor_msgs/Image


*/




/*
G2oInterface g2o_graph;

bool initialization_1 = false;
bool initialization_2 = false;
int optimization_cnt = 0;

void odomCB_1(const nav_msgs::OdometryConstPtr &odom_msgs){
    g2o_graph.UpdateOdomEstMsgs(odom_msgs,"robot1");
    initialization_1 = true;
}

void odomCB_2(const nav_msgs::OdometryConstPtr &odom_msgs){
    g2o_graph.UpdateOdomEstMsgs(odom_msgs,"robot2");
    initialization_2 = true;
}

void loopClosureCB(const geometry_msgs::PoseStampedConstPtr &loop_msgs){
    g2o_graph.addLoopClousreEdge(loop_msgs);
}
*/
/*
void robot1SyncCB(const nav_msgs::OdometryConstPtr robot1_odom_est , const nav_msgs::OdometryConstPtr robot1_odom_gt){
    std::cout<<"robot1SyncCB"<<std::endl;
    g2o_graph.UpdateOdomEstMsgs(robot1_odom_est,"robot1");
    g2o_graph.UpdateOdomGtMsgs(robot1_odom_gt,"robot1");
}

void robot2SyncCB(const nav_msgs::OdometryConstPtr robot2_odom_est , const nav_msgs::OdometryConstPtr robot2_odom_gt){
    std::cout<<"robot2SyncCB"<<std::endl;
    g2o_graph.UpdateOdomEstMsgs(robot2_odom_est,"robot2");
    g2o_graph.UpdateOdomGtMsgs(robot2_odom_gt,"robot2");
}

void robot3SyncCB(const nav_msgs::OdometryConstPtr robot3_odom_est , const nav_msgs::OdometryConstPtr robot3_odom_gt){
    std::cout<<"robot3SyncCB"<<std::endl;
    g2o_graph.UpdateOdomEstMsgs(robot3_odom_est,"robot3");
    g2o_graph.UpdateOdomGtMsgs(robot3_odom_gt,"robot3");
}
*/
/*
void poseCB(const geometry_msgs::PoseStampedConstPtr &pose_msgs){
    if(initialization_1&initialization_2) {
//        geometry_msgs::PoseStampedPtr loop_msgs;
//        loop_msgs->pose.orientation = pose_msgs->pose.orientation;
//        loop_msgs->pose.position = pose_msgs->pose.position;
        if (pose_msgs) {
//            if (pose_msgs->pose.position.z < 1) {
                g2o_graph.addLoopClousreEdge(pose_msgs);
                optimization_cnt++;
//            }
        }
    }
}

*/
