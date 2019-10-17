#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <iostream>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

  PointCloud2 syn_pointcloud;
  Imu syn_imu;

void callback(const PointCloud2ConstPtr& ori_pointcloud, const ImuConstPtr& ori_imu)
{
  // Solve all of perception here...
  syn_pointcloud = *ori_pointcloud;
  syn_imu = *ori_imu;
  cout << "syn velodyne points' timestamp : " << syn_pointcloud.header.stamp << endl;
  cout << "syn Imu's timestamp : " << syn_imu.header.stamp << endl;
  ROS_INFO("pointcloud stamp value is: %d", syn_pointcloud.header.stamp);
  ROS_INFO("imu stamp value is: %d", syn_imu.header.stamp);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_synchronizer");
  ros::NodeHandle nh;

  message_filters::Subscriber<Imu> imu_sub(nh, "/imu/data", 1);
  message_filters::Subscriber<PointCloud2> velodyne_sub(nh, "/velodyne_points", 1);
  typedef sync_policies::ApproximateTime<PointCloud2, Imu> MySyncPolicy;    
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), velodyne_sub, imu_sub); //queue size=10
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}



#include "ros/ros.h"	//包含了使用ROS节点的必要文件
#include "std_msgs/String.h"	//包含了使用的数据类型
#include <sstream>
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_a");	//初始化ROS，节点名命名为node_a，节点名必须保持唯一
	ros::NodeHandle n;	//实例化节点, 节点进程句柄
	ros::Publisher pub = n.advertise<std_msgs::String>("str_message", 1000);	//告诉系统要发布话题了，话题名为“str_message”，类型为std_msgs::String，缓冲队列为1000。
ros::Rate loop_rate(10);	//设置发送数据的频率为10Hz
//ros::ok()返回false会停止运行，进程终止。
	while(ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss<<"Hello World";
		msg.data = ss.str();
		ROS_INFO("node_a is publishing %s", msg.data.c_str());
		pub.publish(msg);	//向话题“str_message”发布消息
		ros::spinOnce();	//不是必须，若程序中订阅话题则必须，否则回掉函数不起作用。
		loop_rate.sleep();	//按前面设置的10Hz频率将程序挂起
	}
 
	return 0;
}



    #include <ros/ros.h>  
      
    class SubscribeAndPublish  
    {  
    public:  
      SubscribeAndPublish()  
      {  
        //Topic you want to publish  
        pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);  
      
        //Topic you want to subscribe  
        sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);  //注意这里，和平时使用回调函数不一样了。
      }  
      
      void callback(const SUBSCRIBED_MESSAGE_TYPE& input)  
      {  
        PUBLISHED_MESSAGE_TYPE output;  
        //.... do something with the input and generate the output...  
        pub_.publish(output);  
      }  
      
    private:  
      ros::NodeHandle n_;   
      ros::Publisher pub_;  
      ros::Subscriber sub_;  
      
    }//End of class SubscribeAndPublish  
      
    int main(int argc, char **argv)  
    {  
      //Initiate ROS  
      ros::init(argc, argv, "subscribe_and_publish");  
      
      //Create an object of class SubscribeAndPublish that will take care of everything  
      SubscribeAndPublish SAPObject;  
      
      ros::spin();  
      
      return 0;  
    }  
