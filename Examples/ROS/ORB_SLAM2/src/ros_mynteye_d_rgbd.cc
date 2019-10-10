/**
小觅深度 ROS
qsh 2019-05-14
https://blog.csdn.net/qq_30356613/article/details/76409367
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    //初始化ROS系统
    ros::init(argc, argv, "RGBD");
    //ROS启动函数
    ros::start();
    //检测给定参数数量是否完整
    if(argc != 3)
    {
        cerr << endl << "用法: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

   /* //参数输出，注释掉
   cout << " 参数: " << endl
    	 << " 字典路径: " << argv[1] << endl
	     << " 配置文件路径: " << argv[2] << endl;
       */
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // 创建SLAM系统，初始化所有系统线程并准备进程帧
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    //定义捕获图像的对象
    ImageGrabber igb(&SLAM);
    //建立ROS节点
    ros::NodeHandle nh;
    //Subscriber滤波器作为ROS消息的最顶层，不能够将其他滤波器的输出作为其输入
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/mynteye/left/image_color", 1);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/mynteye/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
    //下边两条指令是为了将rgb_sub和depth_sub进行数据对齐，滤掉不对齐的帧数
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    //采用registerCallback()方法，注册回调函数ImageGrabber::GrabRGBD。
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    //此函数用于ROS不断回调节点mageGrabber::GrabRGBD函数
    ros::spin();

    // Stop all threads
    //停止所有线程
    SLAM.Shutdown();

    // Save camera trajectory
    //保存相机轨迹
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

//以上类和代码主要就是将采集到的数据检测是否有异常，然后将采集到的数据给system，供整个系统使用。
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}
