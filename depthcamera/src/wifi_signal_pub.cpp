#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/UserData.h>

int main(int argc, char** argv)
{
   //... node initialization stuff

   ros::Rate rate(0.5); 

     ros::Publisher wifiPub = nh.advertise<rtabmap_ros::UserData>("wifi_signal", 1);
     
     while(ros::ok())
     {
        ros::Time stamp = ros::Time::now();
  
        // Get signal strength...
        int dBm = ...
  
        // Create user data [level, stamp].
        // OpenCV matrix is used for convenience. Any format is accepted.
        // However, if CV_8UC1 format is used, make sure rows > 1 as 
        // rtabmap will think it is already compressed.
        cv::Mat data(1, 2, CV_64FC1);
        data.at<double>(0) = double(dBm);
  
        // We should set stamp in data to be able to
        // retrieve it from the saved user data as we need 
        // to get precise position in the graph afterward.
        data.at<double>(1) = stamp.toSec();
  
        rtabmap_ros::UserData dataMsg;
        dataMsg.header.frame_id = "wifi_link";
        dataMsg.header.stamp = stamp;
  
        // Convert OpenCV matrix to UserData msg. Optionally, 
        // we can choose to compress the data now (here we keep uncompressed).
        rtabmap_ros::userDataToROS(data, dataMsg, false);
        
        // Publish data!
        wifiPub.publish<rtabmap_ros::UserData>(dataMsg);
        
        ros::spinOnce();
        rate.sleep();
     }
     return 0;
  }