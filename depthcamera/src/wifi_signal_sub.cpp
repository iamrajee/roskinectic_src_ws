#include <rtabmap_ros/MapData.h>
#include <rtabmap_ros/MsgConversion.h>

std::map<double, int> wifiLevels;
void mapDataCallback(const rtabmap_ros::MapDataConstPtr & mapDataMsg)
{
   // Convert MapData message in more user friendly objects
   rtabmap::Transform mapToOdom;
   std::map<int, rtabmap::Transform> poses;
     std::multimap<int, rtabmap::Link> links;
     std::map<int, rtabmap::Signature> signatures;
  
     rtabmap_ros::mapDataFromROS(*mapDataMsg, poses, links, signatures, mapToOdom);
  
     // The user data of the latest node added to 
     // map is contained in signatures map. Iterate over 
     // to get the data. Handle the case where we can 
     // receive only latest data, or if all data are published
     for(std::map<int, rtabmap::Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
     {
        int id = iter->first;
        rtabmap::Signature & node = iter->second;
  
        if(!node.sensorData().userDataCompressed().empty())
        {
           cv::Mat data;
           node.sensorData().uncompressDataConst(0 ,0, 0, &data);
  
           if(data.type() == CV_64FC1 && data.rows == 1 && data.cols == 2)
           {
              // format [int level, double stamp], see wifi_signal_pub_node.cpp
              int level = data.at<double>(0);
              double stamp = data.at<double>(1);
              wifiLevels.insert(std::make_pair(stamp, level));
           }
           else if(!data.empty())
           {
              ROS_ERROR("Wrong user data format for wifi signal.");
           }
        }
     }
  
     // "wifiLevels" contains all signal values sorted 
     // by timestamp. We can then find their exact position 
     // on the graph. See referred source code for details...
  }
  
  
  
  int main(int argc, char** argv)
  {
     //... node initialization stuff
  
     ros::Subscriber mapDataSub = nh.subscribe("/rtabmap/mapData", 1, mapDataCallback);
  
     ros::spin();
  }