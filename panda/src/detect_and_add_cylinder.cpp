#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

/*
co : collision_object
cp : cylinder_pose
s : planning_scene_interface
 */

class CylinderSegment
{
  public:
  CylinderSegment()
  {
    ros::NodeHandle nh;
    //ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 1, &CylinderSegment::cloudCB, this);
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, &CylinderSegment::cloudCB, this);
    ros::spin();
  }

  //====================== given cylinder_params, add the cylinder to the s ============================
  void addCylinder()
  {
    //define collision object
    moveit::planning_interface::PlanningSceneInterface s;
    moveit_msgs::CollisionObject co;
    //co.header.frame_id = "camera_link";
    co.header.frame_id = "camera_rgb_optical_frame";
    co.id = "cylinder";

    //primitive
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = cylinder_params->height;
    primitive.dimensions[1] = cylinder_params->radius;

    //find orient from vector dir
    geometry_msgs::Pose cp;
    Eigen::Vector3d cylinder_z_direction(cylinder_params->direction_vec[0], cylinder_params->direction_vec[1], cylinder_params->direction_vec[2]);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    cp.orientation.x = axis.x() * sin(angle / 2);
    cp.orientation.y = axis.y() * sin(angle / 2);
    cp.orientation.z = axis.z() * sin(angle / 2);
    cp.orientation.w = cos(angle / 2);

    cp.position.x = cylinder_params->center_pt[0];
    cp.position.y = cylinder_params->center_pt[1];
    cp.position.z = cylinder_params->center_pt[2];

    //add
    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(cp);
    co.operation = co.ADD;
    s.applyCollisionObject(co);
  }

  // ==================== Given the pointcloud containing just the cylinder, compute its center point and its height and store in cylinder_params. =========
  void extractLocationHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    double max_angle_y = 0.0;
    double min_angle_y = std::numeric_limits<double>::infinity();
    double lowest_point[3];
    double highest_point[3];
    
    for (auto const point : cloud->points)// Loop over the entire pointcloud.
    {
      if (atan2(point.z, point.y) < min_angle_y)//Find the coordinates of the highest point
      {
        min_angle_y = atan2(point.z, point.y);
        lowest_point[0] = point.x;
        lowest_point[1] = point.y;
        lowest_point[2] = point.z;
      }
      else if (atan2(point.z, point.y) > max_angle_y)//Find the coordinates of the lowest point
      {
        max_angle_y = atan2(point.z, point.y);
        highest_point[0] = point.x;
        highest_point[1] = point.y;
        highest_point[2] = point.z;
      }
    }
    //Store the center point of cylinder
    cylinder_params->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
    cylinder_params->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
    cylinder_params->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
    /* Store the height of cylinder */
    cylinder_params->height = sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) + pow((lowest_point[2] - highest_point[2]), 2));
  }

  //======================== Given a pointcloud extract the ROI defined by the user. ===================================================
  void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    //pass.setFilterLimits(-0.10, 0.30);// min and max values in z axis to keep //-10cm,30cm
    pass.setFilterLimits(0.3, 1.1);
    pass.filter(*cloud);
  }

  // ============================== Given the pointcloud compute the point normals and store in cloud_normals. ================
  void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(50);// Set the number of k nearest neighbors to use for the feature estimation.
    ne.compute(*cloud_normals);
  }

  // ====================== Given the point normals and point indices, extract the normals for the indices. =================//
  void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals);
  }

  // ================== Given the pointcloud and indices of the plane, remove the plannar region from the pointcloud. ===================//
  void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers_plane)
  {
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;// create a SAC segmenter without using normals
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setMaxIterations(1000);//run at max 1000 iterations before giving up
    segmentor.setDistanceThreshold(0.01);//tolerance for variation from model
    segmentor.setInputCloud(cloud);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);//Create the segmentation object for the planar model and set all the parameters
    segmentor.segment(*inliers_plane, *coefficients_plane);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;//Extract the planar inliers from the input cloud
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(true);//Remove the planar inliers, extract the rest
    extract_indices.filter(*cloud);
  }

  //==== Given cloud, ModelCoefficients and cloud_normals extract the cylinder from the pointcloud and store the cylinder param in coefficients_cylinder.====
  void extractCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;// Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setNormalDistanceWeight(0.1);// Set the normal angular distance weight
    segmentor.setMaxIterations(10000);// run at max 1000 iterations before giving up
    segmentor.setDistanceThreshold(0.05);// tolerance for variation from model
    //segmentor.setRadiusLimits(0.05, 0.10);// min max values of radius in meters to consider //5cm - 10cm
    segmentor.setRadiusLimits(0, 1);
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);
    segmentor.segment(*inliers_cylinder, *coefficients_cylinder);// Obtain the cylinder inliers and coefficients
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;// Extract the cylinder inliers from the input cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud);
  }
  // ========== convert from sensor_msgs to pcl::PointXYZRGB ==================================//
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    printf("\n===> converting sensor_msgs to PointXYZRGB...");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);
    printf("\n===> passthough filter...");
    passThroughFilter(cloud);// Using passthough filter to get region of interest. A passthrough filter just eliminates the point cloud values which do not lie in the user specified range.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);// Declare normals and call function to compute point normals.
    printf("\n===> computeNormals...");
    computeNormals(cloud, cloud_normals);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);// inliers_plane will hold the indices of the point cloud that correspond to a plane.
    printf("\n===> removePlaneSurface...");
    removePlaneSurface(cloud, inliers_plane);// Detect and eliminate the plane on which the cylinder is kept to ease the process of finding the cylinder.
    printf("\n===> extractNormals...");
    extractNormals(cloud_normals, inliers_plane);// extracting the normals that correspond to the plane on which cylinder lies.
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);// ModelCoefficients will hold cylinder params
    printf("\n===> extractCylinder...");
    extractCylinder(cloud, coefficients_cylinder, cloud_normals);//Extract the cylinder using SACSegmentation.
    if (cloud->points.empty())
    {
      ROS_ERROR_STREAM_NAMED("cylinder_segment", "Can't find the cylindrical component.");
      return;
    }
    if (points_not_found)
    {
      cylinder_params->radius = coefficients_cylinder->values[6];
      cylinder_params->direction_vec[0] = coefficients_cylinder->values[3];
      cylinder_params->direction_vec[1] = coefficients_cylinder->values[4];
      cylinder_params->direction_vec[2] = coefficients_cylinder->values[5];
      printf("\n===> extractLocationHeight...");
      extractLocationHeight(cloud);
      printf("\n===> addCylinder...");
      addCylinder();// Use the parameters extracted to add the cylinder to the planning scene as a collision object.
      points_not_found = false;
    }
  }

  private:
  //=========================================== cylinder_params =====================================================//
  struct AddCylinderParams
  {
    double radius;
    double direction_vec[3];//Direction vector towards the z-axis of the cylinder
    double center_pt[3];
    double height;
  };
  AddCylinderParams* cylinder_params;
  bool points_not_found = true;
};

//============================================================= MAIN =================================================//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cylinder_segment");
  CylinderSegment();
}
