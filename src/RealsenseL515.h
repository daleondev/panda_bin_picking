#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>

class RealsenseL515 {
public:
    RealsenseL515();
    ~RealsenseL515();

    bool capture(const tf::StampedTransform& transform);

    sensor_msgs::PointCloud2 getPointcloud(const tf::StampedTransform* transform = nullptr) const;
    std::string getDepthFrame() const;

private:
    void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;
    
    const std::string DEPTH_FRAME = "/camera_depth_optical_frame";
};