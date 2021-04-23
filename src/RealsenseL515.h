#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

class RealsenseL515 {
public:
    RealsenseL515();
    ~RealsenseL515();

    void capture(const tf::StampedTransform& transform);

    sensor_msgs::PointCloud2 getPointcloud(const tf::StampedTransform* transform = nullptr) const;
    std::string getDepthFrame() const;

    void savePointcloud(const std::string& file) const;

private:
    void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;
    
    const std::string DEPTH_FRAME = "/camera_depth_optical_frame";
    const std::string PC_TOPIC = "/realsense/depth/points";
};