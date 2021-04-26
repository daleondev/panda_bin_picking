#pragma once

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
    void filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const;
    void thinOutPointcloud(const float x, const float y, const float z, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;
    
    static const std::string DEPTH_FRAME;
    static const std::string PC_TOPIC;
};
