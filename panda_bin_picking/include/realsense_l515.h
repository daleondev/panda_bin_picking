#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>

/**
 * Abstraction of the Intel RealSense LiDAR Camera L515
 * 
 * Allows capturing pointclouds and processing them
 */
class RealsenseL515 {
public:
    /**
     * Constructor
     */
    RealsenseL515();
    /**
     * Destructor
     */
    ~RealsenseL515();

    /**
     * Captures and preprocesses a pointcloud
     * 
     * @param[in] transform the transformation into a desired frame for preprocessing
     */
    void capturePointcloud(const tf::StampedTransform& transform);
    /**
     * Fuses all previously captured pointclouds into one big cloud
     */
    void fusePointclouds();
    /**
     * Clears all previously captured pointclouds
     */
    void clearPointclouds();

    /**
     * Get the fused pointcloud
     * 
     * @return PointCloud2 the fused pointcloud
     */
    sensor_msgs::PointCloud2 getPointcloud() const;
    /**
     * Get the depth frame name
     * 
     * @return string the depth frame name
     */
    std::string getDepthFrame() const;

    /**
     * Saves the captured pointclouds and the fused cloud to a file (for debugging)
     */
    void savePointclouds() const;

private:
    /**
     * Extracts area of interest from a pointcloud
     * 
     * @param[in,out] pc the pointcloud to modify
     */
    void extractPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const;
    /**
     * Filters a pointcloud
     * 
     * @param[in,out] pc the pointcloud to modify
     */
    void filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const;
    /**
     * Thins out a pointcloud
     * 
     * @param[in] x the threshold in x direction
     * @param[in] y the threshold in y direction
     * @param[in] z the threshold in z direction
     * @param[in,out] pc the pointcloud to modify
     */
    void thinOutPointcloud(const float x, const float y, const float z, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const;

    /**
     * Contains all captured pointclouds
     */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
    /**
     * The fused pointcloud
     */
    sensor_msgs::PointCloud2 pc_;

    /**
     * Depth frame name
     */
    static const std::string DEPTH_FRAME;
    /**
     * Topic for receiving pointclouds
     */
    static const std::string PC_TOPIC;
};
