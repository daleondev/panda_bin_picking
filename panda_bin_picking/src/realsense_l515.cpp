#include "realsense_l515.h"
#include "custom_types.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

const std::string RealsenseL515::DEPTH_FRAME =  "/camera_depth_optical_frame";
const std::string RealsenseL515::PC_TOPIC =     "/camera/depth/color/points";

RealsenseL515::RealsenseL515() = default;

RealsenseL515::~RealsenseL515() = default;

void RealsenseL515::capturePointcloud(const tf::StampedTransform& transform)
{
    PRINT_METHOD();

    // wait for new pointcloud on topic
    sensor_msgs::PointCloud2::ConstPtr pc_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(PC_TOPIC);

    // convert ros msg to pcl pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pc_msg, *pc);

    // extract area of interest from pointcloud
    extractPointcloud(pc);

    // transform pointcloud (e.g. to world frame)
    pcl_ros::transformPointCloud(*pc, *pc, transform);
     
    // filter pointcloud
    filterPointcloud(pc);

    // save pointcloud
    clouds_.push_back(pc);
}

void RealsenseL515::fusePointclouds()
{
    PRINT_METHOD();

    // abort if no pointclouds
    if (clouds_.empty()) {
        return;
    }
    
    // add all pointclouds together
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    for (const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud : clouds_) {
        *pc = *pc + *cloud;
    }

#ifdef SIMULATION
    thinOutPointcloud(0.005f, 0.005f, 0.005f, pc);
#endif

    pcl::toROSMsg(*pc, pc_);
}

void RealsenseL515::clearPointclouds()
{
    PRINT_METHOD();

    clouds_.clear();
}

sensor_msgs::PointCloud2 RealsenseL515::getPointcloud() const
{
    PRINT_METHOD();

    return pc_;
}

std::string RealsenseL515::getDepthFrame() const
{
    PRINT_METHOD();

    return DEPTH_FRAME;
}

void RealsenseL515::savePointclouds() const
{
    PRINT_METHOD();

    // for debugging

    // save individual pointclouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < clouds_.size(); ++i) {
        pcl::io::savePCDFileASCII("cloud_" + std::to_string(time(NULL)) + "_" + std::to_string(i) + ".pcd", *clouds_[i]);
        *pc = *pc + *clouds_[i];
    }

    // save fused pointcloud
    pcl::io::savePCDFile("cloud_" + std::to_string(time(NULL)) + "_fused.pcd", *pc);
}

void RealsenseL515::extractPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const
{
    PRINT_METHOD();

    // extract area of interest from pointcloud
    pcl::PointIndices::Ptr to_remove(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for (int i = 0; i < pc->size(); i++) {
        if (pc->points[i].x > 0.12 || pc->points[i].x < -0.12 || pc->points[i].y > 0.12 || pc->points[i].y < -0.12 || !std::isfinite(pc->points[i].x) || !std::isfinite(pc->points[i].y) || !std::isfinite(pc->points[i].z)) {
            to_remove->indices.push_back(i);
        }
    }
    if (!to_remove->indices.empty()) {
        extract.setInputCloud(pc);
        extract.setIndices(to_remove);
        extract.setNegative(true);
        extract.filter(*pc);
    }
}

void RealsenseL515::filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const
{
    PRINT_METHOD();

    // remove unwanted parts of the pointcloud
    pcl::PointIndices::Ptr to_remove(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for (int i = 0; i < pc->size(); i++) {
        if (pc->points[i].z < -0.03 || !std::isfinite(pc->points[i].x) || !std::isfinite(pc->points[i].y) || !std::isfinite(pc->points[i].z)) {
            to_remove->indices.push_back(i);
        }
    }
    if (!to_remove->indices.empty()) {
        extract.setInputCloud(pc);
        extract.setIndices(to_remove);
        extract.setNegative(true);
        extract.filter(*pc);
    }

    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    // filter.setInputCloud(pc);
    // filter.setMeanK(20);
    // filter.setStddevMulThresh(1.0);
    // filter.filter(*pc);
}

void RealsenseL515::thinOutPointcloud(const float x, const float y, const float z, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const
{
    PRINT_METHOD();

    pcl::VoxelGrid<pcl::PointXYZ> grid;

    grid.setInputCloud(pc);
    grid.setLeafSize(x, y, z);
    grid.filter(*pc);
}