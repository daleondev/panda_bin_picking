#include "RealsenseL515.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

RealsenseL515::RealsenseL515()
: pc_{ new pcl::PointCloud<pcl::PointXYZ>() }
{

}

RealsenseL515::~RealsenseL515() = default;

void RealsenseL515::capture(const tf::StampedTransform& transform)
{
    sensor_msgs::PointCloud2::ConstPtr pc_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/realsense/depth/points");

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pc_msg, *pc);
    pcl_ros::transformPointCloud(*pc, *pc, transform);

    filter(pc);

    *pc_ += *pc;
}

sensor_msgs::PointCloud2 RealsenseL515::getPointcloud(const tf::StampedTransform* transform) const
{
    sensor_msgs::PointCloud2 pc_msg;   

    if (transform) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl_ros::transformPointCloud(*pc_, *pc, *transform);
        pcl::toROSMsg(*pc, pc_msg);
    } else {
        pcl::toROSMsg(*pc_, pc_msg);
    }

    return pc_msg;
}

std::string RealsenseL515::getDepthFrame() const
{
    return DEPTH_FRAME;
}

void RealsenseL515::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const
{
    pcl::PointIndices::Ptr toRemove(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    for (int i = 0; i < pc->size(); i++)
    {
        if (pc->points[i].z < -0.5 || !std::isfinite(pc->points[i].x) || !std::isfinite(pc->points[i].y) || !std::isfinite(pc->points[i].z)) {
            toRemove->indices.push_back(i);
        }
    }
    extract.setInputCloud(pc);
    extract.setIndices(toRemove);
    extract.setNegative(true);
    extract.filter(*pc);

    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setInputCloud(pc);
    grid.setLeafSize (0.01f, 0.01f, 0.01f);
    grid.filter(*pc);
}

void RealsenseL515::savePointcloud(const std::string& file) const
{
    pcl::io::savePCDFileASCII(file, *pc_);
}