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
    sensor_msgs::PointCloud2::ConstPtr pc_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(PC_TOPIC);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pc_msg, *pc);
    pcl_ros::transformPointCloud(*pc, *pc, transform);

    filterPointcloud(pc);
    thinOutPointcloud(0.01f, 0.01f, 0.01f, pc);

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

void RealsenseL515::savePointcloud(const std::string& file) const
{
    pcl::io::savePCDFileASCII(file, *pc_);
}

void RealsenseL515::filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const
{
    pcl::PointIndices::Ptr to_remove(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    for (int i = 0; i < pc->size(); i++)
    {
        if (pc->points[i].z < -0.5 || !std::isfinite(pc->points[i].x) || !std::isfinite(pc->points[i].y) || !std::isfinite(pc->points[i].z)) {
            to_remove->indices.push_back(i);
        }
    }
    extract.setInputCloud(pc);
    extract.setIndices(to_remove);
    extract.setNegative(true);
    extract.filter(*pc);
}

void RealsenseL515::thinOutPointcloud(const float x, const float y, const float z, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) const
{
    pcl::VoxelGrid<pcl::PointXYZ> grid;

    grid.setInputCloud(pc);
    grid.setLeafSize(x, y, z);
    grid.filter(*pc);
}