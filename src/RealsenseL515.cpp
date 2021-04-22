#include "RealsenseL515.h"

RealsenseL515::RealsenseL515()
: pc_{ new pcl::PointCloud<pcl::PointXYZ>() }
{

}

RealsenseL515::~RealsenseL515() = default;

bool RealsenseL515::capture(const tf::StampedTransform& transform)
{
    sensor_msgs::PointCloud2::ConstPtr pc_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/realsense/depth/points");

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pc_msg, *pc);
    pcl_ros::transformPointCloud(*pc, *pc, transform);

    filter(pc);

    *pc_ += *pc;
    header_ = pc_msg->header;
}

sensor_msgs::PointCloud2 RealsenseL515::getPointcloud(const tf::StampedTransform& transform)
{
    sensor_msgs::PointCloud2 pc_msg;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_ros::transformPointCloud(*pc_, *pc, transform);

    pcl::toROSMsg(*pc, pc_msg);
    pc_msg.header = header_;

    return pc_msg;
}

void RealsenseL515::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
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
}