


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>


void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visViewer (new pcl::visualization::PCLVisualizer ("result Viewer"));
    visViewer->addPointCloud<pcl::PointXYZRGB> (cloud,  "result cloud");
    while (!visViewer->wasStopped ())
    {
        visViewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
}

int
main (int argc, char** argv)
{
    std::string projectSrcDir = PROJECT_SOURCE_DIR;
    // Load point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ref (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trg (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (projectSrcDir + "/data/pointcloud_chair0.pcd", *cloud_ref);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (projectSrcDir + "/data/pointcloud_chair1.pcd", *cloud_trg);
    
    //visualize(cloud_ref);
    //visualize(cloud_trg);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visViewer (new pcl::visualization::PCLVisualizer ("initViewer"));
    visViewer->addPointCloud<pcl::PointXYZRGB> (cloud_ref,  "ref cloud");
    visViewer->addPointCloud<pcl::PointXYZRGB> (cloud_trg,  "trg cloud");
    while (!visViewer->wasStopped ())
    {
        visViewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    // Register two point clouds using ICP according to slides
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> iterativeClosestPoint;
    iterativeClosestPoint.setInputSource(cloud_ref);
    iterativeClosestPoint.setInputTarget(cloud_trg);
    pcl::PointCloud<pcl::PointXYZRGB> result;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
    iterativeClosestPoint.align(result);
    result = result + *cloud_trg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr resPtr(&result);
    // Visualize the registered point clouds
    visualize(resPtr); 
    
    return 0;
}




