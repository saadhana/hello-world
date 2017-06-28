#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

int
main (int argc, char** argv)
{
    std::string projectSrcDir = PROJECT_SOURCE_DIR;
    
    // Load [.pcd::] file to pcl::PointCloud
    std::string modelFilename = projectSrcDir + "/data/bun0.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (modelFilename, *pointCloud) == -1) 
    {
        std::cout<< "Couldn't read file test_pcd.pcd \n";
        return (-1);
    }
    
    // Visualize the point clouds
    pcl::visualization::CloudViewer cloudViewer ("Cloud Viewer");
    cloudViewer.showCloud (pointCloud);
    while(!cloudViewer.wasStopped()) 
    {
        
    }
    // Compute normals of the point clouds
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(pointCloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ> ());
    normalEstimation.setSearchMethod (searchTree);
    pcl::PointCloud<pcl::Normal>::Ptr normalCloud (new pcl::PointCloud<pcl::Normal>);
    normalEstimation.setRadiusSearch (0.03);
    normalEstimation.compute (*normalCloud);
    // Visualize the point clouds with computed normals
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visViewer (new pcl::visualization::PCLVisualizer ("Visualization Viewer"));
    visViewer->addPointCloud<pcl::PointXYZ> (pointCloud,  "sample cloud");
    visViewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (pointCloud, normalCloud, 1, 0.008, "normals");
    while (!visViewer->wasStopped ())
    {
        visViewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    return 0;
}



