
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h> 

#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

void normal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals) {

    pcl::PointCloud<pcl::Normal>::Ptr normalCloud(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(pointCloud);
    normalEstimation.setSearchMethod(searchTree);
    normalEstimation.setRadiusSearch(0.03);
    normalEstimation.compute(*normalCloud);
    pcl::concatenateFields(*pointCloud, *normalCloud, *cloudWithNormals);
}

void visualize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string a) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visViewer(new pcl::visualization::PCLVisualizer(a));
    visViewer->addPointCloud<pcl::PointXYZRGB> (cloud, a);
    while (!visViewer->wasStopped()) {
        visViewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}

void visualizeWithNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, std::string a) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visViewer(new pcl::visualization::PCLVisualizer(a));
    visViewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud, a);
    while (!visViewer->wasStopped()) {
        visViewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

}


int
main(int argc, char** argv) {
    std::string projectSrcDir = PROJECT_SOURCE_DIR;

    std::cout<<"Works!";
    // Camera intrinsic parameters of depth camera
    float focal = 570.f; // focal length
    float px = 319.5f; // principal point x
    float py = 239.5f; // principal point y
    
    // loop for depth images on multiple view
    for (int i = 0; i < 8; i++) {

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Loading depth image and color image using OpenCV
        std::string depthFilename = projectSrcDir + "/data/depth/depth" + std::to_string(i) + ".png";
        std::string colorFilename = projectSrcDir + "/data/color/color" + std::to_string(i) + ".png";
        cv::Mat depthImage = cv::imread(depthFilename, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat colorImage = cv::imread(colorFilename, CV_LOAD_IMAGE_UNCHANGED);

        // Create point clouds in pcl::PointCloud type from depth image and color image using camera intrinsic parameters
        // Part of this process is similar to Exercise 5-1, so you can reuse the part of the code.
        // The provided depth image is millimeter scale but the default scale in PCL is meter scale
        // So the point clouds should be scaled to meter scale, during point cloud computation

        for (int i = 0; i < colorImage.size().width; i++) {
            for (int j = 0; j < colorImage.size().height; j++) {

                cv::Vec3b colors = colorImage.at<cv::Vec3b>(j, i);
                uint8_t r = colors[2];
                uint8_t g = colors[1];
                uint8_t b = colors[0];
                int32_t rgb = (r << 16) | (g << 8) | b;
                pcl::PointXYZRGB temp_point = *(new pcl::PointXYZRGB(r, g, b));
                float z = (float) ((depthImage.at<unsigned short>(j, i))) / 1000.0;
                temp_point.z = z;
                temp_point.x = ((i - px) * z) / focal;
                temp_point.y = ((j - py) * z) / focal;
                resultPointCloud->points.push_back(temp_point);
            }
        }
        //        
        // Downsample point clouds so that the point density is 1cm by uisng pcl::VoxelGrid<pcl::PointXYZRGB > function
        // point density can be set by using pcl::VoxelGrid::setLeafSize() function

        pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
        voxelGrid.setInputCloud(resultPointCloud);
        voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f); //density is 1cm
        voxelGrid.filter(*filteredPointCloud);
        // Visualize the point clouds
        visualize(filteredPointCloud, "voxelized_"+std::to_string(i));

        // Save the point clouds as [.pcd] file
        std::string pointFilename = projectSrcDir + "/data/pointclouds/pointclouds" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFile(pointFilename, *filteredPointCloud);

    }
    
    
//    //icp
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
//    std::string pointFilenameTrg = projectSrcDir + "/data/pointclouds/pointclouds" + std::to_string(0) + ".pcd";
//    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pointFilenameTrg, *target) == -1) {
//        std::cout << "Couldn't read file \n";
//        return (-1);
//    }
//
//    for (int i = 1; i < 8; i++) {
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
//        std::string pointFilenameSrc = projectSrcDir + "/data/pointclouds/pointclouds" + std::to_string(i) + ".pcd";
//        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pointFilenameSrc, *source) == -1) {
//            std::cout << "Couldn't read file \n";
//            return (-1);
//        }
//        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> ());
//        icp->setMaximumIterations(50);
//        icp->setInputSource(target);
//        icp->setUseReciprocalCorrespondences(true);
//        icp->setInputTarget(source);
//        icp->align(*target); 
//        //Eigen::Matrix4f transform, finTransform;
//        //transform = icp->getFinalTransformation();      //Will return transformation, but our input is normals
//        //finTransform = transform.inverse();
//        *target = *target + *source;
//
//    }
//    visualize(target, "icp without normals");
//    std::string pointFilename = projectSrcDir + "/data/pointclouds/icp_without_normals.pcd";
//    pcl::io::savePCDFile(pointFilename, *target);
    
    
    
    
    
    
    
    
    //icp with normals
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::string pointFilenameTrg = projectSrcDir + "/data/pointclouds/pointclouds" + std::to_string(0) + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pointFilenameTrg, *target) == -1) {
        std::cout << "Couldn't read file \n";
        return (-1);
        }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
    normal(target, result);
    
    for (int i = 1; i < 8; i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::string pointFilenameSrc = projectSrcDir + "/data/pointclouds/pointclouds" + std::to_string(i) + ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pointFilenameSrc, *source) == -1) {
            std::cout << "Couldn't read file \n";
            return (-1);
        }
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
        normal(source, cloud_source_normals);
        pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp(new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ());
        icp->setMaximumIterations(50);
        icp->setInputSource(result);
        icp->setUseReciprocalCorrespondences(true);
        icp->setInputTarget(cloud_source_normals);
        icp->align(*result); 
        Eigen::Matrix4f transform, finTransform;
        transform = icp->getFinalTransformation();      //Will return transformation, but our input is normals
        //finTransform = transform.inverse();
        *result = *result + *cloud_source_normals;

    }
    visualizeWithNormals(result, "icp with normals");
    std::string pointFilename = projectSrcDir + "/data/pointclouds/icp_with_normals.pcd";
    pcl::io::savePCDFile(pointFilename, *result);
    return 0;
}

