#include <iostream>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <Eigen/Dense>
/**
 * @brief Saving point clouds data as ply file
 * @param [in] filename : filename for saving point clouds (the extention should be .ply)
 * @param [in] vertices : list of vertex point
 * @param [in] colors   : list of vertex color
 * @return Success:true, Failure:false
 */
bool savePointCloudsPLY(std::string filename, std::vector<Eigen::Vector4f>& vertices, std::vector<cv::Vec3b>& colors) {
    std::ofstream fout;
    fout.open(filename.c_str());
    if (fout.fail()) {
        std::cerr << "file open error:" << filename << std::endl;
        return false;
    }
    int pointNum = vertices.size();
    fout << "ply" << std::endl;
    fout << "format ascii 1.0" << std::endl;
    fout << "element vertex " << pointNum << std::endl;
    fout << "property float x" << std::endl;
    fout << "property float y" << std::endl;
    fout << "property float z" << std::endl;
    fout << "property uchar red" << std::endl;
    fout << "property uchar green" << std::endl;
    fout << "property uchar blue" << std::endl;
    fout << "property uchar alpha" << std::endl;
    fout << "end_header" << std::endl;
    for (int i = 0; i < pointNum; i++) {
        Eigen::Vector4f& vertex = vertices[i];
        cv::Vec3b& col = colors[i];
        fout << vertex[0] << " " << vertex[1] << " " << vertex[2] << " " << static_cast<int> (col[2]) << " " << static_cast<int> (col[1]) << " " << static_cast<int> (col[0]) << " " << 255 << std::endl;
    }
    fout.close();
    return true;
}
/**
 * @brief Loading camera pose file (6Dof rigid transformation)
 * @param [in]  filename : filename for loading text file
 * @param [out] pose   : 4x4 transformation matrix
 * @return Success:true, Failure:false
 */
bool loadCameraPose(std::string filename, Eigen::Matrix4f& poseMat) {
    std::ifstream fin;
    fin.open(filename.c_str());
    if (fin.fail()) {
        std::cerr << "file open error:" << filename << std::endl;
        return false;
    }
    // Loading 4x4 transformation matrix
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            fin >> poseMat(y, x);
        }
    }
    return true;
}
int main(int argc, char* argv[]) {
    std::string projectSrcDir = PROJECT_SOURCE_DIR;
    //std::cout<<"Source dir: "<<projectSrcDir;
    
    int index = 0;
    
    
    //for(int index=0;index<=5;index++)
    //{
        std::vector<Eigen::Vector4f> vertices; // 3D points
        std::vector<cv::Vec3b> colors; // color of the points
        // Loading depth image and color image
        std::string depthFilename = projectSrcDir + "/Data/depth/depth" + std::to_string(index) + ".png";
        std::string colorFilename = projectSrcDir + "/Data/color/color" + std::to_string(index) + ".png";
        cv::Mat depthImg = cv::imread(depthFilename, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat colorImg = cv::imread(colorFilename, CV_LOAD_IMAGE_UNCHANGED);
        //std::cout<<"colorImg  "<<colorImg.type()<<"    "<<type2str(colorImg.type())<<"    "<<type2str(depthImg.type())<<std::endl;
        //cv::imshow("colorImage", colorImg);
        //cv::waitKey(0);
        // Loading camera pose
        std::string poseFilename = projectSrcDir + "/Data/pose/pose" + std::to_string(index) + ".txt";
        Eigen::Matrix4f poseMat; // 4x4 transformation matrix
        loadCameraPose(poseFilename, poseMat);
        //std::cout << "Transformation matrix" << std::endl << poseMat << std::endl;
        // Setting camera intrinsic parameters of depth camera
        float focal = 570.f; // focal length
        float px = 319.5f; // principal point x
        float py = 239.5f; // principal point y
        // Data for point clouds consisted of 3D points and their colors
        
        for (int i = 0; i < colorImg.size().width; i++) {
            for (int j = 0; j < colorImg.size().height; j++) {
                Eigen::Vector4f vertex;
                // Create point clouds from depth image and color image using camera intrinsic parameters
                // (1) Compute 3D point from depth values and pixel locations on depth image using camera intrinsic parameters.
                float z = depthImg.at<unsigned short>(j, i);
                vertex(2) = z;
                vertex(0) = ((i - px) * z) / focal;
                vertex(1) = ((j - py) * z) / focal;
                vertex(3) = 1;
                //std::cout<<z<<"   ";
                // (2) Translate 3D point in local coordinate system to 3D point in global coordinate system using camera pose.
                vertices.push_back(poseMat * vertex);   
                
                // (3) Add the 3D point to vertices in point clouds data.
                // (4) Also compute the color of 3D point and add it to colors in point clouds data.
                colors.push_back(colorImg.at<cv::Vec3b>(j, i));
            }
        }
        
         // Save point clouds
    savePointCloudsPLY("pointClouds" + std::to_string(index) + ".ply", vertices, colors);
    //}
   
    return 0;
}