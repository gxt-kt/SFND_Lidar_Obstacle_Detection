//
// Created by hyin on 2020/3/25.
//

// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "yaml_config.h"
using namespace lidar_obstacle_detection;

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// Test load pcd
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
//    ProcessPointClouds<pcl::PointXYZI>pointProcessor;
//    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//    renderPointCloud(viewer,inputCloud,"cloud");
//}

// Initialize the simple Highway

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <pcl/common/pca.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// reference https://pointclouds.org/documentation/tutorials/moment_of_inertia.html
/**
 * 从聚类结果中提取障碍物信息
 * @param obstacle_clouds 输出的障碍物点云集合
 * @param obstacle_centroids 输出的障碍物质心集合
 * @param obstacle_sizes 输出的障碍物尺寸集合
 * @param obstacle_orientations 输出的障碍物旋转集合
 */
void ExtractObstacles(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& obstacle_clouds,
    std::vector<Eigen::Vector3f>& obstacle_centroids,
    std::vector<Eigen::Vector3f>& obstacle_sizes,
    std::vector<Eigen::Quaternionf>& obstacle_orientations)
{
    obstacle_centroids.clear();
    obstacle_sizes.clear();
    obstacle_orientations.clear();
    for (const auto &obstacle_cloud : obstacle_clouds) {
      pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
      feature_extractor.setInputCloud(obstacle_cloud);
      feature_extractor.compute();

      pcl::PointXYZI min_point_OBB;
      pcl::PointXYZI max_point_OBB;
      pcl::PointXYZI position_OBB;
      Eigen::Matrix3f rotational_matrix_OBB;

      feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,
                               rotational_matrix_OBB);

      pcl::visualization::PCLVisualizer::Ptr viewer(
          new pcl::visualization::PCLVisualizer("3D Viewer"));
      viewer->setBackgroundColor(0, 0, 0);
      viewer->addCoordinateSystem(1.0);
      viewer->initCameraParameters();
      viewer->addPointCloud<pcl::PointXYZI>(obstacle_cloud, "sample cloud");

      Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
      Eigen::Quaternionf quat(rotational_matrix_OBB);
      // 添加可视化
      // viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x,
      //                 max_point_OBB.y - min_point_OBB.y,
      //                 max_point_OBB.z - min_point_OBB.z, "OBB");
      // viewer->setShapeRenderingProperties(
      //     pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      //     pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
      obstacle_centroids.push_back(position);
      obstacle_orientations.push_back(quat);
      Eigen::Vector3f size;
      size << (max_point_OBB.x - min_point_OBB.x) ,(max_point_OBB.y - min_point_OBB.y),(max_point_OBB.z - min_point_OBB.z);
      obstacle_sizes.push_back(size);
    }

    // debug
    if(0) {
        std::cout << obstacle_centroids.size();
        for(int i=0;i < obstacle_centroids.size();i++) {
            std::cout << "====" << std::endl;
            std::cout << obstacle_centroids[i] << std::endl;
            std::cout << obstacle_sizes[i] << std::endl;
            std::cout << obstacle_orientations[i] << std::endl;
        }
    }
}

/**
 * 可视化障碍物
 * @param viewer PCL可视化器
 * @param obstacle_clouds 障碍物点云集合
 * @param obstacle_centroids 障碍物质心集合
 * @param obstacle_sizes 障碍物尺寸集合
 * @param obstacle_orientations 障碍物旋转集合
 */
void visualizeObstacles(
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& cloudClusters,
    const std::vector<Eigen::Vector3f>& obstacle_centroids,
    const std::vector<Eigen::Vector3f>& obstacle_sizes,
    const std::vector<Eigen::Quaternionf>& obstacle_orientations)
{
    // 创建可视化窗口
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // 绘制包围盒
    for (size_t i = 0; i < obstacle_centroids.size(); i++)
    {
        
        std::stringstream cloud_name;
        cloud_name << "cloud_" << i;
        viewer->addPointCloud<pcl::PointXYZI>(cloudClusters[i], cloud_name.str());
        
        std::stringstream box_name;
        box_name << "box_" << i;
        
      viewer->addCube(obstacle_centroids[i], obstacle_orientations[i], obstacle_sizes[i][0],
                      obstacle_sizes[i][1],
                      obstacle_sizes[i][2], box_name.str());
         // 设置颜色
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
                                    0.5,0,0, box_name.str());  
      viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, box_name.str());
    }

    // 启动可视化
    while (!viewer->wasStopped()) {
      viewer->spinOnce();
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Test read Lidar data
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Setting hyper parameters

    // FilterCloud
    // float filterRes = 0.4;
    float filterRes=yaml_config["filterRes"].as<float>();
    // Eigen::Vector4f minpoint(-10, -6.5, -2, 1);
    // Eigen::Vector4f maxpoint(30, 6.5, 1, 1);
    double max_x=yaml_config["roi"]["max_x"].as<double>();
    double max_y=yaml_config["roi"]["max_y"].as<double>();
    double max_z=yaml_config["roi"]["max_z"].as<double>();
    double min_x=yaml_config["roi"]["min_x"].as<double>();
    double min_y=yaml_config["roi"]["min_y"].as<double>();
    double min_z=yaml_config["roi"]["min_z"].as<double>();
    Eigen::Vector4f maxpoint(max_x,max_y,max_z,1);
    Eigen::Vector4f minpoint(min_x,min_y,min_z,1);
    // SegmentPlane
    // int maxIterations = 40;
    int maxIterations=yaml_config["ransac"]["maxIterations"].as<int>();
    // float distanceThreshold = 0.3;
    float distanceThreshold=yaml_config["ransac"]["distanceThreshold"].as<float>();
    // 聚类参数设置
    // Clustering
    // float clusterTolerance = 0.5;
    // int minsize = 10;
    // int maxsize = 140;
    float clusterTolerance = yaml_config["cluster"]["clusterTolerance"].as<float>();
    int minsize = yaml_config["cluster"]["minsize"].as<int>();
    int maxsize = yaml_config["cluster"]["maxsize"].as<int>();

    // 1. 降采样+设置roi区域+去除车辆相关点云
    // First:Filter cloud to reduce amount of points
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minpoint,
                                                                                      maxpoint);
    // 2. 把点云分离出路面
    // Second: Segment the filtered cloud into obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacSegmentPlane(
            filteredCloud, maxIterations, distanceThreshold);
//    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
//    renderPointCloud(viewer,inputCloud,"inputCloud");
    // Third: Cluster different obstacle cloud
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first,
    //                                                                                               clusterTolerance,
    //                                                                                               minsize, maxsize);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->GxtEuclideanClustering(segmentCloud.first,
                                                                                                  clusterTolerance,
                                                                                                  minsize, maxsize);

    std::vector<Eigen::Vector3f> centroids;
    std::vector<Eigen::Vector3f> sizes;
    std::vector<Eigen::Quaternionf> orientations;
    
    // std::cout << "begin ExtractObstacles" << std::endl;
    // ExtractObstacles(cloudClusters,centroids,sizes,orientations);
    // visualizeObstacles(cloudClusters,centroids,sizes,orientations);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {

        std::cout << "cluster size";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);
        // Fourth: Find bounding boxes for each obstacle cluster
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;

    }

}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obst = false;
    bool render_plane = false;
    bool render_cluster = true;
    bool render_box = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
//    renderRays(viewer,lidar->position,inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(
            inputCloud, 100, 0.2);
    if (render_obst) {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    }
    if (render_plane) {
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }


    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0,
                                                                                               3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        if (render_cluster) {
            std::cout << "cluster size:  ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),
                             colors[clusterId % colors.size()]);
            ++clusterId;
        }
        if (render_box) {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }
        ++clusterId;
    }
    renderPointCloud(viewer, segmentCloud.second, "planeCloud");

}



//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

// 计算旋转矩阵  
Eigen::Matrix4d TransforMatrix(Eigen::Vector3d translation,double roll,double pitch,double yaw) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())  
                         * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())  
                         * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());  
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity(); // 初始化为单位矩阵  
    transformation_matrix.block<3, 3>(0, 0) = q.toRotationMatrix(); // 设置旋转矩阵部分  
    transformation_matrix.block<3, 1>(0, 3) = translation; // 设置平移向量部分  
    std::cout << "旋转矩阵：" << std::endl << transformation_matrix << std::endl;  
    return transformation_matrix;
}

// char* argv[] means array of char pointers, whereas char** argv means pointer to a char pointer.
int main(int argc, char **argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

// For simpleHighway function
//    simpleHighway(viewer);
//    cityBlock(viewer);
//    while (!viewer->wasStopped ())
//    {
//     viewer->spinOnce ();
//    }
//
    std::string pcd_files_path = "src/sensors/data/pcd/data_1";
    if (argc >= 2) {
      pcd_files_path = argv[1];
    }
    std::cout << "Read pcd file path: " << pcd_files_path << std::endl;

//  Stream cityBlock function
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd(pcd_files_path);
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    Eigen::Vector3d trans;
    trans<<0.3643,0.2078,1.21;
    Eigen::Matrix4d tran_matrix=TransforMatrix(trans,-179.5*M_PI/180.0,0.5*M_PI/180.0,360*M_PI/180.0);
    
    Eigen::Affine3d affine3d_transform(tran_matrix); // 转换为Affine3d类型  
    Eigen::Affine3f affine3f_transform = affine3d_transform.cast<float>(); // 转换为Affine3f类型
    affine3f_transform=affine3f_transform.inverse();
    // 


    while (!viewer->wasStopped()) {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        // 坐标系转换
        pcl::transformPointCloud(*inputCloudI, *inputCloudI, affine3f_transform);
        cityBlock(viewer, pointProcessorI, inputCloudI);
        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
        viewer->spinOnce();
    }
}
