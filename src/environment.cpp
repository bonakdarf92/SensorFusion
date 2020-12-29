/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "quiz/ransac/ransac.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "quiz/cluster/kdtree.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
    Car lambo(Vect3(16, -4, 0), Vect3(3.5,1.8,1),Color(0.5,0.3, 0.1),"Lamborghini");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);
    cars.push_back(lambo);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
        lambo.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10,-5,-2,1), Eigen::Vector4f(30,8,1,1));
    inputCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10,-10,-2,1), Eigen::Vector4f(40,10,2, 1));
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(inputCloud, 50, 0.25);
    std::unordered_set<int> inliersPlane;
    inliersPlane =  RansacPlane3d(inputCloud, 50, 0.25);
    //renderPointCloud(viewer, segmentCloud.first," obstCloud",Color(1,0,0));
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < inputCloud->points.size(); index++)
    {
        pcl::PointXYZI point = inputCloud->points[index];
        if(inliersPlane.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    renderPointCloud(viewer, cloudInliers, "plane", Color(0,0,1));
    renderPointCloud(viewer, cloudOutliers, "obst", Color(1,0,0));
    /*
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(cloudOutliers,0.53,30,300);
    int cloudId = 0;
    std::vector<Color> colors = {Color(0.5,0.5,0), Color(0.4,0.9,0.1), Color(0.33,0.7,1),Color(0.65,0.76,0.21)};
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        std::cout << "Cluster groeße" << std::endl;
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstableCloud"+std::to_string(cloudId),colors[cloudId%colors.size()]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,cloudId);
        ++cloudId;
    }
    */

    KdTree* tree = new KdTree;
    //for (int i=0; i<points.size(); i++)
    //	tree->insert(points[i],i);
    for (int i=0; i<cloudOutliers->size(); i++)
        tree->insert(cloudOutliers->points[i],i);
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //
    std::vector<std::vector<int>> clusters = euclideanCluster(cloudOutliers, tree, 0.5);
    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    // Render clusters
    int clusterId = 0;
    std::vector<Color> colors = {Color(0.5,0.5,1), Color(0,1,0), Color(0,0,1)};
    for(std::vector<int> cluster : clusters)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        for(int indice: cluster)
            clusterCloud->points.push_back(pcl::PointXYZI(cloudOutliers->points.at(indice).x, cloudOutliers->points.at(indice).y, cloudOutliers->points.at(indice).z, cloudOutliers->points.at(indice).intensity));
        //clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
        renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
        Box box =  boundingBox(clusterCloud);
        renderBox(viewer,box, clusterId);
        ++clusterId;
    }


    //renderPointCloud(viewer,cloudClusters,"Cloud clusterd");
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor
    Lidar * lidar = new Lidar(cars, 0.01);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer,inputCloud,"Point Cloud");


    // Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = pointProcessor.SeparateClouds(inliers, inputCloud);
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100,0.2);
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud", Color(0,1,1));

    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first,2,20,150);

    int cloudId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1),Color(0,1,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "Cluster groeße" << std::endl;
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstableCloud"+std::to_string(cloudId),colors[cloudId%colors.size()]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,cloudId);
        ++cloudId;
    }


}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 2);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    //setenv("DISPLAY", "127.0.0.1:0", true);
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        //usleep(50000);
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}