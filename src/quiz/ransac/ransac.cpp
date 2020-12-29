//
// Created by faridb on 23.12.20.
//
#include "ransac.h"


std::unordered_set<int> RansacPlane3d( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    while (maxIterations--) {
        std::unordered_set<int> inliers;
        while (inliers.size() < 3) {
            inliers.insert(rand() % (cloud->points.size()));
        }
        float x1, x2, x3, y1, y2, y3, z1, z2, z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;


        float A = (y2 - y1)*(z3 - z1) - (z2 - z1) * (y3 - y1);
        float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float C = (x2 - x1)*(y3 - y1) - (y2 - y1)* (x3 - x1);
        float D = -(A*x1 + B * y1 + C* z1);

        for (int index = 0; index < cloud->points.size() ; index++) {
            if (inliers.count(index) > 0) {
                continue;
            }

            pcl::PointXYZI point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            float d = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(A * A + B * B + C * C);

            if (d <= distanceTol) {
                inliers.insert(index);
            }
        }
        if(inliers.size()>inliersResult.size()) {
            inliersResult = inliers;
        }
    }
    return inliersResult;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData3D()
{

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI> ("../../../../src/sensors/data/pcd/simpleHighway.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from ../../../../src/sensors/data/pcd/simpleHighway.pcd" << std::endl;

    return cloud;
}
