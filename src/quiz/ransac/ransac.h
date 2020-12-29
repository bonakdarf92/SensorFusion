//
// Created by faridb on 23.12.20.
//

#ifndef PLAYBACK_RANSAC_H
#define PLAYBACK_RANSAC_H

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include "../../render/box.h"
#include <unordered_set>


std::unordered_set<int>RansacPlane3d(pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud, int maxIterations, float distanceTol);


pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData3D();

#endif //PLAYBACK_RANSAC_H
