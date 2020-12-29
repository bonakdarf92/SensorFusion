/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "kdtree.h"
//#include "../ransac/ransac.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area

pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


void render3DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

    if(node!=NULL)
    {
        Box upperWindow = window;
        Box lowerWindow = window;
        // split on x axis
        if(depth%3==0)
        {
            //viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, window.z_min),pcl::PointXYZ(node->point[0], window.y_max, window.z_max),0,0,1,"line"+std::to_string(iteration));
            lowerWindow.x_max = node->point.x;
            upperWindow.x_min = node->point.x;
        }
            // split on y axis
        if (depth%3 == 1)
        {
            //viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], window.z_min),pcl::PointXYZ(window.x_max, node->point[1], window.z_max),1,0,0,"line"+std::to_string(iteration));
            lowerWindow.y_max = node->point.y;
            upperWindow.y_min = node->point.y;
        }
        else
        {
            //viewer->addLine(pcl::PointXYZ(window.x_min, window.y_min, node->point[2]),pcl::PointXYZ(window.x_max, window.y_max, node->point[2]),0,1,0,"line"+std::to_string(iteration));
            lowerWindow.z_max = node->point.z;
            upperWindow.z_min = node->point.z;
        }
        iteration++;
        render3DTree(node->left,viewer, lowerWindow, iteration, depth+1);
        render3DTree(node->right,viewer, upperWindow, iteration, depth+1);
    }
}

void clusterHelper(int index, pcl::PointCloud<pcl::PointXYZI>::Ptr points, std::vector<int> & clusters, std::vector<bool> & processedPoints, KdTree * tree, float distanceTool) {
    processedPoints[index] = true;
    clusters.push_back(index);
    std::vector<int> nearest = tree->search(points->at(index), distanceTool);
    for(int ids: nearest) {
        if(!processedPoints[ids])
            clusterHelper(ids, points, clusters, processedPoints, tree, distanceTool);
    }
}

std::vector<std::vector<int>> euclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr points, KdTree* tree, float distanceTol)
{

    // Fill out this function to return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processedPoints(points->size(), false);
    int k = 0;
    while(k < points->size()) {
        if (processedPoints[k]) {
            k++;
            continue;
        }
        std::vector<int> cluster;
        clusterHelper(k, points, cluster,processedPoints, tree, distanceTol);
        clusters.push_back(cluster);
        k++;
    }
    return clusters;
}

Box boundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

/*
int main ()
{

    setenv("DISPLAY", "127.0.0.1:0", true);
	// Create viewer
	Box window;
  	window.x_min = -10;
  	window.x_max =  10;
  	window.y_min = -10;
  	window.y_max =  10;
  	window.z_min = -10;
  	window.z_max =  10;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();  // window, 10

	// Create data
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };
    //std::vector<std::vector<float>> points = { {-6.2,7, 0.2}, {-6.3,8.4, 0.3}, {-5.2,7.1, 4.1}, {-5.7, 6.3, 3.9}, {7.2, 6.1, -3.0}, {8.0, 5.3, -2.5}, {7.2, 7.1, -3.1}, {0.2,-7.1, 7.2}, {1.7, -6.9, 6.9}, {-1.2,-7.2, 5.5}, {2.2,-8.9, 6.0} };
	//std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3} };
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 = CreateData3D();

    std::unordered_set<int> inliersPlane = RansacPlane3d(cloud2, 1000, 0.2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

    for(int index = 0; index < cloud2->points.size(); index++)
    {
        pcl::PointXYZI point = cloud2->points[index];
        if(inliersPlane.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


	KdTree* tree = new KdTree;
    //for (int i=0; i<points.size(); i++)
    //	tree->insert(points[i],i);
    for (int i=0; i<cloudOutliers->size(); i++)

        tree->insert(cloudOutliers->points[i],i);

    int it = 0;
  	//render3DTree(tree->root,viewer,window, it);

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<std::vector<int>> clusters = euclideanCluster(cloudOutliers, tree, 2);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(0.5,0.5,1), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  		    clusterCloud->points.push_back(pcl::PointXYZ(cloudOutliers->points.at(indice).x, cloudOutliers->points.at(indice).y, cloudOutliers->points.at(indice).z));
  			//clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
        //Box box =  boundingBox(clusterCloud);
        //renderBox(viewer,box, clusterId);
  		++clusterId;
  	}
  	if(clusters.size()==0)
  		renderPointCloud(viewer,cloud2,"data");

  	renderPointCloud(viewer, cloudInliers, "Plane");

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
*/