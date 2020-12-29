/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include "pcl/recognition/linemod/line_rgbd.h"
#include "../../render/box.h"


// Structure to represent node of kd tree
/*
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};
*/
struct Node
{
    //std::vector<float> point;
    pcl::PointXYZI point;
    int id;
    Node* left;
    Node* right;

    Node(pcl::PointXYZI arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}
};



struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}


	void insertHelper(Node ** node, uint depth, pcl::PointXYZI point, int id) {
	    // check if tree is empty
	    if(*node == NULL) {
	        *node = new Node(point, id);
	    }
	    else {
	        uint cd = depth % 3;
	        if (cd == 0) {
                if (point.x < ((*node)->point.x)) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            }
            if (cd == 1) {
                if (point.y < ((*node)->point.y)) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            }
            if (cd == 2) {
                if (point.z < ((*node)->point.z)) {
                    insertHelper(&((*node)->left), depth + 1, point, id);
                } else {
                    insertHelper(&((*node)->right), depth + 1, point, id);
                }
            }
	    }

	}

	void insert(pcl::PointXYZI point, int id)
	{
		// Fill in this function to insert a new point into the tree
		insertHelper(&root,0, point,id);
	}

	void searchHelper(pcl::PointXYZI target, Node * node, int depth, float distanceTol, std::vector<int> &ids) {
        if (node != NULL) {
            /*
            if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) ) {
                float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }
            */
            if ((node->point.x >= (target.x - distanceTol) && node->point.x <= (target.x + distanceTol)) &&
                (node->point.y >= (target.y - distanceTol) && node->point.y <= (target.y + distanceTol)) &&
                (node->point.z >= (target.z - distanceTol) && node->point.z <= (target.z + distanceTol)) ) {
                float distance = sqrt((node->point.x - target.x)*(node->point.x - target.x) +
                                         (node->point.y - target.y)*(node->point.y - target.y) +
                                         (node->point.z - target.z)*(node->point.z - target.z));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }

            if (depth%3 == 0) {
                if ((target.x - distanceTol) < node->point.x)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.x + distanceTol) > node->point.x)
                    searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }

            if (depth%3 == 1) {
                if ((target.y - distanceTol) < node->point.y)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.y + distanceTol) > node->point.y)
                searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
            if (depth%3 == 2) {
                if ((target.z - distanceTol) < node->point.z)
                    searchHelper(target, node->left, depth + 1, distanceTol, ids);
                if ((target.z + distanceTol) > node->point.z)
                searchHelper(target, node->right, depth + 1, distanceTol, ids);
            }
        }
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};


std::vector<std::vector<int>> euclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr points, KdTree* tree, float distanceTol);

Box boundingBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);


