#include "../../render/render.h"
#include <cmath>

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}
};

template <typename PointT>
class KdTree
{
    public:
	Node<PointT>* root;

	KdTree() : root(NULL) {}

	void insertUtil(Node<PointT>*& node, uint depth, PointT point, int id)
	{
		if(node == NULL)
		{
			node = new Node<PointT>(point, id);		
		}
		else
		{
			uint curDim = depth%3;
            
            switch(curDim)
            {
                case 0:{
                    if(point.x > node->point.x)
                        insertUtil(node->right, depth+1, point, id);
                    else
                        insertUtil(node->left , depth+1, point, id); 
                } break;

                case 1:{
                    if(point.y > node->point.y)
                        insertUtil(node->right, depth+1, point, id);
                    else
                        insertUtil(node->left , depth+1, point, id);
                } break;

                case 2:{
                    if(point.z > node->point.z)
                        insertUtil(node->right, depth+1, point, id);
                    else
                        insertUtil(node->left , depth+1, point, id);
                } break;
            }
        }    
	}

	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertUtil(root, 0, point, id);
	}

	// void insert(BinaryTreeNode *&node, int data)
	// {
	// 	if(node == NULL)
	// 	{
	// 		node = getNewNode(data);
	// 	}
	// 	else if(data < node->data)
	// 	{
	// 		insert(node->left, data);
	// 	}
	// 	else
	// 	{
	// 		insert(node->right, data);
	// 	}
	// }

	// void searchUtil(std::vector<float> target, Node*& node, uint depth, float distanceTol, std::vector<int>& ids)
	// {
	// 	if(node != NULL)
	// 	{
	// 		if( (node->point[0] >= (target[0]-distanceTol) && node->point[0] <= (target[0]+distanceTol)) 
	// 			&& (node->point[1] >= (target[1]-distanceTol) && node->point[1] <= (target[1]+distanceTol))
	// 			&& (node->point[2] >= (target[2]-distanceTol) && node->point[2] <= (target[2]+distanceTol)) )
 	// 		{
	// 			float distance = sqrt(pow(node->point[0] - target[0], 2) + pow(node->point[1] - target[1], 2) + pow(node->point[2]- target[2], 2));
	// 			if(distance <= distanceTol)
	// 			{
	// 				ids.push_back(node->id);
	// 			}
	// 		}

	// 		// check across boundary
	// 		uint curDim = depth%3;
	// 		if((target[curDim]-distanceTol) < node->point[curDim])
	// 			searchUtil(target, node->left, depth+1, distanceTol, ids);
	// 		if((target[curDim]+distanceTol) > node->point[curDim])
	// 			searchUtil(target, node->right, depth+1, distanceTol, ids);
	// 	}
	// }

	// // return a list of point ids in the tree that are within distance of target
	// std::vector<int> search(std::vector<float> target, float distanceTol)
	// {
	// 	std::vector<int> ids;
	// 	searchUtil(target, root, 0, distanceTol, ids);
		
	// 	return ids;
	// }
};