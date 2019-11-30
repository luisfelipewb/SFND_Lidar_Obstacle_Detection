/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
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

struct KdTree
{
	Node* root;
	Node* insertLocation = NULL;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		int tree_depth = 1;
		Node* insertLocation = root;

		if (root == NULL)
			root = new Node(point, id);
		else
		{
			while (insertLocation != NULL)
			{
				tree_depth++;

				if (point[tree_depth%2] < insertLocation->point[tree_depth%2])
				{
					if (insertLocation->left == NULL)
					{
						insertLocation->left = new Node(point, id);
						break;
					}
					else
						insertLocation = insertLocation->left;
				}
				else
				{
					if (insertLocation->right == NULL)
					{
						insertLocation->right = new Node(point, id);
						break;
					}
					else
						insertLocation = insertLocation->right;
				}
			}
		}
	}

	void recursiveSearch(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			std::cout << "Looking for node: " << node->id << std::endl;
			// Check distance of current Node
			float xdist = target[0] - node->point[0];
			float ydist =  target[1] - node->point[1];
			float distance = sqrt(xdist*xdist + ydist*ydist);
			std::cout << "Calculated distance: " << distance << std::endl;
			if (distance < distanceTol)
			{
				std::cout << "Added to list" << std::endl << std::endl;
				ids.push_back(node->id);
			}
			else
				std::cout << "Discarded" << std::endl << std::endl;

			// Go deeper into the tree only if withing boundaries
			if((target[depth%2]-distanceTol) < node->point[depth%2]){
				std::cout << "Check left branch" << std::endl;
				recursiveSearch(target, node->left, depth+1, distanceTol, ids);
			}
			if((target[depth%2]+distanceTol) > node->point[depth%2]){
				std::cout << "Check right branch" << std::endl;
				recursiveSearch(target, node->right, depth+1, distanceTol, ids);
			}
		}
		else
			std::cout << "This node was NULL" << std::endl;

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::cout << "Starting serach with parametes: ";
		std::cout << target[0] << " ";
		std::cout << target[1] << " ";
		std::cout << distanceTol << std::endl << std::endl;

		recursiveSearch(target, root, 1, distanceTol, ids);
		printKDTree(root, 1);

		return ids;
	}

	void printKDTree(Node* node, int depth)
	{
		if (node != NULL) {
			std::cout << depth << "  NODE id: " << node->id << std::endl;
			printKDTree(node->left, depth*10+1);
			printKDTree(node->right, depth*10+1);
		}
	}


};
