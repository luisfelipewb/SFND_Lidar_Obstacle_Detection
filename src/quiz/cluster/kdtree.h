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

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}

};
