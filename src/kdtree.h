/* \author Aaron Brown */

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

	void recursiveInsert(std::vector<float> point, int id, Node* node, int depth)
	{
		if (point[depth%2] < node->point[depth%2])
		{
			if (node->left == NULL)
				node->left = new Node(point, id);
			else
				recursiveInsert(point, id, node->left, depth+1);
		}
		else
		{
			if (node->right == NULL)
				node->right = new Node(point, id);
			else
				recursiveInsert(point, id, node->right, depth+1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// Insert a new point into the tree
		Node* insertLocation = root;

		if (root == NULL)
			root = new Node(point, id);
		else
			recursiveInsert(point, id, root, 0);
	}

	void recursiveSearch(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			// Check distance of current Node
			float xdist = target[0] - node->point[0];
			float ydist =  target[1] - node->point[1];
			float distance = sqrt(xdist*xdist + ydist*ydist);
			if (distance < distanceTol)
				ids.push_back(node->id);

			// Go deeper into the tree only if withing boundaries
			if((target[depth%2]-distanceTol) < node->point[depth%2])
				recursiveSearch(target, node->left, depth+1, distanceTol, ids);
			if((target[depth%2]+distanceTol) > node->point[depth%2])
				recursiveSearch(target, node->right, depth+1, distanceTol, ids);
		}

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		recursiveSearch(target, root, 0, distanceTol, ids);

		return ids;
	}

};
