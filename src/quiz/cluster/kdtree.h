#include "/home/banban/workspace/jiangxinyu_ws/SFND_Lidar_Obstacle_Detection/src/render/render.h"
#include <cmath>

struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId):	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree() : root(NULL) {}

	void insert(std::vector<float> point, int id)
	{
		InsertNode( &root, point, id , 0);
	}

	void InsertNode(Node **pnode,  std::vector<float> point, int id, uint depth)
	{
		if((*pnode)==NULL)
		{
			*pnode = new Node(point,id);
		}
		else
		{
			uint flag = depth%2;
			if (point[flag] > (*pnode)->point[flag])
			{
				InsertNode( &(*pnode)->right , point , id , depth + 1 );
			}
			else
			{
				InsertNode(  &(*pnode)->left , point , id , depth + 1 );
			}
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		SearchHelper(&root,ids,target,distanceTol,0);
		return ids;
	}

	void SearchHelper(Node **pnode ,std::vector<int> & idv ,std::vector<float> target , float distanceTol, uint depth)
	{
		if(*pnode == NULL)
		{
			std::cerr <<  " The search is over " << std::endl;
			return ;
		}
		uint flag = depth %2;
		if (isInBox(target, (*pnode)->point, distanceTol) == true)
		{
			idv.push_back((*pnode)->id);
			SearchHelper(&(*pnode)->right, idv, target, distanceTol, depth + 1);
			SearchHelper(&(*pnode)->left, idv, target, distanceTol, depth + 1);
		}
		if(isInBox(target,(*pnode)->point,distanceTol)==false)
		{
			if (target[flag] >=(*pnode)->point[flag] )
			{
				SearchHelper( &(*pnode)->right,  idv , target , distanceTol , depth+1);
			}
			if ( target[flag] <= (*pnode)->point[flag] )
			{
				SearchHelper(&(*pnode)->left, idv, target, distanceTol, depth + 1);
			}
		}

	}

	bool isInBox(std::vector<float> target ,std::vector<float> point, float distanceTol)
	{
		if(abs(point[0]-target[0]) <= distanceTol && abs(point[1]-target[1]<= distanceTol))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	

};




