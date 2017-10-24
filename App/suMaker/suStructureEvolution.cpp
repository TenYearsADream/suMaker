#include "suStructureEvolution.h"
#include "suVolume.h"
#include "suMorton.h"
#include <deque>
//#include <Eigen/Core>
int suStructrueOptimizer::evolve()
{	
	std::deque<SU::OctNode*> floodEdges;

	//Init: find seeds
	/*for (int i = 0; i < nodeArr_.size(); i++)
	{
		if (nodeArr_[i]->location == nMaxValue && 
			nodeArr_[i]->label_ == SU::INTERIOR_CELL)
		{
			floodEdges.push_back(nodeArr_[i]);
		}
	}*/

	//start evolution
	//while (!floodEdges.empty())
	//{
	//	std::deque<SU::OctNode *>::iterator it = floodEdges.begin();


	//}
	while(1/*isConvergence()*/)
	{
		//agents.act();
		//agents.evalue();
	}


	return 0;
}

int suStructrueOptimizer::init_position_field()
{
	std::deque<SU::OctNode*> floodEdges;
	int maxValue = 0;

	//init boundary
	for (int i = 0; i < nodeArr_.size(); i++)
	{
		if (nodeArr_[i]->label_ == SU::BOUNDARY_CELL || nodeArr_[i]->label_ == SU::BOUNDARY_CELL_SPECIAL)
		{
			nodeArr_[i]->location = 1;
			floodEdges.push_back(nodeArr_[i]);
		}
			
	}
	//floodfill from boundary to internal
	while (!floodEdges.empty())
	{
		std::deque<SU::OctNode *>::iterator it = floodEdges.begin();

		std::vector<int> neighbors;
		suMorton::get_6neighbors(neighbors, (*it)->morton, nLevel_);
		for (int i = 0; i < 6; i++)
		{
			int mcode = neighbors[i];
			if (mcode >= nodeArr_.size() || mcode < 0) continue;    
			SU::OctNode *pNode = nodeArr_[mcode];
			if (pNode->location == 0 &&
				pNode->label_ == SU::INTERIOR_CELL)
			{
				maxValue = (*it)->location + 1;
				pNode->location = maxValue;
				floodEdges.push_back(pNode);
			}
		}

		floodEdges.pop_front();
	}

	return maxValue;
	
}

int suStructrueOptimizer::init_morton_coding(std::vector<SU::OctNode*> &voxels)
{
	nodeArr_.resize(1 << (3 * nLevel_));

	for (int i = 0; i < voxels.size(); i++)
	{
		voxels[i]->out = true;
		SU::Point _cp = voxels[i]->center();
		SU::Point _vsize = voxels[i]->size();
		voxels[i]->morton = suMorton::encode(_cp.x / _vsize.x, _cp.y / _vsize.y, _cp.z / _vsize.z, nLevel_);
		nodeArr_[voxels[i]->morton] = voxels[i];
	}	
	return nodeArr_.size();
}
