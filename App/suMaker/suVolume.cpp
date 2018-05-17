#include "suVolume.h"
#include "triboxoverlap.h"
#include "suMathBase.h"
#include <iostream>
#include <sstream> 
namespace SU {


	/*!
	* \function  LoadMeshFromFile
	*
	* \brief  load stl/obj/ply model by OpenMesh
	* \param  @pNode  [IN]current node
	*         @return  [OUT] if success(true/false).
	* \author Yuan Yao
	* \date 2016/6/16
	*/
	bool suVolume::LoadMeshFromFile(const char *pFileName)
	{
		if (!pFileName)
			return false;

		if (isLoad_)
		{
			clear();
			isLoad_ = false;
		}

		if (OpenMesh::IO::read_mesh(mesh_, pFileName))
		{
			suMesh::ConstVertexIter  v_it(mesh_.vertices_begin()),
				v_end(mesh_.vertices_end());

			bbMin_ = bbMax_ = mesh_.point(*v_it);
			for (; v_it != v_end; ++v_it)
			{
				bbMin_.minimize(mesh_.point(*v_it));
				bbMax_.maximize(mesh_.point(*v_it));
			}
			isLoad_ = true;
		}
		return isLoad_;
	}
	bool suVolume::LoadMeshFromMesh(suMesh & m)
	{
		if (isLoad_)
		{
			clear();
			isLoad_ = false;
		}

		mesh_ = m;
		{
			suMesh::ConstVertexIter  v_it(mesh_.vertices_begin()),
				v_end(mesh_.vertices_end());

			bbMin_ = bbMax_ = mesh_.point(*v_it);
			for (; v_it != v_end; ++v_it)
			{
				bbMin_.minimize(mesh_.point(*v_it));
				bbMax_.maximize(mesh_.point(*v_it));
			}
			isLoad_ = true;
		}
		return isLoad_;
	}

	void suVolume::clear()
	{
		if (pRoot_ != NULL)delete pRoot_;
		pRoot_ = NULL;
		mesh_.clear();
	}

	int suVolume::PartitionSpace(unsigned int level)
	{
		level_ = level;
		leafBoundaryNodes_.clear();
		leafInternalNodes_.clear();

		pRoot_ = new OctNode();
		curLevel_ = 0;

		//All mesh in bounding box, but we need to convenient recursivePartition.
		suMesh::VertexIter   v_it(mesh_.vertices_begin());
		suMesh::VertexIter   v_end(mesh_.vertices_end());
		suMesh::FaceIter     f_it(mesh_.faces_begin());
		suMesh::FaceIter     f_end(mesh_.faces_end());
		if (pRoot_)
		{
			//1.fill node
			pRoot_->max_.x = bbMax_[0];
			pRoot_->max_.y = bbMax_[1];
			pRoot_->max_.z = bbMax_[2];
			pRoot_->min_.x = bbMin_[0];
			pRoot_->min_.y = bbMin_[1];
			pRoot_->min_.z = bbMin_[2];

			//regulate bbox to a cube
			SU::Point sz_bbox = pRoot_->max_ - pRoot_->min_;
			float max_ax = max(max(sz_bbox.x, sz_bbox.y), sz_bbox.z);
			pRoot_->max_.x = bbMin_[0] + max_ax;
			pRoot_->max_.y = bbMin_[1] + max_ax;
			pRoot_->max_.z = bbMin_[2] + max_ax;

			///add face handle to node
			for (; f_it != f_end; ++f_it)
			{
				pRoot_->suNode_.AddElement(f_it.handle());
			}
			for (; v_it != v_end; ++v_it)
			{
				suMesh::Point   point = mesh_.point(v_it);
				pRoot_->suNode_.AddElement(v_it);
			}
			pRoot_->label_ = BOUNDARY_CELL;
			pRoot_->level_ = 0;

			if (level >= 9)
			{
				std::cout << "Don't support out-core voxilization\n";
				return 0;
			}

			//2.start partition		
			recursivePartition(pRoot_, level);

			//3.label internal & outernal
			LabelBoundaryNeighbors();

			//4.subdivide all node


			//5.foodfill
			floodFill();

		}

		return 0;
	}
	//recursiely partition for each level
	void suVolume::recursivePartition(OctNode * pParent, int nLevel)
	{
		if (pParent->level_ >= nLevel || pParent->label_ != BOUNDARY_CELL) return;

		//partition each boundary node
		Point delta = pParent->max_ - pParent->min_;
		Point halfLen = delta * 0.5f;
		Point mid = halfLen + pParent->min_;

		///new sub oct node
		for (int i = 0; i < 8; i++)
		{
			pParent->children_[i] = new OctNode();
			///fill node
			pParent->children_[i]->parent_ = pParent;
			pParent->children_[i]->level_ = pParent->level_ + 1;

			///generate code
			int xLocCode = i & 0x1;
			int yLocCode = (i >> 1) & 0x1;
			int zLocCode = (i >> 2) & 0x1;

			///update node size
			float minx = (xLocCode > 0) ? (pParent->min_.x + halfLen.x) : pParent->min_.x;
			float miny = (yLocCode > 0) ? (pParent->min_.y + halfLen.y) : pParent->min_.y;
			float minz = (zLocCode > 0) ? (pParent->min_.z + halfLen.z) : pParent->min_.z;

			float maxx = (xLocCode > 0) ? pParent->max_.x : (pParent->min_.x + halfLen.x);
			float maxy = (yLocCode > 0) ? pParent->max_.y : (pParent->min_.y + halfLen.y);
			float maxz = (zLocCode > 0) ? pParent->max_.z : (pParent->min_.z + halfLen.z);


			Point newMin = Point(minx, miny, minz);
			Point newMax = Point(maxx, maxy, maxz);

			pParent->children_[i]->max_ = newMax;
			pParent->children_[i]->min_ = newMin;

			pParent->children_[i]->xLocCode_ = (xLocCode << pParent->level_ + 1) | pParent->xLocCode_;
			pParent->children_[i]->yLocCode_ = (yLocCode << pParent->level_ + 1) | pParent->yLocCode_;
			pParent->children_[i]->zLocCode_ = (zLocCode << pParent->level_ + 1) | pParent->zLocCode_;
		}

		//Label boundary & add faces to node	
		double triverts[3][3];
		double boxCenter[3];
		double boxHalfSize[3];

		delta = pParent->children_[0]->max_ - pParent->children_[0]->min_;
		halfLen = delta * 0.5f;
		boxHalfSize[0] = halfLen.x;
		boxHalfSize[1] = halfLen.y;
		boxHalfSize[2] = halfLen.z;

		for (int i = 0; i < 8; i++)
		{
			/*delta = pParent->children_[i]->max_ - pParent->children_[i]->min_;
			halfLen = delta * 0.5f;*/
			mid = pParent->children_[i]->min_ + halfLen;

			boxCenter[0] = mid.x;
			boxCenter[1] = mid.y;
			boxCenter[2] = mid.z;
			/*boxHalfSize[0] = halfLen.x;
			boxHalfSize[1] = halfLen.y;
			boxHalfSize[2] = halfLen.z;*/

			//Labeling boundary by faces
			size_t nFaces = pParent->suNode_.FaceVector.size();
			for (size_t k = 0; k < nFaces; k++)
			{
				suMesh::FaceHandle fHandle = pParent->suNode_.FaceVector[k];
				suMesh::FaceVertexIter fv_it = mesh_.fv_iter(fHandle);
				suMesh::Point    point;

				for (int j = 0; j < 3; j++)
				{
					point = mesh_.point(fv_it++);
					triverts[j][0] = point[0];
					triverts[j][1] = point[1];
					triverts[j][2] = point[2];
				}
				/*if (fHandle.idx() == 2094) {
					std::cout << fHandle.idx << std::endl;
				}*/
				if (triBoxOverlap(boxCenter, boxHalfSize, triverts))
				{
					pParent->children_[i]->suNode_.AddElement(fHandle);
				}
			}

			//intesect with faces | It is boundary node.
			if (!(pParent->children_[i]->suNode_.FaceVector.empty()))
			{
				pParent->children_[i]->label_ = BOUNDARY_CELL;

				if (nLevel == pParent->children_[i]->level_)
				{
					leafBoundaryNodes_.push_back(pParent->children_[i]);
					//std::cout << leafBoundaryNodes_.size() << std::endl;
				}

				//continue subdivide
				recursivePartition(pParent->children_[i], nLevel);
			}
		}

		return;
	}

	void suVolume::transverse(OctNode * pNode, Callback * cb)
	{

	}

	/*\brief 这个函数用于继续划分当前的节点，例如一些位于内部的节点，使其划分到与其它叶节点同样的尺寸，并进行标记
	 *\param 返回包含新叶节点的指针数组
	 *\example
	 * ```
	 *  std::vector<OctNode*> nodes;
	 * patitionToLevel(pNode, 5, pNode->label, nodes);
	 * ```
	 */
	void suVolume::patitionToLevel(OctNode * pNode, int nLevel, SU::NODE_LABEL label, std::vector<OctNode*> &nodeArr)
	{
		if (pNode->level_ >= nLevel ||
			pNode->children_[0] != 0)    //has been partitioned, not a leafnode.
			return;

		//partition each boundary node
		Point delta = pNode->max_ - pNode->min_;
		Point halfLen = delta * 0.5f;
		Point mid = halfLen + pNode->min_;

		///new sub oct node
		for (int i = 0; i < 8; i++)
		{
			pNode->children_[i] = new OctNode();
			///fill node
			pNode->children_[i]->parent_ = pNode;
			pNode->children_[i]->level_ = pNode->level_ + 1;

			///generate code
			int xLocCode = i & 0x1;
			int yLocCode = (i >> 1) & 0x1;
			int zLocCode = (i >> 2) & 0x1;

			///update node size
			float minx = (xLocCode > 0) ? (pNode->min_.x + halfLen.x) : pNode->min_.x;
			float miny = (yLocCode > 0) ? (pNode->min_.y + halfLen.y) : pNode->min_.y;
			float minz = (zLocCode > 0) ? (pNode->min_.z + halfLen.z) : pNode->min_.z;

			float maxx = (xLocCode > 0) ? pNode->max_.x : (pNode->min_.x + halfLen.x);
			float maxy = (yLocCode > 0) ? pNode->max_.y : (pNode->min_.y + halfLen.y);
			float maxz = (zLocCode > 0) ? pNode->max_.z : (pNode->min_.z + halfLen.z);


			Point newMin = Point(minx, miny, minz);
			Point newMax = Point(maxx, maxy, maxz);

			pNode->children_[i]->max_ = newMax;
			pNode->children_[i]->min_ = newMin;

			pNode->children_[i]->xLocCode_ = (xLocCode << pNode->level_ + 1) | pNode->xLocCode_;
			pNode->children_[i]->yLocCode_ = (yLocCode << pNode->level_ + 1) | pNode->yLocCode_;
			pNode->children_[i]->zLocCode_ = (zLocCode << pNode->level_ + 1) | pNode->zLocCode_;
		}

		if (pNode->children_[0]->level_ == nLevel)
		{
			//has been partitioned  to nLevel
			for (int i = 0; i < 8; i++)
			{
				pNode->children_[i]->label_ = label;
				nodeArr.push_back(pNode->children_[i]);
			}
			return;
		}
		else {
			//recursive partition
			for (int i = 0; i < 8; i++)
			{
				if (pNode->children_[i]->level_ < nLevel)
				{
					patitionToLevel(pNode->children_[i], nLevel, label, nodeArr);
				}
			}
		}

		return;
	}

	int suVolume::LabelBoundaryNeighbors()
	{
		for (int i = 0; i < leafBoundaryNodes_.size(); i++)
		{
			std::vector<OctNode*> neighbors;
			int N = get6NeighborNodes(leafBoundaryNodes_[i], neighbors);
			{
				for (int j = 0; j < N; j++)
				{
					if (neighbors[j]->label_ == UNDEFINE_CELL
						//&& neighbors[j]->level_ == level_ //only for leaf voxels
						)
					{
						neighbors[j]->label_ = labelNode(neighbors[j]);
						//neighbors[j]->label_ = INTERIOR_CELL;
						if (neighbors[j]->label_ == INTERIOR_CELL)
							leafInternalNodes_.push_back(neighbors[j]);
					}
				}
			}
		}
		return 0;
	}

	/*!
	* \function  labelNode
	*
	* \brief  search the nearest neighbor
	* \param  @pNode  [IN]current node
	*         @return  [OUT]a label.
	* \author Yuan Yao
	* \date 2016/6/16
	*/

	NODE_LABEL suVolume::labelNode(OctNode * pNode)
	{
		//return INTERIOR_CELL;
		Point delta = pNode->max_ - pNode->min_;
		Point halfLen = delta * 0.5f;
		Point mid = halfLen + pNode->min_;

		//find nearest face in neighbor nodes
		std::vector<OctNode*> neighbors;
		std::vector<suMesh::FaceHandle> faces;

		float fMinDist = 3.402823466e+38F;
		suMesh::FaceHandle faceHandle;
		int N = get6NeighborNodes(pNode, neighbors);
		{
			for (int i = 0; i < N; i++)
			{
				//we don't consider about parent node
				if (neighbors[i]->level_ == pNode->level_)
				{
					faces.insert(faces.end(), neighbors[i]->suNode_.FaceVector.begin(),
						neighbors[i]->suNode_.FaceVector.end());
				}
			}


			//for each face surrounding this node
			//find the nereast face
			int f_idx = 0;
			for (int i = 0; i < faces.size(); i++)
			{
				suMesh::FaceHandle fHandle = faces[i];
				suMesh::FaceVertexIter fv_it = mesh_.fv_iter(fHandle);
				suMesh::Point v;
				std::vector<Point> fv;

				for (int j = 0; j < 3; j++)
				{
					v = mesh_.point(fv_it++);
					fv.push_back(Point(v[0], v[1], v[2]));
				}

				Point closestPoint = ClosestPointOnTriangle(fv, mid);

				Point Delta_ = (closestPoint - mid);
				float Dist = Delta_.lenthSqrt();
				if (Dist < fMinDist)
				{
					fMinDist = Dist;
					faceHandle = fHandle;
				}
			}
		}

		suMesh::FaceVertexIter fv_it = mesh_.fv_iter(faceHandle);
		suMesh::Point v;
		std::vector<Point> fv;

		for (int j = 0; j < 3; j++)
		{
			v = mesh_.point(fv_it++);
			fv.push_back(Point(v[0], v[1], v[2]));
		}
		//compute normal			
		Point p1 = fv[2] - fv[0];
		Point p2 = fv[1] - fv[0];
		Point Normal_ = p1 * p2;
		Point closestPoint = ClosestPointOnTriangle(fv, mid);
		Point Delta_ = (closestPoint - mid);

		float fSign = Delta_.dot(Normal_);

		if (fSign < 0) return INTERIOR_CELL;
		else return EXTERIOR_CELL;
		return UNDEFINE_CELL;
	}
	void suVolume::labelNeighbors(OctNode *node)
	{
		std::vector<OctNode*> neighbors;
		int N = get6NeighborNodes(node, neighbors, false);
		for (int j = 0; j < N; j++)
		{
			if (neighbors[j]->label_ == UNDEFINE_CELL)
			{
				neighbors[j]->label_ = node->label_;
				leafInternalNodes_.push_back(neighbors[j]);
				labelNeighbors(neighbors[j]);
			}
		}

	}
	/*!
	* \function  floodFill
	* \brief  floodfill begin with
	*/
	void suVolume::floodFill()
	{
		std::vector<OctNode*> nodeArr = leafInternalNodes_;
		for (size_t i = 0; i < nodeArr.size(); i++)
		{
			std::vector<OctNode*> neighbors;
			int N = get6NeighborNodes(nodeArr[i], neighbors);
			{
				for (int j = 0; j < N; j++)
				{
					if (neighbors[j]->label_ == UNDEFINE_CELL)
					{
						neighbors[j]->label_ = INTERIOR_CELL;

						leafInternalNodes_.push_back(neighbors[j]);
						labelNeighbors(neighbors[j]);
					}
				}
			}

		}
	}

	/*!
	* \function  get6NeighborNodes
	*
	* \brief  return the 6-neighbors node.
	*         if one neighbor node is not exist, add its parent's (or parent's parent's ...) node to array if it exists
	* \param  @pNode  [IN]current node
	*         @nodes  [OUT]neighbor nodes array.
	*         @considerLevel  [IN] true(default):
	* \author Yuan Yao
	* \date 2016/5/25
	*/

	int suVolume::get6NeighborNodes(OctNode * pNode, std::vector<OctNode*>& nodes, bool considerLevel)
	{
		std::vector<OctNode *> neighbors;
		std::vector<Point> trans;
		trans.push_back(Point(1, 0, 0));
		trans.push_back(Point(-1, 0, 0));
		trans.push_back(Point(0, 1, 0));
		trans.push_back(Point(0, -1, 0));
		trans.push_back(Point(0, 0, 1));
		trans.push_back(Point(0, 0, -1));

		OctNode *pNNode_ = NULL;
		for (int i = 0; i < trans.size(); i++)
		{
			switch (i)
			{
				//if on boundary
			case 0: if (isOnTopBoundary(pNode->xLocCode_, pNode->level_)) continue; break;
			case 1: if (isOnBottomBoundary(pNode->xLocCode_, pNode->level_)) continue; break;
			case 2: if (isOnTopBoundary(pNode->yLocCode_, pNode->level_)) continue; break;
			case 3: if (isOnBottomBoundary(pNode->yLocCode_, pNode->level_)) continue; break;
			case 4: if (isOnTopBoundary(pNode->zLocCode_, pNode->level_)) continue; break;
			case 5: if (isOnBottomBoundary(pNode->zLocCode_, pNode->level_)) continue; break;
			}

			unsigned int xx = pNode->xLocCode_;
			unsigned int yy = pNode->yLocCode_;
			unsigned int zz = pNode->zLocCode_;
			if (trans[i].x != 0)
			{
				unsigned int inv_x = reverse_int(pNode->xLocCode_, pNode->level_ + 1);
				inv_x += trans[i].x;
				xx = reverse_int(inv_x, pNode->level_ + 1);
			}
			if (trans[i].y != 0)
			{
				unsigned int inv_y = reverse_int(pNode->yLocCode_, pNode->level_ + 1);
				inv_y += trans[i].y;
				yy = reverse_int(inv_y, pNode->level_ + 1);
			}
			if (trans[i].z != 0)
			{
				unsigned int inv_z = reverse_int(pNode->zLocCode_, pNode->level_ + 1);
				inv_z += trans[i].z;
				zz = reverse_int(inv_z, pNode->level_ + 1);
			}


			pNNode_ = getOctNode(pNode->level_, xx, yy, zz);
			if (pNNode_)
			{
				if (considerLevel)
				{
					if (pNNode_->level_ == pNode->level_)
					{
						nodes.push_back(pNNode_);
					}
				}
				else {
					nodes.push_back(pNNode_);
				}

			}
		}
		if (nodes.size()) //std::cout << nodes.size() << std::endl;
			return nodes.size();
	}
	/*!
	* \function  getOctNode
	*
	* \brief  return a oct node.
	*         if this node is not exist,
	*             1)return its parent's (or parent's parent's ...) node
	*             2)it is not belong to octree, return null
	* \param  @level  locCode level
	*         @xLocCode, yLocCode, zLocCode  Location code.
	* \return @true: if a special-boundary nodes is found. false: if no such node is found.
	* \author Yuan Yao
	* \date 2016/5/25
	*/

	OctNode* suVolume::getOctNode(unsigned int level,
		unsigned int xLocCode, unsigned int yLocCode, unsigned int zLocCode)
	{
		if (!pRoot_) return NULL;

		OctNode *pNode = pRoot_;

		for (unsigned int i = 1; i <= level; i++)
		{
			unsigned int locCode = (unsigned int)1 << i;

			unsigned int index = ((xLocCode & locCode) >> i) |
				(((yLocCode & locCode) >> i) << 1) |
				(((zLocCode & locCode) >> i) << 2);

			if (pNode->children_[index] == NULL)
			{
				return pNode;
			}
			pNode = pNode->children_[index];
		}
		return pNode;
	}

	/*!
	* \function  getOctNode(x,y,z)
	*
	* \brief  return a a leaf oct node, where (x,y,z) belong to.*
	* \todo: out core compatibility
	* \author Yuan Yao
	* \date 2016/5/25
	*/
	OctNode* suVolume::getOctNode(float x, float y, float z)
	{
		Point p = Point(x, y, z);

		if (!pRoot_) return NULL;

		unsigned int xLocCode = 1;
		unsigned int yLocCode = 1;
		unsigned int zLocCode = 1;

		// If current point is not in the Boundary Box.
		if (p< pRoot_->min_ || p>pRoot_->max_)
			return NULL;

		OctNode *pCurNode = pRoot_;
		while (pCurNode)
		{

			// In current node, determine where the point belong to.	
			Point delta = pCurNode->max_ - pCurNode->min_;
			Point midPoint = pCurNode->min_ + delta / 2;
			xLocCode = (p.x >= midPoint.x) ? 1 : 0;
			yLocCode = (p.y >= midPoint.y) ? 1 : 0;
			zLocCode = (p.z >= midPoint.z) ? 1 : 0;

			OctNode *pNode = pCurNode->children_[((zLocCode << 2) | (yLocCode << 1) | xLocCode)];

			if (!pNode)
			{
				return pCurNode;
			}

			//update current node information	
			pCurNode = pNode;
		}

		return pCurNode;
	}


	//from http://www.gamedev.net/topic/552906-closest-point-on-triangle/
	Point suVolume::ClosestPointOnTriangle(std::vector<Point> fv, Point pos)
	{
		Point edge0 = fv[1] - fv[0];
		Point edge1 = fv[2] - fv[0];
		Point v0 = fv[0] - pos;

		float a = edge0.dot(edge0);
		float b = edge0.dot(edge1);
		float c = edge1.dot(edge1);
		float d = edge0.dot(v0);
		float e = edge1.dot(v0);

		float det = a*c - b*b;
		float s = b*e - c*d;
		float t = b*d - a*e;

		if (s + t < det)
		{
			if (s < 0.0f)
			{
				if (t < 0.0f)
				{
					if (d < 0.0f)
					{
						s = suFSaturate(-d / a);
						t = 0.0f;
					}
					else
					{
						s = 0.0f;
						t = suFSaturate(-e / c);
					}
				}
				else
				{
					s = 0.0f;
					t = suFSaturate(-e / c);
				}
			}
			else if (t < 0.0f)
			{
				s = suFSaturate(-d / a);
				t = 0.f;
			}
			else
			{
				float invDet = 1.0f / det;
				s *= invDet;
				t *= invDet;
			}
		}
		else
		{
			if (s < 0.0f)
			{
				float tmp0 = b + d;
				float tmp1 = c + e;
				if (tmp1 > tmp0)
				{
					float numer = tmp1 - tmp0;
					float denom = a - 2.0f*b + c;
					s = suFSaturate(numer / denom);
					t = 1.0f - s;
				}
				else
				{
					t = suFSaturate(-e / c);
					s = 0.0f;
				}
			}
			else if (t < 0.0f)
			{
				if (a + d > b + e)
				{
					float numer = c + e - b - d;
					float denom = a - 2.0f*b + c;
					s = suFSaturate(numer / denom);
					t = 1.0f - s;
				}
				else
				{
					s = suFSaturate(-e / c);
					t = 0.0f;
				}
			}
			else
			{
				float numer = c + e - b - d;
				float denom = a - 2.0f*b + c;
				s = suFSaturate(numer / denom);
				t = 1.0f - s;
			}
		}
		return fv[0] + edge0 * s + edge1 * t;
	}

	bool suVolume::saveVTK(const char *pVTKFileName, int level, const char *pVTKHead /*= "UnKnown Name"*/, float dx /*= 0*/, float dy /*= 0*/, float dz /*= 0*/)
	{
		if (NULL == pVTKFileName || NULL == pVTKHead)
			return false;


		std::ofstream vtk(pVTKFileName);

		if (!vtk)
			return false;

		vtk << "# vtk DataFile Version 2.0" << std::endl;
		vtk << pVTKHead << std::endl;
		vtk << "ASCII" << std::endl;
		vtk << "DATASET STRUCTURED_POINTS" << std::endl;

		SU::Point bMax(bbMax_[0], bbMax_[1], bbMax_[2]);
		SU::Point bMin(bbMin_[0], bbMin_[1], bbMin_[2]);
		SU::Point vSize = bMax - bMin;

		if (dx == 0)
		{
			float nSize = pow(2, level);
			dx = vSize.x / nSize;
			dy = vSize.y / nSize;
			dz = vSize.z / nSize;
		}

		unsigned int Xdim = (unsigned int)(vSize.x / dx);
		unsigned int Ydim = (unsigned int)(vSize.y / dy);
		unsigned int Zdim = (unsigned int)(vSize.z / dz);

		vtk << "DIMENSIONS " << Xdim << " " << Ydim << " " << Zdim << std::endl;

		// 数据区的其他信息
		vtk << "ASPECT_RATIO 1 1 1" << std::endl;
		vtk << "ORIGIN 0 0 0" << std::endl;
		vtk << "POINT_DATA " << Xdim * Ydim * Zdim << std::endl;
		vtk << "SCALARS volume_scalars char 1" << std::endl;
		vtk << "LOOKUP_TABLE default" << std::endl << std::endl;

		for (unsigned int IndexZ = 0; IndexZ < Zdim; IndexZ++)
		{
			float CellCenter_Z = bMin.z + IndexZ * dz + dz / 2.0f;

			for (unsigned int IndexY = 0; IndexY < Ydim; IndexY++)
			{
				float CellCenter_Y = bMin.y + IndexY * dy + dy / 2.0f;

				for (unsigned int IndexX = 0; IndexX < Xdim; IndexX++)
				{
					float CellCenter_X = bMin.x + IndexX * dx + dx / 2.0f;

					OctNode *pNode = getOctNode(CellCenter_X, CellCenter_Y, CellCenter_Z);
					if (NULL != pNode)
					{
						// 内部节点输出 0 
						if (INTERIOR_CELL == pNode->label_)
							vtk << 2 << " ";

						// 边界节点输出 1
						else if (BOUNDARY_CELL == pNode->label_ || BOUNDARY_CELL_SPECIAL == pNode->label_)
							vtk << 1 << " ";

						// 外部节点输出 2
						else if (EXTERIOR_CELL == pNode->label_)
							vtk << 3 << " ";

						// 其他情况输出 3 
						else
							vtk << 0 << " ";
					}
					else
						vtk << 4 << " ";
				}
				vtk << std::endl;
			}
			vtk << std::endl;
		}

		return 0;
	}
	class CmpVec
	{
	public:

		CmpVec(float _eps = FLT_MIN) : eps_(_eps) {}

		bool operator()(const SU::Point& _v0, const SU::Point& _v1) const
		{
			if (fabs(_v0[0] - _v1[0]) <= eps_)
			{
				if (fabs(_v0[1] - _v1[1]) <= eps_)
				{
					return (_v0[2] < _v1[2] - eps_);
				}
				else return (_v0[1] < _v1[1] - eps_);
			}
			else return (_v0[0] < _v1[0] - eps_);
		}

	private:
		float eps_;
	};

	/************************************
	* saveBaseInp
	* 1. nodes and elements are saved.
	* Add element_id to each OctNode
	************************************/
	bool suVolume::saveBaseInp(std::string filename,
		std::set<int> &face_list_force,
		float force_value,
		std::set<int> &face_list_constraint)
	{
		if (leafBoundaryNodes_.empty()) return false;

		struct _Element {
			int node_index[8];
		};
		std::vector<_Element> _elements;
		std::vector<SU::Point> _nodes;
		std::set<int> _nodes_load;
		std::set<int> _nodes_constraints;
		CmpVec comp(FLT_MIN);
		std::map<SU::Point, unsigned int, CmpVec> _nodeMap;

		// define a lambda function to satisfy
		// 1. add element_id to original leafBoundaryNodes_ and leafInternalNodes_
		// 2. ensure the uniqueness of the nodes and elements 
		auto gen_element = [&](std::vector<SU::OctNode*> &oct_nodes) {

			std::vector<SU::OctNode*>::iterator it = oct_nodes.begin();

			while (it != oct_nodes.end())
			{
				if ((*it)->level_ != level_) {
					it++;
					continue;
				}

				SU::Point &m = (*it)->min_;
				SU::Point &M = (*it)->max_;
				_Element ele;

				//generate vertices for each voxel
				SU::Point v[8];
				v[0] = SU::Point(m.x, m.y, m.z);
				v[1] = SU::Point(M.x, m.y, m.z);
				v[2] = SU::Point(M.x, M.y, m.z);
				v[3] = SU::Point(m.x, M.y, m.z);
				v[4] = SU::Point(m.x, m.y, M.z);
				v[5] = SU::Point(M.x, m.y, M.z);
				v[6] = SU::Point(M.x, M.y, M.z);
				v[7] = SU::Point(m.x, M.y, M.z);

				for (int k = 0; k < 8; k++) {
					auto n = _nodeMap.find(v[k]);
					if (n == _nodeMap.end()) {
						_nodes.push_back(v[k]);
						unsigned int idx = _nodes.size();
						_nodeMap[v[k]] = idx;
						ele.node_index[k] = idx;
					}
					else {
						ele.node_index[k] = n->second;
					}
				}
				_elements.push_back(ele);
				(*it)->suNode_.element_id = _elements.size();  //record element id
				it++;
			}
		};

		auto gen_force_nodes = [&](std::set<int> &face_list_force) {
			std::map<int, int> force_elements;

			std::vector<OctNode*>::iterator it = leafBoundaryNodes_.begin();
			for (; it != leafBoundaryNodes_.end(); ++it) {

				//convert face handle to id
				std::set<int> faceid_list;
				std::vector<OpenMesh::FaceHandle>::iterator f_it = (*it)->suNode_.FaceVector.begin();
				for (; f_it != (*it)->suNode_.FaceVector.end(); ++f_it) {
					faceid_list.insert(f_it->idx());
				}
				
				//if selected oct node
				for (int i:face_list_force) {
					if (faceid_list.count(i)) {
						if((*it)->suNode_.element_id != -1)
						   force_elements[(*it)->suNode_.element_id] = (*it)->suNode_.element_id;
					}				
				}
			}

			//get load nodes(point) from load elments
			int ii = 0;
			for (std::map<int, int>::iterator it = force_elements.begin();
				it != force_elements.end(); ++it) {

				int _eid = it->first;
				for (int i = 0; i < 8; i++) {
					int _nid = _elements[_eid].node_index[i];
					_nodes_load.insert(_nid);
				}
			}
		};

		
		gen_element(leafBoundaryNodes_);
		gen_force_nodes(face_list_force);
		gen_element(leafInternalNodes_);

		std::stringbuf strInp;
		std::ofstream inpFile;
		inpFile.open(filename, std::ios::out);

		inpFile << "*Heading" << std::endl;
		inpFile << "** Job name : EXAMPLE Model name :" << filename << std::endl;
		inpFile << "* * Generated by : suDesigner" << std::endl;
		inpFile << "*Preprint, echo = NO, model = NO, history = NO, contact = NO" << std::endl;
		inpFile << "*Node, NSET = Nall" << std::endl;
		// write nodes
		std::stringstream os;
		for (unsigned int i = 0; i < _nodes.size(); i++) {
			os << i + 1 << ",  " << _nodes[i].x << ",   " << _nodes[i].y << ",   " << _nodes[i].z << std::endl;
		}
		inpFile << os.str();

		//write elements
		os.str(std::string());
		os.clear();
		os << "*Element, type = C3D8" << std::endl;
		for (unsigned int i = 0; i < _elements.size(); i++) {
			os << i + 1 << ",  ";
			for (int j = 0; j < 7; j++) {
				os << _elements[i].node_index[j] << ",   ";
			}
			os << _elements[i].node_index[7] << std::endl;
		}
		inpFile << os.str();

		//clear string stream
		os.str(std::string());
		os.clear();

		//write force related
		os << std::endl;
		os << "*NSET, NSET = FemLoadFixed" << std::endl;

		int ii = 0;
		for (std::set<int>::iterator it = _nodes_load.begin();
			it != _nodes_load.end(); ++it) {
			os << *it << ", ";
			if (++ii % 6 == 0) os << std::endl;
		}
		os << std::endl;

		os << "***********************************************************" << std::endl;
		os << "** One step is needed to calculate the mechanical analysis of FreeCAD" << std::endl;
		os << "** loads are applied quasi - static, means without involving the time dimension" << std::endl;
		os << "** written by write_step_begin function" << std::endl;
		os << "*STEP" << std::endl;
		os << "*STATIC" << std::endl;

		os << "***********************************************************" << std::endl;
		os << "** Node loads" << std::endl;
		os << "** written by addForce function" << std::endl;
		os << "*CLOAD" << std::endl;
		os << "** FemLoadFixed" << std::endl;
		os << "** node loads on element Face : Box:Face6" << std::endl;
		os << "FemLoadFixed, 2, " << force_value << std::endl;
		inpFile << os.str() << std::endl;

		os.str(std::string());
		os.clear();

		inpFile.close();
		return false;
	}

	bool suVolume::addForce(std::string filename, std::vector<int> face_list_force)
	{
		std::ifstream File(filename);
		if (File.fail()) return false;


		std::map<int, int> force_elements;

		std::vector<OctNode*>::iterator it = leafBoundaryNodes_.begin();
		for (; it != leafBoundaryNodes_.end(); ++it) {

			//convert face handle to id
			std::vector<int> faceid_list;
			std::vector<OpenMesh::FaceHandle>::iterator f_it = (*it)->suNode_.FaceVector.begin();
			for (; f_it != (*it)->suNode_.FaceVector.end(); ++f_it) {
				faceid_list.push_back(f_it->idx());
			}
			//if selected node
			for (int i = 0; i < face_list_force.size(); i++) {
				if (std::find(faceid_list.begin(),
					faceid_list.end(),
					face_list_force[i]) != faceid_list.end()) {
					force_elements[(*it)->suNode_.element_id] = (*it)->suNode_.element_id;
				}
			}
		}

		std::ofstream inpFile;
		inpFile.open(filename, std::ios::app);
		std::stringstream os;

		os << "*NSET, NSET = FemLoadFixed" << std::endl;


		int ii = 0;
		for (std::map<int, int>::iterator it = force_elements.begin();
			it != force_elements.end(); ++it) {
			os << it->first << ", ";
			if (++ii % 6 == 0) os << std::endl;
		}
		os << std::endl;

		os << "***********************************************************" << std::endl;
		os << "** One step is needed to calculate the mechanical analysis of FreeCAD" << std::endl;
		os << "** loads are applied quasi - static, means without involving the time dimension" << std::endl;
		os << "** written by write_step_begin function" << std::endl;
		os << "*STEP" << std::endl;
		os << "*STATIC" << std::endl;

		os << "***********************************************************" << std::endl;
		os << "** Node loads" << std::endl;
		os << "** written by addForce function" << std::endl;
		os << "*CLOAD" << std::endl;
		os << "** FemLoadFixed" << std::endl;
		os << "** node loads on element Face : Box:Face6" << std::endl;
		os << "FemLoadFixed, 2, 100" << std::endl;
		inpFile << os.str() << std::endl;


		return false;
	}

	bool suVolume::addBoundary(std::string filename, std::vector<int> face_list_fix)
	{

		return false;
	}


}//end namespace SU
