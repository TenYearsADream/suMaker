#pragma once
#include <suMesh.h>
#include <deque>


namespace SU{
	struct Point
	{
		float x;
		float y;
		float z;
		Point() :x(0), y(0), z(0){}
		Point(const Point& p2) : x(p2.x), y(p2.y), z(p2.z) {}
		Point(float in_x, float in_y, float in_z) : x(in_x), y(in_y), z(in_z) {}
		Point(const float p2[3]) : x(p2[0]), y(p2[1]), z(p2[2]) {}
		Point& operator=(const Point& p2) { x = p2.x; y = p2.y; z = p2.z; return *this; }
		operator float*() { return &x; }
		operator const float*() const { return &x; }
		Point operator+(const Point& p2) const { return Point(x + p2.x, y + p2.y, z + p2.z); }
		Point operator-(const Point& p2) const { return Point(x - p2.x, y - p2.y, z - p2.z); }
		Point operator*(float f) const { return Point(x*f, y*f, z*f); }
		Point operator/(float f) const { return Point(x / f, y / f, z / f); }
		bool operator< (const Point& p2) const { return x < p2.x && y < p2.y && z < p2.z; }
		bool operator>(const Point& p2) const { return x > p2.x && y > p2.y && z > p2.z; }
		bool operator>=(const Point& p2) const { return x >= p2.x && y >= p2.y && z >= p2.z; }
		bool operator<=(const Point& p2) const { return x <= p2.x && y <= p2.y && z <= p2.z; }
		Point operator*(Point& p2) { //Cross product
			return Point(
				(y * p2.z) - (z * p2.y),
				(z * p2.x) - (x * p2.z),
				(x * p2.y) - (y * p2.x)
			);
		}  
		float dot(Point& p2) { return (x * p2.x) + (y * p2.y) + (z * p2.z); }
		float lenthSqrt() { return x*x + y*y + z*z; }
	};

	

	//
	typedef enum
	{
		EXTERIOR_CELL = 0,           // �ⲿ�ڵ�
		BOUNDARY_CELL = 1,           // ����ڵ㣬�˱���ڵ�����������
		BOUNDARY_CELL_SPECIAL = 2,   // ����ڵ㣬�˱���ڵ����������棬ֻ��������ıߴ����˸ýڵ�
		INTERIOR_CELL = 3,           // �ڲ��ڵ�
		UNDEFINE_CELL = 4,           // δ����ڵ�
	}NODE_LABEL;

	class suNode{
	public:
		int AddElement(suMesh::VertexIter  CurrentVertexIter, char Label = 'U')
		{
			VertexVector.push_back(CurrentVertexIter);
			LabelVector.push_back(Label);
			return 0;
		}
		int AddElement(suMesh::FaceHandle faceHandle)
		{
			FaceVector.push_back(faceHandle);
			return 0;
		}
		void ClearAll()
		{
			VertexVector.clear();
			FaceVector.clear();
			LabelVector.clear();
			return;
		}
		int SetLabel(suMesh::VertexIter  CurrentVertexIter, char Label)
		{
			if (Label != 'I' && Label != 'S' && Label != 'O')
				return -1;

			for (unsigned int index = 0; index < VertexVector.size(); index++)
			{
				if (VertexVector[index] == CurrentVertexIter)
				{
					LabelVector[index] = Label;
					return 0;
				}
			}
			return -2;
		}
	public:
		//vertex in the node
		std::vector<suMesh::VertexIter>     VertexVector;
		//face handle in the node
		std::vector<suMesh::FaceHandle>     FaceVector;
		std::vector<char>                   LabelVector;
	};

	struct OctNode
	{
		suNode suNode_;
		OctNode* children_[8];
		OctNode* parent_;

		Point max_, min_;
		
		unsigned int   xLocCode_;     // X code
		unsigned int   yLocCode_;     // Y code
		unsigned int   zLocCode_;     // Z code
		unsigned int   level_;        // current level, in 32 system unsigned bit is
		// represented by 32 bit, so MAX(level) = 32
		unsigned int location;
		unsigned int locationNext;
		unsigned int morton;
		bool out;
		bool outNext;
		float strain;

		NODE_LABEL     label_;

		OctNode() :xLocCode_(0), yLocCode_(0), zLocCode_(0), level_(0), label_(UNDEFINE_CELL), 
			parent_(0),location(0),locationNext(0),morton(0),out(false),
			outNext(true),strain(0)
		{
			for (int i = 0; i < 8; i++)
				children_[i] = 0;
		}
		virtual ~OctNode()
		{
			for (int i = 0; i < 8; i++)
			if (children_[i])
				delete children_[i];
			parent_ = 0;
		}

		inline Point center() { return (max_ + min_) / 2; }
		inline Point size() { return max_ - min_; }
	};

	/* \class suVolume
	 * \brief �����࣬�����˰˲����Ļ��ֹ���
	 * \todo  ������
	 */
	class suVolume
	{
	public:
		suVolume() : pRoot_(0), curLevel_(0),level_(0), isLoad_(0){}
		~suVolume(){ clear(); }

		class Callback
		{
		public:
			// Return value: true = continue; false = abort.
			virtual bool operator()(OctNode* pNode) = 0;
		};

		int  PartitionSpace(unsigned int level);
		
		void  recursivePartition(OctNode * pParent, int nLevel);	
		void  transverse(OctNode *pNode, Callback *cb);

		void  patitionToLevel(OctNode *pNode, int nLevel, SU::NODE_LABEL label, std::vector<OctNode*> &nodeArr);  //continue partion current node to level, and set theirs label

		
		//IO
		bool saveVTK(const char *pVTKFileName, int level = 3, const char *pVTKHead = "UnKnown Name",
			float dx = 0, float dy = 0, float dz = 0);

		bool LoadMeshFromMesh(suMesh &m);
		bool LoadMeshFromFile(const char *pFileName);

		int LabelBoundaryNeighbors();
		NODE_LABEL labelNode(OctNode* pNode);
		void floodFill();	
		void labelNeighbors(OctNode *node);
				
		int  get6NeighborNodes(OctNode *pNode, std::vector<OctNode*> &nodes, bool considerLevel = true );
		bool isOnTopBoundary(int code, int level)
		{
			int sum_c = 0;
			for (int i = 0; i <= level; i++)
			{
				int c = (code >> i) & 0x1;
				//std::cout << c << " ,";
				sum_c += c;
			}
			//std::cout << std::endl << sum_c << std::endl;

			if (sum_c == level) return true;
			return false;
		}
		bool isOnBottomBoundary(int code, int level)
		{
			int sum_c = 0;
			for (int i = 0; i <= level; i++)
			{
				int c = (code >> i) & 0x1;
				sum_c += c;
			}
			//std::cout << sum_c << std::endl;

			if (sum_c == 0) return true;
			return false;
		}
		OctNode* getOctNode(unsigned int level,
			unsigned int   xLocCode,
			unsigned int   yLocCode,
			unsigned int  zLocCode);
		OctNode* getOctNode(float x, float y, float z);

		//tool functions
		Point ClosestPointOnTriangle(std::vector<Point> fv, Point pos);

	private:
		void clear();
		
	public://for debug
		suMesh             mesh_;
		suMesh::Point      bbMin_, bbMax_;
		OctNode            *pRoot_;
				
		std::vector<OctNode*>   leafBoundaryNodes_;  //To store all nodes for each level
		std::vector<OctNode*>   leafInternalNodes_;

		bool               isLoad_;

		int                curLevel_;
		int                level_;     //max level

	};

}//end namespace SU