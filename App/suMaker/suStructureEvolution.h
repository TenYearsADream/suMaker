#pragma once
/*
    A volume structure evolution class.
	Morton code is used to index all voxels	
	*******
	\brief suStructrueOptimizer这个类提供了体素结构的演化框架
	\todo：为便于扩展，演化方法和演化框架应设计在不同的对象中。
	\example   
	```
	suStructrueOptimizer  framework;
	framework.set_octree_level(5);
	framework.set_rule_lists(...);
	framework.init(internal_nodes);
	framework.run();
	```
   
**/

#ifdef __GNUG__
     // g++, clang++: [[ gnu::always_inline ]] does not also imply C++ inline
     // [[ gnu::always_inline ]] is an additional implementation-defined
     // attribute using the C++11 unified standard attribute syntax. note: gnu::
     #define FORCEINLINE [[ gnu::always_inline ]] inline

#elif defined _MSC_VER
     // msc++: ____forceinline implies C++ inline
     // msc++ does not provide any additional implementation-defined
     // attributes using the C++11 unified standard attribute syntax
     #define FORCEINLINE __forceinline // inline is implied

#else
     #define FORCEINLINE inline
#endif
#include <vector>

//forecast defination
namespace SU
{
	class OctNode;
};


//evolution framework
class suStructrueOptimizer
{
public:
	suStructrueOptimizer():nLevel_(0), nIterTimes_(1){};
	~suStructrueOptimizer() {};

	void set_octree_level(int level) { nLevel_ = level; }
	void set_iter_times(int nTimes) { nIterTimes_ = nTimes; }
		
	int evolve();     //return num of iteration
	int init_morton_coding(std::vector<SU::OctNode*> &voxels);  //generate a node array in nodeArr_, indexed with morton code
	int init_position_field(); 
	int init(std::vector<SU::OctNode*> &voxels)
	{
		init_morton_coding(voxels);
		int nMaxValue = init_position_field();
		return 0;
	}
	int run()
	{		
		evolve();
		return 0;
	}

	//knowlege base
	void readRule() {};	
	
	
private:
	std::vector<SU::OctNode*> nodeArr_;  //morton coded node array 
	int nLevel_;     //octree level
	int nIterTimes_; //interation times      

};


