#pragma once
#include <string>
#include"voxelinfo.h"
#include"oct.h"
#include <vector>
#include "cache.h"
class generator {
private:
	std::string address;
	int level;
	
	
	double bounX, bounY, bounZ;
	double dx, dy, dz;
public:
	vector<face> faces;
	generator(std::string address_, int level_,vector<face> faces_) :
		address(address_), level(level_),faces(faces_) {//在这里生成初始文件
		for (int i = 1;i <= level;i++) {
			voxels<VoxelInfo> saveOut(address, i, "mark", 4096);
		}
	};
	void generatorOctree(int, vector<face>, float[6], int, bool);
	float *calbb(vector<face>);

};