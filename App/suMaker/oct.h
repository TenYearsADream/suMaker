//#include "read_stl.h"
#pragma once
#include <vector>
#include <cmath>
#include <fstream>
using namespace std;
/*class octree_info
{
public:
	long long morton;
	bool in_out = false;//true is in     false is out
	bool inside_boun = false;//true is boundary    false is inside
	//both false is unkonw
	//in_out==false  inside_boun==true means outside
};

class oct
{
public:
	vector <octree_info> octrees;3
	void generate_oct(string,int);
	oct(string, int);//构造函数中输入STL文件地址，划分深度
};*/
#ifndef coorAndface
#define coorAndface
struct coor{
	float x;
	float y;
	float z;
};
struct face {
	coor coors[3];
};
bool overlap(coor, coor, coor,//tri
	float, float, float, float, float, float);//voxel
int merton(int, int, int, int);
//int *code(int , int);
#endif
/*
class voxelRead{//n表示在第几个文件中  levelLow表示文件中存储的分割次数
private:
	octree_info *voxels = new octree_info;
	int n;
	int levelLow;
public:
	voxelRead(int, string, int);//构造函数读入文件  标记是否存在   	
	~voxelRead();//析构函数中删除数组
	int& getVoxel(int);//返回一个体素的引用
};*/
