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
	oct(string, int);//���캯��������STL�ļ���ַ���������
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
class voxelRead{//n��ʾ�ڵڼ����ļ���  levelLow��ʾ�ļ��д洢�ķָ����
private:
	octree_info *voxels = new octree_info;
	int n;
	int levelLow;
public:
	voxelRead(int, string, int);//���캯�������ļ�  ����Ƿ����   	
	~voxelRead();//����������ɾ������
	int& getVoxel(int);//����һ�����ص�����
};*/
