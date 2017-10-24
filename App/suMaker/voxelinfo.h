#pragma once
#include <iostream>
using namespace std;
class VoxelInfo {
public:
	long long morton;
	bool in_out = false;//true is in     false is out
	bool inside_boun = false;//true is boundary    false is inside
							//both false is unkonw
							//in_out==false  inside_boun==true means outside
	bool operator==(const VoxelInfo & o);
	VoxelInfo& operator=(const VoxelInfo & o);
	bool operator>(const VoxelInfo &o);
	bool operator<(const VoxelInfo &o);
	bool operator!=(const VoxelInfo &o);
	bool operator!=(const int & o);
	void operator=(const int & o);
	bool operator<(const unsigned long long &o);
	unsigned long long operator%(const unsigned long long&o);
	unsigned long long operator/(const unsigned long long & o);
};
ostream& operator<<(ostream & os, const VoxelInfo & o);