#include "voxelinfo.h"
#include <iostream>
bool VoxelInfo::operator==(const VoxelInfo & o)
{
	return morton == o.morton;
}

VoxelInfo& VoxelInfo::operator=(const VoxelInfo & o)
{
	inside_boun = o.inside_boun;
	in_out = o.in_out;
	morton = o.morton;
	return *this;
}

bool VoxelInfo::operator>(const VoxelInfo & o)
{
	return morton > o.morton;
}

bool VoxelInfo::operator<(const VoxelInfo & o)
{
	return morton < o.morton;
}

bool VoxelInfo::operator!=(const VoxelInfo & o)
{
	return morton != o.morton;
}

ostream & operator<<(ostream & os, const VoxelInfo & o)
{
	cout << o.inside_boun << ' ' << o.in_out << ' ' << o.morton;
	return os;
}

bool VoxelInfo::operator!=(const int & o)
{
	return morton == o;
}

void VoxelInfo::operator=(const int & o)
{
	morton = o;
}

bool VoxelInfo::operator<(const unsigned long long & o)
{
	return morton < o;
}

unsigned long long VoxelInfo::operator%(const unsigned long long & o)
{
	return morton%o;
}

unsigned long long VoxelInfo::operator/(const unsigned long long & o)
{
	return morton / o;
}