#include "oct.h"
#include <iostream>
using namespace std;



bool overlap(coor tri1, coor tri2, coor tri3, float voxel_xn, float voxel_xp, float voxel_yn, float voxel_yp, float voxel_zn, float voxel_zp)
{
	if (tri1.x < voxel_xn&&tri2.x < voxel_xn&&tri3.x < voxel_xn)
		return false;
	if (tri1.x > voxel_xp&&tri2.x > voxel_xp&&tri3.x > voxel_xp)
		return false;
	if (tri1.y < voxel_yn&&tri2.y < voxel_yn&&tri3.y < voxel_yn)
		return false;
	if (tri1.y > voxel_yp&&tri2.y > voxel_yp&&tri3.y > voxel_yp)
		return false;
	if (tri1.z < voxel_zn&&tri2.z < voxel_zn&&tri3.z < voxel_zn)
		return false;
	if (tri1.z > voxel_zp&&tri2.z > voxel_zp&&tri3.z > voxel_zp)
		return false;
	//cout << "1";
	vector<coor> polygon;
	polygon.push_back(tri1);
	polygon.push_back(tri2);
	polygon.push_back(tri3);
	vector <int> point_to_cut;
	vector <int> replace_location;
	vector <coor> replace;
	for (int i = 0; i < polygon.size(); i++)
	{
		//cout << "2";
		if (polygon[i].x >= voxel_xn)//判断这个点要被切除还是保留  保留就continue  
			continue;
	
		int location_temp[2];//找这个点的相邻点  判断相邻点有没有交叉  没有交叉就continue
		if (i == 0)
		{
			location_temp[0] = polygon.size() - 1;
			location_temp[1] = 1;
			if (polygon[location_temp[0]].x < voxel_xn&&polygon[location_temp[1]].x < voxel_xn)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}
		else if (i == polygon.size() - 1)
		{
			location_temp[0] = polygon.size() - 2;
			location_temp[1] = 0;
			if (polygon[location_temp[0]].x < voxel_xn&&polygon[location_temp[1]].x < voxel_xn)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}
		else
		{
			location_temp[0] = i - 1;
			location_temp[1] = i + 1;
			if (polygon[location_temp[0]].x < voxel_xn&&polygon[location_temp[1]].x < voxel_xn)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}//至少有一个交叉 计算交点  替换
		if (polygon[i].x == voxel_xn)
			continue;
		//count the intersect coordinate
		if ((polygon[location_temp[0]].x<voxel_xn&&polygon[location_temp[1]].x>voxel_xn) || (polygon[location_temp[1]].x> voxel_xn&&polygon[location_temp[0]].x == voxel_xn))//与后一个有交叉
		{
			coor replace_temp;
			replace_temp.x = voxel_xn;
			replace_temp.y = polygon[i].y + (voxel_xn - polygon[i].x)
				*(polygon[location_temp[1]].y - polygon[i].y) / (polygon[location_temp[1]].x - polygon[i].x);
			replace_temp.z = polygon[i].z + (voxel_xn - polygon[i].x)
				*(polygon[location_temp[1]].z - polygon[i].z) / (polygon[location_temp[1]].x - polygon[i].x);
			replace.push_back(replace_temp);
			replace_location.push_back(i);
		}
		else if ((polygon[location_temp[0]].x>voxel_xn&&polygon[location_temp[1]].x < voxel_xn) || (polygon[location_temp[0]].x> voxel_xn&&polygon[location_temp[1]].x == voxel_xn))//与前一个有交叉
		{
			coor replace_temp;
			replace_temp.x = voxel_xn;
			replace_temp.y = polygon[i].y + (voxel_xn - polygon[i].x)
				*(polygon[location_temp[0]].y - polygon[i].y) / (polygon[location_temp[0]].x - polygon[i].x);
			replace_temp.z = polygon[i].z + (voxel_xn - polygon[i].x)
				*(polygon[location_temp[0]].z - polygon[i].z) / (polygon[location_temp[0]].x - polygon[i].x);
			replace.push_back(replace_temp);
			replace_location.push_back(i);
		}
		else//都有交叉
		{
			coor temp[2];
			//polygon.insert(polygon.begin() + i, temp);

			temp[0].x = voxel_xn;
			temp[0].y = polygon[i].y + (voxel_xn - polygon[i].x)
				*(polygon[location_temp[0]].y - polygon[i].y) / (polygon[location_temp[0]].x - polygon[i].x);
			temp[0].z = polygon[i].z + (voxel_xn - polygon[i].x)
				*(polygon[location_temp[0]].z - polygon[i].z) / (polygon[location_temp[0]].x - polygon[i].x);

			temp[1].x = voxel_xn;
			temp[1].y = polygon[i].y + (voxel_xn - polygon[i].x)
				*(polygon[location_temp[1]].y - polygon[i].y) / (polygon[location_temp[1]].x - polygon[i].x);
			temp[1].z = polygon[i].z + (voxel_xn - polygon[i].x)
				*(polygon[location_temp[1]].z - polygon[i].z) / (polygon[location_temp[1]].x - polygon[i].x);
			polygon[i] = temp[0];
			polygon.insert(polygon.begin() + i + 1, temp[1]);

			//cout << endl;
			for (int i = 0; i < polygon.size(); i++)
			{
				//cout << polygon[i].x << ' ' << polygon[i].y << ' ' << polygon[i].z << endl;
			}
			break;//这种情况已经不用继续判断了
		}
	}
	//该切的切掉  替换的换掉
	if (replace.size() != 0)
	{
		for (int i = 0; i < replace.size(); i++)
		{
			polygon[replace_location[i]] = replace[i];
		}
		replace_location.clear();
		replace.clear();
	}
	for (int i = point_to_cut.size() - 1; i >= 0; i--)
	{
		polygon.erase(polygon.begin() + point_to_cut[i]);
	}
	point_to_cut.clear();

	for (int i = 0; i < polygon.size(); i++)
	{
		//cout << endl << "3";
		if (polygon[i].x <= voxel_xp)//判断这个点要被切除还是保留  保留就continue  
			continue;

		int location_temp[2];//找这个点的相邻点  判断相邻点有没有交叉  没有交叉就continue
		if (i == 0)
		{
			location_temp[0] = polygon.size() - 1;
			location_temp[1] = 1;
			if (polygon[location_temp[0]].x > voxel_xp&&polygon[location_temp[1]].x > voxel_xp)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}
		else if (i == polygon.size() - 1)
		{
			location_temp[0] = polygon.size() - 2;
			location_temp[1] = 0;
			if (polygon[location_temp[0]].x > voxel_xp&&polygon[location_temp[1]].x > voxel_xp)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}
		else
		{
			location_temp[0] = i - 1;
			location_temp[1] = i + 1;
			if (polygon[location_temp[0]].x > voxel_xp&&polygon[location_temp[1]].x > voxel_xp)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}//至少有一个交叉 计算交点  替换
		if (polygon[i].x == voxel_xp)
		{
			continue;
		}
		//count the intersect coordinate
		if ((polygon[location_temp[0]].x> voxel_xp&&polygon[location_temp[1]].x<voxel_xp) || (polygon[location_temp[1]].x< voxel_xp&&polygon[location_temp[0]].x == voxel_xp))//与后一个有交叉
		{
			coor replace_temp;
			replace_temp.x = voxel_xp;
			replace_temp.y = polygon[i].y + (voxel_xp - polygon[i].x)
				*(polygon[location_temp[1]].y - polygon[i].y) / (polygon[location_temp[1]].x - polygon[i].x);
			replace_temp.z = polygon[i].z + (voxel_xp - polygon[i].x)
				*(polygon[location_temp[1]].z - polygon[i].z) / (polygon[location_temp[1]].x - polygon[i].x);
			replace.push_back(replace_temp);
			replace_location.push_back(i);
		}
		else if (polygon[location_temp[0]].x<voxel_xp&&polygon[location_temp[1]].x > voxel_xp || (polygon[location_temp[0]].x< voxel_xp&&polygon[location_temp[1]].x == voxel_xp))//与前一个有交叉
		{
			coor replace_temp;
			replace_temp.x = voxel_xp;
			replace_temp.y = polygon[i].y + (voxel_xp - polygon[i].x)
				*(polygon[location_temp[0]].y - polygon[i].y) / (polygon[location_temp[0]].x - polygon[i].x);
			replace_temp.z = polygon[i].z + (voxel_xp - polygon[i].x)
				*(polygon[location_temp[0]].z - polygon[i].z) / (polygon[location_temp[0]].x - polygon[i].x);
			replace.push_back(replace_temp);
			replace_location.push_back(i);
		}
		else//都有交叉
		{
			coor temp[2];
			//polygon.insert(polygon.begin() + i, temp);

			temp[0].x = voxel_xp;
			temp[0].y = polygon[i].y + (voxel_xp - polygon[i].x)
				*(polygon[location_temp[0]].y - polygon[i].y) / (polygon[location_temp[0]].x - polygon[i].x);
			temp[0].z = polygon[i].z + (voxel_xp - polygon[i].x)
				*(polygon[location_temp[0]].z - polygon[i].z) / (polygon[location_temp[0]].x - polygon[i].x);

			temp[1].x = voxel_xp;
			temp[1].y = polygon[i].y + (voxel_xp - polygon[i].x)
				*(polygon[location_temp[1]].y - polygon[i].y) / (polygon[location_temp[1]].x - polygon[i].x);
			temp[1].z = polygon[i].z + (voxel_xp - polygon[i].x)
				*(polygon[location_temp[1]].z - polygon[i].z) / (polygon[location_temp[1]].x - polygon[i].x);
			polygon[i] = temp[0];
			polygon.insert(polygon.begin() + i + 1, temp[1]);



			break;//这种情况已经不用继续判断了
		}
	}

	//该切的切掉  替换的换掉
	if (replace.size() != 0)
	{
		for (int i = 0; i < replace.size(); i++)
		{
			polygon[replace_location[i]] = replace[i];
		}
		replace_location.clear();
		replace.clear();
	}
	for (int i = point_to_cut.size() - 1; i >= 0; i--)
	{
		polygon.erase(polygon.begin() + point_to_cut[i]);
	}
	point_to_cut.clear();


	int compare = 0;
	for (int i = 0; i < polygon.size(); i++)
	{
		if (polygon[i].y > voxel_yp)
			compare++;
	}
	if (compare == polygon.size())
		return false;
	compare = 0;
	for (int i = 0; i < polygon.size(); i++)
	{
		if (polygon[i].y < voxel_yn)
			compare++;
	}
	if (compare == polygon.size())
		return false;



	for (int i = 0; i < polygon.size(); i++)
	{
		if (polygon[i].y <= voxel_yp)//判断这个点要被切除还是保留  保留就continue  
			continue;

		int location_temp[2];//找这个点的相邻点  判断相邻点有没有交叉  没有交叉就continue
		if (i == 0)
		{
			location_temp[0] = polygon.size() - 1;
			location_temp[1] = 1;
			if (polygon[location_temp[0]].y > voxel_yp&&polygon[location_temp[1]].y > voxel_yp)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}
		else if (i == polygon.size() - 1)
		{
			location_temp[0] = polygon.size() - 2;
			location_temp[1] = 0;
			if (polygon[location_temp[0]].y > voxel_yp&&polygon[location_temp[1]].y > voxel_yp)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}
		else
		{
			location_temp[0] = i - 1;
			location_temp[1] = i + 1;
			if (polygon[location_temp[0]].y > voxel_yp&&polygon[location_temp[1]].y > voxel_yp)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}//至少有一个交叉 计算交点  替换
		//count the intersect coordinate
		if ((polygon[location_temp[0]].y> voxel_yp&&polygon[location_temp[1]].y<voxel_yp) || (polygon[location_temp[1]].y<voxel_yp&&polygon[location_temp[0]].y == voxel_yp))//与后一个有交叉
		{
			coor replace_temp;
			replace_temp.x = polygon[i].x + (voxel_yp - polygon[i].y)
				*(polygon[location_temp[1]].x - polygon[i].x) / (polygon[location_temp[1]].y - polygon[i].y);
			replace_temp.y = voxel_yp;
			replace_temp.z = polygon[i].z + (voxel_yp - polygon[i].y)
				*(polygon[location_temp[1]].z - polygon[i].z) / (polygon[location_temp[1]].y - polygon[i].y);
			replace.push_back(replace_temp);
			replace_location.push_back(i);
		}
		else if (polygon[location_temp[0]].y<voxel_yp&&polygon[location_temp[1]].y > voxel_yp || (polygon[location_temp[0]].y<voxel_yp&&polygon[location_temp[1]].y == voxel_yp))//与前一个有交叉
		{
			coor replace_temp;
			replace_temp.x = polygon[i].x + (voxel_yp - polygon[i].y)
				*(polygon[location_temp[0]].x - polygon[i].x) / (polygon[location_temp[0]].y - polygon[i].y);
			replace_temp.y = voxel_yp;
			replace_temp.z = polygon[i].z + (voxel_yp - polygon[i].y)
				*(polygon[location_temp[0]].z - polygon[i].z) / (polygon[location_temp[0]].y - polygon[i].y);
			replace.push_back(replace_temp);
			replace_location.push_back(i);
		}
		else//都有交叉
		{
			coor temp[2];
			//polygon.insert(polygon.begin() + i, temp);

			temp[0].x = polygon[i].x + (voxel_yp - polygon[i].y)
				*(polygon[location_temp[0]].x - polygon[i].x) / (polygon[location_temp[0]].y - polygon[i].y);
			temp[0].y = voxel_yp;
			temp[0].z = polygon[i].z + (voxel_yp - polygon[i].y)
				*(polygon[location_temp[0]].z - polygon[i].z) / (polygon[location_temp[0]].y - polygon[i].y);

			temp[1].x = polygon[i].x + (voxel_yp - polygon[i].y)
				*(polygon[location_temp[1]].x - polygon[i].x) / (polygon[location_temp[1]].y - polygon[i].y);
			temp[1].y = voxel_yp;
			temp[1].z = polygon[i].z + (voxel_yp - polygon[i].y)
				*(polygon[location_temp[1]].z - polygon[i].z) / (polygon[location_temp[1]].y - polygon[i].y);
			polygon[i] = temp[0];
			polygon.insert(polygon.begin() + i + 1, temp[1]);



			break;//这种情况已经不用继续判断了
		}
	}
	if (replace.size() != 0)
	{
		for (int i = 0; i < replace.size(); i++)
		{
			polygon[replace_location[i]] = replace[i];
		}
		replace_location.clear();
		replace.clear();
	}
	if (point_to_cut.size() != 0)
	{
		for (int i = point_to_cut.size()-1; i >=0; i--)
		{
			
			polygon.erase(polygon.begin() + point_to_cut[i]);
			//cout << polygon.size();
		}
		point_to_cut.clear();
	}

	for (int i = 0; i < polygon.size(); i++)
	{
		//cout << endl << "55";
		if (polygon[i].y >= voxel_yn)//判断这个点要被切除还是保留  保留就continue  
			continue;

		int location_temp[2];//找这个点的相邻点  判断相邻点有没有交叉  没有交叉就continue
		if (i == 0)
		{
			location_temp[0] = polygon.size() - 1;
			location_temp[1] = 1;
			if (polygon[location_temp[0]].y < voxel_yn&&polygon[location_temp[1]].y < voxel_yn)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}
		else if (i == polygon.size() - 1)
		{
			location_temp[0] = polygon.size() - 2;
			location_temp[1] = 0;
			if (polygon[location_temp[0]].y < voxel_yp&&polygon[location_temp[1]].y < voxel_yn)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}
		else
		{
			location_temp[0] = i - 1;
			location_temp[1] = i + 1;
			if (polygon[location_temp[0]].y < voxel_yn&&polygon[location_temp[1]].y < voxel_yn)
			{
				point_to_cut.push_back(i);
				continue;
			}
		}//至少有一个交叉 计算交点  替换
		if ((polygon[location_temp[0]].y< voxel_yn&&polygon[location_temp[1]].y>voxel_yn) || (polygon[location_temp[1]].y>voxel_yn&&polygon[location_temp[0]].y == voxel_yn))//与后一个有交叉
		{
			coor replace_temp;
			replace_temp.x = polygon[i].x + (voxel_yn - polygon[i].y)
				*(polygon[location_temp[1]].x - polygon[i].x) / (polygon[location_temp[1]].y - polygon[i].y);
			replace_temp.y = voxel_yn;
			replace_temp.z = polygon[i].z + (voxel_yn - polygon[i].y)
				*(polygon[location_temp[1]].z - polygon[i].z) / (polygon[location_temp[1]].y - polygon[i].y);
			replace.push_back(replace_temp);
			replace_location.push_back(i);
		}
		else if ((polygon[location_temp[0]].y>voxel_yn&&polygon[location_temp[1]].y < voxel_yn) || (polygon[location_temp[0]].y>voxel_yn&&polygon[location_temp[1]].y == voxel_yn))//与前一个有交叉
		{
			coor replace_temp;
			replace_temp.x = polygon[i].x + (voxel_yn - polygon[i].y)
				*(polygon[location_temp[0]].x - polygon[i].x) / (polygon[location_temp[0]].y - polygon[i].y);
			replace_temp.y = voxel_yn;
			replace_temp.z = polygon[i].z + (voxel_yn - polygon[i].y)
				*(polygon[location_temp[0]].z - polygon[i].z) / (polygon[location_temp[0]].y - polygon[i].y);
			replace.push_back(replace_temp);
			replace_location.push_back(i);
		}

		else//都有交叉
		{
			coor temp[2];
			//polygon.insert(polygon.begin() + i, temp);

			temp[0].x = polygon[i].x + (voxel_yn - polygon[i].y)
				*(polygon[location_temp[0]].x - polygon[i].x) / (polygon[location_temp[0]].y - polygon[i].y);
			temp[0].y = voxel_yn;
			temp[0].z = polygon[i].z + (voxel_yn - polygon[i].y)
				*(polygon[location_temp[0]].z - polygon[i].z) / (polygon[location_temp[0]].y - polygon[i].y);

			temp[1].x = polygon[i].x + (voxel_yn - polygon[i].y)
				*(polygon[location_temp[1]].x - polygon[i].x) / (polygon[location_temp[1]].y - polygon[i].y);
			temp[1].y = voxel_yn;
			temp[1].z = polygon[i].z + (voxel_yn - polygon[i].y)
				*(polygon[location_temp[1]].z - polygon[i].z) / (polygon[location_temp[1]].y - polygon[i].y);
			polygon[i] = temp[0];
			polygon.insert(polygon.begin() + i + 1, temp[1]);



			break;//这种情况已经不用继续判断了
		}
	}

	if (replace.size() != 0)
	{
		for (int i = 0; i <replace.size(); i++)
		{
			polygon[replace_location[i]] = replace[i];
		}
		replace_location.clear();
		replace.clear();
	}
	if (point_to_cut.size() != 0)
	{
		for (int i = point_to_cut.size() - 1; i >= 0; i--)
		{
			polygon.erase(polygon.begin() + point_to_cut[i]);
		}
		point_to_cut.clear();
	}

	compare = 0;
	for (int i = 0; i < polygon.size(); i++)
	{
		if (polygon[i].z > voxel_zp)
			compare++;
	}
	if (compare == polygon.size())
		return false;
	compare = 0;
	for (int i = 0; i < polygon.size(); i++)
	{
		if (polygon[i].z < voxel_zn)
			compare++;
	}
	if (compare == polygon.size())
		return false;
	
	return true;
}

/*

int *code(int merton, int level)//输入一个莫顿序 输出其对应X Y Z编号
{
	int *code = new int[3]{0, 0, 0};
	for (int i = level - 1; i >= 0; i--)
	{
		code[0] += (merton >> (i * 3 + 2)) << i;
		merton -= (merton >> (i * 3 + 2)) << (i * 3 + 2);
		code[1] += (merton >> (i * 3 + 1)) << i;
		merton -= (merton >> (i * 3 + 1)) << (i * 3 + 1);
		code[2] += (merton >> (i * 3)) << i;
		merton -= (merton >> (i * 3)) << (i * 3);
	}
	return code;
}

*/
int merton(int x, int y, int z, int level)
{
	int merton = 0;
	for (int i = level - 1; i >= 0; i--)
	{
		merton += (x >> i) << (3 * i + 2);
		x = x - ((x >> i) << i);
		merton += (y >> i) << (3 * i + 1);
		y = y - ((y >> i) << i);
		merton += (z >> i) << (3 * i);
		z = z - ((z >> i) << i);
	}
	return merton;
}
/*voxelRead::voxelRead(int n_, string address, int levelLow_ = 5){
	n = n_;
	levelLow = levelLow_;
	octree_info *voxels = new octree_info[1 << (3 * levelLow)];
	fstream readFile;
	readFile.open(address, ios::in | ios::binary);//先读入到临时的数组中 再对类中的变量赋值
	vector<long long> temp;
	while (!readFile.eof()){
		long long readTemp;
		readFile >> readTemp;
		temp.push_back(readTemp);
	}
	int j = 0;
	for (int i = n * 32768; i < (n + 1) * 32768; i++){//赋莫顿序的值
		octree_info *pointer;
		pointer = voxels + j;
		pointer->morton = i;
		cout << pointer << ' ' << j << '|';
		j++;
	}
	for (int i = 0; i < temp.size(); i++){
		voxels[temp[i] - n*(1 << (3 * levelLow))].in_out = true;
	}
	for (int i = 0; i < 1 << (3 * levelLow); i++){
		cout << voxels[i].morton << ' ' << voxels[i].in_out << '|';
	}
}
voxelRead::~voxelRead(){
	delete[]voxels;
}*/