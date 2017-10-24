#include "generator.h"
//#include "cache.h"
//#include "triboxoverlap.h"
#include <iostream>
void generator::generatorOctree(int levelNow, vector<face> faceHere, float bb[6],int mortonNow, bool isNull = false) {
	if (levelNow > level) { return; }
	if (isNull == true) { return; }
	std::cout << "level now :" << levelNow << "faces =" << faceHere.size() << std::endl;
	//std::cout << 6;
	//生成此层 生成八个数组 调用八次递归？
	float *boundaryBox;
	//std::cout << 7;
	if (levelNow == 1) {
		boundaryBox = calbb(faceHere);
	}
	else {
		//std::cout << 8;
		boundaryBox = bb;
		//std::cout << 8;
	}//std::cout << 9;
	float *tempmid = new float[3]{ (boundaryBox[3] + boundaryBox[0]) / 2,(boundaryBox[4] + boundaryBox[1]) / 2,(boundaryBox[5] + boundaryBox[2]) / 2 };//x y z的"中点"
	//std::cout << std::endl << boundaryBox[0] << ' ' << boundaryBox[1] << ' ' << boundaryBox[2] << ' ' << boundaryBox[3] << ' ' << boundaryBox[4] << ' ' << boundaryBox[5] << std::endl;																																   //std::cout << 9;
	float subBoundaryBox[48] =//八个"小包围盒"的xmin ymin zmin xmax ymax zmax    八个的顺序为000 001 010 011 100 101 110 111
	{ boundaryBox[0],boundaryBox[1],boundaryBox[2],tempmid[0],tempmid[1],tempmid[2],
		boundaryBox[0],boundaryBox[1],tempmid[2],tempmid[0],tempmid[1],boundaryBox[5],
		boundaryBox[0],tempmid[1],boundaryBox[2],tempmid[0],boundaryBox[4],tempmid[2],
		boundaryBox[0],tempmid[1],tempmid[2],tempmid[0],boundaryBox[4],boundaryBox[5],
		tempmid[0],boundaryBox[1],boundaryBox[2],boundaryBox[3],tempmid[1],tempmid[2],
		tempmid[0],boundaryBox[1],tempmid[2],boundaryBox[3],tempmid[1],boundaryBox[5],
		tempmid[0],tempmid[1],boundaryBox[2],boundaryBox[3],boundaryBox[4],tempmid[2],
		tempmid[0],tempmid[1],tempmid[2],boundaryBox[3],boundaryBox[4],boundaryBox[5]
	};
	/*for (int i = 0;i < 8;i++) {
		for (int j = 0;j < 6;j++) {
			std::cout << subBoundaryBox[i * 6 + j];
		}
		std::cout << endl;
	}*/
	double boxCen[8][3];
	for (int i = 0;i < 8;i++) {
		boxCen[i][0] = (subBoundaryBox[i * 6 + 3] + subBoundaryBox[i * 6]) / 2;
		boxCen[i][1] = (subBoundaryBox[i * 6 + 4] + subBoundaryBox[i * 6 + 1]) / 2;
		boxCen[i][2] = (subBoundaryBox[i * 6 + 5] + subBoundaryBox[i * 6 + 2]) / 2;
		//std::cout << boxCen[i][0] << ' ' << boxCen[i][1] << ' ' << boxCen[i][2] << std::endl;
	}
	
	double boxHalfSize[8][3];
	for (int i = 0;i < 8;i++) {
		boxHalfSize[i][0] = (subBoundaryBox[i * 6 + 3] - subBoundaryBox[i * 6]) / 2;
		boxHalfSize[i][1] = (subBoundaryBox[i * 6 + 4] - subBoundaryBox[i * 6 + 1]) / 2;
		boxHalfSize[i][2] = (subBoundaryBox[i * 6 + 5] - subBoundaryBox[i * 6 + 2]) / 2;
		//std::cout << boxHalfSize[i][0] << ' ' << boxHalfSize[i][1] << ' ' << boxHalfSize[i][2] << std::endl;
	}
	delete[]tempmid;
	//std::cout << 1;
	vector<face> faces8[8];
	//std::cout << 2;
	bool *subIsNull = new bool[8]{ true,true,true,true,true,true,true,true };
	//std::cout << 3;
	//遍历所有三角面片 判断其与八个子节点的哪几个交叉，存到数组中，标记是否为空

	/*for (int i = 0;i < saveToDisk->voxel.size();i++) {
		std::cout << saveToDisk->voxel[i];
	}*/
	vector<face>::iterator it = faceHere.begin();
	for (;it != faceHere.end();it++) {
		double(*tri)[3] = new double[3][3];
		coor coori[3];
		for (int i = 0;i < 3;i++) {
			tri[i][0] = it->coors[i].x;
			tri[i][1] = it->coors[i].y;
			tri[i][0] = it->coors[i].z;
			coori[i].x = it->coors[i].x;
			coori[i].y = it->coors[i].y;
			coori[i].z = it->coors[i].z;
		}
		face faceHere;
		for (int i = 0;i < 3;i++) {
			faceHere.coors[i].x = coori[i].x;
			faceHere.coors[i].y = coori[i].y;
			faceHere.coors[i].z = coori[i].z;
		}
		for (int i = 0;i < 8;i++) {
			//std::cout << triBoxOverlap(boxCen[i], boxHalfSize[i], tri);
			if (overlap(coori[0], coori[1], coori[2], subBoundaryBox[i * 6], subBoundaryBox[i * 6 + 3], subBoundaryBox[i * 6 + 1],
				subBoundaryBox[i * 6 + 4], subBoundaryBox[i * 6 + 2], subBoundaryBox[i * 6 + 5])) {
				subIsNull[i] = false;
				faces8[i].push_back(faceHere);				
			}
			/*if (triBoxOverlap(boxCen[i], boxHalfSize[i], tri)) {
				subIsNull[i] = false;
				faces8[i].push_back(faceHere);
			}*/
		}
		//std::cout << endl;
		delete[]tri;
	}
	voxels<VoxelInfo> *saveToDisk = new voxels<VoxelInfo>(address, levelNow, 4096);
	for (int i = 0;i < 8;i++) {
		if (subIsNull[i] == false) {
			VoxelInfo saveThis;
			saveThis.inside_boun = 1;
			saveThis.in_out = 1;
			saveThis.morton =( mortonNow << 3) + i;
			//cout << "morton:" << saveThis.morton;
			*saveToDisk->getCode(saveThis.morton) = saveThis;
		}
	}
	
	delete saveToDisk;

	int subLevel = levelNow + 1;
	//std::cout << 4;
	for (int i = 0;i < 8;i++) {//八个子节点的生成
		float *subBB = new float[6]{ subBoundaryBox[6 * i] ,subBoundaryBox[6 * i + 1],subBoundaryBox[6 * i + 2],
			subBoundaryBox[6 * i + 3],subBoundaryBox[6 * i + 4],subBoundaryBox[6 * i + 5] };
		generatorOctree(subLevel, faces8[i], subBB, ((mortonNow << 3) + i),subIsNull[i]);
		delete[]subBB;
	}
	//std::cout << 5;
	//空的子节点调用时isNull=true
}


float *generator::calbb(vector<face> faceToBoun)
{
	float *bb = new float[6];//前三个是最小的XYZ后三个是最大的
	vector<face>::iterator it = faceToBoun.begin();
	bb[3] = bb[0] = it->coors[0].x;
	bb[4] = bb[1] = it->coors[0].y;
	bb[5] = bb[2] = it->coors[0].z;
	for (;it != faceToBoun.end();it++) {
		for (int i = 0;i < 3;i++) {
			if (bb[0] > it->coors[i].x)
				bb[0] = it->coors[i].x;
			else if (bb[3] < it->coors[i].x)
				bb[3] = it->coors[i].x;

			if (bb[1] > it->coors[i].y)
				bb[1] = it->coors[i].y;
			else if (bb[4] < it->coors[i].y)
				bb[4] = it->coors[i].y;

			if (bb[2] > it->coors[i].z)
				bb[2] = it->coors[i].z;
			else if (bb[5] < it->coors[i].z)
				bb[5] = it->coors[i].z;
		}
	}
	//std::cout << endl << "boundary box:" << bb[0] << ' ' << bb[1] << ' ' << bb[2] << ' ' << bb[3] << ' ' << bb[4] << ' ' << bb[5] << endl;
	return bb;
}