#pragma once
#define IGL_VIEWER_VIEWER_CPP
#include <igl/viewer/Viewer.h>
#include <suMesh.h>
#include <opencv2/opencv.hpp>
#include "suVolume.h"
#include <vector>
#include "suStructureEvolution.h"
/*\class suMeshViewer
 *\brief 
 *       1. mesh data conversion
 *       2. bounding box computation
 *       3. octree generation
**/
struct point_strain
{
	double x_strain, y_strain, z_strain;
};
struct point_mises_strain
{
	float mises_strain;
	int point_morton;
};
class suMeshViewer:public igl::viewer::Viewer 
{
public:
	suMeshViewer();
	~suMeshViewer() { clear(); }
public:
	//Callback for process bar
	typedef std::function<void(const std::string &, float)> ProgressCallback;
	//IO functions
	void openMesh(std::string filename,  const ProgressCallback &progress = ProgressCallback());
	void openSkeleton(std::string filename, const ProgressCallback &progress = ProgressCallback());

	//UI functions
	void build_UI();
	void set_select_mode(bool bSet);	
	void showProgress(const std::string &_caption, float value);
	void setTitle(std::string strWinTitle);

	//Data functions
	void add_octree();
	void add_bounding_box();
	
	bool export_stl_with_metaball(const char* fileName, std::vector<SU::OctNode> &stlVector);    //根据表面体素生成元球模型
	bool export_inp(std::string fileName);
	
	
	void clear();

public:
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXf N;
	Eigen::MatrixXd C;
	Eigen::MatrixXd bbox;
	Eigen::MatrixXd octreeLeafNodeBox;    //leaf node box for draw

	std::vector<std::set<Eigen::DenseIndex>> AVV;  //adjacent verts list
	std::vector<std::set<Eigen::DenseIndex>> AVF;  //adjacent faces list
	std::vector<point_mises_strain> sort_vector;
	struct CrossSectionOption { int nAxis; float fPos; int imgId; };

	//// UI
	//// FEA 
	//todo: add caliX	
	enum FEA_engine_enum {
		OOFEM = 0, 
		Ansys, 
		Abaqus
	};
	
	enum Layers {
		InputMesh = 0,
		InputMeshWireframe,
		Scheleton
	};
	

	std::string cur_opened_mesh_filename;   
	//UI control
	bool bShowWindow2D;

protected:
	//UI related 
	std::vector<std::pair<int, std::string>> mExampleImages;
	nanogui::ref<nanogui::ComboBox> mFEABox;
	nanogui::ref<nanogui::Window> mWindow2D;

	//For showing progress
	std::function<void(const std::string &, float)> mProgress;
	nanogui::ref<nanogui::Window> mProgressWindow;
	nanogui::ref<nanogui::ProgressBar> mProgressBar;
	nanogui::ref<nanogui::Label>  mProgressLabel;

	

public:	
	suMesh mesh_;	
	
	std::vector<float> all_point_mises_strain;
	std::vector<int> point_morton;
	
	//Option
	FEA_engine_enum curFEAengein;
	int  nDeep_;
	float fThresholdMC;
	CrossSectionOption cross_section_option;   //store cross section settings
	cv::Mat *m_pCross_section_img;
	SU::suVolume v;
	suStructrueOptimizer optimizer;
};

#ifndef voxelOutout
#define voxelOutput
class voxel_output
{
private:
	int x_code;
	int y_code;
	int z_code;
	double point_code[8];
	int level;

public:	
	voxel_output(int x_, int y_, int z_, int levell)
	{
		x_code = x_ + 1;
		y_code = y_ + 1;
		z_code = z_ + 1;
		level = levell;
	};
	
	void output_point1();//计算与输出点的编码
	
	void outAbaqusPoint();


	void output_point2(int number_);//计算与输出点的编码
	

};

double trans(char a1, char a2, char a3, char a4, char a5, char a6, char a7, char a8, char a9, char a10, char a11, char a12);
int asc2(char a);
#endif
