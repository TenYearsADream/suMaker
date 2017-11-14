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

	//Interaction functions
	void build_UI();
	void set_select_mode(bool bSet);	
	void showProgress(const std::string &_caption, float value);

	//Data functions
	void add_octree();
	void add_bounding_box();
	void envolution();
	void init(std::vector<SU::OctNode>&);
	void envolution(std::vector<SU::OctNode>&);
	void caCut(std::vector<SU::OctNode>&);
	void outOofemFile(std::vector <SU::OctNode>&);
	void outForcedOofemFile(std::vector <SU::OctNode>&);
	void outAbaqusFile(std::vector < SU::OctNode>&);
	void outAbaquscoor(float max_x, float max_y, float max_z, int level);
	void read_point_information(std::string address);
	void read_point_coor(std::string address, float box_maxx, float box_maxy, float box_maxz, int box_level);
	void ass_point();
	void coorForcedOutput(float max_x, float max_y, float max_z, int level);//三个包装盒的尺寸  分割次数
	void coor1(float max_x, float max_y, float max_z, int level);
	void assignment(std::vector<SU::OctNode>&);
	float return_max_strain(int morton_code, int level);
	void agentCut(std::vector<SU::OctNode>&);

	void outStlFile(std::vector<SU::OctNode> &stlVector);
	bool export_stl_with_metaball(const char* fileName, std::vector<SU::OctNode> &stlVector);    //根据表面体素生成元球模型
	
	
	void clear();

public:
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::MatrixXf N;
	Eigen::MatrixXd C;
	Eigen::MatrixXd bbox;
	Eigen::MatrixXd octreeLeafNodeBox;    //leaf node box for draw
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

	//UI control
	bool bShowWindow2D;

private:
	bool  bMesh_Open;
	bool  bSelect_Mode;
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
