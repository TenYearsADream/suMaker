#include "suMeshViewer.h"
#include "suGlobalState.h"
#include "morton.h"
#include <nanogui/formhelper.h>
#include <nanogui/screen.h>
#include <nanogui/imageview.h>
#include <nanogui/slider.h>
#include <nanogui/popupbutton.h>
#include <nanogui/VScrollPanel.h>
#include <nanogui/ImagePanel.h>
#include <nanogui/progressbar.h>
#include <nanogui/messagedialog.h>
#include <igl/unproject_onto_mesh.h>
#include <fstream>
#include <string>
#include "suWriteSTL.h"
#include <igl/viewer/ViewerData.h>
#include <resources.h>
#include <suSkeleton.h>

using namespace std;
suMeshViewer::suMeshViewer() : bMesh_Open(false), bSelect_Mode(false), nDeep_(0), m_pCross_section_img(0), fThresholdMC(1)
{
	mFEABox = nullptr;

	bShowWindow2D = false;
}
void suMeshViewer::openMesh(std::string filename, const ProgressCallback &progress)
{
	progress("Read mesh", 0);
	OpenMesh::IO::read_mesh(mesh_, filename);
	progress("Read mesh", 100);

	//convert obj from openmesh to eigen
	suMesh::ConstVertexIter  v_it(mesh_.vertices_begin()), v_end(mesh_.vertices_end());

	suMesh::Point p;
	Eigen::DenseIndex rows = mesh_.n_vertices();
	V.resize(rows, 3);
	rows = mesh_.n_faces();
	F.resize(rows, 3);

	progress("Add vertices", 101);
	//add vertices
	for (; v_it != v_end; ++v_it)
	{
		p = mesh_.point(v_it);
		V.row(v_it->idx()) << p[0], p[1], p[2];
	}
	//add face vertice 
	progress("Add face vertice", 150);
	suMesh::ConstFaceIter f_it(mesh_.faces_begin()), f_end(mesh_.faces_end());
	int idxFace = 0;
	for (; f_it != f_end; f_it++)
	{
		suMesh::FaceVertexIter fv_it = mesh_.fv_begin(f_it.handle());
		suMesh::FaceVertexIter fv_end = mesh_.fv_end(f_it.handle());

		int idxFv = 0;  //vertex index on a face
		for (; fv_it != fv_end; ++fv_it)
		{
			F(idxFace, idxFv++) = fv_it.handle().idx();
		}
		idxFace++;
	}
	progress("Get bounding box", 200);
	//get bounding box
	Eigen::Vector3d m = V.colwise().minCoeff();
	Eigen::Vector3d M = V.colwise().maxCoeff();

	// Corners of the bounding box
	/****************************
   (7) __________(6)
	  /	        /|
  (4)/_________/ |  (5)
	|	       | |
	|(3)       | |(2)
	|	       | /
	|__________|/
  (0)        (1)
	*****************************/


	bbox.resize(8, 3);
	bbox <<
		m(0), m(1), m(2),
		M(0), m(1), m(2),
		M(0), M(1), m(2),
		m(0), M(1), m(2),
		m(0), m(1), M(2),
		M(0), m(1), M(2),
		M(0), M(1), M(2),
		m(0), M(1), M(2);

	bMesh_Open = true;
	data.clear();
	data.set_mesh(V, F);

	add_bounding_box();

	//data.set_colors(C);	
	core.align_camera_center(V, F);

	progress("Data read finished", 250);
}

void suMeshViewer::openSkeleton(std::string filename, const ProgressCallback & progress)
{
	progress("Read skeleton", 0);
	SU::suSkeleton skel;
	if(!skel.load(filename, progress))   return;

	data.lines = Eigen::MatrixXd(0, 9);
	unsigned _N = skel.getEdgesCount();
	
	//get edges
	Eigen::MatrixXd p1, p2;
	p1.resize(1, 3);
	p2.resize(1, 3);
	for (unsigned int i = 0; i < _N; i++) {
		Eigen::Vector2i _e = skel.getEdge(i);
		Eigen::Vector3f sv = skel.getVerts(_e[0]);
		Eigen::Vector3f tv = skel.getVerts(_e[1]);
		p1 << sv[0], sv[1], sv[2];
		p2 << tv[0], tv[1], tv[2];
		data.add_edges
		(p1, p2,
		Eigen::RowVector3d(1, 0, 0)  //color
		);
		progress("Add edge...", 20 + (float)i / _N * 230);
		
	}
	progress("Read finished", 250);
}


void suMeshViewer::build_UI()
{
	//init
	cross_section_option.fPos = 0.5;
	cross_section_option.nAxis = 0;
	suGlobalState::gOnly().setVolume(&v);//！！！！！！关联v与gOnly中的数据
	suGlobalState::gOnly().set_cross_view_dimension(255, 255);

	//gui
	callback_init = [&](igl::viewer::Viewer& viewer){

		using namespace nanogui;

		ngui->setFixedSize(Eigen::Vector2i(60, 20));

		// -----------Create nanogui widgets	

		/*Tool Panel*/
		nanogui::Window *curWindow = ngui->addWindow(Eigen::Vector2i(10, 10), "Tool Panel");

		// ---------------------- Create popup load mesh panel ----------------------		
		PopupButton *openBtn = new nanogui::PopupButton(ngui->window(), "Open mesh");
		openBtn->setBackgroundColor(nanogui::Color(0, 255, 0, 25));
		openBtn->setIcon(ENTYPO_ICON_FOLDER);
		Popup *popup = openBtn->popup();
		VScrollPanel *vscroll = new nanogui::VScrollPanel(popup);
		ImagePanel *panel = new nanogui::ImagePanel(vscroll);

		auto ctx = this->screen->nvgContext();
		
		mExampleImages.insert(mExampleImages.begin(),
			std::make_pair(nvgImageIcon(ctx, loadmesh), ""));
		mExampleImages.insert(mExampleImages.end(),
			std::make_pair(nvgImageIcon(ctx, loadskeleton), ""));


		panel->setImages(mExampleImages);
		panel->setCallback([&, openBtn](int i) {
			openBtn->setPushed(false);

			//load mesh
			if (i == 0) { 
				std::string strOpenFile = nanogui::file_dialog(
				{ { "stl", "Mesh model" },{ "ply", "Mesh model" },{ "obj", "Mesh model" } }, false);
				if (!strOpenFile.empty())
				{
					showProgress("Load mesh", 3);

					//Open and plot the mesh. viewer.data.set_mesh(V, F);
					openMesh(strOpenFile, mProgress);
					//todo: add a process bar
					mProgressWindow->setVisible(false);
				}

			}
			
			//load skeleton
			if (i == 1) {
				std::string strOpenFile = nanogui::file_dialog(
				{ { "cgal", "Mesh skeleton" },{ "ske", "Mesh skeleton" }}, false);
				if (!strOpenFile.empty())
				{
					showProgress("Load skeleton", 3);

					//Open and plot the mesh. viewer.data.set_mesh(V, F);
					openSkeleton(strOpenFile, mProgress);
					//todo: add a process bar
					mProgressWindow->setVisible(false);
				}

			}
		});
		ngui->mLayout->appendRow(0);
		ngui->mLayout->setAnchor(openBtn, nanogui::AdvancedGridLayout::Anchor(1, ngui->mLayout->rowCount() - 1, 3, 1));

		//// ------------------Advanced setting--------------------
		ngui->addPopupButton("View Setting..", Color(150, 0, 0, 25), 0);

		/// Horizotal buttons
		Widget *pStatePanel = new Widget(ngui->mCurPopupWindow);
		pStatePanel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 1));

		//center button
		Button *pCenterBtn = new Button(pStatePanel, "Center", ENTYPO_ICON_HOME);
		pCenterBtn->setCallback([&] { this->core.align_camera_center(this->data.V, this->data.F); });

		//canonical view button
		Button *pSnapBtn = new Button(pStatePanel, "Canonical", ENTYPO_ICON_DRIVE);
		pSnapBtn->setCallback([&] {
			this->snap_to_canonical_quaternion();
		});

		ngui->addGroupOnPopup("Viewing Options");

		ngui->addVariableOnPopup("Zoom", core.camera_zoom);
		ngui->addVariableOnPopup("Orthographic view", core.orthographic);

		ngui->addGroupOnPopup("Draw options");

		ngui->addVariableOnPopup<bool>("Face-based", [&](bool checked)
		{
			this->data.set_face_based(checked);
		}, [&]()
		{
			return this->data.face_based;
		});

		ngui->addVariableOnPopup("Show texture", core.show_texture);

		ngui->addVariableOnPopup<bool>("Invert normals", [&](bool checked)
		{
			this->data.dirty |= igl::viewer::ViewerData::DIRTY_NORMAL;
			this->core.invert_normals = checked;
		}, [&]()
		{
			return this->core.invert_normals;
		});
		ngui->addVariableOnPopup("Show overlay", core.show_overlay);
		ngui->addVariableOnPopup("Show overlay depth", core.show_overlay_depth);
		ngui->addVariableOnPopup("Background", (nanogui::Color &) core.background_color);
		ngui->addVariableOnPopup("Line color", (nanogui::Color &) core.line_color);
		ngui->addVariableOnPopup("Shininess", core.shininess);

		ngui->addGroupOnPopup("Overlays");
		ngui->addVariableOnPopup("Wireframe", core.show_lines);
		ngui->addVariableOnPopup("Fill", core.show_faces);
		ngui->addVariableOnPopup("Show vertex labels", core.show_vertid);
		ngui->addVariableOnPopup("Show faces labels", core.show_faceid);

		// Create volumization Setting
		// ---------------------- Create popup volumization  panel ------------- ---------
		ngui->addPopupButton("Volumization..", nanogui::Color(255, 0, 0, 25), ENTYPO_ICON_LAYOUT);
		ngui->addGroupOnPopup("Parameters");
		ngui->addVariableOnPopup("Octree Deep", nDeep_);
		ngui->addButtonOnPopupWindow("Generate Octree", [&]()
		{
			//generate octree
			add_octree();
		});

		ngui->addVariableOnPopup("Show 2D window", bShowWindow2D);
		ngui->addGroupOnPopup("FEA Plgins");
		ngui->mCurPanel = new Widget(ngui->mCurPopupWindow);
		ngui->mCurPanel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

		//select FEA engine
		mFEABox = new ComboBox(ngui->mCurPanel,
		{ "OOFEM", "Ansys", "Abaqus", "caliX" });
		mFEABox->setIcon(ENTYPO_ICON_AREA_GRAPH);
		mFEABox->setCallback([&](int index) { std::cout << index << std::endl; });
		mFEABox->setId("OOFEM");
		

        ngui->addGroup("Force Setting");
		//select model button
		ngui->addButton("Select Mode", [&] {
			bSelect_Mode = !bSelect_Mode;
			set_select_mode(bSelect_Mode);
		});

		//Optimizaton
		ngui->addGroup("Structure Evolution");
		ngui->addVariable("MC threshold", fThresholdMC);
		ngui->addButton("Evolution", [&]()
		{
			//todo: check 3rd engine 
			std::cout << "Begin evolution..." << std::endl;
			envolution();
		});
		// ---------------------- Create popup export mesh panel ----------------------		
		PopupButton *exportBtn = new nanogui::PopupButton(ngui->window(), "Export mesh");
		exportBtn->setBackgroundColor(nanogui::Color(0, 255, 100, 25));
		exportBtn->setIcon(ENTYPO_ICON_FOLDER);
		Popup *export_popup = exportBtn->popup();
		VScrollPanel *export_vscroll = new nanogui::VScrollPanel(export_popup);
		ImagePanel *export_panel = new nanogui::ImagePanel(export_vscroll);


		/*mExampleImages.insert(mExampleImages.begin(),
			std::make_pair(nvgImageIcon(ctx, loadmesh), ""));*/
		export_panel->setImages(mExampleImages);
		export_panel->setCallback([&, exportBtn](int i) {
			exportBtn->setPushed(false);
			std::string strExportFile = nanogui::file_dialog(
			{ { "stl", "Mesh model" },{ "ply", "Mesh model" },{ "obj", "Mesh model" } }, true);
			if (!strExportFile.empty())
			{
				//Save voxilized model to mesh by metaball and MC;				
				//export_stl_with_metaball(strSaveFile.c_str(), v.leafBoundaryNodes_);
			}
		});
		ngui->mLayout->appendRow(0);
		ngui->mLayout->setAnchor(exportBtn, nanogui::AdvancedGridLayout::Anchor(1, ngui->mLayout->rowCount() - 1, 3, 1));

		//About
		//nanogui::Button *about = new nanogui::Button(ngui->window()->buttonPanel(), "", ENTYPO_ICON_INFO);

		/*about->setCallback([&, ctx]() {
			auto dlg = new nanogui::MessageDialog(
				screen->window(), nanogui::MessageDialog::Type::Information, "About suMaker",
				"suMaker is freely available under a BSD-style license. "
				"If you use the meshes obtained with this software, we kindly "
				"request that you acknowledge this and link to the project page at\n\n"
				"\thttp://cloudsfab.com/research/\n\n");
			dlg->messageLabel()->setFixedWidth(550);
			dlg->messageLabel()->setFontSize(20);
			curWindow->performLayout(ctx);
			dlg->center();
		});
*/



		//Cross section viewer
		{
			using namespace nanogui;
			mWindow2D = ngui->addWindow(Vector2i(710, 15), "Cross section Viewer");

			mWindow2D->setLayout(new GroupLayout());

			auto img = new ImageView(mWindow2D);
			img->setPolicy(ImageView::SizePolicy::Expand);
			img->setFixedSize(Vector2i(275, 275));

			//todo: recoding
			//load images
			//generate cross section
			m_pCross_section_img = new cv::Mat(255, 255, CV_8UC4);
			cv::cvtColor(*m_pCross_section_img, *m_pCross_section_img, CV_BGR2RGBA, 4);
			m_pCross_section_img->setTo(cv::Scalar(0, 255, 255, 255));
			cross_section_option.imgId = nvgCreateImageRGBA(screen->nvgContext(), m_pCross_section_img->cols,
				m_pCross_section_img->rows, 0, m_pCross_section_img->data);

			img->setImage(cross_section_option.imgId);

			//settings of cross section 
			new Label(mWindow2D, "Cross section settings", "sans-bold");

			ngui->addButton("Disp", [img, this]() {
				//genertate cross section for x,y,z				
				if (!v.isLoad_) return;

				m_pCross_section_img->setTo(cv::Scalar(255, 255, 255, 0));
				OpenMesh::Vec3f vSize = v.bbMax_ - v.bbMin_;

				switch (cross_section_option.nAxis)
				{
				case 0:m_pCross_section_img->data = suGlobalState::gOnly().gen_cross_section_X(cross_section_option.fPos); break;
				case 1:m_pCross_section_img->data = suGlobalState::gOnly().gen_cross_section_Y(cross_section_option.fPos); break;
				case 2:m_pCross_section_img->data = suGlobalState::gOnly().gen_cross_section_Z(cross_section_option.fPos); break;
				}

				if (m_pCross_section_img->data)
				{
					nvgUpdateImage(screen->nvgContext(), cross_section_option.imgId, m_pCross_section_img->data);
					img->setImage(cross_section_option.imgId);
				}

			});
			nanogui::Widget *pSettingPanel = new Widget(mWindow2D);
			pSettingPanel->setLayout(new BoxLayout(Orientation::Horizontal,
				Alignment::Maximum, 0, 6));
			TextBox *textBox = new TextBox(pSettingPanel);
			textBox->setFixedSize(Vector2i(60, 25));
			textBox->setValue("50");
			textBox->setUnits("%");

			Slider *slider = new Slider(pSettingPanel);
			slider->setValue(0.5f);
			slider->setFixedWidth(100);

			ComboBox *comboAxis = new ComboBox(pSettingPanel, { "x axis", "y axis", "z axis" });
			comboAxis->setCallback([&](int value) {
				cross_section_option.nAxis = value;
				std::cout << value << std::endl;
			});

			slider->setCallback([textBox, this](float value) {
				cross_section_option.fPos = value;
				textBox->setValue(std::to_string((int)(value * 100)));

			});
			slider->setFinalCallback([&](float value) {
				std::cout << "Final slider value: " << (int)(value * 100) << std::endl;
			});
			textBox->setFixedSize(Vector2i(60, 25));
			textBox->setFontSize(20);
			textBox->setAlignment(TextBox::Alignment::Left);

			mWindow2D->performLayout(ctx);
		}

		//Progress bar
		mProgressWindow = ngui->addWindow(screen->fixedSize() / 2, "Please wait...");
		mProgressLabel = new Label(mProgressWindow, "...");
		mProgressWindow->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Minimum, 15, 15));
		mProgressBar = new ProgressBar(mProgressWindow);
		mProgressBar->setFixedWidth(250);
		mProgressWindow->center();
		mProgressWindow->setVisible(false);


		// Generate menu
		viewer.screen->performLayout();

		// Bind function
		using namespace std::placeholders;
		mProgress = std::bind(&suMeshViewer::showProgress, this, _1, _2);

		return false;
	};

	callback_pre_draw = [&](igl::viewer::Viewer &viewer) {
		mWindow2D->setVisible(bShowWindow2D);
		return false;
	};
}

void suMeshViewer::add_octree()
{
	if (!bMesh_Open) return;

	v.LoadMeshFromMesh(mesh_);
	v.PartitionSpace(nDeep_);

	std::vector<SU::OctNode*> &nodeArr_ = v.leafInternalNodes_;
	std::cout << "internal node: " << v.leafInternalNodes_.size() << std::endl;
	//for test
	nodeArr_.insert(nodeArr_.end(), v.leafBoundaryNodes_.begin(), v.leafBoundaryNodes_.end());
	//add box
	Eigen::MatrixXd box;
	box.resize(8, 3);
	Eigen::MatrixXi edges;
	edges.resize(12, 2);

	if (nodeArr_.empty()) nodeArr_.push_back(v.pRoot_);
	std::vector<SU::OctNode*>::iterator it = nodeArr_.begin();

	int nEdges = nodeArr_.size() * 12;
	int nPoints = nodeArr_.size() * 8;

	Eigen::MatrixXd P_;
	Eigen::MatrixXi E_;
	P_.resize(nPoints, 3);
	E_.resize(nEdges, 2);

	int eIdx_ = 0;
	int vIdx_ = 0;
	while (it != nodeArr_.end())
	{
		SU::Point &m = (*it)->min_;
		SU::Point &M = (*it)->max_;

		//generate vertices for each voxel
		box <<
			m.x, m.y, m.z,
			M.x, m.y, m.z,
			M.x, M.y, m.z,
			m.x, M.y, m.z,
			m.x, m.y, M.z,
			M.x, m.y, M.z,
			M.x, M.y, M.z,
			m.x, M.y, M.z;

		for (int k = 0; k < 8; k++)
			P_.row(vIdx_ + k) = box.row(k);

		//generate edge for each voxel
		edges <<
			vIdx_ + 0, vIdx_ + 1,
			vIdx_ + 1, vIdx_ + 2,
			vIdx_ + 2, vIdx_ + 3,
			vIdx_ + 3, vIdx_ + 0,
			vIdx_ + 4, vIdx_ + 5,
			vIdx_ + 5, vIdx_ + 6,
			vIdx_ + 6, vIdx_ + 7,
			vIdx_ + 7, vIdx_ + 4,
			vIdx_ + 0, vIdx_ + 4,
			vIdx_ + 1, vIdx_ + 5,
			vIdx_ + 2, vIdx_ + 6,
			vIdx_ + 7, vIdx_ + 3;

		for (int l = 0; l < 12; l++)
			E_.row(eIdx_ + l) = edges.row(l);


		eIdx_ += 12;
		vIdx_ += 8;
		it++;
	}
	data.set_edges(P_, E_, Eigen::RowVector3d(1, 0, 0));


	std::cout << "Output: " << nodeArr_.size() << " nodes." << std::endl;


}

void suMeshViewer::envolution()
{
	std::vector<SU::OctNode*> &internalVoxels = v.leafInternalNodes_;
	std::vector<SU::OctNode*> &boundaryVoxels = v.leafBoundaryNodes_;
	std::vector<SU::OctNode*>::iterator itInternal = internalVoxels.begin();
	std::cout << internalVoxels.size() << std::endl;
	std::cout << boundaryVoxels.size() << std::endl;

	std::vector<SU::OctNode> nodesEnvoMorton(1 << (3 * nDeep_));

	//将Internal node中所有未完全划分的叶节点继续划分，扩展的volume.level_
	std::vector<SU::OctNode*> newLeafNodes;
	std::vector<SU::OctNode*>::iterator itNodePointer = internalVoxels.begin();

	for (; itNodePointer != internalVoxels.end(); itNodePointer++)
	{
		if ((*itNodePointer)->level_ < v.level_)
		{
			v.patitionToLevel(*itNodePointer, v.level_, (*itNodePointer)->label_, newLeafNodes);	 //add new node	pointer			
		}
		else {
			newLeafNodes.push_back(*itNodePointer);  //copy original nodes pointer
		}
	}
	//Now we get a new leaf nodes array
	internalVoxels = newLeafNodes;

	//计算莫顿序

	for (itInternal = internalVoxels.begin(); itInternal != internalVoxels.end(); itInternal++)
	{
		(**itInternal).xLocCode_ = (((**itInternal).max_.x + (**itInternal).min_.x) / 2 - v.bbMin_.data()[0]) / ((v.bbMax_.data()[0] - v.bbMin_.data()[0]) / (1 << nDeep_));
		(**itInternal).yLocCode_ = (((**itInternal).max_.y + (**itInternal).min_.y) / 2 - v.bbMin_.data()[1]) / ((v.bbMax_.data()[1] - v.bbMin_.data()[1]) / (1 << nDeep_));
		(**itInternal).zLocCode_ = (((**itInternal).max_.z + (**itInternal).min_.z) / 2 - v.bbMin_.data()[2]) / ((v.bbMax_.data()[2] - v.bbMin_.data()[2]) / (1 << nDeep_));

		if ((*itInternal)->label_ == SU::BOUNDARY_CELL || (*itInternal)->label_ == SU::BOUNDARY_CELL_SPECIAL) {
			(*itInternal)->location = 1;
		}
		(*itInternal)->out = true;
		(*itInternal)->morton = morton((*itInternal)->xLocCode_, (*itInternal)->yLocCode_, (*itInternal)->zLocCode_, nDeep_);
		nodesEnvoMorton[(*itInternal)->morton] = **itInternal;

		//debug
		//std::cout << nodesEnvoMorton[(*itInternal)->morton].center().x << ", " << (**itInternal).center().x<<std::endl;

	}//赋值 内部节点 
	/*std::vector<SU::OctNode>::iterator itNodes = nodesEnvoMorton.begin();
	int temp = 0;
	for (;itNodes != nodesEnvoMorton.end();itNodes++) {
		std::cout << itNodes->xLocCode_ << ' ' << itNodes->yLocCode_ << ' ' << itNodes->zLocCode_ << ' ' << temp << ' ' << itNodes->label_ << ' '
			<< itNodes->location << std::endl;
		temp++;
	}*/

	//修复内部
	//std::vector<SU::OctNode>::iterator repairIt;
	//int isEnd = 0;
	//for (;;) {
	//	repairIt = nodesEnvoMorton.begin();
	//	for (;repairIt != nodesEnvoMorton.end();repairIt++) {
	//		if (repairIt->label_ == 3 && repairIt->out == true) {
	//			int * neighs = six_n_morton(repairIt->morton, nDeep_);
	//			//std::cout << initIt->morton << ' ' << nDeep_;
	//			for (int i = 0;i < 26;i++) {
	//				if (nodesEnvoMorton[neighs[i]].out == false) {
	//					int *code_ = code(neighs[i], nDeep_);
	//					nodesEnvoMorton[neighs[i]].out = true;
	//					nodesEnvoMorton[neighs[i]].xLocCode_ = code_[0];
	//					nodesEnvoMorton[neighs[i]].yLocCode_ = code_[1];
	//					nodesEnvoMorton[neighs[i]].zLocCode_ = code_[2];						
	//					nodesEnvoMorton[neighs[i]].label_ = SU::INTERIOR_CELL;
	//					nodesEnvoMorton[neighs[i]].morton = neighs[i];
	//					isEnd++;
	//				}
	//			}
	//			
	//			delete[]neighs;
	//	
	//		}
	//	}
	//	
	//	if (!isEnd)
	//		break;
	//	isEnd = 0;
	//}

	//std::cout << nodesEnvoMorton.size();
	//距离场
	init(nodesEnvoMorton);
	/*for (int i = 0; i < nodesEnvoMorton.size(); i++)
	{
		if (nodesEnvoMorton[i].label_ == SU::INTERIOR_CELL)
		std::cout << nodesEnvoMorton[i].center().x << std::endl;
	}*/

	//迭代
	//...
	//CA 优化
	for (int k = 0; k < 3; k++)
	{
		caCut(nodesEnvoMorton);
	}
	/*caCut(nodesEnvoMorton);
	caCut(nodesEnvoMorton);*/

	///输出OOFEM加载文件（没交互）

	//outForcedOofemFile(nodesEnvoMorton);
	//system("cd /d r:\\ &oofem.exe &oofem -f oofemOutFile.txt");

	/////读OOFEM输出
	//float x = v.bbMax_.data()[0] - v.bbMin_.data()[0];
	//float y = v.bbMax_.data()[1] - v.bbMin_.data()[1];
	//float z = v.bbMax_.data()[2] - v.bbMin_.data()[2];
	//read_point_information("R:\\Majnun.out.m0.1.vtu");  //应变
	//read_point_coor("r:\\Majnun.out.m0.1.vtu", x,y,z, nDeep_);//坐标
	//assignment(nodesEnvoMorton);//关联应变与目标



	//agentCut(nodesEnvoMorton);


	/*outOofemFile(nodesEnvoMorton);
	system("cd /d D:\\oofem\\build2.3\\Debug &oofem.exe&oofem -f oofemOutFile.txt");*/

	suGlobalState::gOnly().setEnvoMorton(nodesEnvoMorton);


	//outStlFile(nodesEnvoMorton);
	export_stl_with_metaball("r:/test.stl", nodesEnvoMorton);



}

void suMeshViewer::init(std::vector<SU::OctNode> &initVector) {
	std::vector<SU::OctNode>::iterator initIt;
	int isEnd = 0;
	for (;;) {
		initIt = initVector.begin();
		for (; initIt != initVector.end(); initIt++) {
			if (initIt->label_ == 3 && initIt->location == 0) {
				int * neighs = six_n_morton(initIt->morton, nDeep_);
				//std::cout << initIt->morton << ' ' << nDeep_;
				for (int i = 0; i < 6; i++) {
					initIt->locationNext = initVector[neighs[i]].location > initIt->locationNext ? initVector[neighs[i]].location : initIt->locationNext;
				}
				initIt->locationNext++;
				delete[]neighs;
				isEnd++;
			}
		}
		for (initIt = initVector.begin(); initIt != initVector.end(); initIt++) {
			if (initIt->locationNext > 1) {
				initIt->location = initIt->locationNext;
				initIt->locationNext = 0;
			}
			else
				initIt->locationNext = 0;
		}
		if (!isEnd)
			break;
		isEnd = 0;
	}
}

void suMeshViewer::outOofemFile(std::vector<SU::OctNode>& outVector)
{
	std::vector<SU::OctNode>::iterator outIt = outVector.begin();
	int voxelNumber = 0;
	for (; outIt != outVector.end(); outIt++) {
		if ((outIt->label_ == 1 || outIt->label_ == 2 || outIt->label_ == 3) && outIt->out == true)
			voxelNumber++;
	}

	std::fstream outfile;
	std::string fileAddress = "r:\\oofemOutFile.txt";
	outfile.open(fileAddress, std::ios::out);
	outfile << "Majnun.out" << std::endl;
	outfile << "test of Brick elements with nlgeo 1(strain is the Green-Lagrangian strain) rotated as a rigid body" << std::endl;
	outfile << "#NonLinearStatic  nmsteps 1 nsteps 1 " << std::endl;
	outfile << "#LinearStatic  nmsteps 1 nsteps 1 " << std::endl;
	outfile << "#nsteps 5 rtolv 1.e-6 stiffMode 1 controlmode 1 maxiter 100" << std::endl;
	outfile << "#vtkxml tstep_all domain_all primvars 1 1 vars 2 4 1 stype 1" << std::endl;
	outfile << "#domain 3d" << std::endl;
	outfile << "#OutputManager tstep_all dofman_all element_all" << std::endl;
	outfile << "LinearStatic nsteps 3 nmodules 1" << std::endl;
	outfile << "vtkxml tstep_all domain_all primvars 1 1 vars 2 4 1 stype 1" << std::endl;
	outfile << "domain 3d" << std::endl;
	outfile << "OutputManager tstep_all dofman_all element_all" << std::endl;
	outfile << "ndofman " << pow((pow(2, nDeep_) + 1), 3) << " nelem " << voxelNumber << " ncrosssect 1 nmat 1 nbc 2 nic 0 nltf 1 " << std::endl;
	outfile.close();
	float x = v.bbMax_.data()[0] - v.bbMin_.data()[0];
	float y = v.bbMax_.data()[1] - v.bbMin_.data()[1];
	float z = v.bbMax_.data()[2] - v.bbMin_.data()[2];
	coor1(x, y, z, nDeep_);


	outIt = outVector.begin();
	int number = 0;//一个六面体体素生成四个四个四面体体素
	for (; outIt != outVector.end(); outIt++)
	{
		if ((outIt->label_ == 1 || outIt->label_ == 2 || outIt->label_ == 3) && outIt->out == true)
		{
			std::fstream outfile;
			outfile.open(fileAddress, std::ios::app);
			int *asddd = code(outIt->morton, nDeep_);
			//outfile << pNode->xLocCode_ << "  " << pNode->yLocCode_ << "  " << pNode->zLocCode_ << "  " << pNode->level_ << "  " << pNode->label_ << "  ";
			//outfile << IndexX << "  " << IndexY << "  " << IndexZ;
			outfile << "LSpace " << ++number << "	 nodes  8 ";
			voxel_output* asd = new voxel_output(asddd[0], asddd[1], asddd[2], nDeep_);
			outfile.close();
			//voxel_output asd(pChildNode, level);
			asd->output_point1();
			delete asd;
		}
	}


	outfile.open("r:\\oofemOutFile.txt", std::ios::app);
	outfile << "SimpleCS 1" << std::endl << "IsoLE 1 d 0. E 15.0 n 0.25 talpha 1.0" << std::endl << "BoundaryCondition  1 loadTimeFunction 1 prescribedvalue 0.0"
		<< std::endl << "BoundaryCondition  2 loadTimeFunction 1 prescribedvalue 0.5" << std::endl << "PiecewiseLinFunction 1 npoints 2 t 2 0. 1000. f(t) 2 0. 1000." << std::endl;
	outfile.close();

}

void suMeshViewer::outForcedOofemFile(std::vector<SU::OctNode>& outVector) {
	std::vector<SU::OctNode>::iterator outIt = outVector.begin();
	int voxelNumber = 0;
	for (; outIt != outVector.end(); outIt++) {
		if ((outIt->label_ == 1 || outIt->label_ == 2 || outIt->label_ == 3) && outIt->out == true)
			voxelNumber++;
	}
	std::fstream outfile;
	std::string fileAddress = "r:\\oofemOutFile.txt";
	outfile.open(fileAddress, std::ios::out);
	outfile << "Majnun.out" << std::endl
		<< "Simple bending of a cantilever beam, quadratic elements." << std::endl
		<< "LinearStatic nsteps 2 controllmode 1 rtolv 1.e-3 nmodules 1" << std::endl
		<< "vtkxml tstep_all domain_all primvars 1 1 vars 6 1 2 4 5 27 28 stype 1" << std::endl
		<< "domain 3d" << std::endl
		<< "OutputManager tstep_all dofman_all element_all" << std::endl
		<< "ndofman " << pow((pow(2, nDeep_) + 1), 3) << " nelem " << voxelNumber * 5 << " ncrosssect 1 nmat 1 nbc 2 nic 0 nltf 1" << std::endl;
	outfile.close();
	float x = v.bbMax_.data()[0] - v.bbMin_.data()[0];
	float y = v.bbMax_.data()[1] - v.bbMin_.data()[1];
	float z = v.bbMax_.data()[2] - v.bbMin_.data()[2];
	coorForcedOutput(x, y, z, nDeep_);


	outIt = outVector.begin();
	int number = 0;//一个六面体体素生成四个四个四面体体素
	for (; outIt != outVector.end(); outIt++)
	{
		if ((outIt->label_ == 1 || outIt->label_ == 2 || outIt->label_ == 3) && outIt->out == true)
		{

			int *asddd = code(outIt->morton, nDeep_);
			//outfile << pNode->xLocCode_ << "  " << pNode->yLocCode_ << "  " << pNode->zLocCode_ << "  " << pNode->level_ << "  " << pNode->label_ << "  ";
			//outfile << IndexX << "  " << IndexY << "  " << IndexZ;

			voxel_output* asd = new voxel_output(asddd[0], asddd[1], asddd[2], nDeep_);

			//voxel_output asd(pChildNode, level);
			asd->output_point2(number++);
			delete asd;
		}
	}

	outfile.open("r:\\oofemOutFile.txt", std::ios::app);
	outfile << "SimpleCS 1 thick 1.0 width 1.0" << std::endl << "IsoLE 1 d 0.00125 E 2400.0 n 0.429  tAlpha 0.0000780"
		<< std::endl << "BoundaryCondition 1 loadTimeFunction 1 prescribedvalue 0.0"
		<< std::endl << "ConstantSurfaceLoad 2 ndofs 3 loadType 2 Components 3 0.0 -3000 0.0 loadTimeFunction 1"
		<< std::endl << "ConstantFunction 1 f(t) 1.0" << std::endl;
	outfile.close();
}

void suMeshViewer::outAbaqusFile(std::vector<SU::OctNode>& outVector)
{
	std::vector<SU::OctNode>::iterator outIt = outVector.begin();
	int voxelNumber = 0;
	for (; outIt != outVector.end(); outIt++) {
		if ((outIt->label_ == 1 || outIt->label_ == 2 || outIt->label_ == 3) && outIt->out == true)
			voxelNumber++;
	}

	std::fstream outfile;
	std::string fileAddress = "r:\\oofemOutFile.txt";
	outfile.open(fileAddress, std::ios::out);
	outfile.close();

	float x = v.bbMax_.data()[0] - v.bbMin_.data()[0];
	float y = v.bbMax_.data()[1] - v.bbMin_.data()[1];
	float z = v.bbMax_.data()[2] - v.bbMin_.data()[2];
	outAbaquscoor(x, y, z, nDeep_);


	outIt = outVector.begin();
	int number = 0;//一个六面体体素生成四个四个四面体体素
	for (; outIt != outVector.end(); outIt++)
	{
		if ((outIt->label_ == 1 || outIt->label_ == 2 || outIt->label_ == 3) && outIt->out == true)
		{
			std::fstream outfile;
			outfile.open(fileAddress, std::ios::app);
			int *asddd = code(outIt->morton, nDeep_);
			//outfile << pNode->xLocCode_ << "  " << pNode->yLocCode_ << "  " << pNode->zLocCode_ << "  " << pNode->level_ << "  " << pNode->label_ << "  ";
			//outfile << IndexX << "  " << IndexY << "  " << IndexZ;
			outfile << ++number << ',';
			voxel_output* asd = new voxel_output(asddd[0], asddd[1], asddd[2], nDeep_);
			outfile.close();
			//voxel_output asd(pChildNode, level);
			asd->outAbaqusPoint();
			delete asd;
		}
	}


	outfile.open("r:\\oofemOutFile.txt", std::ios::app);
	outfile << "SimpleCS 1" << std::endl << "IsoLE 1 d 0. E 15.0 n 0.25 talpha 1.0" << std::endl << "BoundaryCondition  1 loadTimeFunction 1 prescribedvalue 0.0"
		<< std::endl << "BoundaryCondition  2 loadTimeFunction 1 prescribedvalue 0.5" << std::endl << "PiecewiseLinFunction 1 npoints 2 t 2 0. 1000. f(t) 2 0. 1000." << std::endl;
	outfile.close();
}

void suMeshViewer::envolution(std::vector<SU::OctNode>& envoVector)
{

}

void suMeshViewer::add_bounding_box()
{
	// Edges of the bounding box
	Eigen::MatrixXi E_box(12, 2);
	E_box <<
		0, 1,
		1, 2,
		2, 3,
		3, 0,
		4, 5,
		5, 6,
		6, 7,
		7, 4,
		0, 4,
		1, 5,
		2, 6,
		7, 3;

	// Plot the corners of the bounding box as points
	// data.add_points(bbox, Eigen::RowVector3d(1, 0, 0));
	// Plot the edges of the bounding box
	for (unsigned i = 0; i < E_box.rows(); ++i)
		data.add_edges
		(
			bbox.row(E_box(i, 0)),
			bbox.row(E_box(i, 1)),
			Eigen::RowVector3d(1, 0, 0)  //color
			);
}

void suMeshViewer::clear()
{
	suGlobalState::gOnly().release();
	if (!m_pCross_section_img) delete m_pCross_section_img;
}

void suMeshViewer::set_select_mode(bool bSet)
{
	if (bSet && bMesh_Open)
	{
		//set select model
		C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
		callback_mouse_move =
			[&](igl::viewer::Viewer& viewer, int, int)->bool
		{
			int fid = -1;
			Eigen::Vector3f bc;
			// Cast a ray in the view direction starting from the mouse position
			double x = viewer.current_mouse_x;
			double y = viewer.core.viewport(3) - viewer.current_mouse_y;

			if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), core.view * core.model,
				core.proj, core.viewport, V, F, fid, bc))
			{
				// paint hit red
				C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
				C.row(fid) << 1, 0, 0;
				viewer.data.set_colors(C);
				return true;
			}
			else {
				C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
				viewer.data.set_colors(C);
			}
			return false;
		};
	}
	else {
		
		data.uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
			Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
			Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));
		callback_mouse_move = [&](igl::viewer::Viewer& viewer, int, int)->bool
		{
			return false;
		};
	}
}

void suMeshViewer::showProgress(const std::string & _caption, float value)
{
	std::string caption = _caption + " ..";
	float newValue = mProgressBar->value();
	if (mProgressLabel->caption() != caption) {
		newValue = 0;
		mProgressLabel->setCaption(caption);
	}

	if (value >= 0)
		newValue = value; /* Positive: absolute progress values */
	else
		newValue -= value; /* Negative: relative progress values (OpenMP) */

	mProgressBar->setValue(newValue/250);

	mProgressWindow->setVisible(true);
	screen->drawAll();
	mProgressWindow->requestFocus();
}

void suMeshViewer::coorForcedOutput(float max_x, float max_y, float max_z, int level)
{
	double lines = pow(2, level);//计算每一行有多少体素   dxdydz分别是三个方向上每个体素的尺寸
	double dx = max_x / lines;
	double dy = max_y / lines;
	double dz = max_z / lines;
	float region_x[2], region_y[2], region_z[2];
	//cout << "input the region of the force";

	region_x[0] = 0.095;
	region_x[1] = 0.105;
	region_y[0] = 0.09;
	region_y[1] = 0.11;
	region_z[0] = 0.07;
	region_z[1] = 0.08;
	std::fstream outfile;
	outfile.open("r:\\oofemOutFile.txt", std::ios::app);
	int number = 1;
	lines += 1;
	for (int i = 0; i < lines; i++)//输出每个节点的坐标
	{
		for (int j = 0; j < lines; j++)
		{
			for (int k = 0; k < lines; k++)
			{
				if ((j*dx >= region_x[0]) && (j*dx <= region_x[1]) && (k*dy >= region_y[0]) && (k*dy <= region_y[1]) && (i*dz >= region_z[0]) && (i*dz <= region_z[1]))
				{
					suGlobalState::gOnly().forcedPoint.push_back(number);
				}
				if (k == 0)
				{
					outfile << "node   " << number++ << "   coords 3     " << j *dx << "  " << k *dy << "  " << i *dz << "       bc 3 1 1 1" << std::endl;
				}
				else
					outfile << "node   " << number++ << "   coords 3     " << j *dx << "  " << k *dy << "  " << i *dz << std::endl;
			}
		}
	}
	outfile.close();
}

void voxel_output::output_point2(int number_)//计算与输出点的编码
{
	double q = pow(2, level) + 1;
	point_code[0] = (x_code - 1)*q + y_code + z_code *q*q;
	point_code[1] = (x_code - 1)*q + y_code + 1 + z_code *q*q;
	point_code[3] = x_code *q + y_code + 1 + z_code *q*q;
	point_code[2] = x_code *q + y_code + z_code *q*q;

	point_code[4] = (x_code - 1)*q + y_code + (z_code - 1)*q*q;
	point_code[5] = (x_code - 1)*q + y_code + 1 + (z_code - 1)*q*q;
	point_code[7] = x_code *q + y_code + 1 + (z_code - 1)*q*q;
	point_code[6] = x_code *q + y_code + (z_code - 1)*q*q;

	std::fstream outfile;
	outfile.open("r:\\oofemOutFile.txt", std::ios::app);
	/*for (int i = 0; i <= 7; i++)
	{
	outfile << point_code[i] << "  ";
	}
	//outfile <<"    "<< x_code << "  " << y_code << "  " << z_code;
	outfile << "    	mat 1 crossSect 1	nlgeo 1" << endl;*/

	std::vector<int>::iterator forcedPointIt;
	outfile << "LTRSpace  " << 5 * number_ + 1 << "  " << "nodes  4	  " << point_code[0] << "  " << point_code[3] << "  "
		<< point_code[2] << "  " << point_code[6] << "  mat 1 crossSect 1	NIP  1";
	for (forcedPointIt = suGlobalState::gOnly().forcedPoint.begin(); forcedPointIt != suGlobalState::gOnly().forcedPoint.end(); forcedPointIt++)
	{
		if ((point_code[0] == *forcedPointIt) || (point_code[3] == *forcedPointIt) || (point_code[2] == *forcedPointIt) || (point_code[6] == *forcedPointIt))
		{
			outfile << " BoundaryLoads 2 2 3";
			break;
		}
	}
	outfile << std::endl;
	outfile << "LTRSpace  " << 5 * number_ + 2 << "  " << "nodes  4	  " << point_code[0] << "  " << point_code[3] << "  "
		<< point_code[6] << "  " << point_code[5] << "  mat 1 crossSect 1	NIP  1";
	for (forcedPointIt = suGlobalState::gOnly().forcedPoint.begin(); forcedPointIt != suGlobalState::gOnly().forcedPoint.end(); forcedPointIt++)
	{
		if ((point_code[0] == *forcedPointIt) || (point_code[3] == *forcedPointIt) || (point_code[5] == *forcedPointIt) || (point_code[6] == *forcedPointIt))
		{
			outfile << " BoundaryLoads 2 2 3";
			break;
		}
	}
	outfile << std::endl;
	outfile << "LTRSpace  " << 5 * number_ + 3 << "  " << "nodes  4	  " << point_code[3] << "  " << point_code[6] << "  "
		<< point_code[5] << "  " << point_code[7] << "  mat 1 crossSect 1	NIP  1";
	for (forcedPointIt = suGlobalState::gOnly().forcedPoint.begin(); forcedPointIt != suGlobalState::gOnly().forcedPoint.end(); forcedPointIt++)
	{
		if ((point_code[3] == *forcedPointIt) || (point_code[6] == *forcedPointIt) || (point_code[5] == *forcedPointIt) || (point_code[7] == *forcedPointIt))
		{
			outfile << " BoundaryLoads 2 2 3";
			break;
		}
	}
	outfile << std::endl;
	outfile << "LTRSpace  " << 5 * number_ + 4 << "  " << "nodes  4	  " << point_code[0] << "  " << point_code[1] << "  "
		<< point_code[3] << "  " << point_code[5] << "  mat 1 crossSect 1	NIP  1";
	for (forcedPointIt = suGlobalState::gOnly().forcedPoint.begin(); forcedPointIt != suGlobalState::gOnly().forcedPoint.end(); forcedPointIt++)
	{
		if ((point_code[0] == *forcedPointIt) || (point_code[1] == *forcedPointIt) || (point_code[3] == *forcedPointIt) || (point_code[5] == *forcedPointIt))
		{
			outfile << " BoundaryLoads 2 2 3";
			break;
		}
	}
	outfile << std::endl;
	outfile << "LTRSpace  " << 5 * number_ + 5 << "  " << "nodes  4	  " << point_code[4] << "  " << point_code[6] << "  "
		<< point_code[5] << "  " << point_code[0] << "  mat 1 crossSect 1	NIP  1";
	for (forcedPointIt = suGlobalState::gOnly().forcedPoint.begin(); forcedPointIt != suGlobalState::gOnly().forcedPoint.end(); forcedPointIt++)
	{
		if ((point_code[4] == *forcedPointIt) || (point_code[5] == *forcedPointIt) || (point_code[6] == *forcedPointIt) || (point_code[0] == *forcedPointIt))
		{
			outfile << " BoundaryLoads 2 2 3";
			break;
		}
	}
	outfile << std::endl;
	outfile.close();
}

void suMeshViewer::coor1(float max_x, float max_y, float max_z, int level)
{
	double lines = pow(2, level);//计算每一行有多少体素   dxdydz分别是三个方向上每个体素的尺寸
	double dx = max_x / lines;
	double dy = max_y / lines;
	double dz = max_z / lines;

	std::fstream outfile;
	outfile.open("r:\\oofemOutFile.txt", std::ios::app);
	int number = 1;
	lines += 1;
	for (int i = 0; i < lines; i++)//输出每个节点的坐标
	{
		for (int j = 0; j < lines; j++)
		{
			for (int k = 0; k < lines; k++)
			{

				if (k == 0)
				{
					outfile << "node   " << number++ << "   coords 3     " << j *dx << "  " << k *dy << "  " << i *dz << "       bc 3 2 2 2" << std::endl;
				}
				else
					outfile << "node   " << number++ << "   coords 3     " << j *dx << "  " << k *dy << "  " << i *dz << std::endl;
			}

		}

	}
	outfile.close();
}

void suMeshViewer::outAbaquscoor(float max_x, float max_y, float max_z, int level)
{
	double lines = pow(2, level);//计算每一行有多少体素   dxdydz分别是三个方向上每个体素的尺寸
	double dx = max_x / lines;
	double dy = max_y / lines;
	double dz = max_z / lines;

	std::fstream outfile;
	outfile.open("r:\\oofemOutFile.txt", std::ios::app);
	int number = 1;
	lines += 1;
	for (int i = 0; i < lines; i++)//输出每个节点的坐标
	{
		for (int j = 0; j < lines; j++)
		{
			for (int k = 0; k < lines; k++)
			{
				outfile << number++ << ',' << j *dx << ',' << k *dy << ',' << i *dz << std::endl;
			}
		}
	}
	outfile.close();
}

void voxel_output::output_point1()//计算与输出点的编码
{

	double q = pow(2, level) + 1;
	point_code[0] = (x_code - 1)*q + y_code + z_code *q*q;
	point_code[1] = (x_code - 1)*q + y_code + 1 + z_code *q*q;
	point_code[2] = x_code *q + y_code + 1 + z_code *q*q;
	point_code[3] = x_code *q + y_code + z_code *q*q;

	point_code[4] = (x_code - 1)*q + y_code + (z_code - 1)*q*q;
	point_code[5] = (x_code - 1)*q + y_code + 1 + (z_code - 1)*q*q;
	point_code[6] = x_code *q + y_code + 1 + (z_code - 1)*q*q;
	point_code[7] = x_code *q + y_code + (z_code - 1)*q*q;

	std::fstream outfile;
	outfile.open("r:\\oofemOutFile.txt", std::ios::app);
	for (int i = 0; i <= 7; i++)
	{
		outfile << point_code[i] << "  ";
	}
	//outfile <<"    "<< x_code << "  " << y_code << "  " << z_code;
	outfile << "    	mat 1 crossSect 1	nlgeo 1" << std::endl;

	outfile.close();
}

void voxel_output::outAbaqusPoint()
{

	double q = pow(2, level) + 1;
	point_code[0] = (x_code - 1)*q + y_code + z_code *q*q;
	point_code[1] = (x_code - 1)*q + y_code + 1 + z_code *q*q;
	point_code[2] = x_code *q + y_code + 1 + z_code *q*q;
	point_code[3] = x_code *q + y_code + z_code *q*q;

	point_code[4] = (x_code - 1)*q + y_code + (z_code - 1)*q*q;
	point_code[5] = (x_code - 1)*q + y_code + 1 + (z_code - 1)*q*q;
	point_code[6] = x_code *q + y_code + 1 + (z_code - 1)*q*q;
	point_code[7] = x_code *q + y_code + (z_code - 1)*q*q;

	std::fstream outfile;
	outfile.open("r:\\oofemAbaqusFile.txt", std::ios::app);
	for (int i = 0; i < 7; i++)
	{
		outfile << point_code[i] << ',';
	}
	//outfile <<"    "<< x_code << "  " << y_code << "  " << z_code;
	outfile << point_code[7] << std::endl;

	outfile.close();
}

void suMeshViewer::caCut(std::vector<SU::OctNode>& caCutVector)
{
	int count1 = 0, count2 = 0;
	std::vector<SU::OctNode>::iterator caCutIt = caCutVector.begin();

	for (; caCutIt != caCutVector.end(); caCutIt++) {

		if (caCutIt->label_ == SU::INTERIOR_CELL && caCutIt->out == true) {
			int *neighs = six_n_morton(caCutIt->morton, nDeep_);
			int maxLocation = 0;
			for (int i = 0; i < 26; i++) {
				if (caCutVector[neighs[i]].location > maxLocation && caCutVector[neighs[i]].out == true) {
					maxLocation = caCutVector[neighs[i]].location;
				}
			}
			if (maxLocation <= caCutIt->location) {
				//std::cout << "max:" << maxLocation << ' ' << "location:" << caCutIt->location << count1++ << endl;
				caCutIt->outNext = false;
			}
		}
	}
	for (caCutIt = caCutVector.begin(); caCutIt != caCutVector.end(); caCutIt++) {
		if (!caCutIt->outNext) {
			caCutIt->outNext = true;
			caCutIt->out = false;
			count2++;
		}
	}
	//std::cout << count1 << ' ' << count2 << endl;


}

void suMeshViewer::read_point_information(string address)
{
	all_point_mises_strain.clear();
	fstream in;
	char read_temp;
	in.open(address, ios::in);
	while (!in.eof())
	{
		in >> read_temp;
		if (read_temp == 'n')
		{
			in >> read_temp;
			if (read_temp == 'e')
			{
				in >> read_temp;
				if (read_temp == 'n')
				{
					in >> read_temp;
					if (read_temp == 't')
					{
						in >> read_temp;
						if (read_temp == 's')
						{
							in >> read_temp;
							if (read_temp == '=')
							{
								in >> read_temp;
								if (read_temp == '\"')
								{
									in >> read_temp;
									if (read_temp == '9')
									{
										for (;;)
										{
											in >> read_temp;
											//if (read_temp == '<')
											//break;
											int break_ = 0;
											if (read_temp == '>')
											{
												point_strain inf_temp;
												double temp_vector[9];
												char a[12];
												int count = 0;
												for (;;)
												{
													//in >> read_temp;
													in >> read_temp;
													if (read_temp == '<')
													{
														break_++;
														break;
													}
													if (read_temp == '-')
													{
														for (int i = 0; i < 12; i++)
														{
															in >> a[i];
														}
														temp_vector[count] = -1 * trans(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11]);
													}
													else
													{
														a[0] = read_temp;
														for (int i = 1; i < 12; i++)
														{
															in >> a[i];
														}
														temp_vector[count] = trans(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11]);
													}
													count++;
													if (count == 9)
													{
														count = 0;
														/*inf_temp.x_strain = pow(temp_vector[0] * temp_vector[0] + temp_vector[1] * temp_vector[1] + temp_vector[2] * temp_vector[2], 0.5);
														inf_temp.y_strain = pow(temp_vector[3] * temp_vector[3] + temp_vector[4] * temp_vector[4] + temp_vector[5] * temp_vector[5], 0.5);
														inf_temp.z_strain = pow(temp_vector[6] * temp_vector[6] + temp_vector[7] * temp_vector[7] + temp_vector[8] * temp_vector[8], 0.5);
														float strain_temp = pow(0.5*(pow(inf_temp.x_strain - inf_temp.y_strain, 2) + pow(inf_temp.y_strain - inf_temp.z_strain, 2) +
														pow(inf_temp.z_strain-inf_temp.x_strain, 2)), 0.5);*/
														float strain_temp = pow(pow(temp_vector[0] + temp_vector[3] + temp_vector[6], 2)
															+ pow(temp_vector[1] + temp_vector[4] + temp_vector[7], 2) + pow(temp_vector[2] + temp_vector[5] + temp_vector[8], 2), 0.5);
														cout << strain_temp << " ";
														all_point_mises_strain.push_back(strain_temp);
													}
												}
												break;
											}
											if (break_)
												break;
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	in.close();
}

void suMeshViewer::read_point_coor(string address, float box_maxx, float box_maxy, float box_maxz, int box_level)
{
	point_morton.clear();
	{
		float dx_ = box_maxx / pow(2, box_level);
		float dy_ = box_maxy / pow(2, box_level);
		float dz_ = box_maxz / pow(2, box_level);
		fstream in;
		char read_temp;
		in.open(address, ios::in);
		while (!in.eof())
		{
			in >> read_temp;
			//cout << read_temp;
			if (read_temp == 'i')
			{
				in >> read_temp;

				if (read_temp == 'i')
				{
					in >> read_temp;
					if (read_temp == '"')
					{
						in >> read_temp;
						//cout << read_temp;
						if (read_temp == '>')
						{
							float point_inf_temp[3];
							char a[12];
							int count = 0;
							for (;;)
							{
								//in >> read_temp;
								in >> read_temp;
								if (read_temp == '<')
								{
									break;
								}
								if (read_temp == '-')
								{
									for (int i = 0; i < 12; i++)
									{
										in >> a[i];
									}
									point_inf_temp[count] = -1 * trans(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11]);
								}
								else
								{
									a[0] = read_temp;
									for (int i = 1; i < 12; i++)
									{
										in >> a[i];
									}
									point_inf_temp[count] = trans(a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11]);
								}
								count++;
								if (count == 3)
								{
									count = 0;
									int tran[3];
									tran[0] = (point_inf_temp[0] + 0.5*dx_) / dx_;
									tran[1] = (point_inf_temp[1] + 0.5*dy_) / dy_;
									tran[2] = (point_inf_temp[2] + 0.5*dz_) / dz_;
									point_morton.push_back(morton(tran[0], tran[1], tran[2], box_level + 1));
									cout << tran[0] << " " << tran[1] << " " << tran[2] << " " << morton(tran[0], tran[1], tran[2], box_level + 1) << endl;

									//cout << inf_temp.x_strain << " " << inf_temp.y_strain << " " << inf_temp.z_strain << endl;
								}
							}
							break;
						}
					}
				}
			}
		}
		in.close();
		ass_point();
	}
}

void suMeshViewer::ass_point()
{
	//cout << point_number << endl;
	//cout << point_numbert << endl;
	/*for (int i = 0; i < point_number; i++)
	{
	cout << point_morton[i] << " " << i << " ";
	}*/
	for (int i = 0; i < point_morton.size(); i++)
	{
		point_mises_strain temp;
		temp.mises_strain = all_point_mises_strain[i];
		temp.point_morton = point_morton[i];
		sort_vector.push_back(temp);
	}
	/*for (int i = 0; i < point_morton.size(); i++)
	{
		std::cout << sort_vector[i].point_morton << ' ' << sort_vector[i].mises_strain << endl;
	}*/

}

double trans(char a1, char a2, char a3, char a4, char a5, char a6, char a7, char a8, char a9, char a10, char a11, char a12)
{
	double a = 0;
	a += asc2(a1);
	a += 0.1*asc2(a3);
	a += 0.01*asc2(a4);
	a += 0.001*asc2(a5);
	a += 0.0001*asc2(a6);
	a += 0.00001*asc2(a7);
	a += 0.000001*asc2(a8);
	int b = 0;
	b += asc2(a11) * 10 + asc2(a12);
	if (a10 == '-')
	{
		b = b*(-1);
	}
	a = a*pow(10, b);
	return a;
}

int asc2(char a)
{
	switch (a)
	{
	case '0':
		return 0;
		break;
	case '1':
		return 1;
		break;
	case '2':
		return 2;
		break;
	case '3':
		return 3;
		break;
	case '4':
		return 4;
		break;
	case '5':
		return 5;
		break;
	case '6':
		return 6;
		break;
	case '7':
		return 7;
		break;
	case '8':
		return 8;
		break;
	case '9':
		return 9;
		break;
	}
}

void suMeshViewer::assignment(std::vector<SU::OctNode>& assVector)
{
	std::vector<SU::OctNode>::iterator assIt = assVector.begin();
	for (; assIt != assVector.end(); assIt++)//对所有的auto_cell赋应变值
	{
		assIt->strain = return_max_strain(assIt->morton, nDeep_);
		//cout << auto_cell[ca[i]].strain << "  ";
		//std::cout << assIt->strain << ' ';
	}
}

float suMeshViewer::return_max_strain(int morton_code, int level)
{
	int *voxel_morton = code(morton_code, level);
	//先计算对应的八个顶点的莫顿序
	int point_morton_[8];
	point_morton_[0] = morton(voxel_morton[0], voxel_morton[1], voxel_morton[2], level + 1);
	point_morton_[1] = morton(voxel_morton[0] + 1, voxel_morton[1], voxel_morton[2], level + 1);
	point_morton_[2] = morton(voxel_morton[0], voxel_morton[1] + 1, voxel_morton[2], level + 1);
	point_morton_[3] = morton(voxel_morton[0], voxel_morton[1], voxel_morton[2] + 1, level + 1);
	point_morton_[4] = morton(voxel_morton[0] + 1, voxel_morton[1] + 1, voxel_morton[2] + 1, level + 1);
	point_morton_[5] = morton(voxel_morton[0], voxel_morton[1] + 1, voxel_morton[2] + 1, level + 1);
	point_morton_[6] = morton(voxel_morton[0] + 1, voxel_morton[1], voxel_morton[2] + 1, level + 1);
	point_morton_[7] = morton(voxel_morton[0] + 1, voxel_morton[1] + 1, voxel_morton[2], level + 1);
	float max_strain = 0;
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < point_morton.size(); j++)
		{
			//cout << sort_vector[j].point_morton << " " << point_morton[i]<<":";
			if (sort_vector[j].point_morton == point_morton_[i])
			{
				max_strain = max_strain < abs(sort_vector[j].mises_strain) ? abs(sort_vector[j].mises_strain) : max_strain;
				//cout << "qwe"<<sort_vector[j].mises_strain << endl;
				break;
			}
		}
	}
	delete[]voxel_morton;
	return max_strain;
}

void suMeshViewer::agentCut(std::vector<SU::OctNode>& agentCutIt)
{
	for (int i = 0; i < (v.leafInternalNodes_.size() - v.leafBoundaryNodes_.size()) / 5; i++)//将前5%的体素切除
	{
		float compare_temp = 1000000;
		int merton_min_to_cut;
		for (int j = 0; j < agentCutIt.size(); j++)//遍历 找最小
		{
			/*if (j == 0)//第一次赋值
			{
			compare_temp = auto_cell[ca[j]].strain;
			merton_min_to_cut = auto_cell[ca[j]].merton;
			}*/
			if (agentCutIt[j].out == 1 && agentCutIt[j].label_ == 3)//遍历过程 替换最小
			{
				if (agentCutIt[j].strain < compare_temp)
				{
					compare_temp = agentCutIt[j].strain;
					merton_min_to_cut = j;
				}

			}
		}
		agentCutIt[merton_min_to_cut].out = 0;//把它切掉 把它切掉
		//cout << "cut!!!!!!!!!!!!!!!!!!!!" << merton_min_to_cut << " " << agentCutIt[merton_min_to_cut].strain << " " << compare_temp << endl;
	}
}

void suMeshViewer::outStlFile(std::vector<SU::OctNode> &stlVector) {
	fstream outstl;
	outstl.open("d://STL.stl", ios::out);
	outstl << "solid \"sample\"" << endl;
	outstl.close();

	int lines = pow(2, nDeep_);
	float x = v.bbMax_.data()[0] - v.bbMin_.data()[0];
	float y = v.bbMax_.data()[1] - v.bbMin_.data()[1];
	float z = v.bbMax_.data()[2] - v.bbMin_.data()[2];
	float dx = x / lines;
	float dy = y / lines;
	float dz = z / lines;
	std::vector<SU::OctNode>::iterator stlVectorIt;

	for (stlVectorIt = stlVector.begin(); stlVectorIt != stlVector.end(); stlVectorIt++)
	{
		if (stlVectorIt->out == 1 && stlVectorIt->label_ == SU::INTERIOR_CELL)
		{
			int *n_merton = six_n_morton(stlVectorIt->morton, nDeep_);
			/*cout << "(" << ca[i] << ")" << endl;

			for (int j = 0; j < 6; j++)
			{
			cout << "merton:" << n_merton[j] << " ";
			}
			*/
			int *code_ = code(stlVectorIt->morton, nDeep_);
			/*for (int j = 0; j < 3; j++)
			{
			cout << "code:" << code_[j] << " ";
			}*/

			if (code_[0] == (pow(2, nDeep_) - 1))
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "1 0 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1)*dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] * dz) << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "1 0 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1) *dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			else if (stlVector[n_merton[0]].out == 0)//x+
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "1 0 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1)*dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] * dz) << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "1 0 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1) *dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();



			}
			if (code_[0] == 0)
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "-1 0 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] * dz) << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "-1 0 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1)*dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			else if (stlVector[n_merton[1]].out == 0)//x-
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "-1 0 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] * dz) << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "-1 0 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1)*dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			if (code_[1] == (pow(2, nDeep_) - 1))
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "0 1 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "0 1 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			else if (stlVector[n_merton[2]].out == 0)//y+
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "0 1 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "0 1 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			if (code_[1] == 0)
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "0 -1 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "0 -1 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			else if (stlVector[n_merton[3]].out == 0)//y-
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "0 -1 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1)*dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "0 -1 0" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			if (code_[2] == (pow(2, nDeep_) - 1))
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "0 0 1" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "0 0 1" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1)*dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			else if (stlVector[n_merton[4]].out == 0)//z+
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "0 0 1" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "0 0 1" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1)*dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << code_[2] * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << code_[2] * dz << endl;
				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			if (code_[2] == 0)
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "0 0 -1" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;

				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "0 0 -1" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1)*dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1) * dz << endl;

				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
			else if (stlVector[n_merton[5]].out == 0)//z-
			{
				outstl.open("d://STL.stl", ios::app);
				outstl << "  facet normal " << "0 0 -1" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;

				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;


				outstl << "  facet normal " << "0 0 -1" << endl;
				outstl << "    outer loop" << endl;
				outstl << "      vertex " << (code_[0] - 1) * dx << ' ' << (code_[1] - 1)*dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << code_[1] * dy << ' ' << (code_[2] - 1) * dz << endl;
				outstl << "      vertex " << code_[0] * dx << ' ' << (code_[1] - 1) * dy << ' ' << (code_[2] - 1) * dz << endl;

				outstl << "    endloop" << endl;
				outstl << "  endfacet" << endl;

				outstl.close();
			}
		}
	}
	outstl.open("d://STL.stl", ios::app);
	outstl << "endsolid \"sample\"";

}

//TODO: 根据内部边界节点生成metaball
//      例子中采用所有外部边界节点
bool suMeshViewer::export_stl_with_metaball(const char* fileName, std::vector<SU::OctNode>& stlVector)
{
	/*for (int i = 0; i < stlVector.size(); i++)
	{
		if (stlVector[i].label_ == SU::INTERIOR_CELL)
		std::cout << stlVector[i].center().x << std::endl;
	}*/
	if (stlVector.size() < 2)
	{
		std::cout << "Open a model and make voxelization first!\n";
		return false;
	}
	//generate bounding box
	std::vector<Eigen::Vector3f> bbox;

	Eigen::Vector3d maxP = V.colwise().maxCoeff();
	Eigen::Vector3d minP = V.colwise().minCoeff();

	Eigen::Vector3f fminP, fmaxP;
	fminP << minP(0), minP(1), minP(2);
	fmaxP << maxP(0), maxP(1), maxP(2);

	int nVolume_per_dim = pow(2, nDeep_);
	float volume_size = (maxP - minP).maxCoeff() / nVolume_per_dim;
	//volume_size = 0.0f;
	//extrend bbox with one voxel 
	Eigen::Vector3f voxSize;
	voxSize << volume_size, volume_size, volume_size;
	voxSize *= 2;

	bbox.push_back(fminP);// - voxSize);
	bbox.push_back(fmaxP);// +voxSize);

	//generate metaballs on the model surface.
	std::vector<SU::METABALL> mballs;
	for (unsigned int i = 0; i < stlVector.size(); i++)
	{
		SU::METABALL m;

		//if (stlVector[i]->label_ == SU::BOUNDARY_CELL || stlVector[i]->label_ == SU::BOUNDARY_CELL_SPECIAL)
		if (!(stlVector[i].out) && stlVector[i].label_ == SU::INTERIOR_CELL)
		{
			//SU::Point cP = (stlVector[i].min_ + stlVector[i].max_)/2;
			SU::Point cP = stlVector[i].center();
			std::cout << "position = " << cP.x << ", " << cP.y << ", " << cP.z << std::endl;
			m.position << cP.x, cP.y, cP.z;
			m.squaredRadius = pow(volume_size / 2, 2);     //  metaball.r^2 = (volume box size / 2)^2
			/*std::cout << "r^2 = " << m.squaredRadius << std::endl;
			*/


			mballs.push_back(m);
		}


	}

	///resample
	/*int nSample = 5000;

	if (nSample > mballs.size()) nSample = mballs.size();

	std::vector<int> idxArr;
	std::vector<SU::METABALL> samples;
	for (int i = 0; i <mballs.size(); i++) idxArr.push_back(i);

	for (int i = 0; i < nSample; ++i) {
		int index = rand() % (mballs.size() - i) + i;
		std::swap(idxArr[i], idxArr[index]);
		samples.push_back(mballs[index] );
	}	*/

	/*
	SU::METABALL metaball;
	metaball.position = (bbox[0] + bbox[1]) / 2;
	metaball.squaredRadius = pow(volume_size / 2, 2);
	mballs.push_back(metaball);*/

	int resolution = nVolume_per_dim * 5;
	std::cout << "Num of Metaballs: " << mballs.size() << std::endl;
	std::cout << "Dim of Grid: " << resolution << "^ 3" << std::endl;

	//取势场中threshold=1的等值面	
	if (mballs.size())
		SU::write_metaball_to_stl(fileName, /*samples*/mballs, fThresholdMC, resolution, bbox);

	return true;
}
