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
#include <common/common.h>
#include <ui/variabledialog.h>

using namespace std;
suMeshViewer::suMeshViewer() : bSelect_Mode(false), nDeep_(0), m_pCross_section_img(0), fThresholdMC(1)
{
	mFEABox = nullptr;

	bShowWindow2D = false;
}
void suMeshViewer::openMesh(std::string filename, const ProgressCallback &progress)
{
	progress("Read mesh", 0);
	OpenMesh::IO::read_mesh(mesh_, filename);
	progress("Read mesh", 100);
	convert_openmesh_to_Eigen(mesh_, V, F);
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
	
	data.clear();
	data.set_mesh(V, F);

	add_bounding_box();

	//data.set_colors(C);	
	core.align_camera_center(V, F);

	progress("Data read finished", 250);
	cur_opened_mesh_filename = filename;
	setTitle(cur_opened_mesh_filename);
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
	cur_opened_mesh_filename = filename;
	setTitle(cur_opened_mesh_filename);

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
		auto ctx = this->screen->nvgContext();
		/*Tool Panel*/
		nanogui::Window *curWindow = ngui->addWindow(Eigen::Vector2i(10, 10), "Tool Panel");
		////About
		//nanogui::Button *about = new nanogui::Button(curWindow, "", ENTYPO_ICON_INFO);

		//about->setCallback([&, ctx]() {
		//	auto dlg = new nanogui::MessageDialog(
		//		screen->window(), nanogui::MessageDialog::Type::Information, "About suMaker",
		//		"suMaker is freely available under a BSD-style license. "
		//		"If you use the meshes obtained with this software, we kindly "
		//		"request that you acknowledge this and link to the project page at\n\n"
		//		"\thttp://cloudsfab.com/research/\n\n");
		//	dlg->messageLabel()->setFixedWidth(550);
		//	dlg->messageLabel()->setFontSize(20);
		//	curWindow->performLayout(ctx);
		//	dlg->center();
		//});

		// ---------------------- Create popup load mesh panel ----------------------		
		PopupButton *openBtn = new nanogui::PopupButton(ngui->window(), "Open mesh");
		openBtn->setBackgroundColor(nanogui::Color(0, 255, 0, 25));
		openBtn->setIcon(ENTYPO_ICON_FOLDER);
		Popup *popup = openBtn->popup();
		VScrollPanel *vscroll = new nanogui::VScrollPanel(popup);
		ImagePanel *panel = new nanogui::ImagePanel(vscroll);

		
		
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

		ngui->addButton("Add Force", [&] {
			if (!bSelect_Mode) return;
			if (suGlobalState::gOnly().selected_face_list.empty()) return;
			suGlobalState::gOnly().load_face_list = suGlobalState::gOnly().selected_face_list;
			auto dlg = new nanogui::VariableDialog(screen, VariableDialog::Type::Question,
				"N",
				"Force Setting", 
				"The force wiil be load on the selected faces. "
				"The direction of force is on the face normal. "
				"The force magnity is:",
				"OK",
				"Cancel", true);
			//todo: record force direction
			dlg->setCallback([&](float v) {
				suGlobalState::gOnly().force_value = v;
				std::cout << suGlobalState::gOnly().force_value << std::endl;
			});
		});
		 ngui->addButton("Add Constraint", [&] {
			if (!bSelect_Mode) return;
			if (suGlobalState::gOnly().selected_face_list.empty()) return;
			suGlobalState::gOnly().boundary_face_list = suGlobalState::gOnly().selected_face_list;
			
			std::cout << suGlobalState::gOnly().boundary_face_list.size() << std::endl;
			auto dlg = new nanogui::VariableDialog(screen, VariableDialog::Type::Information, 
				"", //none unit 
				"Constraint setting", "Constraint is set!");
					
		});
		//Optimizaton
		ngui->addGroup("Structure Evolution");
		ngui->addVariable("MC threshold", fThresholdMC);
		ngui->addButton("Evolution", [&]()
		{
			std::cout << "Evolution canceled in this version..." << std::endl;
			return;
			//todo: check 3rd engine 			
			std::cout << "Begin evolution..." << std::endl;
			//evolution();
		});
		// ---------------------- Create popup export mesh panel ----------------------		
		PopupButton *exportBtn = new nanogui::PopupButton(ngui->window(), "Export mesh");
		exportBtn->setBackgroundColor(nanogui::Color(0, 255, 100, 25));
		exportBtn->setIcon(ENTYPO_ICON_FOLDER);
		Popup *export_popup = exportBtn->popup();
		VScrollPanel *export_vscroll = new nanogui::VScrollPanel(export_popup);
		ImagePanel *export_panel = new nanogui::ImagePanel(export_vscroll);

		mExampleImages.clear();
		mExampleImages.insert(mExampleImages.begin(),
			std::make_pair(nvgImageIcon(ctx, export_inp), ""));
		export_panel->setImages(mExampleImages);
		export_panel->setCallback([&, exportBtn](int i) {
			if (v.leafBoundaryNodes_.empty()) {
				std::cout << "Please make voxelization first!" << std::endl;
				return;
			}
			if (suGlobalState::gOnly().boundary_face_list.empty()) {
				std::cout << "Please config boundary conditions!" << std::endl;
				return;
			}
			if (suGlobalState::gOnly().load_face_list.empty()) {
				std::cout << "Please set load conditions!" << std::endl;
				return;
			}
			
			exportBtn->setPushed(true);
			std::string strExportFile = nanogui::file_dialog(
			{ { "inp", "Abquas input model" }}, true);
			if (!strExportFile.empty())
			{
				//Save voxilized model to mesh by metaball and MC;				
				//export_stl_with_metaball(strSaveFile.c_str(), v.leafBoundaryNodes_);
				export_inp(strExportFile);
			}
			exportBtn->setPushed(false);
		});
		ngui->mLayout->appendRow(0);
		ngui->mLayout->setAnchor(exportBtn, nanogui::AdvancedGridLayout::Anchor(1, ngui->mLayout->rowCount() - 1, 3, 1));


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
	if (mesh_.faces_empty()) return;	

	v.LoadMeshFromMesh(mesh_);
	v.PartitionSpace(nDeep_);

	//test: only adding boundary nodes to check if there's fault.
	std::vector<SU::OctNode*> nodeArr_;
	/*= v.leafInternalNodes_;
	std::cout << "internal node: " << v.leafInternalNodes_.size() << std::endl;
	*/
	//for test
	nodeArr_.insert(nodeArr_.end(), v.leafBoundaryNodes_.begin(), v.leafBoundaryNodes_.end());
	std::cout << "boundary node: " << v.leafBoundaryNodes_.size() << std::endl;
	
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
	if (bSet && !mesh_.faces_empty())
	{		
		//set select model
		C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
		
		//add and remove mouse callback
		callback_mouse_move =
			[&](igl::viewer::Viewer& viewer, int x, int y)->bool
		{	
			Eigen::Vector3f bc;
			int fid = -1;
			
			// Cast a ray in the view direction starting from the mouse position
			//double x = viewer.current_mouse_x;
			//double 
			y = viewer.core.viewport(3) - y;

			if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), core.view * core.model,
				core.proj, core.viewport, V, F, fid, bc))
			{				
				// paint hit red
				Eigen::Vector3d ori_color = C.row(fid);
				C.row(fid) << 1, 0, 0;
				viewer.data.set_colors(C);	
				C.row(fid) = ori_color;
				return true;
			}
			else {
				viewer.data.set_colors(C);
			}
			
			return false;
		};

		//set mouse click callback to get select face
		callback_mouse_down =
			[&](igl::viewer::Viewer& viewer, int button, int)->bool 
		{
			Eigen::Vector3f bc;
			int fid = -1;
			// Cast a ray in the view direction starting from the mouse position
			double x = viewer.current_mouse_x;
			double y = viewer.core.viewport(3) - viewer.current_mouse_y;

			if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), core.view * core.model,
				core.proj, core.viewport, V, F, fid, bc))
			{
				if (button == static_cast<int>(MouseButton::Left)) {
					// paint hit red
					std::cout << fid << " is selected " << std::endl;
					suGlobalState::gOnly().selected_face_list.push_back(fid);
					C.row(fid) << 1, 0, 0;
				}
				else if (button == static_cast<int>(MouseButton::Right)) {
					std::cout << " selected faces is cleared " << std::endl;
					suGlobalState::gOnly().selected_face_list.clear();
					C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
				}
				return false;
				
			}
			
			return false;
		};

		//add key_down
		callback_key_down =
			[&](igl::viewer::Viewer& viewer, unsigned char key, int modifiers)
		{
			if (key == 'e') {				
				if (suGlobalState::gOnly().selected_face_list.empty()) return false;
				//doto: extend selection
				//extend_selection_faces_by_normal(
				//suGlobalState::gOnly().selected_face_list.empty()
				//);
				
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
		callback_mouse_down =
			[&](igl::viewer::Viewer& viewer, int, int)->bool
		{
			return false;
		};
		callback_key_down = 
			[&](igl::viewer::Viewer& viewer, unsigned char key, int modifiers)
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

void suMeshViewer::setTitle(std::string strWinTitle)
{
	glfwSetWindowTitle(window, strWinTitle.c_str());
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

bool suMeshViewer::export_inp(std::string fileName)
{
	if (!v.isLoad_ || v.leafBoundaryNodes_.empty()) return false;

	auto idx = fileName.find_last_of('.');
	if (idx == std::string::npos ) {
		fileName = fileName + ".inp";
	}
	else {
		std::string ext = fileName.substr(idx);
		if (ext != ".inp") fileName = fileName + ".inp";
	}

	return v.saveBaseInp(fileName, 
		suGlobalState::gOnly().load_face_list,
		suGlobalState::gOnly().force_value,
		suGlobalState::gOnly().boundary_face_list
		);

}
