//This plgin is derived from ViewerPlugin in libigl to generate .inp file.
//Author: Yuan Yao(fly2mars@gmail.com)

#pragma once
#include <igl/viewer/ViewerPlugin.h>
#include <iostream>
#include "../suGlobalState.h"
#include "../suMeshViewer.h"
#include <common/stdfunc.h>
namespace igl
{
	namespace viewer
	{
		class plugin_export_inp : public ViewerPlugin
		{
			class CmpVec
			{
			public:

				CmpVec(float _eps = FLT_MIN) : eps_(_eps) {}

				bool operator()(const SU::Point& _v0, const SU::Point& _v1) const
				{
					if (fabs(_v0[0] - _v1[0]) <= eps_)
					{
						if (fabs(_v0[1] - _v1[1]) <= eps_)
						{
							return (_v0[2] < _v1[2] - eps_);
						}
						else return (_v0[1] < _v1[1] - eps_);
					}
					else return (_v0[0] < _v1[0] - eps_);
				}

			private:
				float eps_;
			};
		public:
			IGL_INLINE  plugin_export_inp()
			{
				plugin_name = "export_inp";
			}
			IGL_INLINE virtual bool save(std::string filename)
			{
				if (SU::GetExtFileName(filename) != "inp")
				{
					return false;
				}
				
				suMeshViewer *pView = (suMeshViewer*)(viewer);
				if (!pView->v.isLoad_ || pView->v.leafBoundaryNodes_.empty()) return false;
				
				
				/*pView->v.saveBaseInp(filename,
					suGlobalState::gOnly().load_face_list,
					suGlobalState::gOnly().force_value,
					suGlobalState::gOnly().boundary_face_list
					);*/

				struct _Element {
					int node_index[8];
				};
				std::vector<_Element>  _elements;
				std::vector<SU::Point> _nodes;
				std::vector<std::set<int> > _nodes_load;    //define multiple load regions
				std::vector<std::set<int> > _nodes_constraints; //define multiple constraints

				CmpVec comp(FLT_MIN);
				std::map<SU::Point, unsigned int, CmpVec> _nodeMap;
				SU::suVolume &v = pView->v;

				auto gen_element = [&](std::vector<SU::OctNode*> &oct_nodes) {
					std::vector<SU::OctNode*>::iterator it = oct_nodes.begin();

					while (it != oct_nodes.end())
					{
						if ((*it)->level_ != v.level_) {  //verify all leaf nodes
							it++;
							continue;
						}

						SU::Point &m = (*it)->min_;
						SU::Point &M = (*it)->max_;
						_Element ele;

						//generate vertices for each voxel
						SU::Point v[8];
						v[0] = SU::Point(m.x, m.y, m.z);
						v[1] = SU::Point(M.x, m.y, m.z);
						v[2] = SU::Point(M.x, M.y, m.z);
						v[3] = SU::Point(m.x, M.y, m.z);
						v[4] = SU::Point(m.x, m.y, M.z);
						v[5] = SU::Point(M.x, m.y, M.z);
						v[6] = SU::Point(M.x, M.y, M.z);
						v[7] = SU::Point(m.x, M.y, M.z);

						// get index of element and ensure uniqueness
						for (int k = 0; k < 8; k++) {
							auto n = _nodeMap.find(v[k]);
							if (n == _nodeMap.end()) {
								_nodes.push_back(v[k]);
								unsigned int idx = _nodes.size();
								_nodeMap[v[k]] = idx;
								ele.node_index[k] = idx;
							}
							else {
								ele.node_index[k] = n->second;
							}
						}
						_elements.push_back(ele);
						(*it)->suNode_.element_id = _elements.size();  //record element id
						it++;
					}
				};
				////////////////////////////////////// LOAD /////////////////////////////////////
				//for each load, generating corresponding groups of element
				auto gen_nodes_group = [&](std::map<int, suGlobalState::force_setting> &loadArray,
					std::map<int, suGlobalState::constraint_setting> &constraintArray
					) {
					
					if (!loadArray.size()) return;
					_nodes_load.resize(loadArray.size());
					_nodes_constraints.resize(constraintArray.size());

					std::vector<SU::OctNode*>::iterator it = v.leafBoundaryNodes_.begin();
					//for each element
					for (; it != v.leafBoundaryNodes_.end(); ++it) {
						////convert face handle to id (for each element)
						if ((*it)->level_ != v.level_) {  //verify all leaf nodes
							it++;
							continue;
						}
						//convert element's face handle to id (for each element)
						std::set<int> faceid_list;
						std::vector<OpenMesh::FaceHandle>::iterator f_it = (*it)->suNode_.FaceVector.begin();
						for (; f_it != (*it)->suNode_.FaceVector.end(); ++f_it) {
							faceid_list.insert(f_it->idx());
						}						

						////for each load trangle, check if it belongs to the current element
						//for each load
						for (auto obj : loadArray) {
							std::set<int> &faceList = obj.second.face_list;
							for (auto fid : faceList) {
								if (faceid_list.count(fid)) {
									if ((*it)->suNode_.element_id != -1)
										_nodes_load[obj.first].insert( (*it)->suNode_.element_id );
								}
							}
							
						}
						////for each constraint trangle, check if it belongs to the current element
						//for each load
						for (auto obj : constraintArray) {
							std::set<int> &faceList = obj.second.face_list;
							for (auto fid : faceList) {
								if (faceid_list.count(fid)) {
									if ((*it)->suNode_.element_id != -1)
										_nodes_constraints[obj.first].insert((*it)->suNode_.element_id);
								}
							}

						}
					}					
				};
				///////////////////////////////////////Constraint//////////////////////////////////////
				//for each constraint, generating corresponding groups of element
				/*auto gen_constraint_nodes = [&](std::map<int, suGlobalState::constraint_setting> &constraintArray) {
					
				};
*/
				std::vector<SU::OctNode*> &leafBoundaryNodes_ = pView->v.leafBoundaryNodes_;
				std::vector<SU::OctNode*> &leafInternalNodes_ = pView->v.leafInternalNodes_;

				gen_element(leafBoundaryNodes_);
				gen_element(leafBoundaryNodes_);
				gen_nodes_group(suGlobalState::gOnly().loadArr, suGlobalState::gOnly().constraintArr);
				//gen_constraint_nodes(suGlobalState::gOnly().constraintArr);

				//generate inp file
				std::stringbuf strInp;
				std::ofstream inpFile;
				inpFile.open(filename, std::ios::out);

				inpFile << "*Heading" << std::endl;
				inpFile << "** Job name : EXAMPLE Model name :" << filename << std::endl;
				inpFile << "* * Generated by : RMEC suMaker" << std::endl;
				inpFile << "*Preprint, echo = NO, model = NO, history = NO, contact = NO" << std::endl;
				inpFile << "*Node, NSET = Nall" << std::endl;

				// write nodes
				std::stringstream os;
				for (unsigned int i = 0; i < _nodes.size(); i++) {
					os << i + 1 << ",  " << _nodes[i].x << ",   " << _nodes[i].y << ",   " << _nodes[i].z << std::endl;
				}
				inpFile << os.str();

				//write elements
				os.str(std::string());
				os.clear();
				os << "*Element, type = C3D8" << std::endl;
				for (unsigned int i = 0; i < _elements.size(); i++) {
					os << i + 1 << ",  ";
					for (int j = 0; j < 7; j++) {
						os << _elements[i].node_index[j] << ",   ";
					}
					os << _elements[i].node_index[7] << std::endl;
				}
				inpFile << os.str() << std::endl;

				//clear string stream
				os.str(std::string());
				os.clear();

				//load info
				struct loadInfo {
					std::string name;
					float mag;
					int ori;         //orientation
				};
				//write carrying regions
				std::vector<loadInfo> loadInfoList;
				int idx_ = 0;
				for (auto r : _nodes_load) {
					loadInfo info;
					std::string name = "FemLoadFixed" + std::to_string(idx_);  
					info.name = name;
					info.mag = suGlobalState::gOnly().loadArr[idx_].force_mag;
					for (int ii = 0; ii < 3; ii++) {
						if (suGlobalState::gOnly().loadArr[idx_].ori[ii] != 0) {
							info.ori = ii+1; break;
						}
					}
					loadInfoList.push_back(info);
					//write node list
					//todo: remove internal node
					os << "*NSET, NSET = "<< name << std::endl;
					int ii = 0;
					for (auto eid : r) {
						os << eid << ", ";
						if (++ii % 6 == 0) os << std::endl;
					}
					os << std::endl;
					idx_++;					
				}
				os << std::endl;

				//write constraint regions
				struct constraintInfo {
					std::string name;
					int type;   //0: ENCASTRE   1: PINNED					
				};
				std::vector<constraintInfo> constraintInfoList;
				idx_ = 0;
				for (auto r : _nodes_constraints) {
					constraintInfo info;
					std::string name = "SetFixed" + std::to_string(idx_);
					info.name = name;
					info.type = suGlobalState::gOnly().constraintArr[idx_].type;
					constraintInfoList.push_back(info);
					//write node list
					//todo: remove internal node
					os << "*NSET, NSET = " << name << std::endl;
					int ii = 0;
					for (auto eid : r) {
						os << eid << ", ";
						if (++ii % 6 == 0) os << std::endl;
					}
					os << std::endl;
					idx_++;
				}

				os << "***********************************************************" << std::endl;
				os << "** One step is needed to calculate the mechanical analysis of FreeCAD/Abqus" << std::endl;
				os << "** loads are applied quasi - static, means without involving the time dimension" << std::endl;
				os << "** written by write_step_begin function" << std::endl;
				os << "*STEP" << std::endl;
				os << "*STATIC" << std::endl;

				//write each load
				os << "***********************************************************" << std::endl;
				os << "** Node loads" << std::endl;
				os << "** written by addForce function" << std::endl;
				os << "*CLOAD" << std::endl;
				os << "** FemLoadFixed" << std::endl;
				os << "** node loads on element Face : Box:Face6" << std::endl;
				for (auto r : loadInfoList) {
					//FemLoadFixed, 2, -50
					os << r.name << ", " << r.ori << ", " << r.mag << std::endl;					
				}

				//write each constraint
				os << "***********************************************************" << std::endl;
				os << "** ** Name: Boundaries" << std::endl;
				os << "** written by addConstraint function" << std::endl;
				os << "*Boundary" << std::endl;
				os << "** ConstraintFixed" << std::endl;

				std::string cons_type[] = { "ENCASTRE", "PINNED" };
				for (auto r : constraintInfoList) {
					//Set-2, PINNED
					os << r.name << ", " << cons_type[r.type]<< std::endl;
				}
				
				inpFile << os.str() << std::endl;

				os.str(std::string());
				os.clear();


				

				inpFile.close();

				return true;
			}	

		};
	}
}