#pragma once
#include <igl/viewer/ViewerPlugin.h>
#include <iostream>
#include "../suGlobalState.h"
#include "../suMeshViewer.h"
#include <common/common.h>
namespace igl
{
	namespace viewer
	{
		class plugin_extend_selection : public ViewerPlugin
		{
		public:
			IGL_INLINE  plugin_extend_selection()
			{
				plugin_name = "extend_selection";
			}
			IGL_INLINE virtual bool key_down(int key, int modifiers)
			{
				if (key == 'E') {
					if (suGlobalState::gOnly().selected_face_list.empty()) return false;
					//doto: extend selection
					std::cout << "extend selected faces\n";
					std::set<Eigen::DenseIndex> fids;
					std::vector<int> &sFids = suGlobalState::gOnly().selected_face_list;
					for (auto i : sFids) {
						generate_adjacent_faces_by_face(
							static_cast<suMeshViewer*>(viewer)->F, 
							static_cast<suMeshViewer*>(viewer)->AVV, 
							static_cast<suMeshViewer*>(viewer)->AVF, i, fids);
					}
					// paint hit red
					for (auto i : fids) {
						static_cast<suMeshViewer*>(viewer)->C.row(i) << 1, 0, 0;
					}

				}
			}
			

		};
	}
}