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
				

				pView->v.saveBaseInp(filename,
					suGlobalState::gOnly().load_face_list,
					suGlobalState::gOnly().force_value,
					suGlobalState::gOnly().boundary_face_list
					);
				return true;
			}	

		};
	}
}