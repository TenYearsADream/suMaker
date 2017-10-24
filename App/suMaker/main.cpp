#include "suMeshViewer.h"

#include <iostream>
#include <suMesh.h>

/**
 * A framework for volume optimization
 * The GUI is based on Wenzel Jakob's NanoGUI and  igl lib
 */

int main(int argc, char *argv[])
{
	//todo: if argc > 1 then run as console command
	//else

	suMeshViewer viewer;
	viewer.build_UI();

	viewer.launch();
}

