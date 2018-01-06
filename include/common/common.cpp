#include "common.h"

bool convert_openmesh_to_Eigen(suMesh &ms, Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
	
	suMesh::ConstVertexIter  v_it(ms.vertices_begin()), v_end(ms.vertices_end());

	suMesh::Point p;
	Eigen::DenseIndex rows = ms.n_vertices();
	V.resize(rows, 3);
	rows = ms.n_faces();
	F.resize(rows, 3);

	//add vertices
	for (; v_it != v_end; ++v_it)
	{
		p = ms.point(v_it);
		V.row(v_it->idx()) << p[0], p[1], p[2];
	}
	//add face vertice 
	suMesh::ConstFaceIter f_it(ms.faces_begin()), f_end(ms.faces_end());
	for (; f_it != f_end; f_it++)
	{
		suMesh::FaceVertexIter fv_it = ms.fv_begin(f_it.handle());
		suMesh::FaceVertexIter fv_end = ms.fv_end(f_it.handle());

		int idxFv = 0;  //vertex index on a face
		for (; fv_it != fv_end; ++fv_it)
		{
			F(f_it->idx(), idxFv++) = fv_it.handle().idx();
		}
	} 
	return true;
}
