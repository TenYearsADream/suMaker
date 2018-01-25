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

void generate_adjacent_vertexes_by_vertex(
	const Eigen::MatrixXi &F,
	const Eigen::MatrixXd &V,
	std::vector<std::set<Eigen::DenseIndex>>& A)
{
	Eigen::DenseIndex N = F.rows();
	Eigen::DenseIndex M = V.rows();
	A.resize(M);
	for (Eigen::DenseIndex i = 0; i < N; i++) {
		for (int j = 0; j < 3; j++) {
			//add relations: F(i,1) F(i,2) to F(i,0) ... 
			int v1_idx = (j + 1) % 3;
			int v2_idx = (j + 2) % 3;
			A[F(i, j)].insert(F(i,v1_idx) );
			A[F(i, j)].insert(F(i, v2_idx) );
		}
		
	}
}

void generate_adjacent_faces_by_vertex(const Eigen::MatrixXi & F, 
	const Eigen::MatrixXd & V, 
	std::vector<std::set<Eigen::DenseIndex>>& A)
{
	Eigen::DenseIndex N = F.rows();
	Eigen::DenseIndex M = V.rows();
	A.resize(M);
	for (Eigen::DenseIndex i = 0; i < N; i++) {
		for (int j = 0; j < 3; j++) {
			//add relation: F(i) each vertex of F(i) 
			A[F(i, j)].insert(i);
		}
	}
}

void generate_adjacent_faces_by_face(Eigen::MatrixXi & F, 
	std::vector<std::set<Eigen::DenseIndex>>& AVV, 
	std::vector<std::set<Eigen::DenseIndex>>& AVF, 
	int indexF, 
	std::set<Eigen::DenseIndex>& A)
{
	A.clear();
	for (int i = 0; i < 3; i++) {
		Eigen::DenseIndex j = F(indexF, i);
		std::set<Eigen::DenseIndex>::iterator it = AVV[j].begin();
		for (; it != AVV[j].end(); it++) {
			A.insert(AVV[(*it)].begin(), AVV[(*it)].end());
		}	
	}
}


