#pragma once
#include <Eigen/Dense>
#include <suMesh.h>

//convert obj from openmesh to eigen
bool convert_openmesh_to_Eigen(suMesh &ms, Eigen::MatrixXd &V, Eigen::MatrixXi &F);

//gernerate one-ring structure for input mesh(V, F)
void generate_adjacent_vertexes_by_vertex(
	const Eigen::MatrixXi &F,
	const Eigen::MatrixXd &V,
	std::vector<std::set<Eigen::DenseIndex> > &A);
void generate_adjacent_faces_by_vertex(
	const Eigen::MatrixXi &F,
	const Eigen::MatrixXd &V,
	std::vector<std::set<Eigen::DenseIndex> > &A);
/**generate adjacent faces from current face
* \param F: face matrix
* \param AVV: not used
* \param AVF: Adjacent face to face matrix
*/
void generate_adjacent_faces_by_face(
	Eigen::MatrixXi &F,
	std::vector<std::set<Eigen::DenseIndex> > &AVV,
	std::vector<std::set<Eigen::DenseIndex> > &AVF,
	int indexF,
	std::set<Eigen::DenseIndex> &A);
//todo:
void generate_boundingbox(Eigen::MatrixXd &bbox, Eigen::MatrixXd &V);
void generate_cube_edges(Eigen::MatrixXd &bbox, Eigen::MatrixXd &P1, Eigen::MatrixXd &p2);
