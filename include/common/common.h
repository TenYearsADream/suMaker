#pragma once
#include <Eigen/Dense>
#include <suMesh.h>

//convert obj from openmesh to eigen
bool convert_openmesh_to_Eigen(suMesh &ms, Eigen::MatrixXd &V, Eigen::MatrixXi &F);
//todo:
void generate_boundingbox(Eigen::MatrixXd &bbox, Eigen::MatrixXd &V);
void generate_cube_edges(Eigen::MatrixXd &bbox, Eigen::MatrixXd &P1, Eigen::MatrixXd &p2);
