#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace SU {
	/**********/
	/* skel */
	/**********/
	class skel
	{
	public:
		skel(float x, float y, float z){
			v << x, y, z;
		}

		void add_child(skel& n){}
		std::vector<skel*> parents_;
		std::vector<skel*> children_;

		Eigen::Vector3f v;
	};


	/********************/
	/*    suSkeleton    */
	/********************/
	// Extract mesh skeleton and give a sorted, directed graph
	// for each v in verts:
	//    add_vert(v)

	class suSkeleton
	{
	public:
		suSkeleton() {}
		~suSkeleton() { clearup(); }

		unsigned int getRootNodeCount() { return (unsigned int) rootNodeArr_.size(); }
		unsigned int getVertsCount() { return (unsigned int)verts_.size(); }
		unsigned int getEdgesCount() { return (unsigned int)edges_.size(); }

		unsigned int add_vert(Eigen::Vector3f &v) {
			for (unsigned int i = 0; i < verts_.size(); i++) {
				if (verts_[i] == v) return i;
			}

			verts_.push_back(v);
			return (unsigned)verts_.size() - 1;
		}

		void add_edge(int eStart, int eEnd) {
			Eigen::Vector2i e;
			e << eStart, eEnd;
			for (int i = 0; i < edges_.size(); i++) {
				if (edges_[i] == e) return;
			}

			edges_.push_back(e);
		}
		void add_edge(Eigen::Vector3f& vStart, Eigen::Vector3f& vEnd) {
			unsigned iStart = add_vert(vStart);
			unsigned iEnd = add_vert(vEnd);
			add_edge(iStart, iEnd);
		}

		void sort_with_seting_plane(int x = 1, int y = 1, int z=0) {
		}

		bool load(std::string filename, const std::function<void(const std::string &, float)> &progress) {
			std::ifstream File(filename);
			std::vector<std::string> lines;
			std::string curLine;
			Eigen::Vector3f sv, tv;
			

			int nLine = 1;
			
			try {
				clearup();
				while (!File.fail()) {
					std::getline(File, curLine);
					if (curLine.empty()) break;
					if (curLine.substr(0, 1) != "2") break;

					std::vector<std::string> strs;
					//slice string
					std::istringstream f(curLine);
					std::string s;
					while (std::getline(f, s, ' ')) {
						strs.push_back(s);
					}

					sv << std::stof(strs[1]), std::stof(strs[2]), std::stof(strs[3]);
					tv << std::stof(strs[4]), std::stof(strs[5]), std::stof(strs[6]);
					add_edge(sv, tv);
					std::stringstream ss;
					ss << nLine++;
					progress("Read lines " + ss.str(), 20.0f);
				}

			}
			catch (...) {
				std::cout << "Illegal file format! \n";
			}
			
			return true;
		}

		Eigen::Vector3f getVerts(int idx) {
			Eigen::Vector3f _v;
			if (idx >= 0 && idx < getVertsCount()) {
				return verts_[idx];
			}
			return _v;
		}

		Eigen::Vector2i getEdge(int idx) {
			Eigen::Vector2i _e;
			if (idx >= 0 && idx < getEdgesCount()) {
				return edges_[idx];
			}
			return _e;
		}
	private:
		std::vector<skel> rootNodeArr_;
		std::vector<Eigen::Vector3f> verts_;
		std::vector<Eigen::Vector2i> edges_;
		
		Eigen::MatrixXd vMatrix_;
		Eigen::MatrixXi eMatrix_;

		void clearup() {
			rootNodeArr_.clear();
			verts_.clear();
			edges_.clear();
			vMatrix_.resize(0, 0);
			eMatrix_.resize(0, 0);
		}
	};
}
