#pragma once
/*suGlobalState: a singleton class to store global info
 *\note this is a c++ 11 singleton, still need to test
 *\author Yuan Yao
 *\date  2016-07-08
 */
#include "suVolume.h"

class suGlobalState {
public:
	suGlobalState();
	~suGlobalState() { clear(); }
	static suGlobalState& gOnly();
	void release();

	// forbid copy & move
	suGlobalState(const suGlobalState&) = delete;
	suGlobalState& operator = (const suGlobalState&) = delete;
	suGlobalState(suGlobalState&&) = delete;
	suGlobalState& operator =(suGlobalState&&) = delete;

	void clear();

	static suGlobalState *p_gOnly;

	std::vector<int> forcedPoint;
	//global accessible data structures
	
	void setVolume(SU::suVolume *p) { p_volume = p; }
	SU::suVolume* getVolume() { return p_volume; }

	std::vector<SU::OctNode> globalEnvoMorton;
	void setEnvoMorton(std::vector<SU::OctNode> &p) { globalEnvoMorton = p; }

	void set_cross_view_dimension(int h, int w, int per_pixel_len = 4);

	unsigned char* gen_cross_section_X(float fPos);
	unsigned char* gen_cross_section_Y(float fPos);
	unsigned char* gen_cross_section_Z(float fPos);

	unsigned char* envoluted_cross_section_X(float fPos,int level);
	unsigned char* envoluted_cross_section_Y(float fPos);
	unsigned char* envoluted_cross_section_Z(float fPos);

	//app config
	//todo: add FEA engine info

	//boudary conditions setting
	//only one force is supported now
	std::set<int> load_face_list;          //for load setting
	std::set<int> boundary_face_list;      //for freedom setting
	std::set<int> selected_face_list;      //list of selected face indexes
	std::set<int> last_selected_face_list; //list of last selected indexes
	float force_value;   

	//multiple forces
	class force_setting {
	public:
		std::set<int> face_list;
		float force_mag;  //magnity
		float ori[3];     //force direction
	};
	class constraint_setting {
	public:
		std::set<int> face_list;		
		int type;     //constraint type: //0: ENCASTRE   1: PINNED	
	};
	std::map<int, force_setting> loadArr;
	std::map<int, constraint_setting> constraintArr;
	void add_load(std::set<int> &face_list, float mag, float *pNorm);
	void del_load();   //erase last entry
	void add_constraint(std::set<int> &face_list, int nType);
	void del_constraint();
	//UI
	bool bSelect_Mode;
private:

	SU::suVolume *p_volume;

	//store public data
	//by using Pimpl method
	class AppData;
	AppData *pData_;
};