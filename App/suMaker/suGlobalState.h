#pragma once
/*suGlobalState: a singleton class to store global info
 *\note this is a c++ 11 singleton, still need to test
 *\author Yuan Yao
 *\date  2016-07-08
 */
#include "suVolume.h"
class suGlobalState {
public:
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
	SU::suVolume *p_volume;
	void setVolume(SU::suVolume *p) { p_volume = p; }

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

private:
	suGlobalState();
	~suGlobalState() { clear(); }

	//store public data
	//by using Pimpl method
	class AppData;
	AppData *pData_;
};