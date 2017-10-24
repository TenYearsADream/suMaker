#include "suGlobalState.h"
#include <opencv2/opencv.hpp>
#include "morton.h"
///class suGlobalState::AppData
class suGlobalState::AppData
{
public:
	AppData():p_cross_section_img(0){}
	~AppData() { clear(); }
	void clear();

	void set_cross_section_dimemsion(int h, int w, int per_pixel_len);

public:
	cv::Mat* p_cross_section_img;
};
void suGlobalState::AppData::clear()
{
	if (!p_cross_section_img) delete p_cross_section_img;
}
void suGlobalState::AppData::set_cross_section_dimemsion(int h, int w, int per_pixel_len)
{
	p_cross_section_img = new cv::Mat(h, w, CV_8UC4);
}


///class suGlobalState
suGlobalState* suGlobalState::p_gOnly = 0;
suGlobalState & suGlobalState::gOnly() 
{
	// TODO: insert return statement here
	if (!p_gOnly) {
		p_gOnly = new suGlobalState();
		return *p_gOnly;
	}
	return *p_gOnly;
}

void suGlobalState::release()
{
	delete p_gOnly;
	p_gOnly = 0;
}



void suGlobalState::clear()
{
	if (!pData_) delete pData_;
}

void suGlobalState::set_cross_view_dimension(int h, int w, int per_pixel_len)
{
	pData_->set_cross_section_dimemsion(h, w, per_pixel_len);
}

unsigned char* suGlobalState::gen_cross_section_X(float fPos)
{
	if (!p_volume) return NULL;
	if (!(pData_->p_cross_section_img)) return NULL;

	int h = pData_->p_cross_section_img->rows;
	int w = pData_->p_cross_section_img->cols;

	pData_->p_cross_section_img->setTo(cv::Scalar(255, 255, 255, 255));

	OpenMesh::Vec3f vSize = p_volume->bbMax_ - p_volume->bbMin_;
	
	float fy_step = vSize[1] / h;
	float fz_step = vSize[2] / w;
	float fx = p_volume->bbMin_[0] + fPos * vSize[0];
	float fy_step_half = fy_step / 2;
	float fz_step_half = fz_step / 2;
	for (int y = 0; y < h; y++)
	{
		for (int z = 0; z < w; z++)
		{
			float fy = p_volume->bbMin_[1] + y*fy_step + fy_step_half;
			float fz = p_volume->bbMin_[2] + z*fz_step + fz_step_half;
			SU::OctNode *pNode = p_volume->getOctNode(fx, fy, fz);
			if (pNode)
			{
				if (pNode->label_ == SU::INTERIOR_CELL)
					pData_->p_cross_section_img->at<cv::Vec4b>(y, z) = cv::Scalar(255, 255, 0, 255);
				if (pNode->label_ == SU::BOUNDARY_CELL)
					pData_->p_cross_section_img->at<cv::Vec4b>(y, z) = cv::Scalar(255, 0, 0, 255);
			}
		}

	}
	return (unsigned char*)pData_->p_cross_section_img->data;
}

unsigned char* suGlobalState::gen_cross_section_Y(float fPos)
{
	if (!p_volume) return NULL;
	if (!(pData_->p_cross_section_img)) return NULL;

	int h = pData_->p_cross_section_img->rows;
	int w = pData_->p_cross_section_img->cols;
	pData_->p_cross_section_img->setTo(cv::Scalar(255, 255, 255, 255));

	OpenMesh::Vec3f vSize = p_volume->bbMax_ - p_volume->bbMin_;

	float fx_step = vSize[0] / 255;
	float fz_step = vSize[2] / 255;

	float fy = p_volume->bbMin_[1] + fPos * vSize[1];
	float fx_step_half = fx_step / 2;
	float fz_step_half = fz_step / 2;
	for (int x = 0; x < h; x++)
	{
		for (int z = 0; z < w; z++)
		{
			float fx = p_volume->bbMin_[0] + x*fx_step + fx_step_half;
			float fz = p_volume->bbMin_[2] + z*fz_step + fz_step_half;
			SU::OctNode *pNode = p_volume->getOctNode(fx, fy, fz);
			if (pNode)
			{
				if (pNode->label_ == SU::INTERIOR_CELL)
					pData_->p_cross_section_img->at<cv::Vec4b>(x, z) = cv::Scalar(255, 255, 0, 255);
				if (pNode->label_ == SU::BOUNDARY_CELL)
					pData_->p_cross_section_img->at<cv::Vec4b>(x, z) = cv::Scalar(255, 0, 0, 255);
			}
		}

	}
	return (unsigned char*)pData_->p_cross_section_img->data;
}

unsigned char* suGlobalState::gen_cross_section_Z(float fPos)
{
	if (!p_volume) return NULL;
	if (!(pData_->p_cross_section_img)) return NULL;

	int h = pData_->p_cross_section_img->rows;
	int w = pData_->p_cross_section_img->cols;
	pData_->p_cross_section_img->setTo(cv::Scalar(255, 255, 255, 255));

	OpenMesh::Vec3f vSize = p_volume->bbMax_ - p_volume->bbMin_;

	float fx_step = vSize[0] / 255;
	float fy_step = vSize[1] / 255;

	float fz = p_volume->bbMin_[2] + fPos * vSize[2];
	float fx_step_half = fx_step / 2;
	float fy_step_half = fy_step / 2;
	for (int x = 0; x < 255; x++)
	{
		for (int y = 0; y < 255; y++)
		{
			float fx = p_volume->bbMin_[0] + x*fx_step + fx_step_half;
			float fy = p_volume->bbMin_[1] + y*fy_step + fy_step_half;
			SU::OctNode *pNode = p_volume->getOctNode(fx, fy, fz);
			if (pNode)
			{
				if (pNode->label_ == SU::INTERIOR_CELL)
					pData_->p_cross_section_img->at<cv::Vec4b>(x, y) = cv::Scalar(255, 255, 0, 255);
				if (pNode->label_ == SU::BOUNDARY_CELL)
					pData_->p_cross_section_img->at<cv::Vec4b>(x, y) = cv::Scalar(255, 0, 0, 255);
			}
		}

	}
	return (unsigned char*)pData_->p_cross_section_img->data;
}

suGlobalState::suGlobalState() :p_volume(0)
{
	pData_ = new AppData();
}

unsigned char * suGlobalState::envoluted_cross_section_X(float fPos,int level)
{
	//找应力最大的点
	float maxStrain = 0;
	std::vector<SU::OctNode>::iterator envoMortonIt = globalEnvoMorton.begin();
	for (;envoMortonIt != globalEnvoMorton.end();envoMortonIt++) {
		maxStrain = envoMortonIt->strain > maxStrain ? envoMortonIt->strain : maxStrain;
	}
	if (maxStrain == 0) maxStrain = 0.00000000001;

	int h = pData_->p_cross_section_img->rows;
	int w = pData_->p_cross_section_img->cols;

	pData_->p_cross_section_img->setTo(cv::Scalar(255, 255, 255, 255));

	OpenMesh::Vec3f vSize = p_volume->bbMax_ - p_volume->bbMin_;

	float fy_step = vSize[1] / h;
	float fz_step = vSize[2] / w;
	float fx = p_volume->bbMin_[0] + fPos * vSize[0];
	float fy_step_half = fy_step / 2;
	float fz_step_half = fz_step / 2;

	float boxdx = vSize[0] / (pow(2, level));
	float boxdy = vSize[1] / (pow(2, level));
	float boxdz = vSize[2] / (pow(2, level));
	fx -= p_volume->bbMin_[0];
	for (int y = 0; y < h; y++)
	{
		for (int z = 0; z < w; z++)
		{
			float fy = p_volume->bbMin_[1] + y*fy_step + fy_step_half;
			float fz = p_volume->bbMin_[2] + z*fz_step + fz_step_half;

			
			fy -= p_volume->bbMin_[1];
			fz -= p_volume->bbMin_[2];
			int xNum = fx / boxdx;
			int yNum = fy / boxdy;
			int zNum = fz / boxdz;
			float color = 0;
			SU::OctNode *pNode = &globalEnvoMorton[morton(xNum, yNum, zNum, level)];
			if (pNode)
			{
				color = pNode->strain / maxStrain;
				if (color > 0.5) {
					if (pNode->label_ == SU::INTERIOR_CELL&&pNode->out == true)
						pData_->p_cross_section_img->at<cv::Vec4b>(y, z) = cv::Scalar(255, 255-255 * (color - 0.5) * 2 ,
							255-255 * (color - 0.5) * 2 , 255);

					if (pNode->label_ == SU::BOUNDARY_CELL)
						pData_->p_cross_section_img->at<cv::Vec4b>(y, z) = cv::Scalar(255, 255 - 255 * (color - 0.5) * 2,
							255 - 255 * (color - 0.5) * 2, 255);
				}
				else {
					if (pNode->label_ == SU::INTERIOR_CELL&&pNode->out == true)
						pData_->p_cross_section_img->at<cv::Vec4b>(y, z) = cv::Scalar(255 * (color * 2 ), 255 * (color * 2 ),
							128, 255);

					if (pNode->label_ == SU::BOUNDARY_CELL)
						pData_->p_cross_section_img->at<cv::Vec4b>(y, z) = cv::Scalar(255 * (color * 2), 255 * (color * 2),
							255, 255);

				}
				
			}
		}

	}
	return (unsigned char*)pData_->p_cross_section_img->data;
}