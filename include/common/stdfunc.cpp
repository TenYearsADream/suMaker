#include "stdfunc.h"
#include <fstream>
namespace SU
{
	std::string GetExtFileName(const std::string &path) {
		int idx = path.rfind('.');
		std::string ext;
		if (idx != std::string::npos) {
			ext = path.substr(idx + 1);
		}
		return ext;
	}
	bool FileExists(const std::string& path)
	{
		std::ifstream File(path.c_str());
		return (!File.fail());
	}

}