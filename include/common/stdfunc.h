// This file is part of suMaker, a simple mesh processing tool.
//
// Copyright (C) 2018 Yuan Yao <fly2mars@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#pragma once
#include <string>
#include <fstream>
namespace SU 
{
	std::string GetExtFileName(const std::string &path);
	bool FileExists(const std::string& path);
}