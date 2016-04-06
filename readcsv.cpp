/*
 Copyright (C) 2016 by Rui Huang
 huangrui@buaa.edu.cn
 
 This file is part of DenseFace3D.
 
 DenseFace3D is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 DenseFace3D is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with DenseFace3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <istream>
#include <string>
#include <vector>
#include <sstream>

std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
{
  std::vector<std::string> result;
  std::string line;
  std::getline(str, line);
  std::stringstream lineStream(line);
  std::string cell;

  while(std::getline(lineStream, cell, ',')){
    result.push_back(cell);
  }
  return result;
}
