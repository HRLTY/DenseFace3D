/*
 Copyright (C) 2016 by Rui Huang
 huangrui@buaa.edu.cn
 
 This file is part of Dense3DFace.
 
 Dense3DFace is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 Dense3DFace is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with Dense3DFace.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef samplePoint_h
#define samplePoint_h

#include <iostream>
#include <vector>

#include <dlib/image_processing.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace dlib;
//class point
//{
//    double x,y,z;
//    point(int xx,int yy,int zz=0):x(xx),y(yy),z(zz) {}
//};
class Work{
public:
    Work() {}
    void procsee(std::vector<dpoint> &ptPolygon,int wx,int wy,int nCount);
    bool check(double px,double py, std::vector<dpoint> &ptPolygon, int nCount);
//private:
    std::vector<dpoint> RPoints;
};

#endif /* samplePoint_h */
