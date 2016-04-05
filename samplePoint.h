//
//  samplePoint.h
//  virtual
//
//  Created by huangrui on 4/3/16.
//
//

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
