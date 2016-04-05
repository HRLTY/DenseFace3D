//
//  samplePoint.cpp
//  virtual
//
//  Created by huangrui on 4/3/16.
//
#include "samplePoint.h"
extern cv::Subdiv2D delaunay;
extern cv::Rect rect;

//void Work::procsee(std::vector<dpoint> &ptPolygon,int wx,int wy,int nCount)
//{
//    double minx=0,maxx=0,miny=0,maxy=0;
//    for(int i=0;i<nCount;i++)
//    {
//        minx=std::min(minx,ptPolygon[i].x());
//        maxx=std::max(maxx,ptPolygon[i].x());
//        miny=std::min(miny,ptPolygon[i].y());
//        maxy=std::max(maxy,ptPolygon[i].y()); 
//    }
//    int dx=(maxx-minx)/wx;
//    int dy=(maxy-miny)/wy;
//    for(int i=1;i<=wx;i++)
//    {
//        double pointx=minx+i*dx;
//        int sy,ey = wy;
//        for(int j=1;j<=wy;j++)
//        {
//            double pointy=miny+j*dy;
//            if(check(pointx,pointy,ptPolygon,nCount))
//            {
//                sy=j;
//                break;
//            }
//        }
//        for(int j=wy;j>=1;j--)
//        {
//            double pointy=miny+j*dy;
//            if(check(pointx,pointy,ptPolygon,nCount))
//            {
//                ey=j;
//                break;
//            }
//        }
//        //cout«"ey is "«ey«"sy is "«sy«endl;
//        for(int j=sy;j<=ey;j++)
//            RPoints.push_back(dpoint(pointx,(miny+j*dy)));
//    }
//}
//generate sample points and insert them into delaunay class
void Work::procsee(std::vector<dpoint> &tem, int wx, int wy, int nCount)
{
	double minx, maxx, miny, maxy;
	minx = miny = numeric_limits<double>::max();
	maxx = maxy = numeric_limits<double>::min();
	for (int i = 0; i < tem.size(); i++)
	{
		minx = std::min(minx, tem[i].x());
		maxx = std::max(maxx, tem[i].x());
		miny = std::min(miny, tem[i].y());
		maxy = std::max(maxy, tem[i].y());
	}
    
    //scanning steps
	double dx = (maxx - minx) / wx;
	double dy = (maxy - miny) / wy;
    
	double currentX = minx, currentY = miny;
	rect = cv::Rect(minx, miny, maxx - minx, maxy - miny);
	delaunay = cv::Subdiv2D(rect);
	for (int i = 0; i < wx; i++) {
		for (int j = 0; j < wy; j++) {
			if (check(currentX, currentY, tem, nCount)) {
				delaunay.insert(cv::Point2f(currentX, currentY));
				//RPoints.push_back(dpoint(currentX, currentY));
			}
			currentY += dy;
		}
		currentX += dx;
		currentY = miny;
	}
}
//check whether the point is inside the polygon
bool Work::check(double px, double py, std::vector<dpoint> &ptPolygon, int nCount)
{
	int nCross = 0;

	for (int i = 0; i < nCount; i++)
	{
		dpoint p1 = ptPolygon[i];
		dpoint p2 = ptPolygon[(i + 1) % nCount];

		if (p1.y() == p2.y())
			continue;

		if (py < std::min(p1.y(), p2.y())) 
			continue;
		if (py >= std::max(p1.y(), p2.y()))
			continue;

		double x = (double)(py - p1.y()) * (double)(p2.x() - p1.x()) / (double)(p2.y() - p1.y()) + p1.x();

		if (x > px)
			nCross++;
	}
	return (nCross % 2 == 1);
}