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

#include "stdafx.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>

#include "cv.h"
#include "highgui.h"

#include <dlib/opencv.h>
#include <opencv2/highgui/highgui.hpp>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>

#if defined(__APPLE__)
#  include <OpenGL/gl.h>
#  include <OpenGL/glu.h>
#elif defined(__linux__) || defined(__MINGW32__) || defined(WIN32)

#  include <GL/gl.h>
#  include <GL/glu.h>
#else
#  include <gl.h>
#endif

#include "glm.h"
#include "OGL_OCV_common.h"
#include "samplePoint.h"
#include "ThinPlateSpline.h"

using namespace std;
using namespace dlib;
using namespace cv;

void loadNext();
//void loadWithPoints(Mat& ip, Mat& img);


struct triangleIndex{int index[3]; };
std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str);
void drawFace();
void drawFaceWithTex();
void drawPrimitiveFace();
void loadTPSwithControlPoints();
void callAndWait();
std::vector<triangleIndex> triangles;//fixed triangulation for primitive face.
std::vector<Point3d> landmark_points; //fixed landmark (now only used for depth values)
std::vector<double> depth, new_depth;
//full_object_detection realtime_landmark_detection; //the real time landmarks
std::vector<std::vector<point> > history_landmarks; //used for calculating the average
std::vector<std::vector<point> > historyForDepth; //used for calculating the average
std::vector<long> x_coor, y_coor; //current active average landmark coordinates
long x_mean = 0, y_mean = 0;

ThinPlateSpline* tps; //interpolation and sampling
Work sampler;


static const int REVERSE = 50;//auto rotation boundary
double rotateY = 0, dir = 5; //rotation speed
bool AUTOROTATE = true;

double ZSCALE = 2, XSCALE, YSCALE; //scaling factors

double color1, color2, color3; //for randomrized coloring
bool CHANGECOLOR = true;

long XTRANS = 315, YTRANS = 150;
bool READY = false; //remain false until the first face is detected
static bool NEW_DEPTH = false;

//for triangulation
cv::Subdiv2D delaunay;
cv::Rect rect;
std::vector<Vec6f> triangleList;

//dlib detection and image
frontal_face_detector detector;
shape_predictor pose_model;
VideoCapture cap;
std::vector <Point2f> img_last;

//current texture
GLuint textureID;
OpenCVGLTexture imgTex,imgWithDrawing;


const GLfloat light_ambient[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
const GLfloat light_diffuse[]  = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat light_position[] = { 0.0f, 0.0f, 1.0f, 0.0f };

const GLfloat mat_ambient[]    = { 0.7f, 0.7f, 0.7f, 1.0f };
const GLfloat mat_diffuse[]    = { 0.8f, 0.8f, 0.8f, 1.0f };
const GLfloat mat_specular[]   = { 1.0f, 1.0f, 1.0f, 1.0f };
const GLfloat high_shininess[] = { 100.0f };

//double rot[9] = {0};


//Mat backPxls;
//std::vector<double> rv(3), tv(3);
//Mat rvec(rv),tvec(tv);
//Mat camMatrix;


//GLMmodel* head_obj;

void display(void)
{
    // draw the image in the back
    int vPort[4]; glGetIntegerv(GL_VIEWPORT, vPort);
    glEnable2D();
    //drawOpenCVImageInGL(imgTex);
    //glTranslated(vPort[2]/2.0, 0, 0);
    drawOpenCVImageInGL(imgTex);
    glDisable2D();
    
    glClear(GL_DEPTH_BUFFER_BIT); // we want to draw stuff over the image
    
    // draw only on left part
    //glViewport(0, 0, vPort[2]/2, vPort[3]);
    //printf("viewport : (0,0,%d,%d)\n",vPort[2], vPort[3]);
    double ALFA = 0.7;
    if(NEW_DEPTH){
        double max = 0, min = 0;
        for(int i = 0; i < 68; i++){
            depth[i] = new_depth[i] * ALFA + depth[i] * (1 - ALFA);
            if(new_depth[i] > max)
                max = new_depth[i];
            if(new_depth[i] < min)
                min = new_depth[i];
        }
        ZSCALE = 100 / (max - min);
        NEW_DEPTH = false;
    }
    glPushMatrix();
    //must be called before drawFaceTex since this one loads control points
    drawFace();
    
    // rotate it
    // double _d[16] = {	rot[0],rot[1],rot[2],0,
    // 					rot[3],rot[4],rot[5],0,
    // 					rot[6],rot[7],rot[8],0,
    // 					0,	   0,	  0		,1};
    // glMultMatrixd(_d);
    
    // draw the 3D head model
    // glColor4f(1, 1, 1,0.75);
    //glmDraw(head_obj, GLM_SMOOTH);
    
    //----------Axes
    //    glScaled(50, 50, 50);
    //    drawAxes();
    //----------End axes
    glPopMatrix();
    
    glPushMatrix();
    drawPrimitiveFace();
    glPopMatrix();
    
    glPushMatrix();
    drawFaceWithTex();
    glPopMatrix();
    
    // restore to looking at complete viewport
    glViewport(0, 0, vPort[2], vPort[3]);
    
    glutSwapBuffers();
}


//densified face
void drawFace()
{
    gluLookAt(300,130,100,
              300,130,0,
              0,-1,0);
    // put the object in the right position in space
    //Vec3d tvv(tv[0],tv[1],tv[2]);
    //Vec3d tvv(0,0,0);
    //glTranslated(tvv[0], tvv[1], tvv[2]);
    
    //face location
    glTranslated(-230 + XTRANS,	80 + YTRANS, 0);
    glRotated(rotateY, 0, 1, 0);
    //glColor3ub(0, 255, 0);
    
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBegin(GL_TRIANGLES);
    delaunay.getTriangleList(triangleList);
    //int count = 0;
    if (READY) {
        //old way
        //        for (auto i = triangles.begin(); i != triangles.end();i++){
        //            count ++;
        //            for (size_t j = 0; j < 3; j++) {
        //                int index = i->index[j];
        //                Point3d p = landmark_points[index-1];
        //                point pp = realtime_landmark_detection.part(index-1);
        //                double x_pos = x_coor[index-1];
        //                double y_pos = y_coor[index-1];
        //                //glVertex3d(pp.x(), pp.y(), p.z * ZSCALE);
        //                glVertex3d(x_pos - x_mean, y_pos - y_mean, p.z * ZSCALE);
        //                //	printf("triangle %d point %lu: (%f,%f,%f)\n", count, j, p.x, p.y,p.z);
        //            }
        //        }
        //new  way
        
        loadTPSwithControlPoints();
        //autoscale to a fix scale
        XSCALE = 160 /(double) rect.width;
        YSCALE = 160 /(double) rect.height;
        
        double x_pos, y_pos, z_pos;
        
        std::vector<Point2d> pt(3);
        
        if(CHANGECOLOR){
            color1 = std::rand() / (double) RAND_MAX;
            color2 = std::rand() / (double) RAND_MAX;
            color3 = std::rand() / (double) RAND_MAX;
            CHANGECOLOR = false;
        }
        glLineWidth(2.0);
        glColor3f(color1, color2, color3);
        
        for (auto i = triangleList.begin(); i != triangleList.end();i++){
            //count ++;
            Vec6f t = *i;
            pt[0] = Point2d(t[0], t[1]);
            pt[1] = Point2d(t[2], t[3]);
            pt[2] = Point2d(t[4], t[5]);
            if ( rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])) {
                for (size_t j = 0; j < 3; j++) {
                    //int index = i->index[j];
                    x_pos = pt[j].x; y_pos = pt[j].y;
                    x_pos -= x_mean; y_pos -= y_mean;
                    //glVertex3d(pp.x(), pp.y(), p.z * ZSCALE);
                    z_pos = tps->calc_height(x_pos, y_pos);
                    x_pos *= XSCALE * .8; y_pos *= YSCALE * .8;
                    glVertex3d(x_pos, y_pos, z_pos);
                    //	printf("triangle %d point %lu: (%f,%f,%f)\n", count, j, p.x, p.y,p.z);
                }
            }
        }
    }
    glEnd();
}

void drawFaceWithTex()
{
    glDisable(GL_LIGHTING);
    glDisable(GL_BLEND);
    gluLookAt(300,130,100,
              300,130,0,
              0,-1,0);
    // put the object in the right position in space
    //Vec3d tvv(tv[0],tv[1],tv[2]);
    //Vec3d tvv(0,0,0);
    //glTranslated(tvv[0], tvv[1], tvv[2]);
    
    //face location
    glTranslated(-180 + XTRANS,	-130 + YTRANS, 0);
    //drawAxes();
    glRotated(rotateY, 0, 1, 0);
    //glColor3f(0, 1, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, imgTex.tex_id);
    double aw2h = imgTex.aspect_w2h, ithr = imgTex.thr, itwr = imgTex.twr;
    
    //int count = 0;
    if (READY) {
        glColor3ub(255, 255, 255);
        glBegin(GL_TRIANGLES);
        //old way
        //        for (auto i = triangles.begin(); i != triangles.end();i++){
        //            count ++;
        //            for (size_t j = 0; j < 3; j++) {
        //                int index = i->index[j];
        //                Point3d p = landmark_points[index-1];
        //                double w = p.x / imgTex.image.cols * itwr;
        //                double h = (1- p.y / imgTex.image.rows) * ithr;
        //                //glTexCoord2d(w , h);
        //                //printf("part %d: srow %f(w%d), sclo %f.\n", index, p.x, p.y);
        //                point pp = realtime_landmark_detection.part(index-1);
        //                double x_pos = x_coor[index-1];
        //                double y_pos = y_coor[index-1];
        //                //double w_r = (double)pp.x() / imgTex.image.cols * itwr;
        //                double w_r = x_pos / imgTex.image.cols * itwr;
        //                //printf("part %d: row %d, clo %d.\n", index, pp.x(), pp.y());
        //                //printf("image row %d, col %d\n", imgTex.image.rows,imgTex.image.cols);
        //                //double h_r = (1- (double)pp.y() / imgTex.image.rows) * ithr;
        //                double h_r = (1- y_pos / imgTex.image.rows) * ithr;
        //                glTexCoord2d(w_r , h_r);
        //                //glVertex3d(p.x, p.y, p.z * ZSCALE);
        //                //glVertex3d(pp.x(), pp.y(), p.z * ZSCALE);
        //                glVertex3d(x_pos - x_mean, y_pos - y_mean, p.z * ZSCALE);
        //                //	printf("triangle %d point %lu: (%f,%f,%f)\n", count, j, p.x, p.y,p.z);
        //            }
        //        }
        
        //occasional printing scale params.
        //        static int PRINT = 0;
        //        if(PRINT == 0){
        //            std::cout<<"face width:"<<rect.width<<", height:"<<rect.height<<'\n';
        //            std::cout<<"XSCALE:"<<XSCALE<<", YSCALE:"<<YSCALE<<'\n';
        //            PRINT = 50;
        //        }else PRINT--;
        
        std::vector<Point2d> pt(3);
        double x_pos, y_pos, z_pos, w_r, h_r;
        
        for (auto i = triangleList.begin(); i != triangleList.end();i++){
            //count ++;
            Vec6f t = *i;
            pt[0] = Point2d(t[0], t[1]);
            pt[1] = Point2d(t[2], t[3]);
            pt[2] = Point2d(t[4], t[5]);
            if ( rect.contains(pt[0]) && rect.contains(pt[1]) && rect.contains(pt[2])) {
                for (size_t j = 0; j < 3; j++) {
                    //int index = i->index[j];
                    x_pos = pt[j].x; y_pos = pt[j].y;
                    w_r = x_pos / imgTex.image.cols * itwr;
                    h_r = (1- y_pos / imgTex.image.rows) * ithr;
                    
                    x_pos -= x_mean; y_pos -= y_mean;
                    //glVertex3d(pp.x(), pp.y(), p.z * ZSCALE);
                    z_pos  = tps->calc_height(x_pos, y_pos);
                    
                    glTexCoord2d(w_r , h_r);
                    x_pos *= XSCALE; y_pos *= YSCALE;
                    glVertex3d(x_pos, y_pos , z_pos);
                    //printf("tripoint: (%f,%f,%f)\n",x_pos , y_pos, z_pos);
                }
            }
        }
        glEnd();
        glDisable(GL_TEXTURE_2D);
    }
    glEnable(GL_LIGHTING);
    glEnable(GL_BLEND);
}
void drawPrimitiveFace()
{
    gluLookAt(300,130,100,
              300,130,0,
              0,-1,0);
    // put the object in the right position in space
    //Vec3d tvv(tv[0],tv[1],tv[2]);
    //Vec3d tvv(0,0,0);
    //glTranslated(tvv[0], tvv[1], tvv[2]);
    
    //face location
    glTranslated(-140 + XTRANS,	80 + YTRANS, 0);
    glRotated(rotateY, 0, 1, 0);
    //glColor3ub(0, 255, 0);
    glLineWidth(2.0);
    //glColor3ub(255, 255, 255);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    glBegin(GL_TRIANGLES);
    if (READY) {
        //old way
        glColor3f(1-color1, 1-color2, 1-color3);
        double x_pos, y_pos, z_pos;
        
        //use fixed triangulation
        for (auto i = triangles.begin(); i != triangles.end();i++){
            for (size_t j = 0; j < 3; j++) {
                int index = i->index[j];
                Point3d p = landmark_points[index-1];
                //point pp = realtime_landmark_detection.part(index-1);
                
                x_pos = x_coor[index-1]; y_pos = y_coor[index-1];
                x_pos -= x_mean; y_pos -= y_mean;
                //glVertex3d(pp.x(), pp.y(), p.z * ZSCALE);
                x_pos *= XSCALE * .8; y_pos *= YSCALE * .8;
                //z_pos = p.z * ZSCALE;
                z_pos = depth[index-1] * ZSCALE;
                glVertex3d(x_pos, y_pos, z_pos);
                //	printf("triangle %d point %lu: (%f,%f,%f)\n", count, j, p.x, p.y,p.z);
            }
        }
    }
    glEnd();
}

//loading real time spline control points
// SID_EEFFECT: SUBTRACT MEAN FROM REAL TIME LOCATION.
void loadTPSwithControlPoints()
{
    std::vector<long> z_coor;
    //    for (auto i = landmark_points.begin(); i != landmark_points.end(); i++) {
    //        z_coor.push_back(i->z * ZSCALE);
    //    }
    for (auto i = depth.begin(); i != depth.end(); i++) {
        z_coor.push_back(*i * ZSCALE);
    }
    std::vector<long> temx = x_coor, temy = y_coor;
    for (long& j : temx)
        j -= x_mean;
    for (long& j : temy)
        j -= y_mean;
    
    tps->insert(temx, z_coor, temy);
    tps->calc_tps();
}

void updateLandmarkCoor(std::vector<point>& newPoints) //calculate the averate coordinate
{
    static time_t last = 0;
    int ROUND = 60;
    if(history_landmarks.size() >= 3)//add new
        history_landmarks.erase(history_landmarks.begin());
    history_landmarks.push_back(newPoints);
    time_t current = std::time(NULL);
    if(historyForDepth.size() >= ROUND && current - last >= 10){//recover depth once for a while(at least 10 secs)
        //prepare to write tracking to file
        stringstream x, y;
        for (auto i = 0; i < historyForDepth.size(); i++) {
            int landmark_count = historyForDepth[i].size();
            for (int j = 0; j < landmark_count; j++) {
                x << historyForDepth[i][j].x()<<' ';
                y << historyForDepth[i][j].y()<<' ';
            }
            x<<'\n'; y<<'\n';
        }
        fstream track_points("track.txt", ios_base::out);
        x.seekg(0, ios::end);
        y.seekg(0, ios::end);
        track_points.write(x.str().c_str(), x.tellg());
        track_points.write(y.str().c_str(), y.tellg());
        track_points.flush();
        track_points.close();
        historyForDepth.clear();
        
        last = std::time(NULL);
        std::thread callMatlab(callAndWait);
        callMatlab.detach();
        
    }else{
        if(historyForDepth.size() >= ROUND)
            historyForDepth.erase(historyForDepth.begin());
        historyForDepth.push_back(newPoints);
    }
    
    int num = (history_landmarks.end()-1)->size();
    x_coor.clear(); y_coor.clear();
    x_coor.insert(x_coor.begin(),num, 0);
    y_coor.insert(y_coor.begin(),num, 0);
    
    for(auto ith : history_landmarks){
        for (int i = 0; i < num; i++){
            x_coor[i] += ith[i].x();
            y_coor[i] += ith[i].y();
        }
    }
    
    int round = history_landmarks.size();
    for (int i = 0; i < num; i++){
        x_coor[i] /= round;
        y_coor[i] /= round;
    }
    
    for (long& j : x_coor)
        x_mean += j;
    for (long& j : y_coor)
        y_mean += j;
    
    x_mean /= x_coor.size();
    y_mean /= y_coor.size();
    
    //    for (long& j : x_coor)
    //        j -= x_mean;
    //    for (long& j : y_coor)
    //        j -= y_mean;
    //
}

void callAndWait()
{
    string com = R"(matlab -nodisplay -nojvm -nosplash -nodesktop -r "run('/Users/huangrui/Developer/dlib-18.18/examples/3dMesh/em-sfm/main.m'), exit")";
    string oldPath(getenv("PATH"));
    oldPath.append(":/Applications/MATLAB_R2015b.app/bin/");
    setenv("PATH", oldPath.c_str(), 1);
    system(com.c_str());
    
    fstream new_depth_file("new_depth.txt", ios_base::in);
    assert(new_depth_file.is_open());
    new_depth.clear();
    double tem_depth;
    int count = 0;
    while(new_depth_file >> tem_depth){
        new_depth.push_back(tem_depth);
        cout<<"["<<++count<<"]"<<tem_depth<<' ';
    }
    assert(new_depth.size() == 68);
    NEW_DEPTH = true;
}
void calNewSampleAndInsertForTri()
{
    std::vector<dpoint> tem;
    for(int i = 0; i < 27; i++){
        tem.push_back(dpoint(x_coor[i], y_coor[i]));
    }
    
    sampler.procsee(tem, 30, 30, 27);
    
    //printing for debugging
    //    int count = 0;
    //    std::fstream out("samples.txt", fstream::in | fstream::out | fstream::trunc );
    //    for (auto t : sampler.RPoints){
    //        out<<t.x()<<','<<t.y()<<'\n';
    //        //std::cout<<"point "<<++count<< ":("<< t.x() <<","<< t.y() <<")\n";
    //    }
    //    out.flush();
    //    out.close();
    //std::vector<dpoint> collector;
    
}

//Mat op;

void loadNext() {
    //static int counter = 1;
    std::vector<Point3f> imagePoints;
    Mat img,frame;
    if(!cap.read(frame)){
        printf("NO MORE VIDEO FRAME!\n");
        return;
    }
    //cv::resize(frame,img,cv::Size(0.4*frame.cols,0.4*frame.rows));
    
    //resize for speeding up the detection
    double limit = 500;
    if(frame.cols > limit)
        cv::resize(frame,img,cv::Size(limit, limit / frame.cols * frame.rows));
    //printf("load frame %d\n",counter);
    cv_image<bgr_pixel> cimg(img);
    
    // Detect faces
    std::vector<dlib::rectangle> faces = detector(cimg);
    std::vector<full_object_detection> shapes;
    for (unsigned long i = 0; i < faces.size(); ++i)
        shapes.push_back(pose_model(cimg, faces[i]));
    
    //for (size_t i = 0; i < shapes.size(); i++) {
    
    //only use the first detected face
    if (shapes.size() >= 1) {
        if(!READY){
            READY = true;
            printf("OK face here!\n");
            printf("Usage:\t q -- flat;\t w -- steep;\n");
            printf("\t s -- start/stop auto rotation;\n");
            printf("\t a -- rotate left;\t d -- rotate right;\n");
            printf("\t e -- speed up;\t r -- slow down\n");
            printf("\t SPACE -- capture and save;\n");
        }
        //const full_object_detection& d = shapes[0];
        
        //import real time detection into x_coor, y_coor
        full_object_detection& realtime_landmark_detection = shapes[0];
        int num_landmark = realtime_landmark_detection.num_parts();
        std::vector<point> newPoints;
        for (int i = 0; i < num_landmark; i++) {//read new landmarks
            point p = realtime_landmark_detection.part(i);
            newPoints.push_back(p);
        }
        updateLandmarkCoor(newPoints);
        calNewSampleAndInsertForTri();
    }
    
    
    //Mat ip(imagePoints);
    //cout<<ip;
    //sprintf(buf,"%sAngelina_Jolie/Angelina_Jolie_%04d.jpg",workingDir,counter);
    
    // Mat img = imread(buf);
    
    imgTex.set(img); //TODO: what if different size??
    
    // paint 2D feature points
    // for(unsigned int i=0;i<imagePoints.size();i++) circle(img,imagePoints[i],2,Scalar(255,0,255),CV_FILLED);
    //
    // loadWithPoints(ip,img);
    
    //imgWithDrawing.set(img);
    
    //counter = (counter+1);
}

//void loadWithPoints(Mat& ip, Mat& img) {
//    int max_d = MAX(img.rows,img.cols);
//    camMatrix = (Mat_<double>(3,3) << max_d, 0, img.cols/2.0,
//                 0,	max_d, img.rows/2.0,
//                 0,	0,	1.0);
//    cout << "using cam matrix " << endl << camMatrix << endl;
//
//    double _dc[] = {0,0,0,0};
//    solvePnP(op,ip,camMatrix,Mat(1,4,CV_64FC1,_dc),rvec,tvec,false,CV_EPNP);
//
//    Mat rotM(3,3,CV_64FC1,rot);
//    Rodrigues(rvec,rotM);
//    double* _r = rotM.ptr<double>();
//    printf("rotation mat: \n %.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n",
//           _r[0],_r[1],_r[2],_r[3],_r[4],_r[5],_r[6],_r[7],_r[8]);
//
//    printf("trans vec: \n %.3f %.3f %.3f\n",tv[0],tv[1],tv[2]);
//
//    double _pm[12] = {_r[0],_r[1],_r[2],tv[0],
//        _r[3],_r[4],_r[5],tv[1],
//        _r[6],_r[7],_r[8],tv[2]};
//
//    Matx34d P(_pm);
//    Mat KP = camMatrix * Mat(P);
//    //	cout << "KP " << endl << KP << endl;
//
//    //reproject object points - check validity of found projection matrix
//    for (int i=0; i<op.rows; i++) {
//        Mat_<double> X = (Mat_<double>(4,1) << op.at<float>(i,0),op.at<float>(i,1),op.at<float>(i,2),1.0);
//        //		cout << "object point " << X << endl;
//        Mat_<double> opt_p = KP * X;
//        Point2f opt_p_img(opt_p(0)/opt_p(2),opt_p(1)/opt_p(2));
//        //		cout << "object point reproj " << opt_p_img << endl;
//
//        circle(img, opt_p_img, 4, Scalar(0,0,255), 1);
//    }
//    rotM = rotM.t();// transpose to conform with majorness of opengl matrix
//}

void init_webcam(string file)
{
    bool FILE_EXIST = false;
    if(!file.empty()){
        std::fstream test(file);
        if(test.is_open())
            FILE_EXIST = true;
        test.close();
    }
    if (!FILE_EXIST)
        cap = VideoCapture(0);
    else
        cap = VideoCapture(file);
    detector = get_frontal_face_detector();
    deserialize("shape_predictor_68_face_landmarks.dat") >> pose_model;
}



void move(int value)
{
    if(AUTOROTATE){
        if(rotateY >= REVERSE || rotateY <= -REVERSE)
            dir *= -1;
        rotateY += dir;
    }
    //printf("Y:%d\n",rotateY);
    
    if(value % 20 == 0) //change color every 20 frame
        CHANGECOLOR = true;
    
    loadNext();
    glutPostRedisplay();
    glutTimerFunc(20, move, value+1);
}

void saveOpenGLBuffer() {
    static unsigned int opengl_buffer_num = 0;
    
    int vPort[4]; glGetIntegerv(GL_VIEWPORT, vPort);
    Mat_<Vec3b> opengl_image(vPort[3],vPort[2]);
    {
        Mat_<Vec4b> opengl_image_4b(vPort[3],vPort[2]);
        glReadPixels(0, 0, vPort[2], vPort[3], GL_RGBA, GL_UNSIGNED_BYTE, opengl_image_4b.data);
        flip(opengl_image_4b,opengl_image_4b,0);
        mixChannels(&opengl_image_4b, 1, &opengl_image, 1, &(Vec6i(0,0,1,1,2,2)[0]), 3);
    }
    stringstream ss; ss << "opengl_buffer_" << opengl_buffer_num++ << ".jpg";
    imwrite(ss.str(), opengl_image);
}

//int __w=250,__h=250;

void key(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 27 :
            exit(0);
            break;
            
            //depth augmentation
        case 'w':
        case 'W':
            ZSCALE += 5;
            cout<<"z-scale changed to--> "<<ZSCALE<<endl;
            break;
        case 'Q':
        case 'q':
            ZSCALE -= 5;
            cout<<"z-scale changed to--> "<<ZSCALE<<endl;
            break;
            
        case ' ':
            cout<<"pic saved!\n";
            saveOpenGLBuffer();
            //loadNext();
            break;
            
            //manual rotation
        case 'a':
            AUTOROTATE = false;
            rotateY += 5;
            break;
        case 'd':
            AUTOROTATE = false;
            rotateY -= 5;
            break;
            
        case 's':
            AUTOROTATE = !AUTOROTATE;
            if(AUTOROTATE) rotateY = 0; //reset position
            break;
            
            //rotation spped
        case 'e':
            dir *= 1.1;
            std::cout<<"speed up:"<<dir<<'\n';
            break;
        case 'r':
            dir /= 1.1;
            std::cout<<"speed down:"<<dir<<'\n';
            break;
        default:
            break;
    }
    
    glutPostRedisplay();
}

void idle(void)
{
    glutPostRedisplay();
}

void myGLinit() {
    //    glutSetOption ( GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION ) ;
    
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    
    
    glShadeModel(GL_SMOOTH);
    
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glEnable(GL_LIGHT0);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial ( GL_FRONT, GL_AMBIENT_AND_DIFFUSE );
    
    glLightfv(GL_LIGHT0, GL_AMBIENT,  light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    
    glMaterialfv(GL_FRONT, GL_AMBIENT,   mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE,   mat_diffuse);
    glMaterialfv(GL_FRONT, GL_SPECULAR,  mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, high_shininess);
    
    glEnable(GL_LIGHTING);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
}


void resize(int width, int height)
{
    //const float ar = (float) width / (float) height;
    
    glViewport(0, 0, width, height);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //glFrustum(-ar, ar, -1.0, 1.0, 2.0, 100.0);
    //gluPerspective(47,1.0,2,100);
    glOrtho(-300, 300, -200, 200, -800, 800);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
void readcsv()
{
    std::ifstream landmark("landmark.txt");
    if(!landmark.is_open())
        std::cout<<"You need conn.txt in the same directory as the executable!\n";
    std::vector<std::string> tmp_point3d =  getNextLineAndSplitIntoTokens(landmark);
    int count = 0;
    while(tmp_point3d.size() == 3){
        landmark_points.push_back(Point3d(std::stod(tmp_point3d[0]), std::stod(tmp_point3d[1]), std::stod(tmp_point3d[2])));
        //initial depth(fixed)
        depth.push_back(0.);//std::stod(tmp_point3d[2]));
        count ++;
        
        tmp_point3d =  getNextLineAndSplitIntoTokens(landmark);
    }
    landmark.close();
    std::cout<<"there are "<<count<<" landmarks read!\n";
    // landmark.close();
    // std::ifstream is("conn.txt");
    // is.seekg(0, is.end);
    // int length = is.tellg();
    // is.seekg(0, is.beg);
    //
    // std::string str;
    // str.resize(length, ' '); // reserve space
    // char* begin = &*str.begin();
    //
    // is.read(begin, length);
    // is.close();
    //
    // int tri_num = std::count(str.begin(), str.end(), '\n');
    // double tri_points[tri_num][3];
    // std::stringstream connectivities(str);
    count = 0;//cout<<count<<endl;
    std::ifstream connectivities("conn.txt");
    if(!connectivities.is_open())
        std::cout<<"You need conn.txt in the same directory as the executable!\n";
    std::vector<std::string> tmp_tri = getNextLineAndSplitIntoTokens(connectivities);
    while(tmp_tri.size() == 3){
        triangleIndex tmp;
        tmp.index[0] = std::stoi(tmp_tri[0]);
        tmp.index[1] = std::stoi(tmp_tri[1]);
        tmp.index[2] = std::stoi(tmp_tri[2]);
        triangles.push_back(tmp);
        count++;
        tmp_tri = getNextLineAndSplitIntoTokens(connectivities);
    }
    connectivities.close();
    std::cout<<"there are "<<count<<" triangles read!\n";
}

int start_opengl() {
    glutTimerFunc(20, move, 0);
    glutMainLoop();
    return 1;
}

void init_opengl(int argc,char** argv) {
    glutInitWindowSize(900,500);
    glutInitWindowPosition(40,40);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH); // | GLUT_MULTISAMPLE
    glutCreateWindow("DenseFace3D");
    
    myGLinit();
    
    glutReshapeFunc(resize);
    glutDisplayFunc(display);
    glutKeyboardFunc(key);
    //glutSpecialFunc(special);
    //glutIdleFunc(idle);
}

int main(int argc, char** argv)
{
    try {
        init_opengl(argc, argv); // get GL context, for loading textures
        
        string filename;
        if(argc >= 2)
            filename = string(argv[1]);
        //filename = "/Users/huangrui/Developer/dlib-18.18/examples/3dMesh/build/apple.mp4";
        //prepare dlib and cam
        init_webcam(filename);
        
        readcsv();
        // prepare OpenCV-OpenGL images
        imgTex = MakeOpenCVGLTexture(Mat());
        imgWithDrawing = MakeOpenCVGLTexture(Mat());
        
        tps = new ThinPlateSpline(68);
        
        loadNext();
        start_opengl();
        
        return 0;
    }
    catch(serialization_error& e)
    {
        cout << "You need dlib's default face landmarking model file to run this example." << endl;
        cout << "You can get it from the following URL: " << endl;
        cout << "   http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2" << endl;
        cout << endl << e.what() << endl;
    }
    catch(exception& e)
    {
        cout << e.what() << endl;
    }
    
}

void drawAxes() {
    
    //Z = red
    glPushMatrix();
    glRotated(180,0,1,0);
    glColor4d(1,0,0,0.5);
    //	glutSolidCylinder(0.05,1,15,20);
    glBegin(GL_LINES);
    glVertex3d(0, 0, 0); glVertex3d(0, 0, 1);
    glEnd();
    glTranslated(0,0,1);
    glScaled(.1,.1,.1);
    glutSolidTetrahedron();
    glPopMatrix();
    
    //Y = green
    glPushMatrix();
    glRotated(-90,1,0,0);
    glColor4d(0,1,0,0.5);
    //	glutSolidCylinder(0.05,1,15,20);
    glBegin(GL_LINES);
    glVertex3d(0, 0, 0); glVertex3d(0, 0, 1);
    glEnd();
    glTranslated(0,0,1);
    glScaled(.1,.1,.1);
    glutSolidTetrahedron();
    glPopMatrix();
    
    //X = blue
    glPushMatrix();
    glRotated(-90,0,1,0);
    glColor4d(0,0,1,0.5);
    //	glutSolidCylinder(0.05,1,15,20);
    glBegin(GL_LINES);
    glVertex3d(0, 0, 0); glVertex3d(0, 0, 1);
    glEnd();
    glTranslated(0,0,1);
    glScaled(.1,.1,.1);
    glutSolidTetrahedron();
    glPopMatrix();
}
