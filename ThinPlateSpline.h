/*
 Copyright (C) 2016 by Rui Huang
 huangrui@buaa.edu.cn
 
 This code is modified by Rui Huang.
 The original lisencing is provided below.
 
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


/*
 *  Thin Plate Spline demo/example in C++
 *
 *  - a simple TPS editor, using the Boost uBlas library for large
 *    matrix operations and OpenGL + GLUT for 2D function visualization
 *    (curved plane) and user interface
 *
 *  Copyright (C) 2003,2005 by Jarno Elonen
 *
 *  TPSDemo is Free Software / Open Source with a very permissive
 *  license:
 *
 *  Permission to use, copy, modify, distribute and sell this software
 *  and its documentation for any purpose is hereby granted without fee,
 *  provided that the above copyright notice appear in all copies and
 *  that both that copyright notice and this permission notice appear
 *  in supporting documentation.  The authors make no representations
 *  about the suitability of this software for any purpose.
 *  It is provided "as is" without express or implied warranty.
 *
 *  TODO:
 *    - implement TPS approximation 3 as suggested in paper
 *      Gianluca Donato and Serge Belongie, 2002: "Approximation
 *      Methods for Thin Plate Spline Mappings and Principal Warps"
 */

#ifndef ThinPlateSpline_h
#define ThinPlateSpline_h

#include <boost/numeric/ublas/matrix.hpp>

#include "linalg3d.h"
#include "ludecomposition.h"
//using namespace boost::numeric::ublas;

class ThinPlateSpline {
public:
    std::vector< TPS::Vec > control_points;
    std::vector< TPS::Vec > validPoints;
    double regularization = 0.025;
    double bending_energy = 0.0;
    int p;
    boost::numeric::ublas::matrix<double> mtx_l,mtx_v,mtx_orig_k,valid;
    bool READY = false;
    ThinPlateSpline(int p);
    void insert(const std::vector<long>& x_coor, const std::vector<long>& y_coor, const std::vector<long>& z_coor);
    void calc_tps();
    double calc_height(double x, double z);
    
    ~ThinPlateSpline() {}
};





// ========= BEGIN INTERESTING STUFF  =========


static double tps_base_func(double r)
{
    if ( r == 0.0 )
        return 0.0;
    else
        return r*r * log(r);
}


/*
 *  Calculate Thin Plate Spline (TPS) weights from
 *  control points and build a new height grid by
 *  interpolating with them.
 */
ThinPlateSpline::ThinPlateSpline(int num): p(num),
mtx_l(boost::numeric::ublas::matrix<double>(p+3, p+3)),
mtx_v(boost::numeric::ublas::matrix<double>(p+3, 1)),
mtx_orig_k(boost::numeric::ublas::matrix<double>(p, p)),
valid(boost::numeric::ublas::matrix<double>(p+3, 1))
{
    
}

void ThinPlateSpline::insert(const std::vector<long>& x_coor, const std::vector<long>& y_coor, const std::vector<long>& z_coor)
{
    int xnum = x_coor.size();
    if(xnum != y_coor.size() || xnum != z_coor.size() || p != xnum){
        std::cout<<"TPS:Coordinate not consistent among x, y, z!\n";
        return;
    }
    control_points.clear();
    for (int i = 0; i < p; i++) {
        control_points.push_back(TPS::Vec(x_coor[i], y_coor[i], z_coor[i]));
    }
}

void ThinPlateSpline::calc_tps()
{
    // You We need at least 3 points to define a plane
    if ( control_points.size() < 3 )
        return;
    
    unsigned p = control_points.size();
    
    // Allocate the matrix and vector
    
    // Fill K (p x p, upper left of L) and calculate
    // mean edge length from control points
    //
    // K is symmetrical so we really have to
    // calculate only about half of the coefficients.
    double a = 0.0;
    for ( unsigned i=0; i<p; ++i )
    {
        for ( unsigned j=i+1; j<p; ++j )
        {
            TPS::Vec pt_i = control_points[i];
            TPS::Vec pt_j = control_points[j];
            pt_i.y = pt_j.y = 0;
            double elen = (pt_i - pt_j).len();
            mtx_l(i,j) = mtx_l(j,i) =
            mtx_orig_k(i,j) = mtx_orig_k(j,i) =
            tps_base_func(elen);
            a += elen * 2; // same for upper & lower tri
        }
    }
    a /= (double)(p*p);
    
    // Fill the rest of L
    for ( unsigned i=0; i<p; ++i )
    {
        // diagonal: reqularization parameters (lambda * a^2)
        mtx_l(i,i) = mtx_orig_k(i,i) =
        regularization * (a*a);
        
        // P (p x 3, upper right)
        mtx_l(i, p+0) = 1.0;
        mtx_l(i, p+1) = control_points[i].x;
        mtx_l(i, p+2) = control_points[i].z;
        
        // P transposed (3 x p, bottom left)
        mtx_l(p+0, i) = 1.0;
        mtx_l(p+1, i) = control_points[i].x;
        mtx_l(p+2, i) = control_points[i].z;
    }
    // O (3 x 3, lower right)
    for ( unsigned i=p; i<p+3; ++i )
        for ( unsigned j=p; j<p+3; ++j )
            mtx_l(i,j) = 0.0;
    
    
    // Fill the right hand vector V
    for ( unsigned i=0; i<p; ++i )
        mtx_v(i,0) = control_points[i].y;
    mtx_v(p+0, 0) = mtx_v(p+1, 0) = mtx_v(p+2, 0) = 0.0;
    
    // Solve the linear system "inplace"
    if (0 != LU_Solve(mtx_l, mtx_v))
    {
        return;
        //puts( "Singular matrix! Aborting." );
        //exit(1);
    }else{
        valid = mtx_v;
        validPoints = control_points;
        READY = true;
    }
    
}

double ThinPlateSpline::calc_height(double x, double z)
{
    if(!READY)
        return 0;
    // Interpolate grid heights
    double h = valid(p+0, 0) + valid(p+1, 0)*x + valid(p+2, 0)*z;
    TPS::Vec pt_i, pt_cur(x,0,z);
    for ( unsigned i=0; i<p; ++i )
    {
        pt_i = validPoints[i];
        pt_i.y = 0;
        h += valid(i,0) * tps_base_func( ( pt_i - pt_cur ).len());
    }
    return h;
}
#endif /* ThinPlateSpline_h */
