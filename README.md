# DenseFace3D
> This is a software for the reconstruction of a real-time 3D face model from a monocular camera.

####Demo:  [Youtube](https://youtu.be/ljXbnFJ0sCI)  or [Tencent Video](http://v.qq.com/page/s/q/p/s0192xdwdqp.html)  

####Feature:

* Real-time landmark detection, mesh densifying (spline interpolation), triangulation and texture mapping for each frame.

* Two stages buffering — Both the landmark detection and spline interpolation can fail and the last valid instance is kept in those cases.

* Fixed location, stabilization and auto-scaling — The the capture is robust to face movements (both in translation and distance) and presented as the average of the locations of last several frames.

### If you have any question, feel free to reach me at huangruiwizard@gmail.com. *Research/intern opportunities regarding computer science(computer vision would be greater) are welcomed.* I am a senior undergraduate currently, this work was done during my internship at Institute of Automation, Chinese Academy of Sciences. You can learn more about me from [my CV](http://hrlty.github.io/about/).

#####Dependencies: [Boost](http://www.boost.org), [Dlib 18.10](http://dlib.net), [OpenCV 2.4](http://opencv.org), [GLUT](https://www.opengl.org/resources/libraries/glut/).

Note: Please make sure `conn.txt` and `landmark.txt`
and dlib's default face landmarking model file shape_predictor_68_face_landmarks.dat (download [here](http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2))
are in the same directory as the executable.

#####Installation:

`mkdir build`

`cd build`

`cmake .. -Ddlib_DIR=path/to/dlib/`
(make sure the file `dlib_DIR/dlib/cmake` exists)

`make`

#####Usage:

the program will use your webcamera by default.

You can specify a video file as the first command line argument.

#####Acknowledgement:

The inspiration comes from [IntraFace](http://www.humansensing.cs.cmu.edu/intraface/index.php).

- Jose Gonzalez-Mora, Fernando De la Torreb, Nicolas Guil, Emilio L.Zapata, 2010

  Learning a generic 3D face model from 2D image databases using incremental structure from motion

- Vahid Kazemi and Josephine Sullivan

  One Millisecond Face Alignment with an Ensemble of Regression Trees


- Lorenzo Torresani, Aaron Hertzmann and Christoph Bregler, NIPS 16, 2003

  Learning Non-Rigid 3D Shape from 2D Motion


- Gianluca Donato and Serge Belongie, 2002

  Approximation Methods for Thin Plate Spline Mappings and Principal Warps


#####License:

 Copyright (C) 2016 by Rui Huang
 huangrui@buaa.edu.cn

 The program DenseFace3D is licensed under the GNU General Public License.

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
