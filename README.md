# DenseFace3D
> This is a software for the reconstruction of a real-time 3D face model from a monocular camera.

#####Demo:  [Youtube](https://youtu.be/bCJOI4-Odes)  

#####Feature:

* Real-time landmark detection, mesh densifying (spline interpolation), triangulation and texture mapping for each frame.

* Two stages buffering — Both the landmark detection and spline interpolation can fail and the last valid instance is kept in those cases.

* Fixed location, stabilization and auto-scaling — The the capture is robust to face movements (both in translation and distance) and presented as the average of the locations of last several frames.

#####Dependencies: [Boost](http://www.boost.org), [Dlib 18.10](http://dlib.net), [OpenCV 2.4](http://opencv.org), [GLUT](https://www.opengl.org/resources/libraries/glut/).

Note: Please make sure `conn.txt` and `landmark.txt`
and dlib's default face landmarking model file [shape_predictor_68_face_landmarks.dat](http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2)
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

The inspiration comes from **IntraFace**.

- Jose Gonzalez-Mora, Fernando De la Torreb, Nicolas Guil, Emilio L.Zapata, 2010

  Learning a generic 3D face model from 2D image databases using incremental structure from motion

- Vahid Kazemi and Josephine Sullivan

  One Millisecond Face Alignment with an Ensemble of Regression Trees


- Lorenzo Torresani, Aaron Hertzmann and Christoph Bregler, NIPS 16, 2003

  Learning Non-Rigid 3D Shape from 2D Motion


- Gianluca Donato and Serge Belongie, 2002

  Approximation Methods for Thin Plate Spline Mappings and Principal Warps






