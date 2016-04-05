# DenseFace3D
This is a software for the reconstruction of a real-time 3D face model from a monocular camera.

Demo: TODO

Dependencies: [Boost](http://www.boost.org), [Dlib](http://dlib.net), [OpenCV 2.4](http://opencv.org).

Note: Please make sure `conn.txt` and `landmark.txt`
and dlib's default face landmarking model file [shape_predictor_68_face_landmarks.dat](http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2)
are in the same directory as the executable.

Installation:

`mkdir build`

`cd build`

`cmake .. -Ddlib_DIR=path/to/dlib/`
(make sure the file `dlib_DIR/dlib/cmake` exists)

`make`

Usage:

the program will use your webcamera by default.

You can specify a video file as the first command line argument.


