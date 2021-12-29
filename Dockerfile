FROM ubuntu:18.04

RUN apt-get update

RUN apt-get install -y wget nano

RUN apt-get install -y build-essential cmake pkg-config

# OpenBLAS and X11
RUN apt-get install -y libopenblas-dev liblapack-dev libx11-dev 

RUN apt-get install -y unzip

# OpenGL
RUN apt-get install -y libgl1-mesa-dev

# GLUT
RUN apt-get install -y freeglut3-dev

# Boost
RUN apt-get install -y libboost-all-dev

# Load OpenCV 4.x
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip; \
unzip opencv.zip; \
cd opencv-4.x/; \
mkdir build; \
cd build/; \
cmake ..; \
cmake --build .; \
make install; \
ldconfig; \
cd ../../;

RUN rm opencv.zip

# Load dlib 19.22
RUN wget http://dlib.net/files/dlib-19.22.tar.bz2; \
	tar xvf dlib-19.22.tar.bz2; \
	cd dlib-19.22/; \
	mkdir build; \
	cd build; \
	cmake ..; \
	cmake --build . --config Release; \
	make install; \
    ldconfig; \
	cd ../../;

RUN rm dlib-19.22.tar.bz2

RUN apt-get install -y bzip2