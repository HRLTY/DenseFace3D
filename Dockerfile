FROM ubuntu:18.04

RUN apt-get update

# Скачать wget и nano
RUN apt-get install -y wget nano

# Установка пакетов для компиляции
RUN apt-get install -y build-essential cmake pkg-config

# Установка OpenBLAS и X11
RUN apt-get install -y libopenblas-dev liblapack-dev libx11-dev 

# Установка unzip
RUN apt-get install -y unzip

# Установка OpenGL
RUN apt-get install -y libgl1-mesa-dev

# Установка GLUT
RUN apt-get install -y freeglut3-dev

# Установка Boost
RUN apt-get install -y libboost-all-dev

# Скачать OpenCV 2.4
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

# Удалить архив после установки
RUN rm opencv.zip

# Скачать dlib 19.22
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

# Удалить архив dlib после распаковки
RUN rm dlib-19.22.tar.bz2

# Скачать bzip2
RUN apt-get install -y bzip2