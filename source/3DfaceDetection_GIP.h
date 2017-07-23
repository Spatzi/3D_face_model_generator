/*
* 2015, 3D colored face modeling via Kinect camera
* Final Project, GIP Lab, Technion - Israel Institute of Technology
* Authors: Rotem Mordoch
*	         Ori Ziskind
*	         Nadine Toledano
*/

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <cctype>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <math.h>
#include <new>
#include <pcl/gpu/kinfu/pixel_rgb.h>
#include <pcl/gpu/kinfu/kinfu.h>
#include <pcl/impl/point_types.hpp>

#include <vector>
#include <algorithm>
#include <tuple>

using namespace std;
using namespace cv;
using namespace pcl::gpu;

void LoadCascades();
void detectFace(vector<KinfuTracker::PixelRGB>* vec, int rows, int cols, int* CenterX, int* CenterY, int* Radius);
void faceInDepth(vector<unsigned short>* vec, int rows, int cols, int CenterX, int CenterY, int Radius);