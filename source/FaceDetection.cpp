/*
* 2015, 3D colored face modeling via Kinect camera
* Final Project, GIP Lab, Technion - Israel Institute of Technology
* Authors: Rotem Mordoch
*	         Ori Ziskind
*	         Nadine Toledano
*/

#include <iostream>
#include <fstream>
#include "3DfaceDetection_GIP.h"

#define AVERAGE_THRESHOLD_FACTOR 300
#define FACE_DEPTH_LIMIT 1300
#define RADIUS_FACTOR 1.1
#define WINDOW_SIZE 50

typedef tuple<int, int, int> triple;
static vector<triple> facePointWindow;

string Profile = "haarcascade_profileface.xml";
string Frontal = "haarcascade_frontalface_default.xml";

CascadeClassifier FrontalCascade;
CascadeClassifier ProfileCascade;

/******************************Auxiliary functions*******************************/
/*
* Description: 
* Compares 2 tuples according to the first argument.
*
* Parameters: 
* lhs - First tuple.
* rhs - Second tuple.
* 
* Return value: 
* True if the first argument of the first tuple is smaller, false otherwise.
*/
static bool mycompare(const triple &lhs, const triple &rhs)
{
    return get<0>(lhs) < get<0>(rhs);
}

/*
* Description: 
* Converts the image from type PixelRGB to type Vec3b.
* Allocates the new image.
*
* Parameters:
* image - The image to convert.
*
* Return value:
* A pointer to the new image in case of success, NULL otherwise.
*/
static Mat* Convert2Vec3b(Mat* image)
{
	Mat* result = new(std::nothrow) Mat(image -> rows, image -> cols, 16);

	if (!result)
		return NULL;

	for (int i = 0; i < image -> rows; i++) {
		for (int j = 0; j < image -> cols; j++) {

			PixelRGB &pixel = image -> at<PixelRGB>(i, j);
			Vec3b &intensity = result -> at<Vec3b>(i, j);

			intensity.val[0] = pixel.b;
			intensity.val[1] = pixel.g;
			intensity.val[2] = pixel.r;
		}
	}

	return result;
}

/*
* Description: 
* Converts vector of type PixelRGB to OpenCV Mat of type Vec3b.
* Allocates the new colored image.
*
* Parameters: 
* vec - PixelRGB vector.
* rows - Number of rows in the frame.
* cols - Number of columns in the frame.
*
* Return value:
* A pointer to the Mat in case of success, NULL otherwise.
*/
static Mat* Vector2Mat(vector<KinfuTracker::PixelRGB>* vec, int rows, int cols)
{
	Mat* temp = new(std::nothrow) Mat(rows, cols, DataType<KinfuTracker::PixelRGB>::type);

	if (!temp)
		return NULL;

	for (int i = 0; i < rows; i++){
		for (int j = 0; j < cols; j++){
			temp -> at<KinfuTracker::PixelRGB>(i, j) = vec -> at(i * cols + j);
		}
	}

	Mat* result = Convert2Vec3b(temp);
	delete temp;
	return result;
}

/*
* Description: 
* Updates facePointWindow with the new face detection details, circle's center point and radius.
* facePointWindow holds last WINDOW_SIZE center points and radii.
* 
* Parameters: 
* x - X coordinate of the circle's center point.
* y - Y coordinate of the circle's center point.
* r - Circle's radius.
*
* Return value:
* None
*/
static void WindowUpdate(int x, int y, int r)
{

	if (facePointWindow.size() < WINDOW_SIZE) {
		facePointWindow.push_back(make_tuple(x, y, r));
	}
	else {
		facePointWindow.erase(facePointWindow.begin());
		facePointWindow.push_back(make_tuple(x, y, r));
	}
}

/*
* Description: 
* Finds the median tuple of facePointWindow.
*
* Parameters:
* None
*
* Return value:
* The median (x, y, r) tuple of facePointWindow which is calculated using mycompare function.
*/
static triple FindMedian()
{
	vector<triple>sorted(facePointWindow); 
	sort(sorted.begin(), sorted.end(), mycompare);
	return sorted[sorted.size()/2];
}

/*
* Description: 
* Auxiliary function for detecting the face in a given RGB frame.
* Detects a face in the RGB frame using frontal and profile classifiers by
* applying the Haar Cascade Classifier.
*
* Parameters:
* frame - The RGB image in OpenCV Mat format.
* CenterX - The result x coordinate of the center point of the circle around the face, 0 if no face was detected.
* CenterY - The result y coordinate of the center point of the circle around the face, 0 if no face was detected.
* Radius - The result radius of the circle around the face, 0 if no face was detected.
*
* Return value:
* None
*/
static void detectAndDraw(Mat& frame, int* CenterX, int* CenterY, int* Radius)
{
	vector<Rect> faces;
	Mat frame_gray;
	int i = 0, radius = 0;
	triple fp;
	Point center;
	double distance = 0;
	uchar pixValue = 0;
	double scale = 1;
	center.x = 0;
	center.y = 0;

	cvtColor( frame, frame_gray, CV_BGR2GRAY );

	/* Try to find frontal face in the image. */
	FrontalCascade.detectMultiScale( frame_gray, faces, 1.1, 6, 0|
						CV_HAAR_SCALE_IMAGE, Size(100, 100) );

	for (vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++) {
        center.x = cvRound((r->x + r->width*0.5)*scale);
        center.y = cvRound((r->y + r->height*0.5)*scale);
        radius = cvRound((r->width + r->height)*0.25*scale);
	}
	
	/* In case no frontal face was found, try to find profile face in the image. */
	if (radius == 0) {
		 ProfileCascade.detectMultiScale(frame_gray, faces, 1.1, 4, 0 |
		 						CV_HAAR_SCALE_IMAGE, Size(100, 100));

		for (vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++) {
			center.x = cvRound((r->x + r->width*0.5)*scale);
			center.y = cvRound((r->y + r->height*0.5)*scale);
			radius = cvRound((r->width + r->height)*0.25*scale);
		}
	}

	/* Update the face window. */
	if (radius != 0) {
		*CenterX = center.x;
		*CenterY = center.y;
		*Radius = radius * RADIUS_FACTOR;
		WindowUpdate(center.x, center.y, radius);
	} 
	else {
		/* In case no face was found in the current frame, 
		     use the median data of the face window. */
		if (facePointWindow.size() > 0) {
			fp = FindMedian();
			*CenterX = get<0>(fp);
			*CenterY = get<1>(fp);
			*Radius = get<2>(fp) * RADIUS_FACTOR;
		} 
		else {
			*CenterX = *CenterY = *Radius = 0;
		}
	}
}

/*
* Description: 
* Loads Haar Cascade Classifiers.
*
* Parameters:
* None
*
* Return value:
* None
*/
void LoadCascades()
{
	if (!FrontalCascade.load(Frontal) || !ProfileCascade.load(Profile))
		cerr << "ERROR: Could not load classifier cascade" << endl;
}

/*
* Description: 
* A wrapper function for detecting the face in the RGB frame.
* Converts the frame from a vector representation to an OpenCV Mat representation and calls detectAndDraw auxiliary function.
* Saves into a folder each frame in jpg format.
* 
* Parameters:
* vec - The original image represented by RGB vector.
* rows - Number of rows in the frame.
* cols - Number of columns in the frame.
* CenterX - The result x coordinate of the center point of the circle around the face, 0 if no face was detected.
* CenterY - The result y coordinate of the center point of the circle around the face, 0 if no face was detected.
* Radius - The result radius of the circle around the face, 0 if no face was detected.
*
* Return value:
* None
*/
void detectFace(vector<KinfuTracker::PixelRGB>* vec, int rows, int cols, int* CenterX, int* CenterY, int* Radius)
{
	static int rgbCounter = 0;	
	Mat* orig = Vector2Mat(vec, rows, cols);

	if (!orig)
		return;
		
	ostringstream filename;
	filename << "RGB_pictures_cache/rgbImage" << rgbCounter++ << ".jpg";
	String filename2 = filename.str();
	imwrite(filename2, *orig);
	if (!orig -> empty())
        detectAndDraw(*orig, CenterX, CenterY, Radius);

	delete orig;
}

/*
* Description: 
* Updates the depth image to represent only the detected face.
* After the face was detected in the RGB image (defined by a center point and a radius),
* this function is called in order to update the depth image accordingly
* by setting to 0 the "z" values outside the circle, and inside it using thresholding. 
* 
* Parameters:
* vec - depth data vector.
* rows - Number of rows in the frame.
* cols - Number of columns in the frame.
* centerX - The x coordinate of the center point of the circle around the face.
* centerY - The y coordinate of the center point of the circle around the face.
* radius - The radius of the circle around the face.
*
* Return value:
* None
*/
void faceInDepth(vector<unsigned short>* vec, int rows, int cols, int centerX, int centerY, int radius)
{
	double distance = 0, sum = 0, squaredSum = 0;
	unsigned char pixValue = 0;
	int count = 0;

	/* Zero the background out side the circle. */
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			distance = sqrt(pow((double)(centerX - j), 2) + pow((double)(centerY - i), 2));
			if (distance > radius) {
				vec -> at(i * cols + j) = 0;
			} 
			else if (distance <= radius) {
				if (vec -> at(i * cols + j) != 0 || vec -> at(i * cols + j) < FACE_DEPTH_LIMIT) {
					sum += vec -> at(i * cols + j);
					count++;
				}
			}
		}
	}

	double mean = sum / (double) count;
	mean += AVERAGE_THRESHOLD_FACTOR;
	
	/* Zero pixels inside the circle, above the average threshold. */
	int xMin = max(centerX - radius, 0);
	int yMin = max(centerY - radius, 0);
	int xMax = min(centerX + radius, rows-1);
	int yMax = min(centerY + radius, cols-1);
	for (int i = yMin; i <= yMax; i++) {
		for (int j = xMin; j <= xMax; j++) {
			if (vec -> at(i * cols + j) > mean) {
				vec -> at(i * cols + j) = 0;
			}
		}
	}
}
