//LaneFinder.h
#ifndef LINE_FINDER_H
#define LINE_FINDER_H
#include <stdarg.h>
#include <stdlib.h>
#include <cv.h>                 /* Contains all cv* function calls */
#include <highgui.h>
#include <iostream>
#include <string>
#include <stdexcept>
#include <math.h>
#include <fstream>
#include <algorithm>
#include <vector>
#include <list>
#include <queue>
#include <stdio.h>
#include <iomanip>
#include <WinSock.h>

//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <netdb.h>
#include <sys/types.h>
#include <sys/stat.h> 

#pragma  comment(lib,"ws2_32.lib")

IplImage* createImage(int color, int width, int height);

IplImage* createColorImage(int _width, int _height);

/**
 * returns the euclidean distance between two pixels
 */
double getDiffPoints(CvPoint const &p1, CvPoint const &p2);
void fillImageWithColor(int colorOfImage, IplImage *nameOfImage);
void displayImage(IplImage* ImageToDisplay, char* NameOfWindow);

/**
 * returns the angle in radian between two intersecting lines, the first two points are start and end point
 * of the first line..
 * (cos alpha = vector_a x vector_b / vector_a_length * vector_b_length ) (x = skalarproduct)
 * @param start_line1: the first line going from start_line1 to end_line1
 * @param end_line1: 
 * @param start_line2: the second line going from start_line2 to end_line2
 * @param end_line2: 
 * @return angle between the two lines
 */
double angleBetweenLines(CvPoint startL1, CvPoint endL1, CvPoint startL2, CvPoint endL2);

/**
 * returns euclidean distance between two pixels
 */
double distBetweenPoints(CvPoint p1, CvPoint p2);

/**
 * The struct housing line pixels
 */
struct line_points
{
		  CvPoint point;
		  float orientation;
		  bool endpoint;
		  bool startpoint;
		  bool no_neighbour;
};

/**
 * The class implementing functions for line extraction
 */
class LaneFinder
{
		  private:
					 int WIDTH;
					 int HEIGHT;
					 int TCPBUS_SOCKFD;
		  protected:
		  public:
					 CvMemStorage *STORAGE;
					 IplImage *LIST_OF_TEMPLATES[7];
					 /**
					  *The constructor. 
					  *@param int width: the width of the image in which you want to find a line
					  *@param int height: the height of the image in which you want to find a line
					  */
					 LaneFinder(int width, int height);
					 /**
					  *The destructor. 
					  */

					 ~LaneFinder();


					 int getWidth();
					 int getHeight();

					 /**
					  * finds Lines in a given image
					  *
					  * @param img: the original image
					  * @param canny_img: the canny filtered image of the original image
					  * @param img: the canny filtered image of the original image
					  * @param lowerLimit: the left boundary of the interval at the image bottom where the line is supposed to start in
					  * @param higherLimit: the right boundary of the interval at the image bottom where the line is supposed to start in
					  * @param yLimit: the upper boundary of the interval at the image bottom where the line is supposed to start in
					  * @param minSize: the detected line must have a minimal length of minSize pixels, otherwise it is discarded
					  * @param lengthTangent: to extrapolate a found line to the image bottom, the line parameters through two points must be specified. The first point is the starting point of the extracted line, the second point
					  * is lengthTangent points away from this first point.
					  * 
					  * @return alpha: the angle of the tangent of the beginning of the extracted line in degrees
					  * @return alpha: the x-coordinate of the first pixel of the extracted lane
					  * @return longestLine: the extracted line
					  * @return bin: contains line segments going from right to left 
					  * @return binRight: contains line segments going left to right
					  */

					 void findLines(IplImage* img, IplImage* img_out, double* alpha, int &x_pos, std::vector< struct line_points*> &longestLine, int lowerLimit, int higherLimit, int yLowerLimit,int yHigherLimit, std::vector< std::vector< struct line_points* > > &bin, std::vector< std::vector< struct line_points* > > &binRight, int minSize, int lengthTangent );

					 /**
					  * converts pixels from the extracted lane from the lineStruct to CvPoints
					  */
					 void convertLineStruct2Vec(std::vector< CvPoint > &vec, std::vector<struct line_points*> &lp);

					 /**
					  * fit a line based on points
					  * return true if is a horizontal line
					  */
					 bool isHorizontalLine(std::vector<struct line_points* > &vec);


					 /**
					  * takes an canny filtered image and creates line segments that start left and go to right
					  * You can change canny parameters here!
					  * @see binLinesRight(vector< vector< struct line_points* > > &bin, IplImage* img, IplImage* canny_img, std::string cannyParam)
					  */
					 void binLines(std::vector< std::vector< struct line_points* > > &bin, IplImage *img, IplImage* copy_canny_img, std::string cannyParam);


					 /**
					  * takes an image, returns a canny filtered image and creates line segments that start right and go to left.
					  * You can change canny parameters here!
					  * @see binLines(vector< vector< struct line_points* > > &bin, IplImage* img, IplImage* canny_img, std::string cannyParam)
					  */
					 void binLinesRight(std::vector< std::vector< struct line_points* > > &bin, IplImage *img, IplImage* copy_canny_img, std::string cannyParam);

					 void copy_line_points_Struct(line_points **p_target, line_points* p_src);
					 /**
					  * check if the given point is close (dist) to some starting point in the given bin.
					  * if so find the point in the bin that is closest to ep and from there on push back everything into longestLine and clear it in bin
					  * not only the closeness is checked, also good continuation in form of an angle
					  */
					 void doJoiningTwoOrient(CvPoint ep, std::vector< std::vector< struct line_points* > > &bin, std::vector< struct line_points*> &longestLine, int dist );
					 /**
					  * Often the lines go over into other objects, or they turn 
					  * around and run back to the beginning. This is treated here, by assuming good 
					  * continuity. In a sliding window fashion I go over the line by a pixel size of n
					  * (e.g. 10 pixels) I draw a line from the first, to the last pixel and calculate
					  * the summed distance of the other pixels from the line. If it is exceeding a threshold
					  * the line is cutted there.
					  *
					  * NEW!! I discount the tolerable distance the longer I do this, that causes
					  * that at the end I cut the line earlier
					  */
					 int improveGoodCont(std::vector< struct line_points*> & longestLine, int windowSize, int thresh,IplImage *img);

					 /**Actually it only checks which start- and endpoint can be used, but 'calTAngentParams' takes care of all special 
					  * cases, like vertical line, is enpoint<startpoint etc.
					  * Finds out which two points to use from the line to calculate m and b, this is done seperately by calTangentParams
					  */
					 void getTangentParams(std::vector< struct line_points*> &longestLine, int &highest_y_index, double &m, double &b, double &angle, CvPoint &startpoint, CvPoint &endpoint, int length, int img_width, IplImage *img);

					 /**
					  * Sometimes a line is extracted that does not start in the first row at the image bottom. To extend it to the bottom artificially this
					  * function extends the tangent of the beginning of the extracted lane to the image bottom. 
					  */
					 void doExtrapolation(std::vector< struct line_points*> &longestLine, IplImage* img,  double m, double b, CvPoint &point_y);
					 /**
					  * calculates pixel orientations
					  */

					 void getOrientationImage(IplImage* img, IplImage* &orientation_img);
					 void getNeighboursOrderedRight(IplImage* img, int x, int y, std::vector<struct line_points*>& points, IplImage* orientation_img, int orientation);
					 void getNeighboursOrdered(IplImage* img, int x, int y, std::vector<struct line_points*>& points, IplImage* orientation_img, int orientation);
					 void getHighestYIndex(std::vector<struct line_points*>& points, int &highest_y_index, CvPoint &yPoint);

					 /**
					  * writes a vector of CvPoints to a file or the tcp bus
					  */
					 int writeCurve(std::vector< CvPoint >  &curve, char *filename);
					 int publishCurve( std::vector< CvPoint >  &curve);
					 
					 void printLongestLineInColor(IplImage *imgColor, const std::vector< struct line_points*> &longestLine, int defaultC=-1);
					 void printLongestLineInColorTO(IplImage* imgColor, const std::vector< struct line_points*  > &longestLine, int limit);
					 /**
					  *deletes the found line data structure to free memory. 
					  */
					 void deleteLine( std::vector< struct line_points*> const &longestLine );
					 void printLongestLineInColorOverlay(IplImage *imgColor, const std::vector< struct line_points*> &longestLine, int defaultC=-1);


					 int extractLineWithoutKalman(IplImage *imgInput, IplImage *imgOutput, std::vector<CvPoint> &lineRight,  int searchLimitLeft, int searchLimitRight, bool debug, int minSize, int lengthTangent);


					 int extractLine(IplImage *imgInput, IplImage *imgOutput, std::vector<CvPoint> &lineRight, int searchLimitLeft, int searchLimitRight, bool debug, int minSize, int lengthTangent);

					 /**
					  * The function implements a scalar kalman filter
					  * @param double state_mu_prev: mean of the previous estimate
					  * @param double state_var_prev: variance of the previous estimate
					  * @param double measurement: the measurement
					  * @param double R: the variance of the noise variable
					  * @return double state_mu_estimate: mean of the calculated estimate
					  * @return double state_var_estimate: variance of the calculated estimate
					  */
					 void doKalmanFiltering(double state_mu_prev, double state_var_prev, double measurement,  double &state_mu_estimate, double &state_var_estimate );

};

/**
 *class Tangent houses handy functions that refer to tangents and derivatives
 */
class Tangent
{
		  private:
					 int WIDTH;
					 int HEIGHT;
		  protected:
		  public:
					 Tangent(int width, int height);
					 ~Tangent();
					 int getWidth();
					 int getHeight();
					 /**
					  *Calculates line parameters m and b for two given points. 
					  * To calculate line parameters for a line going through points, you
					  * need to consider that the y-values of the pixels are invers to the usual euclidean coordinate system.
					  * Y=0 denotes the top of the image. If you call this function you do NOT need to do this inversion prior to this function call yourself. 
					  * @ img: input Image
					  * @ p_1: the line goes from point p_1 to point p_2
					  * @ p_2: the line goes from point p_1 to point p_2
					  * @see calTangentParamsInv(CvPoint& p_1, CvPoint& p_2, double& m, double& b )
					  */
					 void calTangentParams(CvPoint& p_1, CvPoint& p_2, double& m, double& b );

					 /*
					  * Invert the y-values yourself!!
					  * @see calTangentParams(CvPoint& p_1, CvPoint& p_2, double& m, double& b );

*/
					 static void calTangentParamsInv(CvPoint& p_1, CvPoint& p_2, double& m, double& b );
					 void drawTangents(IplImage* img,  CvPoint& p_1,  CvPoint& p_2, int steps, int color=255);
					 void drawTangentsInv(IplImage* img,  CvPoint& p_1,  CvPoint& p_2, int steps, int color=255);
					 static void drawTangent(IplImage* img, CvPoint p_1, CvPoint p_2,  double m, double b, int length );
					 static double calTangentValue(double m, int x, double b);
};





#endif

