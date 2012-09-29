//LaneFinder.cpp
//#define DEBUG 0
#include "LaneFinder.h"

using namespace std;
#define M_PI 3.14159265358979323846


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
double angleBetweenLines(CvPoint start_line_1, CvPoint end_line_1, CvPoint start_line_2, CvPoint end_line_2)
{
	//cout<<"in between=: "<<start_line_1.x<<" "<<start_line_1.y<<" "<<end_line_1.x<<" "<<end_line_1.y<<" "<<start_line_2.x<<" "<<start_line_2.y<<" "<<end_line_2.x<<" "<<end_line_2.y<<endl;
	int x1=end_line_1.x - start_line_1.x;
	int y1=end_line_1.y - start_line_1.y;

	int x2=end_line_2.x - start_line_2.x;
	int y2=end_line_2.y - start_line_2.y;
	if(x1==0 && y1==0){
		//cout<<"Check if first two points are identical!\n";
		return 0.1;
	}
	if(x2==0 && y2==0){
        //cout<<"Check if last two points are identical!\n";
		return 0.1;
	}
	double a_l = sqrt(double(x1*x1+y1*y1));
	double b_l = sqrt(double(x2*x2+y2*y2));
	double ab= x1*x2 + y1*y2;

	double cos_alpha = ab / (a_l*b_l);
	double alpha = acos(cos_alpha);
	return alpha;
}

/**
 * returns the euclidean distance between two pixels
 */
double getDiffPoints(CvPoint const &p1, CvPoint const &p2)
{
	int diff_x = p1.x - p2.x;
	int diff_y = p1.y - p2.y;
	double dist = static_cast<double>((pow(double(diff_x), 2) + pow(double(diff_y), 2)));
	dist = sqrt(dist);
	return dist;
}

void fillImageWithColor(int colorOfImage, IplImage *nameOfImage)
{
	for(int i = 0; i < nameOfImage->width; i++){
		 for(int j = 0; j < nameOfImage->height; j++){
				CV_IMAGE_ELEM(nameOfImage, uchar, j, i) = colorOfImage;
		 }
	}
}

IplImage* createImage(int color, int width, int height)
{
	IplImage* img = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
	fillImageWithColor(color, img);
	return img;
}

void displayImage(IplImage* ImageToDisplay, char *NameOfWindow)//displayImageFunction
{
	cvNamedWindow( NameOfWindow, CV_WINDOW_AUTOSIZE );
	cvShowImage(NameOfWindow, ImageToDisplay);
	cvWaitKey(0);
	//cvWaitKey(200);
	cvDestroyWindow(NameOfWindow);
}

IplImage* createColorImage(int _width, int _height)
{
	int width= _width;
	int height= _height;

	IplImage* imgColor=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
	return imgColor;
}

/**
 * Constructor LaneFinder
 * @param width: the width of the image in which the line is to be found
 * @param height: the height of the image in which the line is to be found
 */
LaneFinder::LaneFinder(int width, int height)
{
	WIDTH=width;
	HEIGHT=height;
	STORAGE = cvCreateMemStorage(0);
}

/**
 * destructor
 */
LaneFinder::~LaneFinder()
{
	if(STORAGE != NULL)
	{
		 cvClearMemStorage( STORAGE );
		 cvReleaseMemStorage(&STORAGE);
		 STORAGE = NULL;
	}
}

/**
 * converts pixels from the extracted lane from the lineStruct to CvPoints
 */
void LaneFinder::convertLineStruct2Vec(std::vector< CvPoint > &vec, std::vector<struct line_points*> &lp)
{
	for(int i = 0; i< lp.size(); i++)
	{
		 CvPoint p=cvPoint(lp[i]->point.x,lp[i]->point.y);
		 vec.push_back(p);
	}
}

/**
 * writes a vector of CvPoints to a file with the given filename
 */
int LaneFinder::writeCurve( std::vector< CvPoint >  &curve, char *filename)
{
	ofstream file_out(filename);
	if(!file_out){
            cout << "file " << filename << "could not be opened!\n";
	    return -1;
	}
	char term1;
	char term2;
	double x, y;
	int ctr=0;
	for(int i=0; i< curve.size();i++){
	    file_out << curve[i].x << "," << curve[i].y << endl;
	}
	file_out.flush();
	file_out.close();
}

/**
 * writes a vector of CvPoints to a file with the given filename
 */
int LaneFinder::publishCurve( std::vector< CvPoint >  &curve)
{

    int portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    const char *hostname = "localhost";

	WORD wVersionRequested;
	WSADATA wsaData;
	int err;

	/* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
	wVersionRequested = MAKEWORD(2, 2);

	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		/* Tell the user that we could not find a usable */
		/* Winsock DLL.                                  */
		printf("WSAStartup failed with error: %d\n", err);
		return 1;
	}

    char buffer[256];
    portno = 5005;
    TCPBUS_SOCKFD = socket(AF_INET, SOCK_STREAM, 0);
    if (TCPBUS_SOCKFD < 0){
        cout << "ERROR: error opening socket\n";
        return(-1);
    }
    server = gethostbyname(hostname);
    if (server == NULL) {
	cout << "ERROR: host " << hostname << " could not be opened!\n";
        return(-1);
    }

	memset((char *) &serv_addr,0,sizeof(serv_addr));
    //bzreo((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(TCPBUS_SOCKFD,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
	cout << "ERROR: Error connecting to tcp bus\n";
        return(-1);
    }
    
    const char *busprefix = "lanefinder\r\nRightLane:";
    n = send(TCPBUS_SOCKFD,busprefix,strlen(busprefix),0);
    if (n < 0){
	cout << "ERROR: Error writing to the bus socket\n";
        return(-1);
    }

    for(int i=0; i< curve.size();i++){
	    sprintf(buffer,"%d,%d-",curve[i].x,curve[i].y);
//	    file_out<<curve[i].x<<"\t"<<curve[i].y<<endl;
        n = send(TCPBUS_SOCKFD,buffer,strlen(buffer),0);
        if (n < 0){
            cout << "ERROR: Error writing to the bus socket\n";
            return(-1);
        }
    }
    strcpy(buffer,"\r\n");
    n = send(TCPBUS_SOCKFD,buffer,strlen(buffer),0);
  
    if (!closesocket(TCPBUS_SOCKFD))
	TCPBUS_SOCKFD = -1;
		
    return 0;

}

/**
* fit a line based on points
* return true if is a horizontal line
*/
bool LaneFinder::isHorizontalLine(std::vector<struct line_points* >  &vec)
{
	std::vector<cv::Point> points;
	int size = vec.size();
	for (int i =0;i<size;++i)
	{
		points.push_back(cv::Point(vec[i]->point.x,vec[i]->point.y));
	}
	cv::Vec4f line;
	fitLine(cv::Mat(points),line,CV_DIST_L2,0,0.01,0.01);

	double alpha = atan2(line[1],line[0]);

	if (abs(alpha) < 0.08)
		return true;
	else
		return false;
}

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
//minSize: consider a line a line if it has minimum length of minSize 
void LaneFinder::findLines2(IplImage*img, std::vector<std::vector< struct line_points*>> &Lines, std::vector< std::vector< struct line_points* > > &bin, std::vector< std::vector< struct line_points* > > &binRight)
{
	if (bin.size()<=0&&binRight.size()<=0)
	{
		return;
	}
	int n=bin.size();
	for(int i=0; i<n;i++){
		if(bin[i].size() >5){
			if(isHorizontalLine(bin[i]))
			{
				std::vector< struct line_points*> line;
				for(int j=0; j< bin[i].size(); j++){
					struct line_points *p;
					copy_line_points_Struct(&p, bin[i][j]);
					line.push_back(p);
				}
				Lines.push_back(line);
			}
		}
	}

	n=binRight.size();
	for(int i=0; i<n;i++){
		if(binRight[i].size()>5){
			if(isHorizontalLine(binRight[i]))
			{
				std::vector< struct line_points*> line;
				for(int j=0; j< binRight[i].size(); j++){
					struct line_points *p;
					copy_line_points_Struct(&p, binRight[i][j]);
					line.push_back(p);
				}
				Lines.push_back(line);
			}
		}
	}


}
void LaneFinder::findLines(IplImage *img, IplImage *canny_img, double* alpha,int &x_pos, std::vector< struct line_points*> &longestLine, int lowerLimit, int higherLimit, int yLowerLimit,int yHigherLimit,  std::vector< std::vector< struct line_points* > > &bin, std::vector< std::vector< struct line_points* > > &binRight, int minSize, int lengthTangent)
{
    //cout<<"now in findLines\n";
	cvClearMemStorage( STORAGE );
	char image_name[300];
	static int ctr=0;
	ctr++;

	static double alpha_prev = 0;
	static int x_pos_prev = 0;
	//now that you have a frame, make an OpenCv image (IplImage) of it, because LaneFinder needs that
	//now get all points of all lines in the image and bin them
	int width=img->width;
	int height=img->height;

	//test begin  *****************************************************************************************
	//cout<<"size bin: "<<bin.size()<<endl;
	//int yLimit=height/2;//this is sth. that should also be filtered!!
	if(binRight.size()<=0 || bin.size()<=0){
	    //cout<<"skipping, bin is too big or empty"<<endl;
		*alpha = alpha_prev;
		x_pos = x_pos_prev;
		return;
	}
	//debugFindLines(binRight);
	//debugFindLines(bin);
	//check if findLongestLine is there in the desired area
	//find longest Line within a certain area in bin
	IplImage *colIM = createColorImage(getWidth(), getHeight());
	int sizeTmp=-1;
	int helper=-1;
	int n=bin.size();
	// cout<<"n: "<<n<<endl;
	// cout<<"higherLimit: "<<higherLimit<<endl;
	// cout<<"lowerLimit: "<<lowerLimit<<endl;
	// cout<<"yLimit: "<<yLimit<<endl;
	// cout<<"size left: "<<n<<endl;
	for(int i=0; i<n;i++){
	    int a=bin[i].size();
		//cout<<"sizeTmp: "<<sizeTmp<<endl;
		if(bin[i].size() >0){
			if(a> sizeTmp && (bin[i][0]->point.x<=higherLimit) && (bin[i][0]->point.x>=lowerLimit) &&  bin[i][0]->point.y > yLowerLimit && bin[i][0]->point.y < yHigherLimit && isHorizontalLine(bin[i]))
			{

				int j =0;
				int sum = 0;
				int numofout = 0;
				//for (;j<a;j++)
				//{
				//	sum = sum+ CV_IMAGE_ELEM(img,uchar,bin[i][j]->point.y+7,bin[i][j]->point.x) ;
				//	if (CV_IMAGE_ELEM(img,uchar,bin[i][j]->point.y,bin[i][j]->point.x) < CV_IMAGE_ELEM(img,uchar,bin[i][j]->point.y + 3,bin[i][j]->point.x))
				//	{
				//		numofout++;
				//	}
				//	//if(binRight[i][j]->point.y < yLowerLimit || binRight[i][j]->point.y > yHigherLimit)
				//	//break;
				//}
				//int average = sum/a;
				//if( average>190)
				//	continue;

				if( j== a )
				{
					sizeTmp=a;
					helper=i;
				}
			}
        }
	}
	//----now the other side------------------------------
	//find longest Line within a certain area in binRight
	int helperRight=-1;
	sizeTmp=-1;
	n=binRight.size();
	//cout<<"higherLimit: "<<higherLimit<<endl;
	//cout<<"lowerLimit: "<<lowerLimit<<endl;
	for(int i=0; i<n;i++){
	    int a=binRight[i].size();
		if(a > sizeTmp && (binRight[i][0]->point.x<=higherLimit) && (binRight[i][0]->point.x>=lowerLimit) &&  binRight[i][0]->point.y > yLowerLimit && binRight[i][0]->point.y < yHigherLimit&&isHorizontalLine(binRight[i]))
		{
			int j = 0;
			int sum = 0;
			int numofout = 0;
			//for (;j<a;j++)
			//{
			//	sum = sum + CV_IMAGE_ELEM(img,uchar,binRight[i][j]->point.y+7,binRight[i][j]->point.x) ;
			//	if (CV_IMAGE_ELEM(img,uchar,binRight[i][j]->point.y,binRight[i][j]->point.x) < CV_IMAGE_ELEM(img,uchar,binRight[i][j]->point.y + 3,binRight[i][j]->point.x))
			//	{
			//		numofout++;
			//	}
			//	//if(binRight[i][j]->point.y < yLowerLimit || binRight[i][j]->point.y > yHigherLimit)
			//		//break;
			//}
			//int average = sum/a;
			//if( average > 190)
			//	continue;
			if(j == a)
			{
				sizeTmp=a;
				helperRight=i;
			}
		}
	}

    int l_r=-1;
	if(helper>=0 || helperRight>=0){
		if(helper>=0 && helperRight>=0){
			if(bin[helper].size()>=binRight[helperRight].size()){

				  //copy into longestLine
				  for(int i=0; i< bin[helper].size(); i++){
					 struct line_points *p;
					 copy_line_points_Struct(&p, bin[helper][i]);
					 longestLine.push_back(p);
				  }
				  l_r=1;//next check binRight
			}
			else {
				  //copy into longestLine
				  for(int i=0; i< binRight[helperRight].size(); i++){
					 struct line_points *p;
					 copy_line_points_Struct(&p, binRight[helperRight][i]);
					 longestLine.push_back(p);
 		   }
		   l_r=2;//next check bin
		}
	 } else if(helper >=0 && helperRight<0){
		//copy into longestLine
		for(int i=0; i< bin[helper].size(); i++){
		  struct line_points *p;
		  copy_line_points_Struct(&p, bin[helper][i]);
		  longestLine.push_back(p);
		}
		l_r=1;//next check binRight
	 } else if(helperRight >=0 && helper <0){
		//cout<<"hier auch nicht\n";
		for(int i=0; i< binRight[helperRight].size(); i++){
		  struct line_points *p;
		  copy_line_points_Struct(&p, binRight[helperRight][i]);
		  longestLine.push_back(p);
		}
		l_r=2;//next check bin

	 }
	 CvPoint ep = longestLine[longestLine.size()-1]->point;
	 bool notdone=true;
	 int diff=40;
	 while(notdone){
		 
		notdone=false;
		int binsize=longestLine.size();	
		if(l_r%2==0){//l_r==even -> check bin

		  //doJoiningTwoOrient(ep, bin, longestLine, diff);
		  //doJoiningTwoOrientH(ep,bin,longestLine,diff);
			doSelfJoining(ep,binRight,longestLine,diff,'R');
		  //l_r++;
		}
		else//l_r odd -> check binRight
		{
		  //doJoiningTwoOrient(ep, binRight, longestLine, diff);
			doSelfJoining(ep,bin,longestLine,diff,'L');
		  //l_r++;
		}

		diff += 10;
		if(diff>=80){break;}
		if(binsize< longestLine.size())
		{
		  notdone=true;
		}
		else {
		  notdone=false;
		}
	 }

  }
	  if(longestLine.size()>minSize){
		  
		 int highest_y_index;
		 double m, b, angle;
		 CvPoint startpoint, endpoint;
		 getTangentParams(longestLine, highest_y_index, m, b, angle, startpoint, endpoint, lengthTangent, img->width, img);
		 if(startpoint.y < img->height-3  && startpoint.x < img->width)//the extrapolation routine copies a lot, do not call it, if not necessary
		 {
			//doExtrapolation(longestLine, canny_img,  m, b, startpoint);
		 }
		 getTangentParams(longestLine, highest_y_index, m, b, angle, startpoint, endpoint, lengthTangent, img->width, img);
		 *alpha = angle;
		 x_pos = startpoint.x;
		 alpha_prev = angle;
		 x_pos_prev = x_pos;

	  }
	  else { 
		 for(int g=0; g < longestLine.size(); g++)
		 {
					longestLine.pop_back();
		 }
		 longestLine.clear();
		 cout<<"skipping, No Longest Line found"<<endl;
		 *alpha = -1;//alpha_prev;
		 x_pos = -1;//x_pos_prev;
	  }
	  if(colIM !=NULL){
		  
		 cvReleaseImage(&colIM);
		 colIM=NULL;
	  } 
}

/**
 * takes an canny filtered image and creates line segments that start left and go to right
 * You can change canny parameters here!
 * @see binLinesRight(vector< vector< struct line_points* > > &bin, IplImage* img, IplImage* canny_img, std::string cannyParam)
 */
void LaneFinder::binLines(vector< vector< struct line_points* > > &bin, IplImage* img, IplImage* canny_img, std::string cannyParam)
{
		  cvClearMemStorage( STORAGE );
		  //IplImage* orient = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_32F, 1); 
		  static IplImage* orient = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_32F, 1); 
		  //-------------------------get all points of all lines and collect them in a vector-------------------------
		  //get Orientation
		  //allocate memory and then delete it at the end of the loop
		  //static IplImage* orient = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_32F, 1); 
		  //static IplImage* orient = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_32F, 1); 
		  //cout<<"bis hier in binLines?\n";
		  getOrientationImage(img,orient);	
		  //cvSaveImage("orient.pgm", orient);
		  // canny
		  //cout<<"bis hier in binLines nach getOrientation?\n";
		  //kommt bis hierhin, danach gibts keinen Speicher mehr!!
		  //cout<<"bin Lines: width: "<<img->width<<endl;
		  //cout<<"bin Lines: height: "<<img->height<<endl;
		  if(cannyParam.compare("lab")==0)
		  {
					 //cvCanny(img, canny_img, 50,150 );
					 cvCanny(img, canny_img, 100,150 );
					 //displayImage(canny_img,"canny");
					 //cvSaveImage("canny.pgm", img);
		  }//for IR
		  else if(cannyParam.compare("ir")==0)
		  { 
					 cvCanny(img, canny_img, 100, 255 );
		  }
		  else if(cannyParam.compare("real")==0)
		  {
						IplConvKernel * shape = cvCreateStructuringElementEx(2,2,0,0,CV_SHAPE_RECT);
						IplImage * temp = cvCreateImage(cvGetSize(img), 8,1);    
						IplImage * imgM=cvCreateImage(cvGetSize(img), 8, 1);
						cvMorphologyEx(img,imgM,temp,shape,CV_MOP_CLOSE,5);
						cvShowImage("AAA", imgM);
						cvWaitKey(0);
					 cvCanny(imgM, canny_img, 200, 60, 3 );
					 //cout<<"you are choosing REAL\n";
					 //cvCanny(img, canny_img, 2000, 2300, 5 );
					 displayImage(canny_img, "canny");
		  }
		  else
		  {
					 cout<<"please use a valid flag for string cannyParam, valid values are: lab, ir or real\n";
		  }
		  int pic_canny = -2;

		  char img_title[100];
		  sprintf(img_title, "Approx_%d.pgm", 5);


		  for(int b=img->height-1; b>1; b--)
		  {
					 for(int a=img->width-1; a>=0; a--)
					 {
								pic_canny = (int)(CV_IMAGE_ELEM(canny_img, uchar, b,a));
								if(pic_canny == 255 )
								{

										  struct line_points *p;
										  vector<struct line_points*> points;
										  CV_IMAGE_ELEM(canny_img, uchar, b,a)=(uchar)70;//mark as read 
										  p = (struct line_points *) malloc(sizeof(struct line_points));
										  if(p==NULL){cout<<"no more memory, exiting\n";exit(-1);}
										  p->point = cvPoint(a,b);
										  float orientation_curr = (float) CV_IMAGE_ELEM(orient, float,b,a);
										  p->orientation = orientation_curr;
										  p->endpoint = false;
										  p->startpoint = true;
										  p->no_neighbour = false;
										  points.push_back(p);
										  getNeighboursOrdered(canny_img,a,b,points, orient, (int)orientation_curr);
										  if(points.size() >= 10)
										  {
													 bin.push_back(points);
													 //debug start
													 /*for(int im=0;im<points.size();im++)
														{
														float imVal=points[im]->orientation;
														if(imVal != 0 && imVal != 90 && imVal!= 135 &&imVal!= 45)
														{
														cout<<"in binLines: "<<imVal<<endl;
														}
														}*/
													 //debug end

										  }
										  else
										  {
													 for(int c = 0; c < points.size(); c++)
													 {
																//make these pixels black in the image -> denoising
																CV_IMAGE_ELEM(canny_img, uchar, points[c]->point.y, points[c]->point.x)=(uchar)0;//mark as read 

																if(points[c]!=NULL)
																{
																		  free(points[c]);
																		  points[c]=NULL;
																}
													 }
													 points.clear();
										  }
								}
					 }
		  }
}

/**
 * takes an image, returns a canny filtered image and creates line segments that start right and go to left.
 * You can change canny parameters here!
 * @see binLines(vector< vector< struct line_points* > > &bin, IplImage* img, IplImage* canny_img, std::string cannyParam)
 */
void LaneFinder::binLinesRight(vector< vector< struct line_points* > > &bin, IplImage* img, IplImage* canny_img, std::string cannyParam)
{
		  //ofstream measureCannyFile("measureCanny.txt", ios::app);
		  /*if(!measureCannyFile)
		  {
					 cout<<"stupid, cannot open file\n";
					 exit(-1);
		  }*/

		  //-------------------------get all points of all lines and collect them in a vector-------------------------
		  //get Orientation
		  //allocate memory and then delete it at the end of the loop
		  static IplImage* orient = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_32F, 1); 

		  getOrientationImage(img,orient);	

		  // canny
		  //cout<<"bin Lines: width: "<<img->width<<endl;
		  //cout<<"bin Lines: height: "<<img->height<<endl;
		  if(cannyParam.compare("lab")==0)
		  {
					 cvCanny(img, canny_img, 100,150 );

		  }//for IR
		  else if(cannyParam.compare("ir")==0)
		  { 
					 cvCanny(img, canny_img, 100, 255 );
		  }
		  else if(cannyParam.compare("real")==0)
		  { 
			  IplConvKernel * shape = cvCreateStructuringElementEx(2,2,0,0,CV_SHAPE_RECT);
			  IplImage * temp = cvCreateImage(cvGetSize(img), 8,1);    
			  IplImage * imgM=cvCreateImage(cvGetSize(img), 8, 1);
			  cvMorphologyEx(img,imgM,temp,shape,CV_MOP_CLOSE,5);
			  //cvShowImage("BBB", imgM);
			  cvWaitKey(0);
					 cvCanny(imgM, canny_img, 1500, 2300, 5 );
					 //cvCanny(img, canny_img, 500, 100, 3 );
					 displayImage(canny_img, "canny2");
		  }
		  else
		  {
					 cout<<"please use a valid flag for string cannyParam, valid values are: lab, ir or real\n";
		  }
		  int pic_canny = -2;
		  char img_title[100];
		  sprintf(img_title, "Approx_%d.pgm", 5);

		  for(int b=img->height-1; b>1; b--)
		  {
					 for(int a=0; a<img->width; a++)
					 {
								pic_canny = (int)(CV_IMAGE_ELEM(canny_img, uchar, b,a));
								if(pic_canny == 255 )
								{
										  struct line_points *p;
										  vector<struct line_points*> points;
										  CV_IMAGE_ELEM(canny_img, uchar, b,a)=(uchar)70;//mark as read 
										  p = (struct line_points *) malloc(sizeof(struct line_points));
										  if(p==NULL){cout<<"no more memory, exiting\n";exit(-1);}
										  p->point = cvPoint(a,b);
										  float orientation_curr = (float) CV_IMAGE_ELEM(orient, float,b,a);
										  p->orientation = orientation_curr;
										  p->endpoint = false;//weiss man auch noch nicht
										  p->startpoint = true;
										  p->no_neighbour = false;//weiss man ja noch gar nicht
										  points.push_back(p);
										  getNeighboursOrderedRight(canny_img,a,b,points, orient, (int)orientation_curr);
										  if(points.size() >= 10)//if(points.size() >= 2)//this value is connected to forSlope in doJoining ->3!!
										  {
													 bin.push_back(points);
										  }
										  else
										  {
													 for(int c = 0; c < points.size(); c++)
													 {
																if(points[c]!=NULL)
																{
																		  //make these pixels black!
																		  CV_IMAGE_ELEM(canny_img, uchar, points[c]->point.y, points[c]->point.x)=(uchar)0;//mark as read 
																		  free(points[c]);
																		  points[c]=NULL;
																}
													 }
													 points.clear();
										  }
								}
					 }
		  }
}

void LaneFinder::copy_line_points_Struct(line_points **p_target, line_points* p_src)
{
		  *p_target = (struct line_points*) malloc(sizeof(struct line_points));
		  (*p_target)->point = p_src->point;

		  (*p_target)->orientation = p_src->orientation;
		  (*p_target)->endpoint = p_src->endpoint;
		  (*p_target)->startpoint = p_src->startpoint;
		  (*p_target)->no_neighbour = p_src->no_neighbour;
}
bool LaneFinder::shouldBeOneLine(std::vector< struct line_points*> &first,std::vector< struct line_points*> &second)
{
	if (first.empty()||second.empty())
	{
		return false;
	}
	int length = first.size();
	CvPoint sp=first[0]->point;
	CvPoint ep=first[length-1]->point;

	int clength = second.size();
	CvPoint csp=second[0]->point;
	CvPoint cep=second[clength-1]->point;

	if ( isPointsConnected(ep ,csp)  || isPointsConnected(ep ,cep) || isPointsConnected(sp ,csp) ||isPointsConnected(ep ,cep) )
	{
		double angle = angleBetweenLines(sp,ep,csp,cep);
		if (angleBetweenLines(sp,ep,csp,cep) < 0.1)
		{
			return true;
		}
	}

	return false;

	
}
void LaneFinder::connectLines(std::vector< std::vector< struct line_points* > > &lines, std::vector< std::vector< struct line_points* > > &stoplines)
{
	for (int i =0;i<lines.size();i++)
	{
		if (lines[i].empty())
			continue;
		for (int j=i+1;j<lines.size();j++)
		{
			if(shouldBeOneLine(lines[i],lines[j]))
			{
				for(std::vector< struct line_points* >::iterator m=lines[j].begin(); m!=lines[j].end();m++ )
				{
					struct line_points *p;
					copy_line_points_Struct(&p, *m);
					lines[i].push_back(p);
				}
				lines[j].clear();
				
			}
		}
	}

	int maxLength = 0;
	int max = 0;
	for (int i =0;i<lines.size();i++)
	{
		if (lines[i].size()>maxLength)
		{
			maxLength = lines[i].size();
			max = i;
		}
	}

	int secondLength = 0;
	int secondMax = 0;
	for (int i =0;i<lines.size();i++)
	{
		if (i!=max&&lines[i].size()>secondLength)
		{
			secondLength = lines[i].size();
			secondMax = i;
		}
	}

	if (maxLength > 180 && abs(lines[max][0]->point.y - lines[secondMax][0]->point.y)<20)
	{
		stoplines.push_back(lines[max]);
		stoplines.push_back(lines[secondMax]);
	}
}

void LaneFinder::doSelfJoining(CvPoint ep,std::vector< std::vector< struct line_points* > > &bin,std::vector< struct line_points*> &longestLine,int dist,char orientation)
{
	int l=10;
	double maxAngle = M_PI/3;//0.6;//in radian [0, pi];//we seek the segment with the greates radian


	double minDist=1000;
	double minDistB=1000;
	int inx=-1;
	int inxB=-1;
	//check if there is a pixel close to end of longestLine and if it is good continued
	for(int i=0; i<bin.size();i++)
	{
		if( longestLine.size() > l)
		{
			CvPoint sp=bin[i][0]->point;
			CvPoint cp = longestLine[longestLine.size()-l]->point;
			ep = longestLine[longestLine.size()-1]->point;
			if(((orientation=='L' && sp.x <= ep.x+10)||(orientation=='R'&&sp.x >= ep.x-10))&&abs(sp.y-ep.y)<4)
			{
				if(!isHorizontalLine(bin[i]))
					continue;

				double diff =distBetweenPoints(ep ,sp);
				if(diff<minDist)
				{
					double angle = angleBetweenLines(ep, cp, ep, sp);
					if(angle==0 && diff< dist && diff<minDistB)
					{
						inxB =i;
						maxAngle=M_PI/2+0.1;//0.9;
						minDistB=diff;
					}
					else if(diff <minDist)
					{
						inx =i;
						minDist=diff;
					}
					else if(angle > maxAngle && diff < dist)
					{
						maxAngle=angle;
						inx =i;
						minDist=diff;
					}
				}
			}
		}
	}
	int matchingSeg=-1;
	if(minDist<=16){matchingSeg = inx;}
	else if(maxAngle >0.9 ){matchingSeg=inxB;}
	else if(minDist<dist)
	{
		CvPoint ep1=longestLine[longestLine.size()-1]->point;
		CvPoint ep2= longestLine[longestLine.size()-l]->point;
		CvPoint ep3=bin[inx][bin[inx].size()-1]->point;
		double angleTmp=angleBetweenLines(ep1, ep2, ep1, ep3);
		if(angleTmp > 0.4)
		{	
			matchingSeg=inx;
		}
	}
	if(matchingSeg==-1)
	{
		return;
	}
	int minDist2=1000;
	int inx2=-1;
	for(int i=0; i<bin[matchingSeg].size();i++)
	{
		double diff2=distBetweenPoints(bin[matchingSeg][i]->point, ep);
		if(diff2<minDist2)
		{
			minDist2=static_cast<int>(floor(diff2));
			inx2=i;
		}
		if(diff2==0)
		{inx2 ++; break;}
	}
	if(inx2 > 0 && inx2<bin[matchingSeg].size())
	{
		for(int i=0; i<bin[matchingSeg].size()-inx2; i++)
		{
			struct line_points *p;
			copy_line_points_Struct(&p, bin[matchingSeg][inx2+i]);
			longestLine.push_back(p);
		}
	}
	else
	{
		for(int i=0; i<bin[matchingSeg].size(); i++)
		{
			struct line_points *p;
			copy_line_points_Struct(&p, bin[matchingSeg][i]);
			longestLine.push_back(p);
		}
	}
	return;
}

/**
* the horizontal version for doJoiningTwoOrient
*/
void LaneFinder::doJoiningTwoOrientH(CvPoint ep, std::vector< std::vector< struct line_points* > > &bin, std::vector< struct line_points*> &longestLine, int dist )
{
	int l=10;
	double maxAngle = M_PI/3;//0.6;//in radian [0, pi];//we seek the segment with the greates radian


	double minDist=1000;
	double minDistB=1000;
	int inx=-1;
	int inxB=-1;
	//check if there is a pixel close to end of longestLine and if it is good continued
	for(int i=0; i<bin.size();i++)
	{
		if( longestLine.size() > l)
		{
			CvPoint sp=bin[i][0]->point;
			CvPoint cp = longestLine[longestLine.size()-l]->point;
			ep = longestLine[longestLine.size()-1]->point;
			if(sp.x >= ep.x)
			{
				vector< struct line_points*>::iterator iter=bin[i].begin();
				while((*iter)->point.x >=ep.x)
				{
					sp=(*iter)->point;
					if((iter+1) != bin[i].end())
					{
						iter++;
					}
					else
					{
						break;
					}
				}
			}
			double diff =distBetweenPoints(ep ,sp);
			if(diff<minDist)
			{
				double angle = angleBetweenLines(ep, cp, ep, sp);
				if(angle==0 && diff< dist && diff<minDistB)
				{
					inxB =i;
					maxAngle=M_PI/2+0.1;//0.9;
					minDistB=diff;
				}
				else if(diff <minDist)
				{
					inx =i;
					minDist=diff;
				}
				else if(angle > maxAngle && diff < dist)
				{
					maxAngle=angle;
					inx =i;
					minDist=diff;
				}
			}
		}
	}
	int matchingSeg=-1;
	if(minDist<=2){matchingSeg = inx;}
	else if(maxAngle >0.9 ){matchingSeg=inxB;}
	else if(minDist<dist)
	{
		CvPoint ep1=longestLine[longestLine.size()-1]->point;
		CvPoint ep2= longestLine[longestLine.size()-l]->point;
		CvPoint ep3=bin[inx][bin[inx].size()-1]->point;
		double angleTmp=angleBetweenLines(ep1, ep2, ep1, ep3);
		if(angleTmp > 0.4)
		{	
			matchingSeg=inx;
		}
	}
	if(matchingSeg==-1)
	{
		return;
	}
	int minDist2=1000;
	int inx2=-1;
	for(int i=0; i<bin[matchingSeg].size();i++)
	{
		double diff2=distBetweenPoints(bin[matchingSeg][i]->point, ep);
		if(diff2<minDist2)
		{
			minDist2=static_cast<int>(floor(diff2));
			inx2=i;
		}
		if(diff2==0)
		{inx2 ++; break;}
	}
	if(inx2 > 0 && inx2<bin[matchingSeg].size())
	{
		for(int i=0; i<bin[matchingSeg].size()-inx2; i++)
		{
			struct line_points *p;
			copy_line_points_Struct(&p, bin[matchingSeg][inx2+i]);
			longestLine.push_back(p);
		}
	}
	else
	{
		for(int i=0; i<bin[matchingSeg].size(); i++)
		{
			struct line_points *p;
			copy_line_points_Struct(&p, bin[matchingSeg][i]);
			longestLine.push_back(p);
		}
	}
	return;
}


/**
 * check if the given point is close (dist) to some starting point in the given bin.
 * if so find the point in the bin that is closest to ep and from there on push back everything into longestLine and clear it in bin
 * not only the closeness is checked, also good continuation in form of an angle
 */
void LaneFinder::doJoiningTwoOrient(CvPoint ep, std::vector< std::vector< struct line_points* > > &bin, std::vector< struct line_points*> &longestLine, int dist )
{
		  int l=10;
		  double maxAngle = M_PI/3;//0.6;//in radian [0, pi];//we seek the segment with the greates radian


		  double minDist=1000;
		  double minDistB=1000;
		  int inx=-1;
		  int inxB=-1;
		  //check if there is a pixel close to end of longestLine and if it is good continued
		  for(int i=0; i<bin.size();i++)
		  {
					 if( longestLine.size() > l)
					 {
								CvPoint sp=bin[i][0]->point;
								CvPoint cp = longestLine[longestLine.size()-l]->point;
								ep = longestLine[longestLine.size()-1]->point;
								if(sp.y >= ep.y)
								{
										  vector< struct line_points*>::iterator iter=bin[i].begin();
										  while((*iter)->point.y >=ep.y)
										  {
													 sp=(*iter)->point;
													 if((iter+1) != bin[i].end())
													 {
																iter++;
													 }
													 else
													 {
																break;
													 }
										  }
								}
								double diff =distBetweenPoints(ep ,sp);
								if(diff<minDist)
								{
										  double angle = angleBetweenLines(ep, cp, ep, sp);
										  if(angle==0 && diff< dist && diff<minDistB)
										  {
													 inxB =i;
													 maxAngle=M_PI/2+0.1;//0.9;
													 minDistB=diff;
										  }
										  else if(diff <minDist)
										  {
													 inx =i;
													 minDist=diff;
										  }
										  else if(angle > maxAngle && diff < dist)
										  {
													 maxAngle=angle;
													 inx =i;
													 minDist=diff;
										  }
								}
					 }
		  }
		  int matchingSeg=-1;
		  if(minDist<=5){matchingSeg = inx;}
		  else if(maxAngle >0.9 ){matchingSeg=inxB;}
		  else if(minDist<dist)
		  {
					 CvPoint ep1=longestLine[longestLine.size()-1]->point;
					 CvPoint ep2= longestLine[longestLine.size()-l]->point;
					 CvPoint ep3=bin[inx][bin[inx].size()-1]->point;
					 double angleTmp=angleBetweenLines(ep1, ep2, ep1, ep3);
					 if(angleTmp > 0.4)
					 {	
								matchingSeg=inx;
					 }
		  }
		  if(matchingSeg==-1)
		  {
					 return;
		  }
		  int minDist2=1000;
		  int inx2=-1;
		  for(int i=0; i<bin[matchingSeg].size();i++)
		  {
					 double diff2=distBetweenPoints(bin[matchingSeg][i]->point, ep);
					 if(diff2<minDist2)
					 {
								minDist2=static_cast<int>(floor(diff2));
								inx2=i;
					 }
					 if(diff2==0)
					 {inx2 ++; break;}
		  }
		  if(inx2 > 0 && inx2<bin[matchingSeg].size())
		  {
					 for(int i=0; i<bin[matchingSeg].size()-inx2; i++)
					 {
								struct line_points *p;
								copy_line_points_Struct(&p, bin[matchingSeg][inx2+i]);
								longestLine.push_back(p);
					 }
		  }
		  else
		  {
					 for(int i=0; i<bin[matchingSeg].size(); i++)
					 {
								struct line_points *p;
								copy_line_points_Struct(&p, bin[matchingSeg][i]);
								longestLine.push_back(p);
					 }
		  }
		  return;
}


/**Actually it only checks which start- and endpoint can be used, but 'calTAngentParams' takes care of all special 
 * cases, like vertical line, is enpoint<startpoint etc.
 * Finds out which two points to use from the line to calculate m and b, this is done seperately by calTangentParams
 */
void LaneFinder::getTangentParams(std::vector< struct line_points*> &longestLine,  int &highest_y_index, double &m, double &b, double &angle, CvPoint &startpoint, CvPoint &endpoint, int length, int img_width, IplImage *img)
{
		  int multiplier=1;//if the line is the other way round, you must invert the angle
		  getHighestYIndex(longestLine, highest_y_index, startpoint);
		  CvPoint p_1 = startpoint;
		  CvPoint p_2 = cvPoint(-1,-1);

		  int tol = length;
		  if( highest_y_index - tol > 0 && highest_y_index -tol <longestLine.size())
		  {
					 p_2 = longestLine[highest_y_index - tol]->point;
		  }
		  if(p_2.x==-1)
		  {
					 if( highest_y_index + tol > 0 && highest_y_index +tol <longestLine.size())
					 {
								p_2 = longestLine[highest_y_index + tol]->point;
					 }
		  }
		  if(p_1.y >= p_2.y){ /*cout<<"or just here?\n";*/startpoint = p_1; endpoint = p_2;}
		  else{ /*cout<<"am i here?\n";*/startpoint = p_2; endpoint = p_1; multiplier=-1;} 
		  if(startpoint.x == endpoint.x)
		  {
					 if(startpoint.x -1 >0 && startpoint.x -1<img_width)
					 {
								startpoint.x -= 1;
					 }
					 else
					 {
								startpoint.x += 1;
					 }
		  }
		  Tangent *myTangent = new Tangent(getWidth(), getHeight());
		  myTangent->calTangentParams(startpoint, endpoint, m, b);

		  angle=atan2(double((getHeight()-endpoint.y)- (getHeight()-startpoint.y)), double(endpoint.x-startpoint.x));
		  angle=(angle/M_PI)*180;//change to deg
		  angle -= 90;
		  angle = angle *-1;
		  if(endpoint.x==-1 && endpoint.y ==-1)
		  {
					 angle=-1000;
					 cout<<" angle could not be calculated! \n";
					 exit(-1);
		  }
		  m = m*multiplier;
}

/**
 * Sometimes a line is extracted that does not start in the first row at the image bottom. To extend it to the bottom artificially this
 * function extends the tangent of the beginning of the extracted lane to the image bottom. 
 */
void LaneFinder::doExtrapolation(std::vector< struct line_points*> &longestLine, IplImage* img, double m, double b, CvPoint &point_y)
{
		  longestLine[0]->startpoint=false;
		  //then copy longest line to a tmp vector:
		  vector<struct line_points*> tmp_longestLine;
		  int s= longestLine.size();
		  for(int i=0; i< s; i++)
		  {
					 tmp_longestLine.push_back(longestLine.back());
					 longestLine.pop_back();
		  }
		  CvPoint p_1 = point_y;
		  int y_start = point_y.y;
		  int y_diff = img->height;
		  for(int y=y_start; y<img->height; y++)
		  {
					 int x = static_cast<int>(floor((y-b)/(m)));

					 if(x>0 && x < img->width && y>0 && y<img->height)
					 {
								line_points* points = (line_points*) malloc(sizeof(line_points));
								points->point=cvPoint(x,y);
								longestLine.push_back(points); //
					 }
		  }
		  //now copy everything back
		  int tmps=tmp_longestLine.size();
		  for(int i=0; i<tmps; i++)
		  {
					 longestLine.push_back(tmp_longestLine.back());
					 tmp_longestLine.pop_back();
		  }
		  //delete tmp
		  tmp_longestLine.clear();
		  //again set startpoint

		  point_y = longestLine[longestLine.size()-1]->point;
		  longestLine[0]->startpoint=true;

}

/**
 * calculates pixel orientations
 */
void LaneFinder::getOrientationImage(IplImage *img, IplImage* &orientation_img)
{
		  //if you want to make this faster you should change the code in OpenCV cvcann.cpp
		  static IplImage* sobel_dx = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_16S, 1); 
		  static IplImage* sobel_dy = cvCreateImage( cvSize(img->width,img->height), IPL_DEPTH_16S, 1); 

		  cvSobel(img, sobel_dx, 1, 0);//----------------------------
		  cvSobel(img, sobel_dy, 0, 1);//----------------------------

		  //displayImage(sobel_dx, "dx");
		  //displayImage(sobel_dy,"dy");

		  int dx_pic=-1;
		  int dy_pic=-1;
		  float orient=-1;

		  for(int i = 0; i < orientation_img->width; i++)
		  {
					 for(int j = 0; j < orientation_img->height; j++)
					 {
								dy_pic = (short int)(CV_IMAGE_ELEM(sobel_dy, short int, j,i));
								//do this only, if there is an edge!
								if(dx_pic != 0 || dy_pic != 0)
								{
										  if(dx_pic == 0 && dy_pic != 0)//
										  {
													 orient = 90;
										  }
										  if(dx_pic != 0 && dy_pic != 0)//atan kann beruhigt berechnet werden
										  {
													 orient = atan2( (float)(dy_pic) , (float)(dx_pic) );
													 orient = ((180/3.141592653)*orient);
										  }
										  //now discretize
										  float dummy = orient;
										  if((dummy >=0.0 && dummy <22.5) || (dummy <0.0 && dummy >-22.5))
										  {
													 orient=90;
										  }
										  if((dummy >= 157.5 && dummy<= 180.0) || (dummy <= -157.5 && dummy>= -180.0) )
										  {
													 orient=90;
										  }
										  if((dummy >= 22.5 && dummy < 67.5) || (dummy <= -112.5 && dummy > -157.5))
										  {
													 orient = 45;
										  }
										  if((dummy >= 67.5  && dummy< 112.5)  || (dummy <= -67.5  && dummy> -112.5))
										  {
													 orient = 0;
										  }
										  if((dummy >= 112.5 && dummy< 157.5) || (dummy <= -22.5 && dummy> -67.5) )
										  {
													 orient = 135;
										  }
										  CV_IMAGE_ELEM(orientation_img, float,j,i)= orient;
								}
								else//if(dx_pic != 0 || dy_pic != 0)
								{
										  CV_IMAGE_ELEM(orientation_img, float,j,i)= 0;
								}
					 }
		  }
}

void LaneFinder::getNeighboursOrderedRight(IplImage* img, int x, int y, std::vector<struct line_points*> &points, IplImage* orientation_img, int orientation)
{

		  int dist=2;
		  if((x - dist/2) > 0 && (x + dist/ 2) < img->width && (y- dist/2) > 0 && (y + dist/ 2) < img->height)
		  {
					 //-------------------1------x+1, y-------------------------
					 int pic_val_1 = CV_IMAGE_ELEM(img, uchar, y,x+1);
					 if(pic_val_1 == 255)//if this point has a neighbour in 8 neighbourhood
					 {
								//then check the orientation
								if((int)CV_IMAGE_ELEM(orientation_img, float,y,x+1) == orientation)
								{
										  struct line_points *p;
										  p = (struct line_points *) malloc(sizeof(struct line_points));
										  p->point = cvPoint(x+1,y);
										  float orientation_curr = (float) CV_IMAGE_ELEM(orientation_img, float,y,x+1);
										  p->orientation = orientation_curr;
										  p->endpoint = false;
										  p->startpoint = false;
										  p->no_neighbour = false;
										  points.push_back(p);
										  CV_IMAGE_ELEM(img, uchar, y,x+1)=(uchar)70;//mark as read 
										  getNeighboursOrderedRight(img,x+1, y, points, orientation_img, (int)orientation_curr);
										  return;
								}
					 }	
					 //-------------------3------x, y-1-------------------------
					 int pic_val_3 = CV_IMAGE_ELEM(img, uchar, y-1,x);
					 if(pic_val_3 == 255)
					 {
								if((int)CV_IMAGE_ELEM(orientation_img, float,y-1,x) == orientation)
								{
										  struct line_points *p;
										  p = (struct line_points *) malloc(sizeof(struct line_points));
										  p->point = cvPoint(x, y-1);
										  float orientation_curr = (float) CV_IMAGE_ELEM(orientation_img, float,y-1,x);
										  p->orientation = orientation_curr;
										  p->endpoint = false;
										  p->startpoint = false;
										  p->no_neighbour = false;
										  points.push_back(p);///
										  CV_IMAGE_ELEM(img, uchar, y-1,x)=(uchar)70;//als gelesen markieren
										  getNeighboursOrderedRight(img,x, y-1, points, orientation_img, (int)orientation_curr);
										  return;
								}
					 }
					 //-------------------2------x+1, y-1-------------------------
					 int pic_val_2 = CV_IMAGE_ELEM(img, uchar, y-1,x+1);
					 if(pic_val_2 == 255)
					 {
								if((int)CV_IMAGE_ELEM(orientation_img, float,y-1,x+1) == orientation)
								{
										  struct line_points *p;
										  p = (struct line_points *) malloc(sizeof(struct line_points));
										  p->point = cvPoint(x+1, y-1);
										  float orientation_curr = (float) CV_IMAGE_ELEM(orientation_img, float,y-1,x+1);
										  p->orientation = orientation_curr;
										  p->endpoint = false;
										  p->startpoint = false;
										  p->no_neighbour = false;
										  points.push_back(p);
										  CV_IMAGE_ELEM(img, uchar, y-1,x+1)=(uchar)70;//als gelesen markieren
										  getNeighboursOrderedRight(img,x+1, y-1, points, orientation_img, (int)orientation_curr);
										  return;
								}
					 }	
					 static CvPoint oldPoint= cvPoint(-1,-1);
					 static int pointCtr=0;
					 if(oldPoint.x == x && oldPoint.y ==y)
					 {
								pointCtr++;
					 }
					 else
					 {
								pointCtr=0;
					 }
					 oldPoint= cvPoint(x,y);
					 int orientation_new;

					 if(pointCtr < 4)
					 {
								switch(orientation)
								{
										  case 0: 
													 orientation_new = 45;
													 getNeighboursOrderedRight(img,x, y, points, orientation_img, orientation_new);
													 break;

										  case 45:
													 orientation_new = 90;
													 getNeighboursOrderedRight(img,x, y, points, orientation_img, orientation_new);
													 break;

										  case 90:
													 orientation_new=135;
													 getNeighboursOrderedRight(img,x, y, points, orientation_img, orientation_new);
													 break;

										  case 135:
													 orientation_new=0;
													 getNeighboursOrderedRight(img,x, y, points, orientation_img, orientation_new);
													 break;
													 //default: points[points.size()-1]->endpoint=true;
													 break;
								}
								return;
					 }

					 else
					 {
								points[points.size()-1]->endpoint=true;
								return;
					 }
		  }//end if not an edge point
		  else
		  {
					 points[points.size()-1]->endpoint = true;
					 return;
		  }
		  return;
}

void LaneFinder::getNeighboursOrdered(IplImage* img, int x, int y, std::vector<struct line_points*> &points, IplImage* orientation_img, int orientation)
{

		  int dist=2;
		  if((x - dist/2) > 0 && (x + dist/ 2) < img->width && (y- dist/2) > 0 && (y + dist/ 2) < img->height)//prüfen, ob der nächste Pixel auch noch im Bild ist
		  {

					 //-------------------3------x, y-1-------------------------
					 int pic_val_3 = CV_IMAGE_ELEM(img, uchar, y-1,x);
					 if(pic_val_3 == 255)
					 {
								if((int)CV_IMAGE_ELEM(orientation_img, float,y-1,x) == orientation)
								{
										  struct line_points *p;
										  p = (struct line_points *) malloc(sizeof(struct line_points));
										  p->point = cvPoint(x, y-1);
										  float orientation_curr = (float) CV_IMAGE_ELEM(orientation_img, float,y-1,x);
										  p->orientation = orientation_curr;
										  p->endpoint = false;
										  p->startpoint = false;
										  p->no_neighbour = false;
										  points.push_back(p);///
										  CV_IMAGE_ELEM(img, uchar, y-1,x)=(uchar)70;//als gelesen markieren
										  getNeighboursOrdered(img,x, y-1, points, orientation_img, (int)orientation_curr);
										  return;
								}
					 }
					 //-------------------5------x-1, y-------------------------
					 int pic_val_5 = CV_IMAGE_ELEM(img, uchar, y,x-1);
					 if(pic_val_5 == 255)
					 {
								if((int)CV_IMAGE_ELEM(orientation_img, float,y,x-1) == orientation)
								{
										  struct line_points *p;
										  p = (struct line_points *) malloc(sizeof(struct line_points));
										  p->point = cvPoint(x-1,y);
										  float orientation_curr = (float) CV_IMAGE_ELEM(orientation_img, float,y, x-1);
										  p->orientation = orientation_curr;
										  p->endpoint = false;
										  p->startpoint = false;
										  p->no_neighbour = false;
										  points.push_back(p);///
										  CV_IMAGE_ELEM(img, uchar, y,x-1)=(uchar)70;//als gelesen markieren
										  getNeighboursOrdered(img,x-1, y, points, orientation_img,(int)orientation_curr);
										  return;
								}
					 }
					 //-------------------4------x-1, y-1-------------------------
					 int pic_val_4 = CV_IMAGE_ELEM(img, uchar, y-1,x-1);
					 if(pic_val_4 == 255)
					 {
								if((int)CV_IMAGE_ELEM(orientation_img, float,y-1,x-1) == orientation)
								{
										  struct line_points *p;
										  p = (struct line_points *) malloc(sizeof(struct line_points));
										  p->point = cvPoint(x-1,y-1);
										  float orientation_curr = (float) CV_IMAGE_ELEM(orientation_img, float,y-1, x-1);
										  p->orientation = orientation_curr;
										  p->endpoint = false;
										  p->startpoint = false;
										  p->no_neighbour = false;
										  points.push_back(p);
										  CV_IMAGE_ELEM(img, uchar, y-1,x-1)=(uchar)70;//als gelesen markieren
										  getNeighboursOrdered(img,x-1, y-1, points, orientation_img, (int)orientation_curr);
										  return;
								}
					 }
					 static CvPoint oldPoint= cvPoint(-1,-1);
					 static int pointCtr=0;
					 if(oldPoint.x == x && oldPoint.y ==y)
					 {
								pointCtr++;
					 }
					 else
					 {
								pointCtr=0;
					 }
					 oldPoint= cvPoint(x,y);
					 int orientation_new;
					 if(pointCtr < 4)
					 {
								switch(orientation)
								{
										  case 0: 
													 orientation_new = 45;
													 getNeighboursOrdered(img,x, y, points, orientation_img, orientation_new);
													 break;

										  case 45:
													 orientation_new = 90;
													 getNeighboursOrdered(img,x, y, points, orientation_img, orientation_new);
													 break;

										  case 90:
													 orientation_new=135;
													 getNeighboursOrdered(img,x, y, points, orientation_img, orientation_new);
													 break;

										  case 135:
													 orientation_new=0;
													 getNeighboursOrdered(img,x, y, points, orientation_img, orientation_new);
													 break;
													 break;
								}
								return;
					 }
					 else
					 {
								points[points.size()-1]->endpoint=true;
								return;
					 }
		  }//end if no edge point
		  else
		  {
					 points[points.size()-1]->endpoint = true;
					 return;
		  }
		  return;
}

/**
 * returns euclidean distance between two pixels
 */
double distBetweenPoints(CvPoint p1, CvPoint p2)
{
		  double distCurr = (double)(pow(double((p1.x - p2.x)),2) + pow(double((p1.y - p2.y)),2));
		  distCurr = sqrt(distCurr);
		  return distCurr;
}

bool isPointsConnected(CvPoint p1, CvPoint p2)
{
	if (abs(p2.y-p1.y)<4)
	{
		return true;
	}
	return false;
}

int LaneFinder::getWidth()
{
		  return WIDTH;
}
int LaneFinder::getHeight()
{
		  return HEIGHT;
}

void LaneFinder::getHighestYIndex(std::vector<struct line_points*>& longestLine, int &highest_y_index, CvPoint &yPoint)
{
		  int y_curr, y_highest;
		  y_highest = 0;
		  for(int i= 0; i<longestLine.size(); i++)
		  {
					 y_curr = longestLine[i]->point.y;
					 if(y_curr > y_highest)
					 {
								y_highest = y_curr; 
								highest_y_index = i;
								yPoint = longestLine[i]->point;
					 }
		  }	
}
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
int LaneFinder::improveGoodCont(std::vector< struct line_points*> & longestLine, int windowSize, int thresh, IplImage* img)
{
		  int length=longestLine.size();
		  int d = length / (windowSize+1);
		  double height=static_cast<double>(getHeight());
		  double sum_diff=0;
		  if(d > 0)
		  {
					 for(int i=length/2; i<length; i+=windowSize/2)//not the entire image, only part of it
					 {
								CvPoint start, end;
								//take pixel at position i and if there at i+windowSize
								static int ctr=0;
								if(i+windowSize<length)
								{
										  start.x = longestLine[i]->point.x;
										  start.y = longestLine[i]->point.y;
										  end.x = longestLine[i+windowSize]->point.x;
										  end.y = longestLine[i+windowSize]->point.y;
										  //now every y value must be inverted
										  //now draw a line between these two point and check for 
										  //the other points on longestLine if they differ a lot from that
										  double alpha, b, m;
										  double diffy = static_cast<double>(((height-end.y)- (height-start.y)));
										  double diffx = static_cast<double>((end.x -start.x));
										  m = diffy / diffx;
										  b = static_cast<double>(-m * start.x + static_cast<double>((getHeight()-start.y))); // offset	
										  //cout<<"b: "<<b<<endl;
										  double angle = atan2(diffy, diffx);
										  double cosM =cos(angle);
										  double sinM =sin(angle);
										  //everything that is calculated now, must also be inverted, with respect to y

										  //on that line you need windowSize-1 equidistant points.	
										  double lineLength = sqrt(pow(static_cast<double>(end.x - start.x),2)+pow(static_cast<double>(end.y-start.y),2));
										  double equ = lineLength/(static_cast<double>(windowSize));
										  ctr++;

										  for(int a=0; a<=windowSize;a++)
										  {
													 double c = static_cast<double>(a)*equ;
													 CvPoint p;

													 p.x=static_cast<int>(floor(longestLine[i]->point.x+cosM*c));
													 p.y=static_cast<int>(floor(height-(height-(longestLine[i]->point.y)+sinM*c)));
													 //cout<<"point on lane: "<<longestLine[i+a]->point.x<<"  "<<longestLine[i+a]->point.y<<endl;
													 //this is the point on the line, we need to compare with the 
													 //corresponding point on the street lane
													 //now calc the difference and sum it up
													 double diff = getDiffPoints(p, longestLine[i+a]->point);
													 sum_diff +=diff;
										  }

										  if(sum_diff > thresh)
										  {
													 sum_diff=0;
													 return i;
										  }
										  else
										  {
													 sum_diff = 0;
										  }
								}

					 }
		  }
		  return -1;
}
//prints all the bin lines.
void LaneFinder::printBinInColor(IplImage *imgColor, const std::vector< std::vector< struct line_points* > > &bin,char color)
{
	int width =imgColor->width;
	int height=imgColor->height;
	CvScalar s;
	s.val[0]=0;//blue
	s.val[1]=0;//green
	s.val[2]=0;//red
	switch (color)
	{
		case 'R':s.val[2]=255;break;
		case 'G':s.val[1]=255;break;
		case 'B':s.val[0]=255;break;
		default:s.val[2]=255;
	}
	int dotsize = 1;
	int length = bin.size();
	for (int i = 0;i<length;i++)
	{
		s.val[1]=0;
		int a = bin[i].size();
		for(int j = 0;j<a;j++)
		{
			s.val[1] = s.val[1]+5;
			cvCircle(imgColor, cvPoint(bin[i][j]->point.x, bin[i][j]->point.y),1, s, dotsize);
		}
	}
}

void LaneFinder::testShadow(IplImage *imgColor, const std::vector< struct line_points*> &longestLine)
{
	int width =imgColor->width;
	int height=imgColor->height;
	int sum = 0;
	int length=longestLine.size();
	if (length ==0)
		return;
	for (int i=0;i<length;i++)
	{
		int x = longestLine[i]->point.x;
		int y = longestLine[i]->point.y;
		sum = sum + (CV_IMAGE_ELEM(imgColor, uchar, y, x*3) + CV_IMAGE_ELEM(imgColor, uchar, y, x*3+1) + CV_IMAGE_ELEM(imgColor, uchar, y, x*3+2))/3;
	}
	int average=sum/length;
	return;
}
//prints the given vector in a light color and adds and adds. This was done for demonstration purpose
void LaneFinder::printLongestLineInColorOverlay(IplImage* imgColor, const std::vector< struct line_points*  > &longestLine, int defaultC)
{
	int width =imgColor->width;
	int height=imgColor->height;

	if(defaultC==-1){
		 //the color should be in the range from red to blue where red denotes start and blue the endpoint
		 CvScalar s;
		 s.val[0]=0;//blue
		 s.val[1]=0;//green
		 s.val[2]=255;//red
		 int length=longestLine.size();
		 cout<<"img width: "<<imgColor->width<<" heigt: "<<imgColor->height<<endl;
		 for (int i = 0; i<length; i++)
		 {
				s.val[0]++;
				s.val[2]--;
				if(s.val[0] >255){s.val[0]=255;}
				if(s.val[2] <0){s.val[2]=0;}
				//cout<<"s: "<<s.val[2]<<endl;
				//cout<<"values: "<<longestLine[i]->point.x<<" "<<longestLine[i]->point.y<<endl;
				int x = longestLine[i]->point.x;
				int y = longestLine[i]->point.y;
				if(x-2 > 0 && x+2 > 0 && x-2 < width && x+2 < width)
				{
						  cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x-2, s);
						  cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x-1, s);
						  cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x, s);
						  cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x+1, s);
						  cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x+2, s);
				}
		 }
		 //cout<<"START=RED, END=BLUE\n";
	}
	else
	{
		 CvScalar s;
		 s.val[0]=1;//defaultC;//blue
		 s.val[1]=1;//green
		 s.val[2]=0;//defaultC;//red
		 int length=longestLine.size();
		 // cout<<"img width: "<<imgColor->width<<" heigt: "<<imgColor->height<<endl;
		 for (int i = 0; i<length; i++)
		 {
			s.val[0]=5;
			s.val[1]=5;
			s.val[2]=5;
			int x = longestLine[i]->point.x;
			int y = longestLine[i]->point.y;
			if(x-2 > 0 && x+2 > 0 && x-2 < width && x+2 < width)
			{
				  s.val[0] += cvGet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x).val[0];
				  s.val[1] += cvGet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x).val[1];
				  s.val[2] += cvGet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x).val[2];
				  if(s.val[0]>=255){s.val[0]=255;}
				  cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x, s);
			}
		 }

	  }
}

void LaneFinder::printLongestLineInColor(IplImage* imgColor, const std::vector< struct line_points*  > &longestLine, int defaultC)
{
	  int width =imgColor->width;
	  int height=imgColor->height;

	  int b=2;
	  if(defaultC==-1)
	  {	
			 //the color should be in the range from red to blue where red denotes start and blue the endpoint
			 CvScalar s;
			 s.val[0]=25;//blue
			 s.val[1]=25;//green
			 s.val[2]=255;//red
			 int length=longestLine.size();
			 int j = 5;
			 //cout<<"img width: "<<imgColor->width<<" heigt: "<<imgColor->height<<endl;
			 for (int i = 0; i<length; i+=j)
			 {
					if (j>1)
						j = j - 1;
						
					s.val[0]++;
					s.val[2]--;
					
					if(s.val[0] >255)
						s.val[0]=255;
						
					if(s.val[2] <0)
						s.val[2]=0;
					//cout<<"s: "<<s.val[2]<<endl;
					//cout<<"values: "<<longestLine[i]->point.x<<" "<<longestLine[i]->point.y<<endl;
					int x = longestLine[i]->point.x;
					int y = longestLine[i]->point.y;
					// cout<<"The orientation is:"<< longestLine[i]->orientation<<endl;

					int dotsize = 2;
					if (j < 2)
						dotsize = 1;
					
					cvCircle(imgColor, cvPoint(longestLine[i]->point.x, longestLine[i]->point.y),1, s, dotsize);
			 }
			 //cvCircle(imgColor, cvPoint(50, 50),1, s, 2);
			 //cout<<"START=RED, END=BLUE\n";
	  }
	  else
	  {
			 CvScalar s;
			 s.val[0]=188;//defaultC;//blue
			 s.val[1]=188; //defaultC;//green
			 s.val[2]=0;//defaultC;//red
			 int length=longestLine.size();
			 cout<<"img width: "<<imgColor->width<<" heigt: "<<imgColor->height<<endl;
			 for (int i = 0; i<length; i++)
			 {
						int x = longestLine[i]->point.x;
						int y = longestLine[i]->point.y;
						//debug start
						//float imVal=longestLine[i]->orientation;
						//if(imVal != 0 && imVal != 90 && imVal!= 135 &&imVal!= 45)
						//{
						//		  cout<<"The orientation is: "<<imVal<<endl;
						//	}
						//debug end
						cvCircle(imgColor, cvPoint(longestLine[i]->point.x, longestLine[i]->point.y),1, s, 2);

						//if(x-b > 0 && x+b > 0 && x-b < width && x+b < width)
						//{
						//		  for(int a=b; a>=0; a--)
						//		  {
						//					 cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x-a, s);
						//					 cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x+a, s);
						//		  }
						//		  /*cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x-1, s);
						//			 cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x, s);
						//			 cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x+1, s);
						//		  cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x+2, s);*/
						//}

			 }

	  }

}
void LaneFinder::printLongestLineInColorTO(IplImage* imgColor, const std::vector< struct line_points*  > &longestLine, int limit)
{
	  //the color should be in the range from red to blue where red denotes start and blue the endpoint
	  //cout<<"in color\n";
	  CvScalar s;
	  s.val[0]=0;//blue
	  s.val[1]=255;//green
	  s.val[2]=255;//red
	  for (int i = 0; i<limit; i++)
	  {
			 //cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x, s);
			 //if(longestLine[i]->point.y < getWidth() && longestLine[i]->point.y >0 longestLine[i]->point.y <getHeight() &&longestLine[i]->point.x >0 )
			 {
						//cout<<"x,y: "<< longestLine[i]->point.x<<" "<<longestLine[i]->point.x<<endl;
						//cout<<imgColor->width<<endl;
						//cout<<imgColor->height<<endl;
						//cout<<"s: "<<s.val[0]<<endl;
						//cout<<"s: "<<s.val[1]<<endl;
						//cout<<"s: "<<s.val[2]<<endl;
						//cvSet2D(imgColor,longestLine[i]->point.y,longestLine[i]->point.x, s);

						cvCircle(imgColor, cvPoint(longestLine[i]->point.x, longestLine[i]->point.y),1, cvScalar(0,0,255), 2);//der Startpunkt ist schwarz der Endpunkt blau
			 }
	  }
}

void LaneFinder::deleteLine(std::vector< struct line_points*> const &longestLine )
{
	  for(int i=0; i<longestLine.size();i++)
	  {
			 if(longestLine[i] != NULL)
			 {
						free(longestLine[i]);
						//longestLine[i] = NULL;
			 }
	  }
}

/**
 * returns the angle in deg between two pixels. NOTE: zero is the vertical on the image, left from that
 * is a negative angle, and right from it is positive.
 */
double angleBetweenTwoPixels(CvPoint p1, CvPoint p2, int height)
{
	  int x = p2.x -p1.x;
	  int y = (height-p2.y) - (height-p1.y);
	  double angle = atan2(double(y),double(x));
	  if(p2.y==p1.y)
	  {
				 if(p2.x<p1.x)
				 {
							angle=M_PI;
				 }

	  }
	  angle = (angle/M_PI) * 180;

	  return angle;
}

/**
 *  class Tangent
 */
/**
 * constructor
 */
Tangent::Tangent(int width, int height)
{
		  WIDTH=width;
		  HEIGHT = height;
}
Tangent::~Tangent()
{

}

int Tangent::getWidth()
{
		  return WIDTH;
}

int Tangent::getHeight()
{
		  return HEIGHT;
}

//Invert the y-values yourself!!
void Tangent::calTangentParamsInv(CvPoint& p_1, CvPoint& p_2, double& m, double& b )
{
		  if(p_1.x == p_2.x)//the curve is a vertical line, m and b cannot be calculated!!
		  {
					 p_2.x -= 1;
		  }
		  if(p_1.y == p_2.y)//the curve is a horizontal line, m and b cannot be calculated!!
		  {
					 m=0;
					 b=p_1.y;
					 return;
		  }
		  m = static_cast<float>(p_2.y  -p_1.y) /  (p_2.x - p_1.x); // slope
		  b = static_cast<float>(-m * p_1.x + p_1.y); // offset	

}

/**
 * to calculate line parameters for a line going through points, you
 * need to consider that the y-values of the pixels are invers to the usual euclidean coordinate system.
 * Y=0 denotes the top of the image. If you call this function you do NOT need to do this inversion prior to this function call yourself. 
 * @ img: input Image
 * @ p_1: the line goes from point p_1 to point p_2
 * @ p_2: the line goes from point p_1 to point p_2
 * @see calTangentParamsInv(CvPoint& p_1, CvPoint& p_2, double& m, double& b )
 */
void Tangent::calTangentParams(CvPoint& p_1, CvPoint& p_2, double& m, double& b )
{
		  if(p_1.x == p_2.x)//the curve is a vertical line, m and b cannot be calculated!!
		  {
					 p_2.x -= 1;
		  }
		  if(p_1.y == p_2.y)//the curve is a horizontal line, m and b cannot be calculated!!
		  {
					 p_2.y -= 1;
		  }
		  m = static_cast<float>((getHeight()-p_2.y)  - (getHeight()-p_1.y)) /  (p_1.x - p_2.x); // slope

		  b = static_cast<float>(-m * p_1.x + p_1.y); // offset	
}

void Tangent::drawTangent(IplImage* img, CvPoint p_1, CvPoint p_2,  double m, double b, int length )
{
		  int x=p_1.x;
		  int y=0;
		  if(p_1.x < p_2.x)
		  {
					 cout<<"debug drawTangent\n";
					 for(int i=0; i<length; i++)
					 {
								x ++;
								y = static_cast<int>(calTangentValue(m, x, b));
								if(y < img->height && y > 0 && x < img->width && x > 0)
								{
										  CV_IMAGE_ELEM(img, uchar, y,x)=(uchar)255;
										  //for colored output:
										  if((x+2) < img->width && (x+2) >0)
										  {
													 CV_IMAGE_ELEM(img, uchar, y,x+1)=(uchar)255;
													 CV_IMAGE_ELEM(img, uchar, y,x+2)=(uchar)255;
										  }

								}
					 }
		  }
		  else
		  {
					 cout<<"debug drawTangent 2\n";
					 for(int i=0; i<length; i++)
					 {
								x --;
								y = static_cast<int>(calTangentValue(m, x, b));
								if(y < img->height && y > 0 && x < img->width && x >0)
								{
										  CV_IMAGE_ELEM(img, uchar, y,x)=(uchar)255;

										  if((x+2) < img->width && (x+2) >0)
										  {
													 CV_IMAGE_ELEM(img, uchar, y,x+1)=(uchar)255;
													 CV_IMAGE_ELEM(img, uchar, y,x+2)=(uchar)255;
										  }
								}
					 }
		  }
}

void Tangent::drawTangentsInv(IplImage* img,  CvPoint& p_1,  CvPoint& p_2, int steps, int color)
{
		  double m,b;
		  int y=p_1.y;
		  int x=p_1.x;

		  Tangent::calTangentParamsInv(p_1, p_2, m,b);
		  cout<<"m= "<<m<<"  b= "<<b<<endl;
		  int w = img->width;
		  int h = img->height;
		  if(p_1.x < p_2.x)
		  {
					 for(int i=0; i<steps; i++)
					 {
								x ++;
								y = static_cast<int>(Tangent::calTangentValue(m, x, b));
								if(y<h && y >0  && x<w && x>0)
								{
										  CV_IMAGE_ELEM(img, uchar, y,x)=(uchar)color;
								}
					 }
		  }
		  else
		  {
					 for(int i=0; i<steps; i++)
					 {
								x --;
								y = static_cast<int>(Tangent::calTangentValue(m, x, b));
								if(y<h && y >0  && x<w && x>0)
								{
										  CV_IMAGE_ELEM(img, uchar, y,x)=(uchar)color;
								}
					 }
		  }
}

void Tangent::drawTangents(IplImage* img,  CvPoint& p_1,  CvPoint& p_2, int steps, int color)
{
		  double m,b;
		  int y=p_1.y;
		  int x=p_1.x;

		  calTangentParams(p_1, p_2, m,b);

		  int w = WIDTH;
		  int h = HEIGHT;
		  if(p_1.x < p_2.x)
		  {
					 for(int i=0; i<steps; i++)
					 {
								x ++;
								y = static_cast<int>(calTangentValue(m, x, b));
								if(y<h && y >0  && x<w && x>0)
								{
										  CV_IMAGE_ELEM(img, uchar, y,x)=(uchar)color;
								}
					 }
		  }
		  else
		  {
					 for(int i=0; i<steps; i++)
					 {
								x --;
								y = static_cast<int>(calTangentValue(m, x, b));
								if(y<h && y >0  && x<w && x>0)
								{
										  CV_IMAGE_ELEM(img, uchar, y,x)=(uchar)color;
								}
					 }
		  }
}

double Tangent::calTangentValue(double m, int x, double b)
{
		  double y = m*x +b;
		  return y;
}

int LaneFinder::extractLine(IplImage *imgInput, IplImage *imgOutput, IplImage *test, std::vector<CvPoint> &lineRight,  int lowerLimit, int higherLimit, bool debug, int minSize, int lengthTangent)
{
	  IplImage* img_out =  cvCloneImage(imgInput);
	  //put processing here
	  double* _alpha;
	  double dummy = 10;
	  int x_pos_tmp = 100;
	  _alpha = &dummy;

	  IplImage* canny_img = cvCloneImage(imgInput);
	  vector< vector< struct line_points* > > bin;
	  vector< vector< struct line_points* > > binRight;

	  binLines(bin, imgInput, canny_img, "real");
	  if(canny_img!=NULL)
	  {
		 cvReleaseImage(&canny_img);
		 canny_img=NULL;
	  }

	  canny_img = cvCloneImage(imgInput);
	  binLinesRight(binRight, imgInput, canny_img, "real");
	  if(canny_img!=NULL)
	  {
		 cvReleaseImage(&canny_img);
		 canny_img=NULL;
	  }

	  std::vector< struct line_points*> longestLine;
	  std::vector<std::vector< struct line_points*>> lines;
	  std::vector<std::vector< struct line_points*>> stoplines;
	  int yLowerLimit=0;
	  int yHigherLimit=imgInput->height;
	  
	  //findLines(imgInput, img_out, _alpha, x_pos_tmp, longestLine, lowerLimit, higherLimit, yLowerLimit,yHigherLimit, bin, binRight, minSize, lengthTangent);
	  findLines2(imgInput,lines,bin,binRight);
	  connectLines(lines,stoplines);
	  if(debug)
	  {
		 cvCvtColor(imgInput, imgOutput, CV_GRAY2BGR);
		 testShadow(test,longestLine);
		 //printLongestLineInColor(imgOutput, longestLine, -1);
		 //printBinInColor(imgOutput,bin,'R');
		 //printBinInColor(imgOutput,lines,'B');
		 printBinInColor(imgOutput,stoplines,'B');
	  }
	  

	  //if you want to use the goodContinuation smoothing uncomment this part:
	  //int breakpoint = myLaneFinder->improveGoodCont(longestLine, 10, 10, imgInput);
	  //now you can transform it into vector of CvPoints
	  //myLaneFinder->convertLineStruct2Vec(lineRight, longestLine);
	  convertLineStruct2Vec(lineRight, longestLine);
	  //if(breakpoint>-1 && debug)
	  //{
	  //	 myLaneFinder->printLongestLineInColorTO(imgOutput, longestLine, breakpoint);
	  //}
	  //end - if you want to use the goodContinuation smoothing uncomment this part:

	  char imgFileName[200];
	  if(longestLine.size()<=0)
	  {
		 cout<<"no line could be detected\n";
	  }

	  cvReleaseImage(&img_out);
	  //free memory
	  for(int ci=0; ci<bin.size();ci++)
	  {
		 for(int a=0; a<bin[ci].size(); a++){
			if(bin[ci][a] != NULL){
			  free((bin[ci][a]));
			  bin[ci][a]=NULL;
			}
		 }
		 bin[ci].clear();
	  }
	  bin.clear();

	  for(int ci=0; ci<binRight.size();ci++)
	  {
		 for(int a=0; a<binRight[ci].size(); a++)
		 {
			if(binRight[ci][a] != NULL)
			{
					  free((binRight[ci][a]));
					  binRight[ci][a]=NULL;
			}
		 }
		 binRight[ci].clear();
	  }
	  binRight.clear();
	  deleteLine(longestLine);
	  longestLine.clear();//delete polyCurve
	  return 0;
}

int LaneFinder::extractLineWithoutKalman(IplImage *imgInput, IplImage *imgOutput, std::vector<CvPoint> &lineRight,  int lowerLimit, int higherLimit, bool debug, int minSize, int lengthTangent)
{
		  IplImage* img_out =  cvCloneImage(imgInput);
		  double* _alpha;
		  double dummy = 10;
		  _alpha=&dummy;
		  int x_pos_tmp = 100;
		  IplImage* canny_img = cvCloneImage(imgInput);
		  vector< vector< struct line_points* > > bin;
		  vector< vector< struct line_points* > > binRight;
		  //myLaneFinder->binLines(bin, imgInput, canny_img, "real");
		  binLines(bin, imgInput, canny_img, "real");
		  if(canny_img!=NULL)
		  {
					 cvReleaseImage(&canny_img);
					 canny_img=NULL;
		  }
		  canny_img = cvCloneImage(imgInput);
		  binLinesRight(binRight, imgInput, canny_img, "real");
		  if(canny_img!=NULL)
		  {
					 cvReleaseImage(&canny_img);
					 canny_img=NULL;
		  }

		  std::vector< struct line_points*> longestLine;
		  int yLowerLimit=0;
		  int yHigherLimit=imgInput->height;
		  findLines(imgInput, img_out, _alpha, x_pos_tmp, longestLine, lowerLimit, higherLimit, yLowerLimit,yHigherLimit, bin, binRight, minSize, lengthTangent);
		  if(debug)
		  {
					 cvCvtColor(imgInput, imgOutput, CV_GRAY2BGR);
					 printLongestLineInColor(imgOutput, longestLine, 255);
		  }

		  //int breakpoint = improveGoodCont(longestLine, 10, 10, imgInput);
		  //now you can transform it into vector of CvPoints
		  //myLaneFinder->convertLineStruct2Vec(lineRight, longestLine);
		  convertLineStruct2Vec(lineRight, longestLine);

		  //if(breakpoint>-1 && debug)
		  //{
		  // 		 printLongestLineInColorTO(imgOutput, longestLine, breakpoint);
		  //}
		  char imgFileName[200];
		  if(longestLine.size()<=0)
		  {
					 cout<<"no line could be detected\n";
		  }

		  cvReleaseImage(&img_out);
		  //free memory
		  for(int ci=0; ci<bin.size();ci++)
		  {
					 for(int a=0; a<bin[ci].size(); a++)
					 {
								if(bin[ci][a] != NULL)
								{
										  free((bin[ci][a]));
										  bin[ci][a]=NULL;
								}
					 }
					 bin[ci].clear();
		  }
		  bin.clear();

		  for(int ci=0; ci<binRight.size();ci++)
		  {
					 for(int a=0; a<binRight[ci].size(); a++)
					 {
								if(binRight[ci][a] != NULL)
								{
										  free((binRight[ci][a]));
										  binRight[ci][a]=NULL;
								}
					 }
					 binRight[ci].clear();
		  }
		  binRight.clear();
		  deleteLine(longestLine);
		  longestLine.clear();//delete polyCurve
		  return 0;
}

/**
 * The function implements a scalar kalman filter
 * @param double state_mu_prev: mean of the previous estimate
 * @param double state_var_prev: variance of the previous estimate
 * @param double measurement: the measurement
 * @return double state_mu_estimate: mean of the calculated estimate
 * @return double state_var_estimate: variance of the calculated estimate
 */
void LaneFinder::doKalmanFiltering(double state_mu_prev, double state_var_prev, double measurement,  double &state_mu_estimate, double &state_var_estimate)
{
		  //For debugging you can write these things to a file,
		  //just uncomment every line containing "debugKF".
		  //ofstream debugKF("debugKF.txt", ios::app); 
		  //debugKF.open();
		  //if(!debugKF){cout<<"could not open the file debugKF.txt, exiting! \n";exit(-1);}
		  //debugKF<<"measurement: "<<setw(9)<<setprecision(3)<<measurement<<"\t";
		  //debugKF<<"state_mu_prev: "<<setw(9)<<setprecision(3)<<state_mu_prev<<"\t";
		  //debugKF<<"state_var_prev: "<<setw(9)<<setprecision(3)<<state_var_prev<<"\t";

		  // variance (sigma^2) process noise
		  //if the process noise is big, you rather trust the measurement, that's why this is a relatively small value
		  double Q = 0.01 ; 
		  static double R = 1000; // variance measurement noise
		  //static double pj_post=1;//just guessing
		  //static double pj_priori= pj_post + Q;// the a priori error variance, just a wild guess in the beginning

		  //make prediction
		  double state_mu_pred  = state_mu_prev;
		  double state_var_pred =  state_var_prev + Q;
		  //debugKF<<"state_mu_pred: " << setw(9)<<setprecision(3)<<state_mu_pred<<"\t";
		  //debugKF<<"state_var_pred: "<<setw(9)<<setprecision(3)<<state_var_pred<<"\t";
		  double residual = measurement - state_mu_pred;

		  //calculate the Kalman gain 
		  double K = state_var_pred / ( state_var_pred + R ); //the Kalman Gain, uses the a priori error variance
		  //debugKF<<setw(9)<<setprecision(3)<<"K: "<<K<<"\t";

		  //update or correction: compute the final estimate by incorporating the measurement
		  if(fabs(residual) <= 50)
		  {
					 state_mu_estimate = state_mu_pred + K * ( residual );  //the Kalman Gain
					 state_var_estimate = (state_var_pred) * (1.0-K);//pj_priori * ( 1-K);
					 //Since the filter is sometimes almost estimating a constant, the variance is rapidly converging
					 //towards zero. However, we want to artificially maintain a wider variance, thus we do it
					 //here manually
					 //this is to prevent the sigma of getting too small, since we want to use this intervall for tracking!
					 if(state_var_estimate<500)
					 {
								state_var_estimate=500;
					 }
		  }
		  //because we know that if nothing was found the measurement is zero,
		  //we can improve the filter a bit manually.
		  //in this case we do not want to shift the estimate towards 0, but to trust the prediction
		  //and discard the measurement entirely
		  else if(fabs(residual)>50)
		  {
					 //the idea is, that sometimes nothing is detected, because evtl. the search interval 
					 //is too small. Then the search intervall must be increased, this can be done, by changing the
					 //measurment variance
					 //cout<<"big residual, increase the variance, i.e. search width!\n";
					 state_var_estimate += 200;
		  }
		  //because we know that if nothing was found the measurement is zero, the
		  //residual is large. In this case we want to discard the measurement completely.
		  if(fabs(residual)>200)					
		  {
					 state_mu_estimate = state_mu_pred;
		  }
		  //debugKF<<"state_mu_estimate: "<<setw(9)<<setprecision(3)<<state_mu_estimate<<"\t";
		  //debugKF<<"state_var_estimate: "<<setw(9)<<setprecision(3)<<state_var_estimate<<"\n";

		  //debugKF.flush();
		  //debugKF.close();
}
