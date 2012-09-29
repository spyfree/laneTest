#include "LaneFinder.h"
using namespace std;


int main(int args, char *argv[])
{

	// Check that the input file exists
	struct stat stFileInfo;
	int intStat = stat(argv[1],&stFileInfo);
	if (intStat != 0) {
		cout << "Error: Input file " << argv[1] << " could not be found\n";
		system("pause");
		return(1);
	}

	// You need to read in an image. Note, you can use different file formats,
	// everything supported by OpenCv
	IplImage* imgInTmp = cvLoadImage(argv[1], 0);//this is the image you want to process
	
	//displayImage(imgInTmp, "find right lane on this image, click image to proceed");
	//you could use it as is, but here I downscale it, for faster processing
	CvRect myRoi = cvRect(0,0, imgInTmp->width, imgInTmp->height);
	cvSetImageROI(imgInTmp, myRoi);
	IplImage* imgIn = createImage(0, imgInTmp->width/2, (imgInTmp->height/2));

	IplImage* testImg = cvLoadImage(argv[1]);
	IplImage* testImgIN = cvCreateImage(cvSize(testImg->width,testImg->height),IPL_DEPTH_8U,3);
	cvResize(testImg,testImgIN,CV_INTER_LINEAR);
	//cvCvtColor(testImg, testImgIN, CV_BGR2YCrCb);
	displayImage(testImgIN, "test pic");


	cvResize(imgInTmp, imgIn);
	cvReleaseImage(&imgInTmp);
	// displayImage(imgIn, "this is the downscaled image, click image to proceed");

	//the function call to extractRightLane() can display an output image, if
	//the debug flag (showOutput) is set to true
	//if you set the debug flag to true, this image shows what was extracted
	IplImage* imgOut = createColorImage(imgIn->width, imgIn->height);

	//will contain the extracted lane boundaries	 
	vector<CvPoint> lineRight; 		 
	//the debug flag
	bool showOutput = true;

	//this object is needed for the lane extraction, initialize it with size of the image to be processed
	LaneFinder *myLaneFinder = new LaneFinder(imgIn->width, imgIn->height);
	//extractLine() expects initial search intervall where the line you are looking
	//for starts. This is a left boundary (horizontal (x-)image coordinate)  for the
	//bottom row in the image and a right boundary.

	int searchLimitLeft = 0;//imgIn->width/2;
	int searchLimitRight = imgIn->width ;
	//the last two integers in this function call, 70 and 10 specify "fiddle parameters", which 
	//can be adjusted according to your needs.
	//If the found line has a length of < 70 it is discarded.
	//Furthermore, sometimes it is not possible to find the real starting point of the line, instead
	//the line starts somewhere more up in the image. In this case it is automatically extrapolated
	//to go towards the image bottom. The extrapolation is done by computing a line equation to the part of the lane
	//close to the image bottom. For that two points are necessary, the starting point of the found line
	//and another point on the found line. It must be specified how many pixels difference this other
	//point should have, in this case I use 10.
	double t = (double)cvGetTickCount();
	myLaneFinder->extractLine(imgIn, imgOut,testImgIN, lineRight, searchLimitLeft, searchLimitRight, showOutput, 100, 20);
	t = ((double)cvGetTickCount() - t)/(cvGetTickFrequency()*1000); 
	printf( "exec time = %gms\n", t );
	cout<<"The length of the detected curve is: "<<lineRight.size()<<" pixels. "<<endl;
	//you can write the extracted curve to a file
	//each row in this file is: x-coordinate y-coordinate
	//of a lane pixel

	cout<<"Sending data to bus...\n";
	if (myLaneFinder->publishCurve(lineRight)==-1){
		char *outfilename = "rightlane.txt";
		cout<<"calling write Curve to file:" << outfilename << "\n";
		myLaneFinder->writeCurve(lineRight, outfilename);
	}


	displayImage(imgOut, "the extracted right lane, click image to proceed");

	//delete the extracted lane 
	lineRight.clear();

	//clean up
	delete myLaneFinder;
	cvReleaseImage(&imgIn);
	cvReleaseImage(&imgOut);
}