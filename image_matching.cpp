/*
 * Compiling
 * g++ image_matching.cpp -o image_matching -std=c++11 `pkg-config --cflags --libs opencv`
 * example of usage
 * ./image_matching ../middlebury/Art/view1.png  ../middlebury/Art/view5.png 
 */
 
 
 /*
  * hints:
  * I have generated a disparity map using the OpenCV StereoBM and StereoSGBM functions and a pair of cameras.
  * 
  */ 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>

#define window 11

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
    if( argc != 3)
    {
     cout <<" Usage: image_mattching imgL imgR" << endl;
     return -1;
    }

    Mat imgL;
    Mat imgR;
    Mat disp;
    imgL = imread(argv[1], cv::IMREAD_COLOR);   // Read input images
    imgR = imread(argv[2], cv::IMREAD_COLOR);   

	//costructing dipsarity representing obj (gray scale - 1 channel)
	disp = Mat(imgL.rows,imgL.cols,CV_8UC1);

    if(! imgL.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image"<<argv[1]<< std::endl ;
        return -1;
    }
	if(! imgR.data )                              
    {
        cout <<  "Could not open or find the image"<<argv[2]<< std::endl ;
        return -1;
    } 
    
    //accessing each pixel
	for(int i=0; i<imgR.rows; i++){
		for(int j=0; j<imgR.cols; j++) {
			
			//foreach pixel I'm getting the disparty looking in the window of the left heand side img
			int minScore = INT_MAX;
			int dMin;
			
			int r = i - window;
			int rLimit = i + window;
			
			//cheking not to go out of bounds
			if (r < 0)
				r = 0;
			if (rLimit > imgR.rows)
				rLimit = imgR.rows;
				
			for (; r < rLimit; r++){
				
				int k = j - window;
				int kLimit = j + window;
				
				
				//cheking not to go out of bounds
				if (k<0)
					k = 0;
				if (kLimit > imgR.cols)
					kLimit = imgR.cols;
					
				for (; k < kLimit; k++){
					int scoreR = pow(imgL.at<cv::Vec3b>(i,j)[0] - imgR.at<cv::Vec3b>(i,k)[0],2);
					int scoreG = pow(imgL.at<cv::Vec3b>(i,j)[1] - imgR.at<cv::Vec3b>(i,k)[1],2);
					int scoreB = pow(imgL.at<cv::Vec3b>(i,j)[2] - imgR.at<cv::Vec3b>(i,k)[2],2);
					
					int matchingScore = scoreR + scoreG + scoreB;
					
					if (minScore > matchingScore) {
						minScore = matchingScore;
						dMin = k;
					}
				}
			}
			
			//dispartiy is distance from the pixel
			
			disp.at<uchar>(i,j) = dMin - j;
		}
	}
	
	
    namedWindow( "Left hend side image", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Left hend side image", imgL );                   // Show our image inside it.
    
    namedWindow( "Rigth hend side image", WINDOW_AUTOSIZE );
    imshow( "Rigth hend side image", imgR );  
    
    namedWindow( "Disparity image", WINDOW_AUTOSIZE );
    imshow( "Disparity image", disp );  


    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}

