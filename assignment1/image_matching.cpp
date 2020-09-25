#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>
#include <fstream>

#define window 15

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
	// Create and open a text file
  	ofstream MyFile("out.txt");
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
	//disp = Mat(imgL.rows,imgL.cols,CV_8UC1);
	disp = Mat::zeros(imgL.rows,imgL.cols,CV_8UC1);
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

	int maxDisparty = 0;
	int minDisparity = INT_MAX;

    
    //accessing each pixel
	for(int y=0; y<imgR.rows; y++){
		for(int x=0; x<imgR.cols; x++) {
			
			//foreach pixel I'm accessing all the pixel in the corresponding window Omega
			int minScore = INT_MAX;
			int dMin;

			//for each pixel, exits multiples windows, each one for each d
			for (int d=0; (d+x)<imgR.cols ; d++){
				int windowDisparity = 0;
				
				int v = y - window;
				int vLimit = y + window;
				
				//cheking not to go out of bounds
				if (v < 0)
					v = 0;
				if (vLimit > imgR.rows)
					vLimit = imgR.rows;
					
				for (; v < vLimit; v++){
					
					int u = x - window;
					int uLimit = x + window;
					
					
					//cheking not to go out of bounds
					if (u<0)
						u = 0;
					if (uLimit > imgR.cols)
						uLimit = imgR.cols;
						
					for (; (u+d) < uLimit; u++){
						int scoreR = pow(imgL.at<cv::Vec3b>(v,u)[0] - imgR.at<cv::Vec3b>(v,u+d)[0],2);
						 int scoreG = pow(imgL.at<cv::Vec3b>(v,u)[1] - imgR.at<cv::Vec3b>(v,u+d)[1],2);
						 int scoreB = pow(imgL.at<cv::Vec3b>(v,u)[2] - imgR.at<cv::Vec3b>(v,u+d)[2],2);
						
						windowDisparity += scoreR ;
						
					}
				}

				if (windowDisparity < minScore){
					minScore = windowDisparity;
					dMin = d;
				}
				
			}
			
			//dispartiy is distance from the pixel
			disp.at<uchar>(y,x) = dMin;
			if ( disp.at<uchar>(y,x) > maxDisparty)
				maxDisparty = disp.at<uchar>(y,x);
			else if (disp.at<uchar>(y,x) < minDisparity)
				minDisparity = disp.at<uchar>(y,x);
		
		}
	}
	
	//Normalization
	cout << "MAX disparity:" << maxDisparty << " min disparity: " << minDisparity <<endl;
	for(int y=0; y<imgR.rows; y++){
		for(int x=0; x<imgR.cols; x++) {
		float tmp = (int)disp.at<uchar>(y,x);
			tmp = tmp / maxDisparty;	
			tmp = tmp * 254;

			MyFile <<(int)tmp <<", ";
			
			disp.at<uchar>(y,x) = (int)tmp;

		
		}
	}
	
    namedWindow( "Left hend side image", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Left hend side image", imgL );                   // Show our image inside it.
    
    namedWindow( "Rigth hend side image", WINDOW_AUTOSIZE );
    imshow( "Rigth hend side image", imgR );  
    
    namedWindow( "Disparity image", WINDOW_AUTOSIZE );
    imshow( "Disparity image", disp );  

	MyFile.close();

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}

