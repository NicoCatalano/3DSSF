#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "3Destimation.h"

int
main (int argc, char **argv)
{

    /////////////////
    // Parameters ///
    /////////////////

    // camera setup parameters
    const double focal_legth = 3740; //px
    const double baseline = 160; //mm

    // stereo estimation parameters
	const int dmin = 200;
	int window_size = 5;
	float weight = 5000;
	const double scale = 3;


    ////////////////////////////
    // Command line arguments //
    ////////////////////////////

    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " IMAGE 1 IMAGE 2 OUTPUT_FILE" <<
                  std::endl;
        return 1;
    }

    cv::Mat image1 = cv::imread (argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat image2 = cv::imread (argv[2], cv::IMREAD_GRAYSCALE);
    const std::string output_file = argv[3];

    if (!image1.data)
    {
        std::cerr << "No img1 data" << std::endl;
        return EXIT_FAILURE;
    }

    if (!image2.data)
    {
        std::cerr << "No img1 data" << std::endl;
        return EXIT_FAILURE;
    }
    
	std::cout<<"window size: ";
	std::cin >> window_size;
	std::cout<<"weight:";
	std::cin >> weight;


    std::cout << "-------------- Parameters --------------" << std::endl;
    std::cout << "focal_legth = " << focal_legth << std::endl;
    std::cout << "baseline = " << baseline << std::endl;
    std::cout << "window_size" << window_size << std::endl;
    std::cout << "occlusion weigths" << weight << std::endl;
    std::cout << "disparity added due to iamge cropping = " << dmin << std::endl;
    std::cout << "scaling of disparity images to show = " << scale << std::endl;
    std::cout << "output filename = " << output_file << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    int height = image1.size ().height;
    int width = image1.size ().width;

    ///////////////////
    // Recostruction //
    ///////////////////

    // Naive disparity image
    cv::Mat naive_disparities = cv::Mat::zeros (height, width, CV_8UC1);

	std::cout  <<" select Algorithm:" <<std::endl<<
					"\t 1 Naive"<<std::endl<<
					"\t 2 Naive Optimized"<<std::endl<<
					"\t 3 Dynamic prog Approach"<<std::endl;
	int selection;
	
	std::cin >> selection;
	
	switch(selection){
		case 1:
			
			StereoEstimation_Naive(
				   window_size,dmin, height, width,
				   image1, image2,
				   naive_disparities, scale);
			break;
			
		case 2:    
			StereoEstimation_naive_optimized (window_size, dmin, height, width,
                       image1, image2, naive_disparities, scale);
			break;
		case 3:
			StereoEstimation_dynamic (window_size, dmin, height, width, weight,
                                image1, image2, naive_disparities, scale);
	
		
	}



    ////////////
    // Output //
    ////////////
	
	std::cout << "Creating Point cloud" << std::endl;
	
    // recostruction
    Disparity2PointCloud (output_file,
                          height, width, naive_disparities,
                          window_size, dmin, baseline, focal_legth);

    //save / display images
    std::stringstream out1;
    out1 << output_file << ".png";
    cv::imwrite (out1.str (), naive_disparities);


    cv::namedWindow ("disparity map", cv::WINDOW_AUTOSIZE);
    cv::imshow ("disparity map", naive_disparities);


    cv::waitKey (0);

    return 0;
}

