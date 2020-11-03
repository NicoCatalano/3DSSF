#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>

/*
 * 
 * https://stackoverflow.com/questions/4858553/converting-disparity-map-to-3d-points
 */

// TODO: consider slinding windows (lab ses ~ minute34), or parallelization
void StereoEstimation_Naive(
	const int& window_size,
	const int& dmin,
	int height,
	int width,
	cv::Mat& image1, cv::Mat& image2, cv::Mat& naive_disparities, const double& scale){
	
	int half_window_size = window_size / 2;
	
	for (int i = half_window_size; i < height - half_window_size; i++){
		std::cout
			<< "Calculating disparitiesfor the naive approch... "
			<< std::ceil(((i - half_window_size + 1) / static_cast<double>(height-window_size +1)) *100) << "%\r"
			<< std::flush;
		for (int j = half_window_size; j < width - half_window_size; j++){
		int min_ssd = INT_MAX;
		int disparity = 0;
		
			for(int d = -j + half_window_size; d < width - j - half_window_size; d++){
				int ssd = 0;
				
				/* for ache pixel in [i][j] i go trough all the pixel 
				 * corresponding wind using [u][v]
				*/
				int norm_left = 0;
				int norm_right = 0;
				/*
				 * NORMALIZATION SSD
				 * sum |  val_left / norm_left - val_rigth /norm_rigth |²
				 * norm_left is constant for all piexls eg the avarege of all pixels
				 * norm_rigth is constant for all piexls
				 * 
				 * some alternatives are:
				 * Cross correlation
				 * Normalized cross correlation
				 * Zero -meannormalized correlation
				*/
				//TODO: find somethng more efficient
				for (int u = -half_window_size; u <= half_window_size; u++){
					for (int v= -half_window_size; v <= half_window_size; v++){
						int val_left = image1.at<uchar>(i + u, j +v); // sampling in the rigth image
						int val_right = image2.at<uchar>(i + u, j + v +d); //sampling in the left image (considering disparity)
						
						norm_left += val_left;
						norm_right +=  val_right;
					}
				}
				
				norm_left /= window_size * window_size;
				norm_right /= window_size * window_size;
				
				norm_left = std::max(1, norm_left);
				norm_right = std::max(1, norm_right);
				
				for (int u = -half_window_size; u <= half_window_size; u++){
					for (int v= -half_window_size; v <= half_window_size; v++){
						int val_left = image1.at<uchar>(i + u, j +v); // sampling in the rigth image
						int val_right = image2.at<uchar>(i + u, j + v +d); //sampling in the left image (considering disparity)
						
						//ssd += (val_left - val_right) * (val_left - val_right);
						ssd += (val_left / norm_left - val_right / norm_right ) * (val_left / norm_left - val_right / norm_right );
					}
				}
				
				
				
				if(ssd < min_ssd){
					min_ssd = ssd;
					disparity = d;
			}
		}
		
		naive_disparities.at<uchar>(i - half_window_size, j - half_window_size) = std::abs(disparity)* scale;
		}
	}

  std::cout << "Calculating disparities for the naive approach... Done.\r" << std::flush;
  std::cout << std::endl;
}

void Disparity2PointCloud(
	const std::string& output_file,
	int height, int width, cv::Mat& disparities,
	const int& window_size,
	const int& dmin, const double& baseline, const double& focal_length){
		  std::stringstream out3d;
  out3d << output_file << ".xyz";
  std::ofstream outfile(out3d.str());
  for (int i = 0; i < height - window_size; ++i) {
    std::cout << "Reconstructing 3D point cloud from disparities... " << std::ceil(((i) / static_cast<double>(height - window_size + 1)) * 100) << "%\r" << std::flush;
    for (int j = 0; j < width - window_size; ++j) {
      if (disparities.at<uchar>(i, j) == 0) continue;

      // TODO
      //const double Z = ...
      //const double X = ...
      //const double Y = ...
	  //
      //outfile << X << " " << Y << " " << Z << std::endl;
    }
  }

  std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" << std::flush;
  std::cout << std::endl;
}
	
int main(int argc, char** argv){

	/////////////////
	// Parameters ///
	/////////////////
	
	// camera setup parameters 
	const double focal_legth = 1247;
	const double baseline = 213;
	
	// stereo estimation parameters
	const int dmin = 67;
	const int window_size = 7;
	const double weight = 500;
	const double scale = 3;
	
	////////////////////////////
	// Command line arguments //
	////////////////////////////
	
	if (argc < 4){
		std::cerr << "Usage: " << argv[0] << " IMAGE 1 IMAGE 2 OUTPUT_FILE" <<std::endl;
		return 1;
	}
	
	cv::Mat image1 = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
	cv::Mat image2 = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
	const std::string output_file = argv[3];
	
	if (!image1.data){
		std::cerr << "No img1 data" << std::endl;
		return EXIT_FAILURE;
	}	
	
	if (!image2.data){
		std::cerr << "No img1 data" << std::endl;
		return EXIT_FAILURE;
	}
	
	
	std::cout << "-------------- Parameters --------------" << std::endl;
	std::cout << "focal_legth = " << focal_legth << std::endl;
	std::cout << "baseline = " << baseline << std::endl;
	std::cout << "window_size" << window_size << std::endl;
	std::cout << "occlusion weigths" << weight << std::endl;
	std::cout << "disparity added due to iamge cropping = " << dmin << std::endl;
	std::cout << "scaling of disparity images to show = " << scale << std::endl;
	std::cout << "output filename = " << output_file << std::endl;
	std::cout << "----------------------------------------" << std::endl;
	
	int height = image1.size().height;
	int width = image1.size().width;
	
	///////////////////
	// Recostruction //
	///////////////////
	
	// Naive disparity image
	cv::Mat naive_disparities = cv::Mat::zeros(height, width, CV_8UC1);
	
	StereoEstimation_Naive(
		window_size,dmin, height, width,
		image1, image2,
		naive_disparities, scale);
	
	////////////
	// Output //
	////////////
	
	// recostruction
	Disparity2PointCloud(
		output_file,
		height, width, naive_disparities,
		window_size, dmin, baseline, focal_legth);
		
	//save / display images
	std::stringstream out1;
	out1 << output_file << "_naive.png";
	cv::imwrite(out1.str(), naive_disparities);
	
	cv::namedWindow("Naive", cv::WINDOW_AUTOSIZE);
	cv::imshow("Naive", naive_disparities);
	
	cv::waitKey(0);

	return 0;
}
