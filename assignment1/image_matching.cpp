#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "main.h"

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


void
StereoEstimation_Naive (const int &window_size,
                        const int &dmin,
                        int height,
                        int width,
                        cv::Mat & image1, cv::Mat & image2,
                        cv::Mat & naive_disparities, const double &scale)
{

    int half_window_size = window_size / 2;

    for (int i = half_window_size; i < height - half_window_size; i++)
    {
        std::cout
                << "Calculating disparitiesfor the naive approch... "
                << std::ceil (((i - half_window_size + 1) / static_cast <
                               double >(height - window_size +
                                        1)) *100) << "%\r" << std::flush;
        for (int j = half_window_size; j < width - half_window_size; j++)
        {
            int min_ssd = INT_MAX;
            int disparity = 0;

            for (int d = -j + half_window_size;
                    d < width - j - half_window_size; d++)
            {
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
                for (int u = -half_window_size; u <= half_window_size; u++)
                {
                    for (int v = -half_window_size; v <= half_window_size; v++)
                    {
                        int val_left = image1.at < uchar > (i + u, j + v);	// sampling in the rigth image
                        int val_right = image2.at < uchar > (i + u, j + v + d);	//sampling in the left image (considering disparity)

                        norm_left += val_left;
                        norm_right += val_right;
                    }
                }

                norm_left /= window_size * window_size;
                norm_right /= window_size * window_size;

                norm_left = std::max (1, norm_left);
                norm_right = std::max (1, norm_right);

                for (int u = -half_window_size; u <= half_window_size; u++)
                {
                    for (int v = -half_window_size; v <= half_window_size; v++)
                    {
                        int val_left = image1.at < uchar > (i + u, j + v);	// sampling in the rigth image
                        int val_right = image2.at < uchar > (i + u, j + v + d);	//sampling in the left image (considering disparity)

                        //ssd += (val_left - val_right) * (val_left - val_right);
                        ssd +=
                            (val_left / norm_left -
                             val_right / norm_right) * (val_left / norm_left -
                                                        val_right / norm_right);
                    }
                }



                if (ssd < min_ssd)
                {
                    min_ssd = ssd;
                    disparity = d;
                }
            }

            naive_disparities.at < uchar > (i - half_window_size,
                                            j - half_window_size) =
                                                std::abs (disparity) * scale;
        }
    }

    std::cout << "Calculating disparities for the naive approach... Done.\r" <<
              std::flush;
    std::cout << std::endl;
}

void StereoEstimation_naive_optimized (const int &window_size,
                          const int &dmin,
                          int height,
                          int width,
                          cv::Mat & image1, cv::Mat & image2,
                          cv::Mat & naive_disparities, const double &scale){

    int half_window_size = window_size / 2;


	//The array where to store  previous locals norms of columns
	int r_local_norms[width];
	int l_local_norms[width];
	
	//init locals norms of columns
	for (int v = 0; v <= width; v++){
		int local_l_sum = 0;
		int local_r_sum = 0;
		
		for (int u = 0; u < window_size; u++){
			int val_left = image1.at < uchar > ( u, v);	
			int val_right = image2.at < uchar > ( u, v);	
			
			local_l_sum += val_left;
			local_r_sum += val_right;
		}
		
		// column scan ended, store the value in the array
		l_local_norms[v] = local_l_sum;
		r_local_norms[v] = local_r_sum;
	}
	
    for (int i = half_window_size; i < height - half_window_size; i++)
    {
        std::cout << "Calculating disparitiesfor the naive approch... " << std::ceil (((i - half_window_size + 1) / static_cast <double >(height - window_size + 1)) *100) << "%\r" << std::flush;	
		
        
		//removing the last raw, and adding the new own from the sum
		if( i > half_window_size && i < (height - half_window_size)){	
			for (int v = 0; v < width; v++){
				int last_val_left = image1.at < uchar > ( i - half_window_size - 1, v);	
				int last_val_right = image2.at < uchar > (i - half_window_size - 1, v);	
				
				int new_val_left = image1.at < uchar > (i + half_window_size , v);	
				int new_val_right = image2.at < uchar > (i + half_window_size , v);	
			
				l_local_norms[v] += (new_val_left - last_val_left) ;
				r_local_norms[v] += (new_val_right - last_val_right);
				
			}
			
		}
        
        for (int j = half_window_size; j < width - half_window_size; j++)
        {
            int min_ssd = INT_MAX;
            int disparity = 0;

			
            for (int d = -j + half_window_size; d < width - j - half_window_size; d++)
            {
                int ssd = 0;

                int norm_left = 0;
                int norm_right = 0;


                //summing the windows_size partal sum already computed
                for(int k  = j - half_window_size; k <= j + half_window_size; k++){
					norm_left += l_local_norms[k];
                    norm_right += r_local_norms[k+d];
				}

                norm_left /= window_size * window_size;
                norm_right /= window_size * window_size;

                norm_left = std::max (1, norm_left);
                norm_right = std::max (1, norm_right);


                for (int u = -half_window_size; u <= half_window_size; u++)
                {
                    for (int v = -half_window_size; v <= half_window_size; v++)
                    {
                        int val_left = image1.at < uchar > (i + u, j + v);	// sampling in the rigth image
                        int val_right = image2.at < uchar > (i + u, j + v + d);	//sampling in the left image (considering disparity)

                        //ssd += (val_left - val_right) * (val_left - val_right);s
                        ssd +=(val_left / norm_left - val_right / norm_right) * (val_left / norm_left - val_right / norm_right);
                    }
                }



                if (ssd < min_ssd){
                    min_ssd = ssd;
                    disparity = d;
                }
            }

            naive_disparities.at < uchar > (i - half_window_size,j - half_window_size) = std::abs (disparity) * scale;
        }
    }


    std::cout << "Calculating disparities for the naive approach... Done.\r" << std::flush;
    std::cout << std::endl;
    
}
void StereoEstimation_dynamic (	  const int &window_size,
								  const int &dmin,
								  int height,
								  int width,
								  double occlusionVal,
								  cv::Mat & image1, cv::Mat & image2,
								  cv::Mat & dynamic_disparities, const double &scale){

	int half_window_size = window_size / 2;
	int **PreMap, **disparityMap, **result;
	int min1, min2, min3; 
	int *scanLine_Left, *scanLine_Right;
	
	// --- init ---
	scanLine_Left = (int*) malloc(sizeof(int)*width);		
	scanLine_Right = (int*) malloc(sizeof(int)*width);
	
	PreMap = (int**) malloc(sizeof(int*)*width);			
	disparityMap = (int**) malloc(sizeof(int*)*width);
	result = (int**) malloc(sizeof(int*)*height);
	
	for(int i=0;i<height;i++)		
		result[i] = (int *) malloc(sizeof(int)*width); 
	
	for(int j=0;j<height;j++) 		
		for(int k=0;k<width;k++)	
			result[j][k] = 0;
	
	
	for(int i=0;i<width;i++) {
		PreMap[i] = (int *) malloc(sizeof(int)*width);
		disparityMap[i] = (int *) malloc(sizeof(int)*width);
	}
	
	for(int j=0;j<width;j++) {
		for(int k=0;k<width;k++) {	
			PreMap[j][k] = 0;
			disparityMap[j][k] = 0;
		}
	}
	
	// --- matching every pair of scanLine ---
	for(int i=half_window_size;i< (height - half_window_size);i++) {
				
		std::cout << "Calculating di sparities for the dynamic approch... "<< std::ceil ( ((i- half_window_size + 1) /((double)height- half_window_size + 1) *100)) << "%\r" << std::flush;	
		
		// scanLine
		for(int j=0;j<width;j++) {
			scanLine_Left[j] = image1.at < uchar > (i, j );
			scanLine_Right[j] = image2.at < uchar > (i, j );
		}
		

		
		// disparityMap init
		for(int j=0;j<width;j++) {
			disparityMap[j][0] = occlusionVal;
			disparityMap[0][j] = occlusionVal;
			PreMap[0][j] = 2;	// left occluded	
			PreMap[j][0] = 3;	// rigth occluded	
		}
		
		for(int k = 1; k < width; k++) {
			for(int j = 1; j < width; j++) {
				
				// SSD				
				int ssd = 0;

                for (int u = -half_window_size; u <= half_window_size; u++)
                {
                    for (int v = -half_window_size; v <= half_window_size; v++)
                    {
                        int val_left = image1.at < uchar > (i + u, k+ v );	
                        int val_right = image2.at < uchar > (i + u, j+ v);	

                        ssd += (val_left - val_right) * (val_left - val_right);
                    }
                }



		
				// store the best path (min difference) (thus, the difference and occlusion value are accumulated to disparityMap)
				int min1 = disparityMap[j-1][k-1] + ssd;		// top-left value + difference	 --> update the difference for previous matching
				int min2 = disparityMap[j-1][k] + occlusionVal; // top value	  + occlusionVal --> left occluded
				int min3 = disparityMap[j][k-1] + occlusionVal; // left value	  + occlusionVal --> right occluded
			
				// the best current path (min(m1, m2, m3)) is chosen
				PreMap[j][k] = -1;
				if(min1 <= min2 && min1 <= min3) {	// min(m1, m2, m3) = m1
					PreMap[j][k] = 1;
					disparityMap[j][k] = min1;	
				} 
				
				if(min2 <= min1 && min2 <= min3) {	 // min(m1, m2, m3) = m2
					PreMap[j][k] = 2;	
					disparityMap[j][k] = min2;	
				}
				
				if(min3 <= min1 && min3 <= min2) {	// min(m1, m2, m3) = m3
					PreMap[j][k] = 3;	
					disparityMap[j][k] = min3;	
				} 
				
			}
		}
		
		
		
		int P = width-1;
		int Q = width-1;
		int BeforeState = 1;
		int disparityValue = 0;

		while(Q>0 && P>0) {
				
			
			switch(PreMap[P][Q]) {
				case 1: // no occlusion 
					result[i][Q] = disparityValue;
					if(BeforeState == 3 || BeforeState == 2) {
						//TODO remove and put ABS
						int val = P-Q;
						
						if(val<0) 
							val = val * -1;
						disparityValue = val; // disparityValue = abs(columnOfLeft - columnOfRight)
					}
					BeforeState = 1;
					P--;
					Q--;
					break;
				case 2: // left occluded	
					result[i][Q] = disparityValue;
					BeforeState = 2;		
					P--;		
					break;
				case 3: // right occluded	
					result[i][Q] = disparityValue;
					BeforeState = 3;		
					Q--;		
					break;
				default:
					return ;
			}
		
		}
			
		
		
	}
	

	
	for(int i=0;i<height;i++) 	
		for(int j=0;j<width;j++) 	
			dynamic_disparities.at < uchar > (i, j ) = (result[i][j]*255)/dmin;	

	
	
    std::cout << "Calculating disparities for the dynamic programmig approach... Done.\r" <<std::flush;
    std::cout << std::endl;
}







void
Disparity2PointCloud (const std::string & output_file,
                      int height, int width, cv::Mat & disparities,
                      const int &window_size,
                      const int &dmin, const double &baseline,
                      const double &focal_length)
{
    std::stringstream out3d;
    out3d << output_file << ".xyz";
    std::ofstream outfile (out3d.str ());
    for (int i = 0; i < height; ++i)
    {
        std::cout << "Reconstructing 3D point cloud from disparities... " <<
                  std::ceil (((i) / static_cast <
                              double >(height - window_size +
                                       1)) *100) << "%\r" << std::flush;
        for (int j = 0; j < width; ++j)
        {
            if (disparities.at < uchar > (i, j) == 0)
                continue;

            const double u = j - width / 2;
            const double v = i - height / 2;
            
            const double d = disparities.at < uchar > (i, j) +dmin;

            const double Z = (double) focal_length * baseline / d;
            const double X = -(baseline * (2 * j - d)) / (2 * d);
            const double Y = i * baseline / d;


            outfile << X << " " << Y << " " << Z << std::endl;
        }
    }

    std::cout << "Reconstructing 3D point cloud from disparities... Done.\r" <<
              std::flush;
    std::cout << std::endl;
}
