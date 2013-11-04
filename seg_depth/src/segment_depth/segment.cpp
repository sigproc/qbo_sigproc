/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include <cstdio>
#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "pnmfile.h"
#include "segment-image.h"
#include <string>
#include <cv_bridge/cv_bridge.h>

//int segment(int argc, char **argv) {
/*int segment(float sigma, float k, int min_size, const char* inputStr, const char* outputStr){
  /*if (argc != 6) {
    fprintf(stderr, "usage: %s sigma k min input(ppm) output(ppm)\n", argv[0]);
    return 1;
  }
  
  float sigma = atof(argv[1]);
  float k = atof(argv[2]);
  int min_size = atoi(argv[3]);
  string input = argv[4];
  string output = argv[5];
	
  printf("loading input image.\n");
  image<double> *input = loadPPMd(inputStr);
	
  printf("processing\n");
  int num_ccs; 
  image<double> *seg = segment_image1(input, sigma, k, min_size, &num_ccs); 
  savePPMd(seg, outputStr);

  printf("got %d components\n", num_ccs);
  printf("done! uff...thats hard work.\n");

  return 0;
}
*/

int segment(float sigma, float k, int min_size, cv::Mat image){
  /*if (argc != 6) {
    fprintf(stderr, "usage: %s sigma k min input(ppm) output(ppm)\n", argv[0]);
    return 1;
  }
  
  float sigma = atof(argv[1]);
  float k = atof(argv[2]);
  int min_size = atoi(argv[3]);
  string input = argv[4];
  string output = argv[5];*/
	
  int width = image.size().width;
  int height = image.size().height;

  image<uchar> *input[width][height];

  for(int i=0; i<height; i++)
    {
        //Go through all the columns
        for(int j=0; j<width; j++)
        {
                imRef(input,i,j) = image.at<uchar>(i,j);
        }
    }
	
  printf("processing\n");
  int num_ccs; 
  image<uchar> *seg = segment_image1(input, sigma, k, min_size, &num_ccs); 
  savePPMd(seg, "tmpIm.ppm");

  printf("got %d components\n", num_ccs);
  printf("done! uff...thats hard work.\n");

  return 0;
}

