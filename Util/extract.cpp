//opencv
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/ocl.hpp>
//C
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
//C++
#include<iostream>
#include <iomanip> 
#include<sstream>
#include<vector>
#include<algorithm>
#include<cstdio>

using namespace cv;
using namespace std;

#define BYTES_PER_PIXEL 2

int main (int argc, char *argv[])
{

if (argc < 6)
{
   std::cout << "Arguments (5 of them): input_filename, output_filename, fseek_bytes, width, height" << std::endl;
   exit(0);
}
int w = atoi(argv[4]);
int h = atoi(argv[5]);
Mat M(h, w, CV_32FC1);
Mat Mn(h, w, CV_32FC1);
Mat Mw, Mwe;
Mat element = getStructuringElement( MORPH_RECT, Size( 2*1 + 1, 2*1+1 ), Point( 1, 1 ) );
FILE *fin  = fopen (argv[1], "rb");
float norm_value;
unsigned char tin_value[BYTES_PER_PIXEL];
uint16_t pix_value;

fseek(fin, atoi(argv[3]), SEEK_SET);
std::cout << "Rewind this many bytes:" << atoi(argv[2]) << std::endl;

for(int i = 0; i < M.rows; i++)
{
    float* Mi = M.ptr<float>(i);
    if(i >= 32 && i < 48) std::cout << std::endl;
    for(int j = 0; j < M.cols; j++)
    {
      fread(tin_value, sizeof(unsigned char), BYTES_PER_PIXEL, fin);
      pix_value = *((uint16_t *)&tin_value[0]);
      Mi[j] = (float)pix_value;
      if(j >= 16 && j < 48) {
        if(i >= 32 && i < 48) {
          printf ("%5d ", pix_value);
        }
      }
    }
}

erode( M, Mwe, element );

normalize(Mwe, Mn, 0, 1.0, NORM_MINMAX, CV_32FC1);
Mn.convertTo(Mw, CV_8UC1, 255, 0);
std::cout << std::endl << "Matrix created!" << std::endl;
imwrite (argv[2], Mw);

std::cout << std::endl;
fclose (fin);
return 0;
}
