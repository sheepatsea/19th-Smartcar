#pragma once

#include <opencv2/opencv.hpp>
#include "../include/common.hpp"

using namespace std;
using namespace cv;
#define using_max_edge_num 500
#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3

#define using_top_row 20
#define using_min_block_length 100

#define origin_image 0
#define ipm_image 1

typedef struct points
{
	int row;
	int col;
}Points;

typedef struct edge
{
	Points point[using_max_edge_num];
	int real_size;
}Edge;

//extern int top_left, top_right, count_time;
extern double Mat1[3][3], Mat2[3][3];
bool findline_maze(Mat& img, int *start_row, Edge *left_edge, Edge *right_edge, uint8_t img_type,  POINT left, POINT right);
Mat binaryzation(Mat& frame);
void draw_on_img(Mat& img, Edge& left_edge, Edge& right_edge);
