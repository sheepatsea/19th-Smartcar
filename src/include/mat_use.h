#pragma once
#include <opencv2/opencv.hpp>
//#include "imgproc.hpp"

#define using_kernel_num 7
#define using_resample_dist 0.05
#define PI 3.141593
#define pixel_per_meter 235
using namespace std;
using namespace cv;

typedef struct points_angle
{
	float angle[500];
	int real_size;
}Angle;

Mat auto_init_ipm_mat();
Mat init_ipm_mat();
vector<POINT> convertCvPointsToPoints(const vector<Point2f>& cvPoints);
vector<Point2f> convertPointsToCvPoints(const vector<POINT>& edge);
Mat draw_boundary_ipm(vector<POINT> left_edge, vector<POINT> right_edge, Mat invMat);
vector<POINT> track_leftline(vector<POINT> edge_in, int approx_num, float dist);
vector<POINT> track_rightline(vector<POINT> edge_in, int approx_num, float dist);
int clip(int x, int low, int up);
vector<POINT> blur_points(vector<POINT> edge_input, int kernel);
vector<POINT> resample_points(vector<POINT> input_edge, float dist);
Angle get_angle(vector<POINT> input_edge, int dist);