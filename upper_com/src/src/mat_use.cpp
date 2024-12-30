#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include "../include/mat_use.h"

using namespace cv;
using namespace std;


vector<Point2f> convertPointsToCvPoints(vector<POINT>& edge) 
{
    vector<Point2f> cvPoints;
    for (int i = 0; i < edge.size(); i++) 
    {
        cvPoints.push_back(Point2f(edge[i].y, edge[i].x));
    }
    return cvPoints;
}//POINT to Point2f

vector<POINT> convertCvPointsToPoints(const vector<Point2f>& cvPoints) 
{
    vector<POINT> edge;
    for (int i = 0; i < cvPoints.size(); ++i) {
        POINT p(0,0);
        p.y= static_cast<int>(cvPoints[i].x);
        p.x = static_cast<int>(cvPoints[i].y);
        edge.push_back(p);
    }
    return edge;
}//Point2f to POINT

Mat init_ipm_mat()
{
    double Mat1[3][3]=       { { 1.18659065731237, -0.642407397045294, 17.7383147134322},
                    { 0.00544372463849122, 0.240244584874997, 17.6073324494164},
                    { -9.42092524688407E-06, -0.00256170659004855, 0.814878630771311}, }; 

                    
    double Mat2[3][3]= { { 0.834722165114306, 1.65660063393502, -53.9649475395608},
                    { -0.0159471431249632, 3.35134491469077, -72.0663990986057},
                    { -4.04821592695126E-05, 0.010554662669316, 1}, }; 
    Mat invM = (Mat_<double>(3, 3) <<
        Mat2[0][0], Mat2[0][1], Mat2[0][2],
        Mat2[1][0], Mat2[1][1], Mat2[1][2],
        Mat2[2][0], Mat2[2][1], Mat2[2][2]);

    return invM;
}
Mat auto_init_ipm_mat()
{
    int offsety = 0;
    vector<Point2f> srcPoints = {
    Point2f(107, 58),
    Point2f(213, 58),
    Point2f(78, 120),
    Point2f(242, 120)
    };// ԭͼ�궨��
    vector<Point2f> dstPoints = {
        Point2f(110, 90 + offsety),
        Point2f(210, 90 + offsety),
        Point2f(110, 190 + offsety),
        Point2f(210, 190 + offsety)
    };
    Mat M = getPerspectiveTransform(srcPoints, dstPoints);// ��͸�Ӻ�������

    return M;
}

int clip(int x, int low, int up) 
{
    return x > up ? up : x < low ? low : x;
}//ȡ��

vector<POINT> blur_points(vector<POINT> edge_input, int kernel) 
{
    vector<POINT> output_edge;
    //output_edge. = edge_input.real_size;

    int half = kernel / 2;
    for (int i = 0; i < edge_input.size(); i++) 
    {
        POINT output(0, 0);
        for (int j = -half; j <= half; j++) {
            output.y += edge_input[clip(i + j, 0, edge_input.size() - 1)].y * (half + 1 - abs(j));
            output.x += edge_input[clip(i + j, 0, edge_input.size() - 1)].x * (half + 1 - abs(j));
        }
        output.y /= (2 * half + 2) * (half + 1) / 2;
        output.x /= (2 * half + 2) * (half + 1) / 2;
        output_edge.push_back(output);
    }

    return output_edge;
}//�����˲�

vector<POINT> resample_points(vector<POINT> input_edge, float dist)
{
    vector<POINT> output_edge;

    int remain = 0; //len = 0;
    for (int i = 0; i < input_edge.size() - 1; i++) {
        float x0 = input_edge[i].y;
        float y0 = input_edge[i].x;
        float dx = input_edge[i + 1].y - x0;
        float dy = input_edge[i + 1].x - y0;
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        while (remain < dn) {
            POINT p(0,0);
            x0 += dx * remain;
            p.y = x0;
            y0 += dy * remain;
            p.x = y0;
            output_edge.push_back(p);

            //len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    return output_edge;
}

Angle get_angle(vector<POINT> input_edge, int dist)
{
    Angle angle_output;

    angle_output.real_size = input_edge.size();
    for (int i = 0; i < input_edge.size(); i++) 
    {
        if (i <= 0 || i >= input_edge.size() - 1)
        {
            angle_output.angle[i] = 0;
            continue;
        }
        float dx1 = input_edge[i].y - input_edge[clip(i - dist, 0, input_edge.size() - 1)].y;
        float dy1 = input_edge[i].x - input_edge[clip(i - dist, 0, input_edge.size() - 1)].x;
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = input_edge[clip(i + dist, 0, input_edge.size() - 1)].y - input_edge[i].y;
        float dy2 = input_edge[clip(i + dist, 0, input_edge.size() - 1)].x - input_edge[i].x;
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_output.angle[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);//��һ������ȷ���������ڶ�������ȷ���Ƕȴ�С
        angle_output.angle[i] = 180 / PI * angle_output.angle[i];
    }

    return angle_output;
}

// float angleBetweenPoints(const POINT& p1, const POINT& p2, const POINT& p3) {
//     float dx1 = p2.x - p1.x;
//     float dy1 = p2.y - p1.y;
//     float dx2 = p3.x - p2.x;
//     float dy2 = p3.y - p2.y;
//     float dot = dx1 * dx2 + dy1 * dy2;
//     float cross = dx1 * dy2 - dy1 * dx2;
//     float angle = atan2f(cross, dot);
//     return fabs(angle);
// }

vector<POINT> track_leftline(vector<POINT> edge_in, int approx_num, float dist)
{
    vector<POINT> edge_out;
    for (int i = 0; i < edge_in.size(); i++) 
    {
        POINT p(0, 0); 
        float dx = edge_in[clip(i + approx_num, 0, edge_in.size() - 1)].y - edge_in[clip(i - approx_num, 0, edge_in.size() - 1)].y;
        float dy = edge_in[clip(i + approx_num, 0, edge_in.size() - 1)].x - edge_in[clip(i - approx_num, 0, edge_in.size() - 1)].x;

        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;

        p.y = edge_in[i].y - dy * dist;
        p.x = edge_in[i].x + dx * dist;
        edge_out.push_back(p);
    }

    return edge_out;
}

vector<POINT> track_rightline(vector<POINT> edge_in, int approx_num, float dist)
{
   vector<POINT> edge_out;
    for (int i = 0; i < edge_in.size(); i++) 
    {
        POINT p(0, 0);
        float dx = edge_in[clip(i + approx_num, 0, edge_in.size() - 1)].y - edge_in[clip(i - approx_num, 0, edge_in.size() - 1)].y;
        float dy = edge_in[clip(i + approx_num, 0, edge_in.size() - 1)].x - edge_in[clip(i - approx_num, 0, edge_in.size() - 1)].x;
        float dn = sqrt(dx * dx + dy * dy);
        dx /= dn;
        dy /= dn;
        p.y = edge_in[i].y + dy * dist;
        p.x = edge_in[i].x - dx * dist;
        edge_out.push_back(p);
    }

    return edge_out;
}//�������Ѱ����?

Mat draw_boundary_ipm(vector<POINT> left_edge, vector<POINT> right_edge, Mat invMat)
{
    //cout << "ԭ������:" << left_edge.real_size;
    left_edge = blur_points(left_edge, using_kernel_num);
    right_edge = blur_points(right_edge, using_kernel_num);//�����˲����ú˰Ѹ�����ɢ����ز�ȥ�ر�������?
    left_edge = resample_points(left_edge, using_resample_dist);
    right_edge = resample_points(right_edge, using_resample_dist);//�²���������ʵ�ʾ���ȥ���ֵ�
    cout << "���������?:" << left_edge.size();
    vector<Point2f> pointsToTransform_left = convertPointsToCvPoints(left_edge), pointsToTransform_right = convertPointsToCvPoints(right_edge), transformedPoints;

    Size imageSize(320, 240);
    Mat only_boundary = Mat::zeros(imageSize, CV_8UC1);
    //��ʾ�����?
    //invMat_auto=invMat_auto.inv();//透�?�矩�?
    perspectiveTransform(pointsToTransform_left, transformedPoints, invMat);
    for (size_t i = 0; i < transformedPoints.size(); i++) 
    {
        circle(only_boundary, transformedPoints[i], 0, Scalar(255, 255, 255), 2);
    }
    left_edge = convertCvPointsToPoints(transformedPoints);
    

    //��ʾ�ұ���
    perspectiveTransform(pointsToTransform_right, transformedPoints, invMat);
    for (size_t i = 0; i < transformedPoints.size(); i++)
    {   
        circle(only_boundary, transformedPoints[i], 0, Scalar(255, 255, 255), 2);
    }

    //��һ�����ߣ�����������ߵ�?)
    vector<POINT> center_line = track_leftline(left_edge, 5, 27);
    Angle angle_pts = get_angle(center_line, 10);
    vector<Point2f> pointsToTransform_center = convertPointsToCvPoints(center_line);
    //perspectiveTransform(pointsToTransform_center, transformedPoints, invMat);
    for (size_t i = 0; i < pointsToTransform_center.size(); i++)
    {
        circle(only_boundary, pointsToTransform_center[i], 0, Scalar(255, 255, 255), 2);
        //imshow("11", only_boundary);
        //cout << angle_pts.angle[i] << endl;
        //waitKey(500);
    }
    //imshow("����", only_boundary);
    
    return only_boundary;
}