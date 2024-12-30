/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file calibration.cpp
 * @author Leo
 * @brief 基于OpenCV的相机标定
 * @version 0.1
 * @date 2022-02-23
 * @copyright Copyright (c) 2022
 * @note 采用标准棋盘格进行标定，标定流程如下：
 *      1. 拍摄标定图片（遥控手柄采图/collection.cpp）
 *      2. 提取每张图片的角点信息
 *      3. 提取图像的亚像素角点信息
 *      4. 在棋盘图上绘制内角点（检验标定结果）
 *      5. 开始标定
 *      6. 评价标定参数
 *      7. 矫正图像
 */

#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <sys/stat.h>
#include "../include/common.hpp"

using namespace cv;
using namespace std;

Size sizeBoard = Size(11, 8);   // 标定板的角点数(行,列)
Size sizeSquare = Size(20, 20); // 实际测量标定板上每个棋盘格的边长（长,宽）：单位mm
string getFilename(string sourcePath);

/**
 * @brief 相机标定程序
 * @note 输出标定参数文件.yml 和 评价结果.txt
 * @param imagesFolder 标定板图像路径（文件夹）
 */
int main(int argc, char *argv[])
{
    string imagesFolder = "../res/calibration/temp/";       // 打开标定图像
    Size sizeImage;                                         // 图像的尺寸
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));   // 相机的畸变矩阵
    vector<String> imagesPath;
    Display display(2);                    // 初始化UI显示窗口
    glob(imagesFolder, imagesPath, false); // OpenCV提取文件夹中的所有文件
    struct stat buffer;

    //[1] 提取每张图片的角点信息
    cout << "[1] 提取每张图片的角点信息" << endl;
    vector<vector<Point2f>> pointsCorners; // 保存检测到的所有角点
    bool getImageSize = false;
    for (int i = 0; i < imagesPath.size(); i++)
    {
        Mat imageInput = imread(imagesPath[i]); // OpenCV读取图像
        if (!getImageSize)                      // 初始化图像Size信息
        {
            sizeImage.width = imageInput.cols;
            sizeImage.height = imageInput.rows;
            getImageSize = true;
            cout << "Image.width = " << sizeImage.width << "  |  Image.height = " << sizeImage.height << endl;
        }

        vector<Point2f> pointCorners;                                   // 每幅图像上检测到的角点坐标
        if (findChessboardCorners(imageInput, sizeBoard, pointCorners)) // 寻找图像角点
        {
            Mat imageGray;
            cvtColor(imageInput, imageGray, CV_RGB2GRAY);
            //[2] 亚像素精确化
            find4QuadCornerSubpix(imageGray, pointCorners, Size(5, 5)); // 对粗提取的角点信息进一步优化
            // cornerSubPix(imageGray,pointCorners,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));

            drawChessboardCorners(imageInput, sizeBoard, pointCorners, true); // 在标定板图像上绘制已识别的角点信息

            std::string windowName = "Corners";
            cv::namedWindow(windowName, WINDOW_NORMAL); // 图像名称
            imshow("Corners", imageInput);              //[3] 在原始图形中绘制角点信息并展示

            // 存储角点绘制图像
            string filename = getFilename(imagesPath[i]);
            string img_path = "../res/calibration/corners/" + filename;

            if (stat(img_path.c_str(), &buffer) != 0) // 判断文件夹是否存在
            {
                string command;
                command = "mkdir -p ../res/calibration/corners/";
                system(command.c_str()); // 利用os创建文件夹
            }

            imwrite(img_path, imageInput);

            uint16_t countCorner = sizeBoard.width * sizeBoard.height; // 每张图像上该有的角点数量
            if (pointCorners.size() >= countCorner)
            {
                pointsCorners.push_back(pointCorners); // 保存亚像素角点
            }

            cout << imagesPath[i] << " 样片合格!" << endl;
            waitKey(500); // 停顿500ms
        }
        else // 图像不存在角点
        {
            cout << "Can not find chessboard corners! Path:" << imagesFolder << endl;
            cout << imagesPath[i] << " 样片不合格!!!" << endl;
            cout << imagesPath[i] << " 请删除该照片!!!" << endl;
            exit(0); // 直接退出标定：否者后续文件名无法对应
        }
    }
    if (imagesPath.size() < 1)
    {
        cout << "Can't find any images!!" << endl;
        return false;
    }
    if (pointsCorners.size() < 1)
    {
        cout << "Can't find any images!!" << endl;
        return false;
    }

    cout << "已筛选 " << pointsCorners.size() << "张合格照片进行标定 | Loading...." << endl;

    //[4] 开始标定图像
    vector<vector<Point3f>> pointsObject; // 标定板角点的三维坐标

    vector<int> countCorners; // 每幅图像中角点的数量

    vector<Mat> rvecsMat; // 图像的旋转向量
    vector<Mat> tvecsMat; // 图像的平移向量

    int i, j, t; // 初始化标定板上角点的三维坐标
    for (t = 0; t < pointsCorners.size(); t++)
    {
        vector<Point3f> tempPointSet;
        for (i = 0; i < sizeBoard.height; i++)
        {
            for (j = 0; j < sizeBoard.width; j++)
            {
                Point3f realPoint;
                // 假设标定板放在世界坐标系中z=0的平面上
                realPoint.x = i * sizeSquare.width;
                realPoint.y = j * sizeSquare.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        pointsObject.push_back(tempPointSet);
    }

    // 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
    for (i = 0; i < pointsCorners.size(); i++)
    {
        countCorners.push_back(sizeBoard.width * sizeBoard.height);
    }

    calibrateCamera(pointsObject, pointsCorners, sizeImage, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0); // 开始标定

    cout << "相机标定完成!" << endl;
    cout << "正在评价标定结果 | Loading...." << endl;

    // 输出标定误差的评价结果
    double errorTotal = 0.0;                            // 所有图像的误差总和
    double error = 0.0;                                 // 每幅图像的平均误差
    vector<Point2f> pointsImage;                        // 重新计算得到的投影点
    ofstream fout("../res/calibration/assessment.txt"); // 保存标定的评价结果

    cout << "相机标定分辨率: " << sizeImage.width << "x" << sizeImage.height << "\n";
    fout << "相机标定分辨率: " << sizeImage.width << "x" << sizeImage.height << "\n";

    cout << "每幅图像的标定误差: \n";
    fout << "每幅图像的标定误差: \n";
    for (i = 0; i < pointsCorners.size(); i++)
    {
        vector<Point3f> tempPointSet = pointsObject[i];
        /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
        projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, pointsImage);
        /* 计算新的投影点和旧的投影点之间的误差*/
        vector<Point2f> tempImagePoint = pointsCorners[i];
        Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
        Mat image_points2Mat = Mat(1, pointsImage.size(), CV_32FC2);
        for (int j = 0; j < tempImagePoint.size(); j++)
        {
            image_points2Mat.at<Vec2f>(0, j) = Vec2f(pointsImage[j].x, pointsImage[j].y);
            tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
        }
        error = norm(image_points2Mat, tempImagePointMat, NORM_L2);
        errorTotal += error /= countCorners[i];

        string filename = getFilename(imagesPath[i]);
        cout << "图像[" << filename << "]的平均误差: " << error << "像素" << endl;
        fout << "图像[" << filename << "]的平均误差: " << error << "像素" << endl;
    }

    cout << "总体平均误差: " << errorTotal / pointsCorners.size() << "像素" << endl;
    fout << "总体平均误差: " << errorTotal / pointsCorners.size() << "像素" << endl
         << endl;

    Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
    fout << "相机内参数矩阵: " << endl;
    fout << cameraMatrix << endl
         << endl;
    fout << "畸变系数: \n";
    fout << distCoeffs << endl
         << endl
         << endl;
    for (int i = 0; i < pointsCorners.size(); i++)
    {
        string filename = getFilename(imagesPath[i]);
        fout << "图像[" << filename << "]的旋转向量: " << endl;
        fout << rvecsMat[i] << endl;
        /* 将旋转向量转换为相对应的旋转矩阵 */
        Rodrigues(rvecsMat[i], rotation_matrix);
        fout << "图像[" << filename << "]的旋转矩阵: " << endl;
        fout << rotation_matrix << endl;
        fout << "图像[" << filename << "]的平移向量: " << endl;
        fout << tvecsMat[i] << endl
             << endl;
    }

    fout << endl;

    //[5] 将相机标定参数写入xml文件
    FileStorage fs1("../res/calibration/calibration.xml", FileStorage::WRITE);
    fs1 << "cameraMatrix" << cameraMatrix;
    fs1 << "distCoeffs" << distCoeffs;
    fs1.release();

    //[6] 开始矫正图像
    cout << "正在显示图像矫正结果..." << endl;
    for (int i = 0; i < imagesPath.size(); i++)
    {
        Mat imageSource = imread(imagesPath[i]); // OpenCV读取图像
        Mat mapx = Mat(sizeImage, CV_32FC1);     // 经过矫正后的X坐标重映射参数
        Mat mapy = Mat(sizeImage, CV_32FC1);     // 经过矫正后的Y坐标重映射参数
        Mat R = Mat::eye(3, 3, CV_32F);          // 内参矩阵与畸变矩阵之间的旋转矩阵

        // 采用initUndistortRectifyMap + remap进行图像矫正
        initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, sizeImage, CV_32FC1, mapx, mapy);
        Mat imageCorrect = imageSource.clone();
        remap(imageSource, imageCorrect, mapx, mapy, INTER_LINEAR);

        display.setNewWindow(1, "imgSource", imageSource);   // 添加需要显示的图像
        display.setNewWindow(2, "imgCorrect", imageCorrect); // 添加需要显示的图像
        display.show();                                      // 图像窗口显示

        // 存储矫正图像
        string filename = getFilename(imagesPath[i]);
        string img_path = "../res/calibration/correct/" + filename;
        if (stat(img_path.c_str(), &buffer) != 0) // 判断文件夹是否存在
        {
            string command;
            command = "mkdir -p ../res/calibration/correct/";
            system(command.c_str()); // 利用os创建文件夹
        }
        imwrite(img_path, imageCorrect);

        waitKey(500); // 延时2s
    }

    return true;
}
/**
 * @brief 矫正图像
 *
 * @param imagesPath 图像路径
 */
void imageCorrecte(string imagesPath)
{
    Size sizeImage;     // 图像的尺寸
    Display display(2); // 图像显示窗口重绘
    // 读取xml中的相机标定参数
    Mat matrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参矩阵
    Mat coeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); // 相机的畸变矩阵
    FileStorage fs;
    fs.open("../res/calibration/valid/calibration.xml", FileStorage::READ);
    fs["cameraMatrix"] >> matrix;
    fs["distCoeffs"] >> coeffs;

    Mat imageSource = imread(imagesPath); // 读取矫正图像
    sizeImage.width = imageSource.cols;
    sizeImage.height = imageSource.rows;
    Mat mapx = Mat(sizeImage, CV_32FC1); // 经过矫正后的X坐标重映射参数
    Mat mapy = Mat(sizeImage, CV_32FC1); // 经过矫正后的Y坐标重映射参数
    Mat R = Mat::eye(3, 3, CV_32F);      // 内参矩阵与畸变矩阵之间的旋转矩阵

    Mat imageCorrect = imageSource.clone();
    // 采用initUndistortRectifyMap+remap进行图像矫正
    initUndistortRectifyMap(matrix, coeffs, R, matrix, sizeImage, CV_32FC1, mapx, mapy);
    remap(imageSource, imageCorrect, mapx, mapy, INTER_LINEAR);

    // 采用undistort进行图像矫正
    //  undistort(imageSource, imageCorrect, matrix, coeffs);

    display.setNewWindow(0, "imgSource", imageSource);   // 添加需要显示的图像
    display.setNewWindow(1, "imgCorrect", imageCorrect); // 添加需要显示的图像
    display.show();                                      // 图像窗口显示

    while (1)
    {
        waitKey(200);
    }
}

/**
 * @brief Get the Filename object
 *
 * @param path 文件地址
 * @return 文件名称
 */
string getFilename(string sourcePath)
{
    // 获取不带路径的文件名
    string::size_type iPos = sourcePath.find_last_of('/') + 1;
    string filename = sourcePath.substr(iPos, sourcePath.length() - iPos);

    return filename;
}