#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
int main() {
    std::cerr << "Opening0" << std::endl;
    // 打开第一个摄像头（索引为0）
    cv::VideoCapture cap1(0);
    // 打开第二个摄像头（索引为1）
    cv::VideoCapture cap2(1);

    std::cerr << "Opening1" << std::endl;

    cap1.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'V', 'Y', 'U'));
    cap2.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'V', 'Y', 'U'));


    cap1.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap1.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    cap2.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap2.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

     std::cerr << "Opening2" << std::endl;

    // 检查摄像头是否成功打开
    if (!cap1.isOpened() || !cap2.isOpened()) {
        std::cerr << "无法打开某个摄像头" << std::endl;
        return -1;
    }

    cv::namedWindow("Camera 1", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Camera 2", cv::WINDOW_AUTOSIZE);

    cv::Mat frame1, frame2;
    int cnt=0;
    while (true) {
        std::cerr << "Running" << std::endl;
        // 从第一个摄像头读取帧
        cap1.read(frame1);
        if (frame1.empty())
        {
           std::cerr << "frame1 is empty" << std::endl;
           break;
        }
        cap2.read(frame2);
        if(frame2.empty())
        {
            std::cerr << "frame2 is empty" << std::endl;
            break;
        }
         cv::imshow("Camera 1", frame1);
         cv::imshow("Camera 2", frame2);
        // if(!cnt)
        // {cap1.read(frame1);cnt++;cv::imshow("Camera 1", frame1);waitKey(100);}
        // else 
        // {
        //     cap2.read(frame2);cnt=0;cv::imshow("Camera 2", frame2);waitKey(100);
        // }
        // 从第二个摄像头读取帧
       // cap2.read(frame2);

        // 检查是否成功读取帧
        // if (frame2.empty() ) {
        //     std::cerr << "无法获取某个画面" << cnt << std::endl;
        //     break;
        // }

        // 显示第一个摄像头的画面
        // cv::imshow("Camera 1", frame1);
        // // 显示第二个摄像头的画面
        // cv::imshow("Camera 2", frame2);

        // 检测键盘按键，如果按下 q 键则退出循环
        if (cv::waitKey(30) == 'q') {
            break;
        }
    }

    // 关闭摄像头
    cap1.release();
    cap2.release();
    cv::destroyAllWindows();

    return 0;
}
