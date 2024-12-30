#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
	cv::Mat rgb_img, small_img, hsv_img, gray_img, binary_img1, binary_img2, binary_img;
	rgb_img = cv::imread("/home/edgeboard/workspace/autocar/res/samples/test/125.jpg");
	
	//cv::resize(rgb_img, small_img, cv::Size(160, 120), cv::INTER_AREA);
	cv::cvtColor(rgb_img, hsv_img, cv::COLOR_BGR2HSV);

	//cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
	//cv::threshold(gray_img, binary_img, 140, 255, cv::THRESH_OTSU);
	//cv::imwrite("C:/Users/86137/Pictures/Camera Roll/(56)_.jpg", binary_img);

	//cv::inRange(hsv_img, cv::Scalar(150, 100, 60), cv::Scalar(165, 255, 255), binary_img1);
	cv::inRange(hsv_img, cv::Scalar(100, 50, 50), cv::Scalar(140, 255, 255), binary_img);
	//cv::inRange(hsv_img, cv::Scalar(170, 100, 60), cv::Scalar(180, 255, 255), binary_img2);
	//cv::inRange(hsv_img, cv::Scalar(65, 100, 20), cv::Scalar(85, 255, 120), binary_img); //��ɫ
	//cv::inRange(hsv_img, cv::Scalar(26, 100, 60), cv::Scalar(35, 255, 255), binary_img); //��ɫ
	//cv::inRange(hsv_img, cv::Scalar(20, 100, 60), cv::Scalar(25, 255, 255), binary_img); //��ɫ
	/*
	uint16_t green_count = 0;

	for (uint8_t x = 0; x < 120; ++x)
	{
		for (uint8_t y = 80; y < 160; ++y)
		{
			cv::Vec3b hsv = hsv_img.at<cv::Vec3b>(x, y);
			if (hsv[0] >= 15 && hsv[1] >= 100 && hsv[2] >= 80 &&
				hsv[0] <= 22 && hsv[1] <= 255 && hsv[2] <= 255)
				++green_count;
		}
	}
	if (green_count >= 100)
	{
		circle(small_img, cv::Point(140, 30), 2, cv::Scalar(0, 0, 255));
	}
	std::cout << green_count << std::endl;
	*/
	std::vector<std::pair<int, int>> temp;
	uint8_t used[120][160] = {0};
	
	for (int x = 119; x >= 0; x --)
	{
		for (int y = 0; y < 160; y ++)
		{
			if (used[x][y] || binary_img.at<uint8_t>(x, y) == 0) continue;

			temp.clear();
			temp.push_back(std::make_pair(x, y));
			used[x][y] = 1;
			int y_next = y;
			while (y_next < 159 && !used[x][y_next + 1] && binary_img.at<uint8_t>(x, y_next + 1) == 255)
			{
				y_next ++;
				temp.push_back(std::make_pair(x, y_next));
				used[x][y_next] = 1;
			}
			int cx = x;
			int cy = (y + y_next) / 2;

			for (uint16_t i = 0; i < temp.size(); ++i)
			{
				int tx = temp[i].first;
				int ty = temp[i].second;
				
				if (tx > 0 && tx > cx - 2 * cx / 10 - 8 && !used[tx - 1][ty] && binary_img.at<uint8_t>(tx - 1, ty) == 255)
				{
					temp.push_back(std::make_pair(tx - 1, ty));
					used[tx - 1][ty] = 1;
				}
				if (ty > 0 && ty > cy - cx / 6 - 2 && !used[tx][ty - 1] && binary_img.at<uint8_t>(tx, ty - 1) == 255)
				{
					temp.push_back(std::make_pair(tx, ty - 1));
					used[tx][ty - 1] = 1;
				}
				if (tx < 119 && tx < cx && !used[tx + 1][ty] && binary_img.at<uint8_t>(tx + 1, ty) == 255)
				{
					temp.push_back(std::make_pair(tx + 1, ty));
					used[tx + 1][ty] = 1;
				}
				if (ty < 159 && ty < cy + cx / 6 + 2 && !used[tx][ty + 1] && binary_img.at<uint8_t>(tx, ty + 1) == 255)
				{
					temp.push_back(std::make_pair(tx, ty + 1));
					used[tx][ty + 1] = 1;
				}
			}
			
			if (temp.size() >= 30)
			{
				circle(small_img, cv::Point(cy, cx), 2, cv::Scalar(0, 0, 255));
				cv::Rect rect = cv::Rect(cy - cx / 6 - 2, cx - 2 * cx / 10 - 8, 2 * cx / 6 + 5, 2 * cx / 10 + 9); // ����һ�����Σ����Ͻ�x�����Ͻ�y�������ߣ�
				cv::Scalar color = cv::Scalar(255, 0, 0);
				cv::rectangle(small_img, rect, color, 1, cv::LINE_8);
			}
		}
	}
	
	imshow("ԭͼ��", binary_img);
	//imshow("��ֵ��ͼ��1", small_img);
	//imshow("��ֵ��ͼ��2", binary_img1);

	cv::waitKey(0);
	cv::destroyAllWindows();

	return 0;
}