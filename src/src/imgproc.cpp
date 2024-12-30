#include <opencv2/opencv.hpp>
#include "imgproc.hpp"


using namespace std;
using namespace cv;

uint16_t count_time=0;

int turn_left(int dir)
{
	dir--;
	if (dir < 0) return 3;
	return dir;
}

int turn_right(int dir)
{
	dir++;
	if (dir > 3) return 0;
	return dir;
}

bool IS_POINT_BLACK(Points this_point,Mat& img)
{
	if (this_point.col <= 0 || this_point.col >= 319)
	{
		return true;
	}
	else if (img.at<uchar>(this_point.row, this_point.col) == 0)
	{
		return true;
	}

	return false;
}

Points get_next_point(Points now_point, int dir)
{
	switch (dir)
	{
	case UP:
		now_point.row--;
		break;
	case DOWN:
		now_point.row++;
		break;
	case LEFT:
		now_point.col--;
		break;
	case RIGHT:
		now_point.col++;
		break;
	default:
		break;
	}

	return now_point;
}

Points get_left_point(Points now_point, int dir)
{
	switch (dir)
	{
	case UP:
		now_point.col--;
		break;
	case DOWN:
		now_point.col++;
		break;
	case LEFT:
		now_point.row++;
		break;
	case RIGHT:
		now_point.row--;
		break;
	default:
		break;
	}

	return now_point;
}

Points get_right_point(Points now_point, int dir)
{
	switch (dir)
	{
	case UP:
		now_point.col++;
		break;
	case DOWN:
		now_point.col--;
		break;
	case LEFT:
		now_point.row--;
		break;
	case RIGHT:
		now_point.row++;
		break;
	default:
		break;
	}

	return now_point;
}

void recursion_maze_left(Mat& img, Edge* left_boundary,int dir,Points now_point, uint8_t img_type)
{
	//Points now_point = left_boundary->point[left_boundary->real_size - 1];
	Points left_point = get_left_point(now_point, dir);//������ǽ
	Points next_point = get_next_point(now_point, dir);//ǰ��������һ��
	if (now_point.row >= using_top_row && (next_point.col <= 318 && next_point.col >= 1)  &&count_time <= 600)
	{
		count_time++;
		if (IS_POINT_BLACK(left_point,img))//������ǽ
		{
			//cout << "11" << endl;
			if (IS_POINT_BLACK(next_point, img))//ǰ����ǽ
			{
				//cout << "22" << endl;
				dir = turn_right(dir);
				recursion_maze_left(img, left_boundary, dir, now_point, img_type);
			}//��ת
			else
			{
				//cout << "33" << endl;
				if (img_type == origin_image)
				{
					if (dir == UP)
					{
						left_boundary->point[left_boundary->real_size] = next_point;
						left_boundary->real_size++;
					}
				}
				else
				{
					left_boundary->point[left_boundary->real_size] = next_point;
					left_boundary->real_size++;
				}
				recursion_maze_left(img, left_boundary, dir, next_point,img_type);
			}//ǰ��
		}
		else
		{
			//cout << "44" << endl;
			dir = turn_left(dir);
			if (img_type == origin_image)
			{
				if (dir == UP)
				{
					left_boundary->point[left_boundary->real_size] = left_point;
					left_boundary->real_size++;
				}
			}
			else
			{
				left_boundary->point[left_boundary->real_size] = left_point;
				left_boundary->real_size++;
			}
			recursion_maze_left(img, left_boundary, dir, left_point, img_type);
		}//��ת��ǰ��
	}
	else
	{
		//cout << count_time << endl;
	}
}

void recursion_maze_right(Mat& img, Edge* right_boundary, int dir, Points now_point, uint8_t img_type)
{
	//Points now_point = left_boundary->point[left_boundary->real_size - 1];
	Points right_point = get_right_point(now_point, dir);//������ǽ
	Points next_point = get_next_point(now_point, dir);//ǰ��������һ��
	if (now_point.row >= using_top_row && (next_point.col >=1 && next_point.col <= 318) && count_time<=600)
	{
		count_time++;
		if (IS_POINT_BLACK(right_point, img))//������ǽ
		{
			//cout << "11" << endl;
			if (IS_POINT_BLACK(next_point, img))//ǰ����ǽ
			{
				//cout << "22" << endl;
				dir = turn_left(dir);
				recursion_maze_right(img, right_boundary, dir, now_point, img_type);
			}//��ת
			else
			{
				//cout << "33" << endl;
				if (img_type == origin_image)
				{
					if (dir == UP)
					{
						right_boundary->point[right_boundary->real_size] = next_point;
						right_boundary->real_size++;
					}
				}
				else
				{
					right_boundary->point[right_boundary->real_size] = next_point;
					right_boundary->real_size++;
				}
				recursion_maze_right(img, right_boundary, dir, next_point, img_type);
			}//ǰ��
		}
		else
		{
			//cout << "44" << endl;
			dir = turn_right(dir);
			if (img_type == origin_image)
			{
				if (dir == UP)
				{
					right_boundary->point[right_boundary->real_size] = right_point;
					right_boundary->real_size++;
				}
			}
			else
			{
				right_boundary->point[right_boundary->real_size] = right_point;
				right_boundary->real_size++;
			}

			recursion_maze_right(img, right_boundary, dir, right_point, img_type);
		}//��ת��ǰ��
	}
	else
	{
		//cout << count_time << endl;
	}
}

void lefthand_maze(Mat& img,Edge *left_boundary, int start_row, int start_col, uint8_t img_type)
{
	left_boundary->point[0].row = start_row;
	left_boundary->point[0].col = start_col;
	left_boundary->real_size = 1;
	int dir = UP;
	//��ʼ����߽�
	count_time = 0;
	recursion_maze_left(img, left_boundary, dir, left_boundary->point[0], img_type);//�ݹ�Ѱ�ұ���

	/*
	for (int i = 0;i < left_boundary->real_size;i++)
	{
		cout << left_boundary->point[i].row << " " << left_boundary->point[i].col << endl;
	}
	*/
}

void righthand_maze(Mat& img,Edge *right_boundary, int start_row, int start_col, uint8_t img_type)
{
	right_boundary->point[0].row = start_row;
	right_boundary->point[0].col = start_col;
	right_boundary->real_size = 1;
	int dir = UP;
	count_time = 0;
	recursion_maze_right(img, right_boundary, dir, right_boundary->point[0], img_type);//�ݹ�Ѱ�ұ���
}

bool search_bottom_boundary(Mat& img, int* start_row,int* left,int* right)
{
	// //�������ѡ����ɫ��ѡ��,˳���ͰѰ�����һ��ʶ����,Ҫ�ǳ���ʮ����׿���ֱ�Ӽ�
	int block_num = 0, start[15], end[15],biggest_block=0;
	int col = 0;
	// while(img.at<uchar>(*start_row, 0) != 0 ) (*start_row)--;
	
	while (col < 319)
	{
		while (img.at<uchar>(*start_row, col) == 0 && col < 319) col++;//��ɫ
		start[block_num] = col;
		while (img.at<uchar>(*start_row, col) != 0 && col < 319) col++;//��ɫ
		end[block_num++] = col;
	}//�Ұ׿�
	if (block_num == 1)
	{
		*left = start[0];
		*right = end[0]-1;
		return true;
	}//������������Ҫ����
	else 
	{
		int temp = 0;
		if (block_num <= 2)
		{
			while (block_num >= 0)
			{
				if (end[block_num] - start[block_num] > temp)
				{
					temp = end[block_num] - start[block_num];
					biggest_block = block_num;
				}
				block_num--;
			}
			if (temp >= using_min_block_length)
			{
				*left = start[biggest_block];
				*right = end[biggest_block] - 1;
				return true;
			}//˵����ͦ���ģ��͵����ǰ�
		}
		else
		{
			//cout << "zebra!!!" << endl;
			(*start_row) -= 5;
			search_bottom_boundary(img, start_row, left, right);
		}//������	�ⲿ��û�����ף������ǻ������ɫ���м俪ʼ�����Ǻ�����
	}


	return false;
}

void search_bottom_left(Mat& img, int* start_row,int* left)
{
	int col=0;
	if(*left==0)
	{
		while(img.at<uchar>(*start_row, col) != 0 && (*start_row >= using_top_row)) (*start_row)--;//找到最左边不是白色的行
		while(img.at<uchar>(*start_row, col) == 0 && col <= 319) col++;//找到最左边不是黑色的点
		*left = col; 
	}
}

void search_bottom_right(Mat& img, int* start_row,int* right)
{
	int col=319;
	if(*right==319)
	{
		while(img.at<uchar>(*start_row, col) != 0 && (*start_row >= using_top_row)) (*start_row)--;//找到最右边不是白色的行
		while(img.at<uchar>(*start_row, col) == 0 && col >=0) col--;//找到最右边不是黑色的点
		*right = col; 
	}
}

bool findline_maze(Mat& img, int *start_row, Edge *left_edge, Edge *right_edge, uint8_t img_type,  POINT left, POINT right)
{
	//��δ��������ѵĵط�����ײ����ж�������
	//top_left = top_right = 0;
	if (img.data)
	{
		int left_boundary, right_boundary;
		/**/

		
		// search_bottom_boundary(img, start_row, &left_boundary, &right_boundary);//Ѱ�����·����ұ���
		//cout << left_boundary << " " << right_boundary << endl;

		//Point2d left_start(left_boundary, *start_row);
		//Point2d right_start(right_boundary, *start_row);
		//circle(img, left_start, 1, Scalar(0, 255, 255), 3);
		//circle(img, right_start, 1, Scalar(0, 255, 255), 3);
		//point��ֱ�Ӷ�ͼ�෴��������ǰ���õ�������(x,y)
		*start_row = left.x;left_boundary = left.y;
		search_bottom_left(img,start_row,&left_boundary);
		lefthand_maze(img, left_edge, *start_row, left_boundary, img_type);
		*start_row = right.x;right_boundary = right.y;
		search_bottom_right(img,start_row,&right_boundary);
		righthand_maze(img, right_edge, *start_row, right_boundary, img_type);

		//cout << "111" << endl;
	}
	else
	{
		cout << "NO BINARY IMG" << endl;
		return false;
	}

	return true;
}

Mat binaryzation(Mat& frame)
{
	Mat imageGray, imageBinary;

	cvtColor(frame, imageGray, COLOR_BGR2GRAY); // RGBת�Ҷ�ͼ

	threshold(imageGray, imageBinary, 0, 255, THRESH_OTSU); // OTSU��ֵ������

	return imageBinary;
}

void draw_on_img(Mat& img, Edge& left_edge, Edge& right_edge)
{
	for (int i = 0;i < left_edge.real_size;i++)
	{
		circle(img, Point(left_edge.point[i].col, left_edge.point[i].row), 0, Scalar(255, 255, 0), 2);
	}
	for (int i = 0;i < right_edge.real_size;i++)
	{
		circle(img, Point(right_edge.point[i].col, right_edge.point[i].row), 0, Scalar(0, 255, 255), 2);
	}
}

/*
void main()
{
	string	img_path = "source\\cross.jpg";
	Mat img = imread(img_path);
	Mat imgBinary = binaryzation(img);
	Edge left_edge, right_edge;

	findline_maze(imgBinary,220,&left_edge,&right_edge);

	for (int i = 0;i < left_edge.real_size;i++)
	{
		circle(img, Point(left_edge.point[i].col, left_edge.point[i].row), 0, Scalar(255, 255, 0), 2);
		circle(img, Point(right_edge.point[i].col, right_edge.point[i].row), 0, Scalar(0, 255, 255), 2);
	}

	imshow("img binary", imgBinary);
	imshow("img rgb", img);

	waitKey(0);
	exit(0);
}*/