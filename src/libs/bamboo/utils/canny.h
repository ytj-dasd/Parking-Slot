#pragma once
#include "bamboo/base/macro.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace welkin::bamboo {
// sobel算子
//阶乘
int factorial(int n);
 
//获得Sobel平滑算子
cv::Mat getSobelSmoooth(int wsize);
 
//获得Sobel差分算子
cv::Mat getSobeldiff(int wsize);
 
//卷积实现
void conv2D(const cv::Mat& src, cv::Mat& dst, cv::Mat kernel, int ddepth, 
    cv::Point anchor = cv::Point(-1, -1), int delta = 0, int borderType = cv::BORDER_DEFAULT);
 
//可分离卷积———先垂直方向卷积，后水平方向卷积
void sepConv2D_Y_X(const cv::Mat& src, cv::Mat& dst, cv::Mat kernel_Y, cv::Mat kernel_X, int ddepth, 
    cv::Point anchor = cv::Point(-1, -1), int delta = 0, int borderType = cv::BORDER_DEFAULT);
 
//可分离卷积———先水平方向卷积，后垂直方向卷积
void sepConv2D_X_Y(const cv::Mat& src, cv::Mat& dst, cv::Mat kernel_X, cv::Mat kernel_Y, int ddepth, 
    cv::Point anchor = cv::Point(-1, -1), int delta = 0, int borderType = cv::BORDER_DEFAULT);
 
//Sobel算子边缘检测
//dst_X 垂直方向
//dst_Y 水平方向
void Sobel(const cv::Mat& src, cv::Mat& dst_X, cv::Mat& dst_Y, cv::Mat& dst, int wsize, int ddepth, 
    cv::Point anchor = cv::Point(-1, -1), int delta = 0, int borderType = cv::BORDER_DEFAULT);

//确定一个点的坐标是否在图像内
bool checkInRang(int r,int c, int rows, int cols);
 
//从确定边缘点出发，延长边缘
void trace(const cv::Mat &edgeMag_noMaxsup, cv::Mat &edge, float TL, int r, int c, int rows, int cols);

enum EdgeAngles { // 边缘角度
    EDGE_ANGLE_0 = 0x1,       //< 水平边缘
    EDGE_ANGLE_90 = 0x2,      //< 垂直边缘
    EDGE_ANGLE_45 = 0x4,      //< 45度边缘
    EDGE_ANGLE_135 = 0x8,     //< 135度边缘
    EDGE_ALL_ANGLES = EDGE_ANGLE_0 | EDGE_ANGLE_90 | EDGE_ANGLE_45 | EDGE_ANGLE_135
};
// 自定义Canny边缘检测 - 可控制输出边缘角度
void BAMBOO_EXPORT Canny(const cv::Mat &src, cv::Mat &edge, float TL, float TH, 
    EdgeAngles angles = EDGE_ALL_ANGLES, int wsize = 3, bool L2graydient = false);
}