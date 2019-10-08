#pragma once
#include<iostream>
#include<opencv2/opencv.hpp>

void colorReduce(const cv::Mat& input, cv::Mat& output, int div)
{
    cv::Mat Table(1, 256, CV_8U);
    uchar* p = Table.data;
    for (int i = 0; i < 256; ++i)
    {
        p[i] = i / div * div + div / 2;
    }
    cv::LUT(input, Table, output);
}
void myShold (const cv::Mat &mat1,cv::Mat &mat2,uchar(*tb)(int) )
{
    cv::Mat Table(1, 256, CV_8U);
    auto p = Table.data;
    for (int i = 0; i < 256; ++i)
    {
        p[i] =tb(i);
    }
    cv::LUT(mat1, Table, mat2);
};