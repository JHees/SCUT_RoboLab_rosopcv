#include"ros/ros.h"
#include<iostream>
#include <visualization_msgs/Marker.h>
#include<opencv2/opencv.hpp>
#include<stdlib.h>
#include "functions.h"

int main(int argc,char**argv)
{
    ros::init(argc,argv,"lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("vis_maker",10);
    ros::Rate loop_rate(30);

    cv::VideoCapture cap("/home/jhees/Desktop/water.avi");
    cv::Mat frame;
    while(ros::ok())
    {
        
       
       cap>>frame;
        if(frame.empty())
        {
            break;
        }
    cv::Mat reduce;
    colorReduce(frame,reduce,16);
    cv::Mat HSV;
    cv::cvtColor(reduce,HSV,cv::COLOR_RGB2HSV);
    std::vector <cv::Mat> channels;
    split(HSV,channels);
    cv::Mat Value_sholded;
    myShold(channels[2],Value_sholded,[](int i)->uchar{return i>125?255:0;});
    cv::morphologyEx(Value_sholded,Value_sholded,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(7,7)));
    std::vector<std::vector<cv::Point>>contours;
    cv::findContours(Value_sholded,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    std::vector<cv::RotatedRect> rRect;
    for(auto i:contours)
    {
        rRect.push_back(cv::minAreaRect(i));
    }
    for (auto &i : rRect)
    {
        if (i.size.width < i.size.height)
        {
            i.angle += 90;
            int buf = i.size.width;
            i.size.width = i.size.height;
            i.size.height = buf;
        }
        i.angle+=i.angle<0?180:0;
    }
    for(size_t i=0;i<rRect.size();++i)
    {
        if(rRect[i].size.width/rRect[i].size.height<3.2||(rRect[i].angle<40||rRect[i].angle>140))
        {
            rRect.erase(rRect.begin()+i);
           --i;
        }
    }

    std::vector<cv::Point2f> corner;
    cv::Point2f vertices[4];
    auto i = rRect[0].center.x<rRect[1].center.x?rRect[0]:rRect[1];
    i.points(vertices);
    corner.push_back(vertices[0]);
    corner.push_back(vertices[3]);
    i = rRect[0].center.x<rRect[1].center.x?rRect[1]:rRect[0];
    i.points(vertices);
    corner.push_back(vertices[2]);
    corner.push_back(vertices[1]);

    const std::vector<cv::Point3f>corner_w{cv::Point3f(-6.75,-2.75,0),
                                                                                        cv::Point3f(-6.75,2.75,0),
                                                                                        cv::Point3f(6.75,2.75,0),
                                                                                        cv::Point3f(6.75,-2.75,0)};
    const cv::Mat cameraM = (cv::Mat_<float>(3,3)<<1128.048344,0                            ,339.421769,
                                                                                                        0                       ,1127.052190    ,236.535242,
                                                                                                        0                       ,0                            ,1);
    const cv::Mat distCoeffs = (cv::Mat_ <float>(1,5)<<-0.568429,0.514592,-0.000126,0.000500,0);
    cv::Mat Rvec(3,1,CV_64FC1),Tvec(3,1,CV_64FC1);
    cv::Mat rotMat(3,3,CV_64FC1);
    cv::solvePnP(corner_w,corner,cameraM,distCoeffs,Rvec,Tvec);
    Rvec.convertTo(Rvec,CV_32FC1);
    Tvec.convertTo(Tvec,CV_64FC1);
    Rvec.ptr<float>(0)[0]= abs(Rvec.ptr<float>(0)[0]);
    Rvec.ptr<float>(0)[1]= abs(Rvec.ptr<float>(0)[1]);
    cv::Rodrigues(Rvec,rotMat);
    rotMat.convertTo(rotMat,CV_64FC1);


    std::cout<<Rvec<<std::endl;
   // cv::transpose(Tvec,Tvec);
    std::vector<cv::Point3f>corner_cam(corner_w);
    for(auto &i:corner_cam)
    {
        cv::Mat buf(1,3,CV_64FC1);
        cv::Mat(i).convertTo(buf,CV_64FC1);
        buf = rotMat*buf+Tvec;
        i=cv::Point3f(buf);
        //std::cout<<i<<std::endl<<std::endl;
    }

    visualization_msgs::Marker lines,point;
    lines.header.frame_id="/my_frame";
    lines.header.stamp = ros::Time::now();
    lines.ns = "lines";
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w =1.0;

    point.header.frame_id="/my_frame";
    point.header.stamp = ros::Time::now();
    point.ns = "lines";
    point.action = visualization_msgs::Marker::ADD;
    point.pose.orientation.w =1.0;

    lines.id=0;
    point.id=1;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    point.type = visualization_msgs::Marker::POINTS;

    lines.scale.x =0.1;
    lines.color.r=1.0;
    lines.color.a=1.0;

    point.scale.x=1;
    point.color.g =1.0;
    point.color.a=1.0;

    for(size_t i=0;i<4;++i)
    {        
        geometry_msgs::Point p[2];
        p[0].x=corner_cam[i].x;
        p[0].y=corner_cam[i].y;
        p[0].z=corner_cam[i].z;
        p[1].x=corner_cam[(i+1)%4].x;
        p[1].y=corner_cam[(i+1)%4].y;
        p[1].z=corner_cam[(i+1)%4].z;
        lines.points.push_back(p[0]);
        lines.points.push_back(p[1]);
    }
        geometry_msgs::Point p;
        p.x=cv::Point3f(Tvec).x;
        p.y=cv::Point3f(Tvec).y;
        p.z=cv::Point3f(Tvec).z;
        point.points.push_back(p);
        marker_pub.publish(lines);
        marker_pub.publish(point);
        loop_rate.sleep();

    }
    cap.release();
}