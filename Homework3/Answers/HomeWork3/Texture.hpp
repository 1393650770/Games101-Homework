//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        u=std::clamp(u,0.0f,1.0f);
        v=std::clamp(v,0.0f,1.0f);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u=std::clamp(u,0.0f,1.0f);
        v=std::clamp(v,0.0f,1.0f);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        float uMax=std::min((float)width,std::ceil(u_img));
        float uMin=std::max(0.0f,std::floor(u_img));
        float vMax=std::min((float)height,std::ceil(v_img));
        float vMin=std::max(0.0f,std::floor(v_img));

        //Up Left Point
        auto colorUL = image_data.at<cv::Vec3b>(vMax,uMin );
        //Up Right Point
        auto colorUR = image_data.at<cv::Vec3b>(vMax, uMax);
        //Down Left Point
        auto colorDL = image_data.at<cv::Vec3b>(vMin, uMin);
        //Down Right Point
        auto colorDR = image_data.at<cv::Vec3b>(vMin, uMax);
        
        float uLerpNum=(u_img-uMin)/(uMax-uMin);
        float vLerpNum=(v_img-vMin)/(vMax-vMin);

        //U Up Lerp
        auto colorUp_U_Lerp=uLerpNum*colorUL+(1-uLerpNum)*colorUR;
        //U Down Lerp
        auto colorDown_U_Lerp= uLerpNum*colorDL+(1-uLerpNum)*colorDR;
        //V Lerp
        auto color = vLerpNum*colorDown_U_Lerp+(1-vLerpNum)*colorUp_U_Lerp;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    
};
#endif //RASTERIZER_TEXTURE_H
