#pragma once
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "VisualOdometry.hpp"



class vSLAM {

    private:
        MonocularOdometry *mono_odom;
        StereoOdometry *stereo_odom;
        std::vector<cv::Mat> poses;
        std::vector<cv::Mat> features;
        std::vector<cv::Mat> pointclouds;

    public:
        vSLAM();

        vSLAM(bool stereo);

        /**
         * @brief 
         * 
         * @param image 
         * @param img_descriptors 
         * @param img_keypoints 
         * @param detector 
         * @return cv::Mat 
         */
        cv::Mat BagOfWords(cv::Mat image, cv::Mat img_descriptors,std::vector<cv::KeyPoint> img_keypoints, cv::Ptr<cv::ORB> detector);

        /**
         * @brief 
         * 
         * @param hist1 
         * @param hist2 
         * @return double 
         */
        double Compare_Histograms(cv::Mat hist1, cv::Mat hist2);


        void Optimize();


        void RunMono(cv::Mat img);


        void RunStereo(cv::Mat left_img, cv::Mat right_img);

};



