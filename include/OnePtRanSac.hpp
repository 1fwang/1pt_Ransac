#ifndef POLYVIEW_MESOLVER_HPP
#define POLYVIEW_MESOLVER_HPP

#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <types.hpp>
#include <polyview/io/launchHelpers.hpp>

namespace polyview{
    namespace demos{
        void OnePtsolver(std::vector<cv::Mat> &Prev_Imgs,
                         std::vector<cv::Mat> &Curr_Imgs,
                         const polyview::cameras::CameraSystem::Ptr &cam_system,
                         Eigen::Matrix3d &Relative_R);

        void FeatureExtraction(cv::Mat &Prev_Img,
                               cv::Mat &Curr_Img,
                               const polyview::cameras::Camera &camera,
                               opengv::bearingVectors_t  &bearingvector1,
                               opengv::bearingVectors_t  &bearingvector2);

        class Histogram_1pt {
        public:
            explicit Histogram_1pt(std::vector<double> data);
            void peak(double &peak, size_t &index);

            int binNum;
            double binWidth;
            std::vector<std::vector<double>> values;
            std::vector<std::vector<size_t>> idx;
        };

        std::vector<size_t> sort_indexes(const std::vector<double> &v);
    }
}

#endif //POLYVIEW_SCALESOLVER_HPP