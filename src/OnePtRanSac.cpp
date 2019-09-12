//#include <iostream>
//#include <Eigen/Eigen>
//#include <opencv2/opencv.hpp>
//#include <EigensolverSacProblem.hpp>
//#include <EigensolverAdapter.hpp>
//#include <RanSac/MultiRansac.hpp>
#include <OnePtRanSac.hpp>
#include <thread>

void polyview::demos::OnePtsolver(std::vector<cv::Mat> &Prev_Imgs,
                                  std::vector<cv::Mat> &Curr_Imgs,
                                  const polyview::cameras::CameraSystem::Ptr &cam_system,
                                  Eigen::Matrix3d &Relative_R) {

    opengv::rotation_t rotation;
    opengv::translation_t position;
    opengv::bearingVectors_t bearingvectors1;
    opengv::bearingVectors_t bearingvectors2;

    rotation = Eigen::Matrix3d::Identity();

    /*Initialize the containers*/

    // ORB extraction
    std::vector<std::thread> OpticalTracking;

    for (int cam = 0; cam < Prev_Imgs.size(); cam++) {
        OpticalTracking.emplace_back(&FeatureExtraction, std::ref(Prev_Imgs[cam]), std::ref(Curr_Imgs[cam]), std::ref(cam_system->camera(cam)), std::ref(bearingvectors1), std::ref(bearingvectors2));
    }

    for (int cam = 0; cam < Prev_Imgs.size(); cam++) {
        OpticalTracking[cam].join();
    }


    Eigen::Vector2d d_value;
    std::vector<double> data;
    std::vector<Eigen::Vector2d> org_D;

    for (int i = 0; i < bearingvectors1.size(); i++) {
        double_t theta;
        Eigen::Vector3d p1, p2;
        p1 = cam_system->orientation(0) * bearingvectors1[i];
        p2 = cam_system->orientation(0) * bearingvectors2[i];
        p1 = p1 / p1.norm();
        p2 = p2 / p2.norm();
//        std::cout<<"p1: "<<p1.transpose()<<std::endl;
//        std::cout<<"p2: "<<p2.transpose()<<std::endl;

        double val;
        val = ((p2[1] * p1[2] - p2[2] * p1[1]) / (p2[0] * p1[2] + p2[2] * p1[0]));
        theta = -2 * std::atan(val);
//        std::cout << "theta: " << theta << std::endl;

        d_value[0] = p2[0] * p1[2] + p2[2] * p1[0];
        d_value[1] = p2[1] * p1[2] - p2[2] * p1[1];
        data.push_back(theta);
        org_D.push_back(d_value);
    }

    //HISTOGRAM VOTING
    double updated_t;
    size_t index;
    polyview::demos::Histogram_1pt histogram(data);
    histogram.peak(updated_t, index);

    //SVD decomposition
    size_t inlierNum = histogram.idx[index].size();
    std::cout<<"inlier Num: "<<inlierNum<<std::endl;
    Eigen::MatrixXd D_mat(inlierNum, 2);

    std::cout<<inlierNum<<std::endl;

    for (size_t i = 0; i < inlierNum; i++) {
        size_t inlier = histogram.idx[index][i];
        D_mat.block(i, 0, 1, 2) = org_D[inlier].transpose();
    }

    Eigen::BDCSVD<Eigen::MatrixXd> svd(D_mat, Eigen::ComputeThinV);
    Eigen::MatrixXd V = svd.matrixV();
    double final_t;
    final_t = 2 * std::atan(V(0, 1) / V(1, 1));
    std::cout << "final_t theta: " << final_t << std::endl;
    Relative_R(0,0) = std::cos(final_t);
    Relative_R(0,1) = -std::sin(final_t);
    Relative_R(1,0) = std::sin(final_t);
    Relative_R(1,1) = std::cos(final_t);

}

void polyview::demos::FeatureExtraction(cv::Mat &Prev_Img,
                                        cv::Mat &Curr_Img,
                                        const polyview::cameras::Camera &camera,
                                        opengv::bearingVectors_t &bearingvector1,
                                        opengv::bearingVectors_t &bearingvector2){

        std::vector<cv::Point2f> prev_kp, curr_kp;

        int max_count = 200  ;	  // maximum number of features to detect
        double qlevel = 0.05;    // quality level for feature detection
        double minDist = 5.0;

        cv::goodFeaturesToTrack(Prev_Img, // the image
                                prev_kp,   // the output detected features
                                max_count,  // the maximum number of features
                                qlevel,     // quality level
                                minDist);

        std::cout<<"feature number:"<<prev_kp.size()<<std::endl;
        std::vector<uchar> features_found;
        int win_size = 10;
        cv::calcOpticalFlowPyrLK(
                Prev_Img,                         // Previous image
                Curr_Img,                         // Next image
                prev_kp,                          // Previous set of corners (from imgA)
                curr_kp,                          // Next set of corners (from imgB)
                features_found,                   // Output vector, each is 1 for tracked
                cv::noArray(),                    // Output vector, lists errors (optional)
                cv::Size(win_size * 2 + 1, win_size * 2 + 1),  // Search window size
                5,                                // Maximum pyramid level to construct
                cv::TermCriteria(
                        cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
                        20,                         // Maximum number of iterations
                        0.3                         // Minimum change per iteration
                )
        );

        for (int pt = 0; pt < prev_kp.size(); pt++) {

            Eigen::Vector2d pt2d_prev(prev_kp[pt].x, prev_kp[pt].y);
            Eigen::Vector2d pt2d_curr(curr_kp[pt].x, curr_kp[pt].y);

            bearingvector1.push_back(camera.camToWorld(pt2d_prev));
            bearingvector2.push_back(camera.camToWorld(pt2d_curr));
        }
}

polyview::demos::Histogram_1pt::Histogram_1pt(std::vector<double> v) {
    std::vector<size_t> sortIndex = sort_indexes(v);

    double IQR = v[sortIndex[v.size() * 3 / 4]] - v[sortIndex[v.size() / 4]];

    binWidth = 2 * IQR / (std::cbrt(double(v.size())));
    double max = v[sortIndex[v.size() - 1]];
    double min = v[sortIndex[0]];
    binNum = ceil((max - min) / binWidth);

    values.resize(binNum);
    idx.resize(binNum);
    for (size_t i = 0; i < v.size(); i++) {
        size_t index = floor((v[i] - min) / binWidth);
        values[index].push_back(v[i]);
        idx[index].push_back(i);
    }
}

void polyview::demos::Histogram_1pt::peak(double &peak, size_t &index) {
    size_t maxSize = 0;

    index = 0;
    for (size_t i = 0; i < values.size(); i++) {
        if (values[i].size() > maxSize) {
            maxSize = values[i].size();
            index = i;
        }
    }

    auto sortIndex = sort_indexes(values[index]);
    peak = values[index][sortIndex[values[index].size() / 2]];
}

std::vector<size_t> polyview::demos::sort_indexes(const std::vector<double> &v) {

    // initialize original index locations
    std::vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

    return idx;
}