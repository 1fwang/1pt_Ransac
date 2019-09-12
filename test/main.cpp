#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <chrono>

#include <polyview/io/launchHelpers.hpp>
#include <OnePtRanSac.hpp>

polyview::cameras::CameraSystem::Ptr
init_cams(std::vector<polyview::tools::OptionSet> & optionSets)
{
    polyview::tools::options::OptionType type;

    // extract the options for the camera
    Eigen::MatrixXd imageSize(2, 1);        imageSize<< 1936, 1216;
    Eigen::MatrixXd principalPoint(2, 1);   principalPoint<< 968.5593,608.854;
    Eigen::MatrixXd focalLength(2,1);       focalLength<< 300.5377,300.5377;
    Eigen::MatrixXd camPosition(3, 1);      camPosition<< 0, 0, 0;
//    Eigen::MatrixXd camOrientation(3, 3);   camOrientation<< 1, 0, 0, 0, 0, 1, 0, -1, 0;
    Eigen::MatrixXd camOrientation(3, 3);   camOrientation<< 1, 0, 0,0, -1, 0, 0, 0, -1;

    polyview::tools::OptionSet optionSet1;
    type = polyview::tools::options::IMAGE_SIZE;        optionSet1.push_back(polyview::tools::Option(type, imageSize));
    type = polyview::tools::options::PRINCIPAL_POINT;   optionSet1.push_back(polyview::tools::Option(type, principalPoint));
    type = polyview::tools::options::FOCAL_LENGTH;      optionSet1.push_back(polyview::tools::Option(type, focalLength));
    type = polyview::tools::options::CAM_POSITION;      optionSet1.push_back(polyview::tools::Option(type, camPosition));
    type = polyview::tools::options::CAM_ORIENTATION;   optionSet1.push_back(polyview::tools::Option(type, camOrientation));

    optionSets.push_back(optionSet1);

    //create the cameras
    polyview::cameras::CameraSystem::Ptr cameraSystem;
    polyview::io::createCameraSystem( optionSets, cameraSystem );

    return cameraSystem;
}


int main() {

    int camera_Num = 1;
    /*READ IMAGES*/
    std::string GTPath("/home/ifwang/datasets/GOVO/association.txt");
    std::vector<std::vector<std::string>> Img_Path;
    std::ifstream GTInput(GTPath, std::ifstream::in);

    std::vector<std::string> tmp;
    tmp.resize(camera_Num);
    char line[512];
    GTInput.getline(line, 512);

    while (!GTInput.eof() && !GTInput.fail()) {
        std::stringstream ss(line, std::stringstream::in);
        for (int i = 0; i < camera_Num; ++i) {
            ss >> tmp[i];
            tmp[i] = "/home/ifwang/datasets/GOVO/" + tmp[i];
        }
        Img_Path.push_back(tmp);
        GTInput.getline(line, 512);
    }


    /* READ THE EXTRINSIC PARAMS */
    std::vector<polyview::tools::OptionSet> optionSets;
    polyview::cameras::CameraSystem::Ptr cameraSystem = init_cams( optionSets );

    Eigen::Matrix3d Relative_R;
    Relative_R = Eigen::Matrix3d::Identity();
    double Angle = 0;

    for (int frame = 0;  frame < Img_Path.size() - 1; frame++) {
        std::vector<cv::Mat> Prev_Imgs, Curr_Imgs;
        Prev_Imgs.resize(camera_Num);
        Curr_Imgs.resize(camera_Num);

        for (int cam = 0; cam < Img_Path[frame].size(); cam++) {
            Prev_Imgs[cam] = cv::imread(Img_Path[frame][cam],0);
            Curr_Imgs[cam] = cv::imread(Img_Path[frame + 1][cam],0);
            std::cout<<Img_Path[frame + 1][cam]<<std::endl;
        }


        auto start = std::chrono::system_clock::now();
        polyview::demos::OnePtsolver(Prev_Imgs,Curr_Imgs,cameraSystem, Relative_R);

        Eigen::Matrix3d C1;
        Eigen::Matrix3d C2;
        Eigen::Matrix3d C;
        C1 = Relative_R-Eigen::Matrix3d::Identity();
        C2 = Relative_R+Eigen::Matrix3d::Identity();
        C = C1 * C2.inverse();

        double z = -C(0,1);
        Angle = Angle + asin((2 * z) / (1 + pow(z, 2)))  / (M_PI / 180.0) ;

        std::cout<<"Angle: "<<Angle<<std::endl;

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    //    std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";


        //Visualizer
        for (int cam = 0; cam < Img_Path[frame].size(); cam++) {
            Prev_Imgs[cam] = cv::imread(Img_Path[frame][cam],0);
            Curr_Imgs[cam] = cv::imread(Img_Path[frame + 1][cam],0);

            cv::Mat Plot_Img = cv::imread(Img_Path[frame][cam]);

            std::vector<cv::Point2f> prev_kp, curr_kp;

            int max_count = 200;	  // maximum number of features to detect
            double qlevel = 0.05;    // quality level for feature detection
            double minDist = 5.0;

            cv::goodFeaturesToTrack(Prev_Imgs[cam], // the image
                                    prev_kp,   // the output detected features
                                    max_count,  // the maximum number of features
                                    qlevel,     // quality level
                                    minDist);

            std::vector<uchar> features_found;
            int win_size = 10;
            cv::calcOpticalFlowPyrLK(
                    Prev_Imgs[cam],                         // Previous image
                    Curr_Imgs[cam],                         // Next image
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

            cv::Mat img_keypoints;

            for (int i = 0; i < static_cast<int>(prev_kp.size()); ++i) {
                if (!features_found[i]) {
                    continue;
                }
                cv::line(
                        Plot_Img,             // Draw onto this image
                        prev_kp[i],                 // Starting here
                        curr_kp[i],                 // Ending here
                        cv::Scalar(0, 255, 0),       // This color
                        1,                           // This many pixels wide
                        cv::LINE_AA                  // Draw line in this style
                );
            }

            cv::imshow("LK Optical Flow Example", Plot_Img);
            cv::waitKey(0);
        }
    }

    return 0;
}