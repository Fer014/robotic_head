#include "include.h"
#include "Visualizer.h"
#include "time.h"
#include "rclcpp/rclcpp.hpp"
#include "camera_custom_interfaces/msg/detection.hpp"
#include "camera_custom_interfaces/msg/detections.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// PCL and OpenCV headers
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/dnn.hpp>
// #include <librealsense2/rs.hpp>
// #include <dlib/gui_widgets.h>

using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace rs2;

const size_t inWidth = 416;
const size_t inHeight = 416;
const float WHRatio = inWidth / (float)inHeight;
const float inScaleFactor = 1 / 255.f;
const float meanVal = 127.5;
Eigen::Matrix<float, 3, 3> MTR;  // Camera coordinate rotation matrix
Eigen::Vector3f V_T;  // Translation vector T
Eigen::Matrix<float, 3, 3> Inner_Transformation_Depth, InnerTransformation_Color;  // Camera intrinsics
cv::Mat Depthmate, Dec_mat, color_mat;
Net net;  // DNN net
std::vector<String> outNames;
rs2::stream_profile dprofile;
rs2::stream_profile cprofile;
vector<string> classNamesVec;
const auto window_name1 = "RGB Image";
const auto window_name2 = "Test Image";
rs2::pipeline pipes;  // Realsense pipeline
vector<Objection> ObjectionOfOneMat;  // Targets in one frame

string package_share_directory = ament_index_cpp::get_package_share_directory("dnn_yolo");
String yolo_tiny_model = package_share_directory + "/engine/enetb0-coco_final.weights";
String yolo_tiny_cfg = package_share_directory + "/engine/enet-coco.cfg";
String classname_path = package_share_directory + "/engine/coco.names";


class DetectionNode : public rclcpp::Node
{
public:
    DetectionNode() : Node("detection_node") {
        publisher_ = this->create_publisher<camera_custom_interfaces::msg::Detections>("detection_info", 10);
        timer_ = this->create_wall_timer(
            1ms, std::bind(&DetectionNode::timer_callback, this));

        rs2::colorizer color_map;
        Realsense_config();
        Get_referance();
        image_detection_Cfg();
        
        auto config = pipes.start();
        auto profile = config.get_stream(RS2_STREAM_COLOR).as<video_stream_profile>();
        rs2::align align_to(RS2_STREAM_COLOR);
        Size cropSize;
        if (profile.width() / (float)profile.height() > WHRatio) {
            cropSize = Size(static_cast<int>(profile.height() * WHRatio), profile.height());
        } else {
            cropSize = Size(profile.width(), static_cast<int>(profile.width() / WHRatio));
        }

        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.55f);
        namedWindow(window_name1, WINDOW_AUTOSIZE);
        namedWindow(window_name2, WINDOW_AUTOSIZE);
        Win_3D.set_title("ALL Objection 3D Point Cloud");
    }

private:
    void timer_callback() {
        auto data = pipes.wait_for_frames();
        data = align_to.process(data);

        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();
        depth_frame = Hole_Filling_filter.process(depth_frame);

        color_mat = frame_to_mat(color_frame);
        Mat depth_mat(Size(640, 480), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        Depthmate = depth_mat;

        Win_3D.clear_overlay();
        ObjectionOfOneMat.clear();
        Dec_mat = Dectection(color_mat);
        Win_3D.add_overlay(Visualizer{ObjectionOfOneMat}.Report_PCLOneMat());

        auto detections_msg = gather_detections();
        publisher_->publish(detections_msg);

        imshow(window_name1, Dec_mat);
        imshow(window_name2, Depthmate);
        if (cv::waitKey(1) == 32) {
            Win_3D.wait_until_closed();
        }
    }

    camera_custom_interfaces::msg::Detections gather_detections()
    {
        camera_custom_interfaces::msg::Detections msg;
        for (const auto& objection : ObjectionOfOneMat) {
            camera_custom_interfaces::msg::Detection detection_msg;
            detection_msg.class_name = objection.Classname;
            detection_msg.class_id = objection.ClassID;
            detection_msg.confidence = objection.confidence;
            detection_msg.x = objection.getAreaObjectionR().x;
            detection_msg.y = objection.getAreaObjectionR().y;
            detection_msg.width = objection.getAreaObjectionR().width;
            detection_msg.height = objection.getAreaObjectionR().height;
            //RCLCPP_INFO(this->get_logger(), "class_name=%s, class_id=%d, (X,Y)=(%d, %d)", detection_msg.class_name.c_str(), detection_msg.class_id, detection_msg.x, detection_msg.y);
            msg.detections.push_back(detection_msg);
        }

        return msg;
    }

    Mat Dectection(Mat color_mat)
    {
        Mat inputBlob = blobFromImage(color_mat, inScaleFactor, Size(inWidth, inHeight), Scalar(), true, false);
        net.setInput(inputBlob);
        std::vector<Mat> outs;
        net.forward(outs, outNames);
        vector<double> layersTimings;
        double freq = getTickFrequency() / 1000;
        double time = net.getPerfProfile(layersTimings) / freq;
        double FPS = 1000 / time;
        ostringstream ss;
        ss << "FPS: " << FPS;
        putText(color_mat, ss.str(), Point(0, 10), 0, 0.5, Scalar(255, 0, 0));

        vector<Rect> boxes;
        vector<int> classIds;
        vector<float> confidences;
        for (size_t i = 0; i < outs.size(); ++i) {
            float* data = (float*)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > 0.5) {
                    int centerX = (int)(data[0] * color_mat.cols);
                    int centerY = (int)(data[1] * color_mat.rows);
                    int width = (int)(data[2] * color_mat.cols);
                    int height = (int)(data[3] * color_mat.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;
                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height));
                }
            }
        }

        vector<int> indices;
        NMSBoxes(boxes, confidences, 0.5, 0.2, indices);
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            Rect box = boxes[idx];
            Objection NewObjection(box, classIds[idx]);
            NewObjection.setConfidence(confidences[idx]);
            NewObjection.setClassname(classNamesVec[classIds[idx]]);
            ObjectionOfOneMat.push_back(NewObjection);
            rectangle(color_mat, box, Scalar(0, 0, 255), 2, 8, 0);
        }
        return color_mat;
    }

    // void Realsense_config() {
    //     rs2::config cfg;
    //     cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //     cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    //     rs2::pipeline_profile profile = pipes.start(cfg);
    //     auto sensor = profile.get_device().first<rs2::depth_sensor>();
    //     if (sensor && sensor.is<rs2::depth_stereo_sensor>()) {
    //         sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_DENSITY);
    //         sensor.set_option(RS2_OPTION_MAX_DISTANCE, 2);
    //     }
    // }

    int Realsense_config()
    {
        rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

        rs2::context ctx;
        auto devs = ctx.query_devices();
        int device_num = devs.size();
        std::cout<<"device num: "<<device_num<<std::endl;

        rs2::device dev = devs[0];
        char serial_number[100] = {0};
        strcpy(serial_number, dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        printf("serial_number: %s\n",serial_number);

        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        pipes.start(cfg);

        rs2::frameset data;
        data = pipes.wait_for_frames();

        rs2::depth_frame depth = data.get_depth_frame();
        rs2::video_frame color = data.get_color_frame();
        dprofile =  depth.get_profile();
        cprofile =  color.get_profile();
        return EXIT_SUCCESS;
    }   


    Mat frame_to_mat(const rs2::frame& f)
    {
        using namespace cv;
        using namespace rs2;
        auto vf = f.as<video_frame>();
        const int w = vf.get_width();
        const int h = vf.get_height();
        if (vf.get_profile().format() == RS2_FORMAT_BGR8)
            return Mat(Size(w, h), CV_8UC3, (void*)vf.get_data(), Mat::AUTO_STEP);
        else if (vf.get_profile().format() == RS2_FORMAT_RGB8) {
            auto r = Mat(Size(w, h), CV_8UC3, (void*)vf.get_data(), Mat::AUTO_STEP);
            cvtColor(r, r, COLOR_RGB2BGR);
            return r;
        } else if (vf.get_profile().format() == RS2_FORMAT_Y8)
            return Mat(Size(w, h), CV_8UC1, (void*)vf.get_data(), Mat::AUTO_STEP);
        else if (vf.get_profile().format() == RS2_FORMAT_Z16)
            return Mat(Size(w, h), CV_16UC1, (void*)vf.get_data(), Mat::AUTO_STEP);
        throw std::runtime_error("Frame format is not supported yet!");
    }

    int Get_referance()
    {
        auto sensor = pipes.get_active_profile().get_device().first<rs2::depth_sensor>();
        auto depth_scale = sensor.get_depth_scale();
        dprofile = pipes.get_active_profile().get_stream(RS2_STREAM_DEPTH);
        cprofile = pipes.get_active_profile().get_stream(RS2_STREAM_COLOR);
        rs2::video_stream_profile cvsprofile(cprofile);
        rs2_intrinsics color_intrin = cvsprofile.get_intrinsics();
        InnerTransformation_Color << color_intrin.fx, 0, color_intrin.ppx, 0, color_intrin.fy, color_intrin.ppy, 0, 0, 1;

        rs2::video_stream_profile dvsprofile(dprofile);
        rs2_intrinsics depth_intrin = dvsprofile.get_intrinsics();
        Inner_Transformation_Depth << depth_intrin.fx, 0, depth_intrin.ppx, 0, depth_intrin.fy, depth_intrin.ppy, 0, 0, 1;

        rs2_extrinsics extrin = dprofile.get_extrinsics_to(cprofile);
        MTR << extrin.rotation[0], extrin.rotation[1], extrin.rotation[2],
              extrin.rotation[3], extrin.rotation[4], extrin.rotation[5],
              extrin.rotation[6], extrin.rotation[7], extrin.rotation[8];
        V_T << extrin.translation[0], extrin.translation[1], extrin.translation[2];
        pipes.stop();
        return EXIT_SUCCESS;
    }

    void image_detection_Cfg()
    {
        net = readNetFromDarknet(yolo_tiny_cfg, yolo_tiny_model);
        net.setPreferableBackend(DNN_BACKEND_OPENCV);
        net.setPreferableTarget(DNN_TARGET_CPU);
        outNames = net.getUnconnectedOutLayersNames();
        ifstream classNamesFile(classname_path);
        if (classNamesFile.is_open()) {
            string className = "";
            while (std::getline(classNamesFile, className))
                classNamesVec.push_back(className);
        }
    }

    rclcpp::Publisher<camera_custom_interfaces::msg::Detections>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rs2::align align_to{RS2_STREAM_COLOR};
    rs2::spatial_filter spat_filter;
    rs2::hole_filling_filter Hole_Filling_filter{1};
    dlib::perspective_window Win_3D;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionNode>());
    rclcpp::shutdown();
    return 0;
}
