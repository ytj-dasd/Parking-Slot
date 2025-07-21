#include "corner_detector.h"
#include <boost/filesystem.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <onnxruntime_cxx_api.h>
#include <common/base/dir.h>
#include <uface/uface.hpp>
#include "file_system.h"

namespace welkin::bamboo {
CornerDetector::CornerDetector(ParklotFileSystem* fs) : _file_system(fs) {
    Q_ASSERT(fs);
}
CornerDetector::CornerDetector(const std::string& inference_mode, ParklotFileSystem* fs) 
        : _inference_mode(inference_mode), _file_system(fs) {
    Q_ASSERT(fs);
}
CornerDetector::~CornerDetector() {}

const std::string& CornerDetector::getInferenceMode() const {
    return _inference_mode;
}
void CornerDetector::setInferenceMode(const std::string& mode) {
    _inference_mode = mode;
}

const std::string& CornerDetector::getOnnxModelPath() const {
    return _onnx_model_path;
}
void CornerDetector::setOnnxModelPath(const std::string& model_path) {
    _onnx_model_path = model_path;
}

bool CornerDetector::detect() {
    // 创建交点文件夹
    common::Dir::Mkdir(_file_system->getCornerDir());

    UProgressTextValue(QObject::tr("Onnx model initializing"), 0, 100);
    // Loading ONNX model
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
    Ort::SessionOptions session_options;
    if (_inference_mode == "GPU") { // gpu推理模式
        OrtCUDAProviderOptions cuda_options;
        session_options.AppendExecutionProvider_CUDA(cuda_options);
        UInfo(QObject::tr("GPU inference is enabled"));
    } else { // cpu推理模式
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        UInfo(QObject::tr("CPU inference is enabled"));
    }
    // onnx模型初始化
    Ort::Session _onnx_session(env, _onnx_model_path.c_str(), session_options);
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::AllocatedStringPtr input_name_ptr = _onnx_session.GetInputNameAllocated(0, allocator);
    const char* input_name = input_name_ptr.get();
    // std::cout << "Input Name: " << input_name << std::endl;

    Ort::TypeInfo input_type_info = _onnx_session.GetInputTypeInfo(0);
    auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> input_node_dims = input_tensor_info.GetShape();

    size_t num_output_nodes = _onnx_session.GetOutputCount();
    std::vector<Ort::AllocatedStringPtr> output_name_ptrs;
    std::vector<const char*> output_node_names;
    for (size_t i = 0; i < num_output_nodes; i++) {
        output_name_ptrs.emplace_back(_onnx_session.GetOutputNameAllocated(i, allocator));
        const char* output_name = output_name_ptrs.back().get();
        output_node_names.push_back(output_name);
        // std::cout << "Output Name: " << output_name << std::endl;
    }

    // 获取图片全路径名
    auto images_folder = _file_system->getPatchImagesDir();
    auto image_files = common::Dir(images_folder).getFiles(".png", true, true);
    std::vector<KeyPoint> keypoints;
    
    // 角点检测
    int index = 0;
    int nums = image_files.size();
    for (const auto& img_path : image_files) {
        UProgressTextValue(QObject::tr("Search keypoints"), (index++) * 100.0 / nums, 100);
        cv::Mat image = cv::imread(img_path);
        if (image.empty()) {
            UWarn(QObject::tr("Failed to load image: %1").arg(UTQ(img_path)));
            continue;
        }
        cv::resize(image, image, cv::Size(512, 512));
        auto path_without_ext = boost::filesystem::path(img_path).replace_extension("").string();
        std::string pos_txt_path = path_without_ext + "_position.txt";
        std::ifstream pos_file(pos_txt_path);
        std::pair<int, int> patch_position = {0, 0};
        if (pos_file.is_open()) {
            std::string line;
            std::getline(pos_file, line);
            pos_file.close();

            size_t start = line.find('(');
            size_t end = line.find(')');
            if (start != std::string::npos && end != std::string::npos && end > start) {
                std::string pos_info = line.substr(start + 1, end - start - 1);
                size_t comma = pos_info.find(',');
                if (comma != std::string::npos) {
                    int x = std::stoi(pos_info.substr(0, comma));
                    int y = std::stoi(pos_info.substr(comma + 1));
                    patch_position = {x, y};
                }
            }
        } else {
            UWarn(QObject::tr("Failed to open position file: %1").arg(UTQ(pos_txt_path)));
        }

        cv::Mat input_blob;
        cv::dnn::blobFromImage(image, input_blob, 1.0 / 255.0, cv::Size(), cv::Scalar(), false, false, CV_32F);

        float* input_data = (float*)input_blob.data;

        std::vector<int64_t> input_dims = {1, input_blob.size[1], input_blob.size[2], input_blob.size[3]};

        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info, input_data, input_blob.total(),
            input_dims.data(), input_dims.size());

        auto output_tensors = _onnx_session.Run(Ort::RunOptions{nullptr},
                                        &input_name, &input_tensor, 1,
                                        output_node_names.data(), output_node_names.size());


        float* points_pred_data = output_tensors[0].GetTensorMutableData<float>();
        std::vector<int64_t> prediction_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
       
        int height = static_cast<int>(prediction_shape[2]);
        int width = static_cast<int>(prediction_shape[3]);

        float point_thresh = 0.005f;    
        float boundary_thresh = 0.001f; 

        std::vector<std::vector<float>> points_pred;

        for (int i = 0; i < height; ++i) {     
            for (int j = 0; j < width; ++j) { 
                
                int idx_conf = (0 * height + i) * width + j;        
                int idx_offset_x = (1 * height + i) * width + j;    
                int idx_offset_y = (2 * height + i) * width + j;  

                float confidence = points_pred_data[idx_conf];

                if (confidence >= point_thresh) {
                    float offset_x = points_pred_data[idx_offset_x];
                    float offset_y = points_pred_data[idx_offset_y];

                    float xval = (j + offset_x) / static_cast<float>(width);
                    float yval = (i + offset_y) / static_cast<float>(height);

                    if (xval < boundary_thresh || xval > 1.0f - boundary_thresh ||
                        yval < boundary_thresh || yval > 1.0f - boundary_thresh) {
                        continue;
                    }

                    std::vector<float> point = {confidence, xval, yval};
                    points_pred.push_back(point);
                }
            }
        }
        
        // 绘制特征点
        drawParkingSlot(image, points_pred, patch_position, keypoints);
        
        // 保存关键点图片
        std::string save_dir = _file_system->getPredictionsDir();
        boost::filesystem::create_directories(save_dir);
        std::string img_filename = common::Dir(img_path).getAbsFileName();
        std::cout << save_dir + "/" + img_filename << std::endl;
        cv::imwrite(save_dir + "/" + img_filename, image);
        UInfo(QObject::tr("Search keypoints in image: %1 completed.").arg(UTQ(img_filename)));
    }
    std::vector<KeyPoint> final_keypoints = nmsKeypoints(keypoints);
    auto keypoints_path = _file_system->getKeypointsPath();
    std::ofstream ofs(keypoints_path);
    for (const auto& kp : final_keypoints) {
        ofs << kp.x << " " << kp.y << " " << kp.confidence << std::endl;
    }
    ofs.close();
    
    cv::Mat map_image = cv::imread(_file_system->getRgbImagePath());
    if (!map_image.empty()) { // 全图写出
        drawFinalKeypoints(map_image, final_keypoints);
        cv::imwrite(_file_system->getKeypointsImagePath(), map_image);
    }
    UInfo(QObject::tr("Final keypoints saved to %1").arg(UTQ(keypoints_path)));
    return true;
}

std::vector<KeyPoint> CornerDetector::nmsKeypoints(
        std::vector<KeyPoint>& keypoints, float distance_threshold) {
    if (keypoints.empty())
        return {};

    std::sort(keypoints.begin(), keypoints.end(), [](const KeyPoint& a, const KeyPoint& b) {
        return a.confidence > b.confidence;
    });

    std::vector<KeyPoint> nms_keypoints;

    while (!keypoints.empty()) {
        KeyPoint kp = keypoints.front();
        keypoints.erase(keypoints.begin());
        nms_keypoints.push_back(kp);

        keypoints.erase(std::remove_if(keypoints.begin(), keypoints.end(),
            [&kp, &distance_threshold](const KeyPoint& k) {
                float dist = std::sqrt(std::pow(k.x - kp.x, 2) + std::pow(k.y - kp.y, 2));
                return dist <= distance_threshold;
            }), keypoints.end());
    }

    return nms_keypoints;
}
void CornerDetector::drawParkingSlot(
        cv::Mat& image, const std::vector<std::vector<float>>& points_pred,
        const std::pair<int, int>& patch_position, std::vector<KeyPoint>& keypoints) {
    for (const auto& point : points_pred) {
        float confidence = point[0];
        float x_pos = point[1];
        float y_pos = point[2];

        int px = static_cast<int>(std::round(512 * x_pos)) + patch_position.first;
        int py = static_cast<int>(std::round(512 * y_pos)) + patch_position.second;

        keypoints.push_back({px, py, confidence});

        cv::circle(image, cv::Point(static_cast<int>(std::round(512 * x_pos)), 
            static_cast<int>(std::round(512 * y_pos))), 3, cv::Scalar(0, 255, 255), 4);
    }
}
void CornerDetector::drawFinalKeypoints(cv::Mat& image, const std::vector<KeyPoint>& keypoints) {
    for (const auto& kp : keypoints) {
        cv::circle(image, cv::Point(kp.x, kp.y), 6, cv::Scalar(0, 255, 255), 6);
    }
}
}