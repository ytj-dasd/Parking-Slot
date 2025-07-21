#pragma once
#include <string>
#include <memory>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "ocr_model.h"

namespace welkin::bamboo {

class OCRRecognizer
{
private:
    std::unique_ptr<OCRModel> modle; 
    std::string ctc_path_str;
    char* ctc_path;
    std::vector<std::string> ctc_data;

    std::vector<cv::UMat> lineSplit(cv::UMat& inimg);
    std::vector<std::pair<std::string, float>> ocrForSingleLines(const std::vector<cv::UMat>& inimgs);
    std::string ctcBest(const std::vector<uint16_t>& data);

public:
    OCRRecognizer(std::string themodle = "densenet_lite", std::string dict_path = "label_cn.txt", USE_DEVICE device = USE_DEVICE::CPU);

    std::vector<std::pair<std::string, float>> ocr(const std::string& path);
    std::vector<std::pair<std::string, float>> ocr(cv::UMat& img);

    std::vector<std::string> ocrtable(cv::UMat img);

    ~OCRRecognizer();
};

}
