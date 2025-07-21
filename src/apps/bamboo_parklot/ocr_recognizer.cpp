#include "ocr_recognizer.h"
#include <algorithm>
#include <iostream>

namespace welkin::bamboo {
OCRRecognizer::OCRRecognizer(std::string themodle, std::string dict, USE_DEVICE device)
{
    // std::cout << "Load OCRRecognizer Model" << std::endl;
    
    this->ctc_path_str = dict;
    this->ctc_path = const_cast<char*>(this->ctc_path_str.c_str());
    this->modle = std::make_unique<OCRModel>(themodle, device);
       
    std::ifstream ctc_char(ctc_path);
    if (!ctc_char.is_open()) {
        std::cerr << "无法打开字符集文件: " << ctc_path << std::endl;
        return;
    }

    std::string line;
    while (std::getline(ctc_char, line)) {
        if (!line.empty()) {
            ctc_data.push_back(line);
        }
    }
    ctc_char.close();
}

/// @brief 读入图片路径返回图片中的字符串
/// @param path 图片文件路径
/// @return 识别到的字符串列表
std::vector<std::pair<std::string, float>> OCRRecognizer::ocr(const std::string& path){
    cv::Mat inimg = cv::imread(path, cv::IMREAD_GRAYSCALE);
    cv::UMat outimg = inimg.getUMat(cv::ACCESS_RW);
    std::vector<std::pair<std::string, float>> res;
    int imgcol = outimg.cols;
    int imgrow = outimg.rows;
    if (std::min(imgrow, imgcol) < 2){
        return res;
    }
    if (cv::sum(outimg.col(0))[0] / outimg.rows < 145) { 
        cv::subtract(255, outimg, outimg);
    }
    auto imgs = lineSplit(outimg);
    res = ocrForSingleLines(imgs);
    return res;
}

std::vector<std::pair<std::string, float>> OCRRecognizer::ocr(cv::UMat& inimg){
    cv::UMat outimg;
    cv::cvtColor(inimg, outimg, cv::COLOR_RGB2GRAY);
    std::vector<std::pair<std::string, float>> res;
    int imgcol = outimg.cols;
    int imgrow = outimg.rows;
    if (std::min(imgrow, imgcol) < 2){
        return res;
    }
    if (cv::sum(outimg.col(0))[0] < 145) { 
        cv::subtract(255, outimg, outimg);
    }
    auto imgs = lineSplit(outimg);
    res = ocrForSingleLines(imgs);
    return res;
}

/// @brief 读取Mat格式的图片 横向切分成 单列有文字的 图片列表
/// @param inimg 
/// @return 单列有文字的 图片列表
std::vector<cv::UMat> OCRRecognizer::lineSplit(cv::UMat& inimg){
    std::vector<cv::UMat> list;
    int imgcol = inimg.cols;
    int imgrow = inimg.rows;
    cv::UMat outimg;
    cv::subtract(255, inimg, outimg);
    cv::reduce(outimg, outimg, 1, cv::REDUCE_MAX);
    cv::Mat outimgr = outimg.getMat(cv::ACCESS_READ);
    int lineforchar = 0;
    int lineforcharstart = 0;
    for (int i = 0; i < imgrow; i++){
        if (outimgr.at<uchar>(i) > 100){
            if (lineforchar + 1 == i){
                lineforchar++;
            }
            else{
                if (lineforchar - lineforcharstart > 7){
                    int start = lineforcharstart;
                    int end = lineforchar;
                    if (start > 0){
                        start -= 1;
                    }
                    if (end < imgcol - 1){
                        end += 1;
                    }
                    list.push_back(inimg(cv::Rect(0, start, inimg.cols, end - start)));
                }
                lineforcharstart = i;
                lineforchar = i;
            }
        }
    }
    if (lineforchar - lineforcharstart > 5){
        int start = lineforcharstart;
        int end = lineforchar;
        if (start > 0){
            start -= 1;
        }
        if (end < imgcol - 1){
            end += 1;
        }
        list.push_back(inimg(cv::Rect(0, start, inimg.cols, end - start)));
    }
    return list;
}

/// @brief 
/// @param input 
void softmax(cv::Mat &input){
    for (int i = 0; i < input.rows; i++){
        cv::Mat ncdata = input.row(i);
        double t = *std::max_element(ncdata.begin<float>(), ncdata.end<float>());
        cv::exp(ncdata - t, ncdata);
        double t1 = cv::sum(ncdata)[0];
        ncdata = ncdata / t1;
    }
}

template<class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last)
{
    return std::distance(first, std::max_element(first, last));
}

std::vector<uint16_t> vargmax(cv::Mat input) {
    std::vector<uint16_t> res;
    for (int i = 0; i < input.rows; i++){
        cv::Mat ncdata = input.row(i);
        res.push_back(static_cast<uint16_t>(argmax(ncdata.begin<float>(), ncdata.end<float>())));
    }
    return res;
}

std::vector<std::pair<std::string, float>> OCRRecognizer::ocrForSingleLines(const std::vector<cv::UMat>& imgs){
    std::vector<std::pair<std::string, float>> res;
    if (imgs.empty()){
        return res;
    }
    // std::cout << "\thave lines x for ocr: " << imgs.size() << std::endl;
    for (const auto& img : imgs){
        int imgcol = img.cols;
        int imgrow = img.rows;
        // std::cout << "\timgSize: " << imgcol << "," << imgrow << std::endl;
        cv::UMat imgmat = img;
        float ratio = static_cast<float>(imgrow) / 32.0f;
        cv::Size sz(static_cast<int>(imgcol / ratio), 32);

        cv::UMat imgresize(sz, CV_8UC1);
        cv::resize(imgmat, imgresize, sz);
        long long input_height = imgresize.rows;
        long long input_width = imgresize.cols;
        cv::Mat ncdata;
        std::vector<void*> ret_data;
        
        ret_data = this->modle->runRecognize(input_width, input_height * input_width, imgresize.getMat(cv::ACCESS_READ).data);
        {
            ncdata = cv::Mat(*static_cast<int64_t*>(ret_data[0]), 6683, CV_32FC1, static_cast<float*>(ret_data[1]));
            // ncdata = cv::Mat(*static_cast<int64_t*>(ret_data[0]), 11, CV_32FC1, static_cast<float*>(ret_data[1]));
            softmax(ncdata);
        }
        
        cv::Mat matdata = ncdata.clone();
        std::vector<uint16_t> best_path = vargmax(ncdata);
        cv::reduce(matdata, matdata, 1, cv::REDUCE_MAX);
        cv::reduce(matdata, matdata, 0, cv::REDUCE_MIN);
        float zql = matdata.at<float>(0, 0); 
        res.emplace_back(std::make_pair(ctcBest(best_path), zql));
    }
    return res;
}

std::string OCRRecognizer::ctcBest(const std::vector<uint16_t>& data){
    std::string res;
    std::vector<uint32_t> vui; 
    for (auto it = data.begin(); it != data.end(); ++it){
        if (!vui.empty()){
            if (vui.back() != static_cast<uint32_t>(*it)){
                vui.push_back(static_cast<uint32_t>(*it));
            }
        }
        else{
            vui.push_back(static_cast<uint32_t>(*it));
        }
    }
    for (auto i : vui){
        if (i < ctc_data.size()){
            res += ctc_data[i];
        }
    }
    // for (auto data : ctc_data)
    // {
    //     std::cout << data << std::endl;
    // }
    
    return res;
}


OCRRecognizer::~OCRRecognizer()
{
}
}