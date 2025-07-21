#include "auto_detection_action.h"
#include "auto_detection_dialog.h"
#include <uface/uface.hpp>

namespace welkin::bamboo {
AutoDetectionAction::AutoDetectionAction(BProjectWidget* project_widget)
        : BProjectAction(project_widget, true) {
    _inference_mode = "CPU";
    this->setText(tr("Auto Detection"));
    this->setToolTip(tr("Auto Detection"));
    this->setIcon(QIcon(":/bamboo/images/auto_detection.png"));
    _enable_progress = true;

    _file_system.reset(new ParklotFileSystem());
    _preprocessor.reset(new DataPreProcessor(_file_system.get()));
    _corner_detector.reset(new CornerDetector(_inference_mode, _file_system.get()));
    auto corner_onnx_model_path = UFQ(IMainWindow::Instance()->getConfPath()) + "/onnx/gcn_model.onnx";
    _corner_detector->setOnnxModelPath(corner_onnx_model_path);
    _slot_generator.reset(new SlotGenerator(_file_system.get()));
    _template_matcher.reset(new TemplateMatcher(_file_system.get()));
     auto ocr_detect_onnx_model_path = UFQ(IMainWindow::Instance()->getConfPath()) + "/onnx/ch_PP-OCRv3_det_infer.onnx";
    _template_matcher->setDetectOnnxModelPath(ocr_detect_onnx_model_path);
    auto ocr_recognize_onnx_model_path = UFQ(IMainWindow::Instance()->getConfPath()) + "/onnx/densenet_lite_136-gru.onnx";
    // auto ocr_recognize_onnx_model_path = UFQ(IMainWindow::Instance()->getConfPath()) + "/onnx/number-densenet_lite_136-gru.onnx";    
    _template_matcher->setRecognizeOnnxModelPath(ocr_recognize_onnx_model_path);
    auto dict_path = UFQ(IMainWindow::Instance()->getConfPath()) + "/onnx/label_cn.txt";
    // auto dict_path = UFQ(IMainWindow::Instance()->getConfPath()) + "/onnx/label_number.txt";
    _template_matcher->setDictPath(dict_path);
}

AutoDetectionAction::~AutoDetectionAction() {}

bool AutoDetectionAction::isDataPrepared() {
    // dom和cloud_list均有效
    auto dom_folder = this->getProject()->getDOMFolder();
    auto cloud_list_folder = this->getProject()->getCloudListFolder();
    return dom_folder->isValid() && cloud_list_folder->isValid();
}

bool AutoDetectionAction::prepareParameter() {
    AutoDetectionDialog dlg;
    if (dlg.exec() != QDialog::Accepted) {
        return false;
    }
    _inference_mode = UFQ(dlg.getInferenceMode());
    _corner_detector->setInferenceMode(_inference_mode);
    return true;
}

void AutoDetectionAction::process() {
    // 设置文件系统根目录
    std::string proj_path = this->getProject()->getPath();
    _file_system->setRootDir(proj_path);

    // 数据预处理
    int nums = 4;
    int index = 0;
    // UProgressPrefixRange(tr("Step %1/%2 - data preprocess: ").arg(index + 1).arg(nums), index, nums);
    // _preprocessor->setProject(this->getProject());
    // if (!_preprocessor->process()) {return;}

    // // 角点检测
    // index = 1;
    // UProgressPrefixRange(tr("Step %1/%2 - corner detect: ").arg(index + 1).arg(nums), index, nums);
    // if (!_corner_detector->detect()) {return;}

    // // slots生成
    // index = 2;
    // UProgressPrefixRange(tr("Step %1/%2 - slot generate: ").arg(index + 1).arg(nums), index, nums);
    // if (!_slot_generator->generate()) {return;}

    // 模板匹配
    index = 3;
    UProgressPrefixRange(tr("Step %1/%2 - template match: ").arg(index + 1).arg(nums), index, nums);
    if (!_template_matcher->match()) {return;}
}
}