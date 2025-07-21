#pragma once
#include <bamboo/widget/bproject_action.h>
#include "file_system.h"
#include "data_preprocessor.h"
#include "corner_detector.h"
#include "slot_generator.h"
#include "template_matcher.h"

namespace welkin::bamboo {
class AutoDetectionAction : public BProjectAction {
    Q_OBJECT
public:
    explicit AutoDetectionAction(BProjectWidget* project_widget = nullptr);
    virtual ~AutoDetectionAction();
    
    QString getID() override {
        return QString("auto_detection");
    }
    
protected:
    bool isDataPrepared() override;
    bool prepareParameter() override;
    void process() override;

private:
    std::string _inference_mode;
    ParklotFileSystem::Ptr _file_system = nullptr;
    DataPreProcessor::Ptr _preprocessor = nullptr;
    CornerDetector::Ptr _corner_detector = nullptr;
    SlotGenerator::Ptr _slot_generator = nullptr;
    TemplateMatcher::Ptr _template_matcher = nullptr;
};
}