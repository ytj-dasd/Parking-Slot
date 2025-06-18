#pragma once
#include "bamboo/base/macro.h"
#include <bamboo/base/osg_utils.h>
#include <bamboo/widget/bproject_action.h>

namespace welkin::bamboo {
class BAMBOO_EXPORT MakeModelListAction : public BProjectAction {
    Q_OBJECT
public:
    explicit MakeModelListAction(BProjectWidget* project_widget = nullptr);
    virtual ~MakeModelListAction();
    QString getID() override {
        return QString("make_model_list");
    }
protected:
    bool isDataPrepared() override;
    bool prepareParameter() override;
    void process() override;
private:
    int _color_map = 0;
    double _min_ratio = 0.05;
    double _max_ratio = 0.9;
};

}
