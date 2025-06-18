#pragma once
#include <bamboo/widget/bproject_action.h>

namespace welkin::bamboo {
class BAMBOO_EXPORT MakeDOMAction : public BProjectAction {
    Q_OBJECT
public:
    explicit MakeDOMAction(BProjectWidget* project_widget = nullptr);
    virtual ~MakeDOMAction();
    
    QString getID() override {
        return QString("make_dom");
    }
protected:
    bool isDataPrepared() override;
    bool prepareParameter() override;
    void process() override;
private:
    double _min_z = -1e6;
    double _max_z = 1e6;

    int _type = 0; 
    int _colormap = 0; 
    double _res = 0.01;
    int _min_value = 2;
    int _max_value = 255;
};
}
