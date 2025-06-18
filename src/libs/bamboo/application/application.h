#pragma once
#include <QApplication>
#include <common/param/conf_param_list.h>
#include "bamboo/base/macro.h"
namespace welkin {
class UStyleSheet;
class UTranslator;
}

namespace welkin::bamboo {
class BAMBOO_EXPORT Application : public QApplication {
public:
    Application(int &argc, char **argv);
    virtual ~Application();

    common::ConfParamList* getParamList() {
        return &_param_list;
    }
    void initSheetStyle();
    void initTranslations();
    
    bool installHelpMenu();

    // for OSG
    static void SetSurfaceFormat();
private:
    UStyleSheet* _ssht = nullptr;
    UTranslator* _trsl = nullptr;
    common::ConfParamList _param_list;
};

}
