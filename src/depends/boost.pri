win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = boost-1_82
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = -vc143-mt-gd-x64-1_82
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        SUBFIX = -vc143-mt-x64-1_82
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS +=-L$$BAMBOO_LIB_PATH -llibboost_filesystem$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -llibboost_system$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -llibboost_program_options$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -llibboost_exception$$SUBFIX
}
unix {
    
}