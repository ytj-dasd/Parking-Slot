win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = ceres-2.1.0
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = -debug
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS  +=-L$$BAMBOO_LIB_PATH -lceres$$SUBFIX
}
unix{
    
}