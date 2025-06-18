win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = sam
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = -gd
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        SUBFIX = 
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS +=-L$$BAMBOO_LIB_PATH -lonnxruntime$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lonnxruntime_providers_shared$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lwelkin_sam$$SUBFIX
}
unix {
    
}
