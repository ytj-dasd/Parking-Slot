win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = FLANN
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = -gd
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        SUBFIX = 
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
#    LIBS +=-L$$BAMBOO_LIB_PATH -lflann$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lflann_s$$SUBFIX
#    LIBS +=-L$$BAMBOO_LIB_PATH -lflann_cpp$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -lflann_cpp_s$$SUBFIX
}
unix {
    
}
