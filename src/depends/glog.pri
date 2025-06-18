win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = glog
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME

    DEFINES += GLOG_NO_ABBREVIATED_SEVERITIES

    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = d
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS +=-L$$BAMBOO_LIB_PATH -lglog$$SUBFIX
}
unix{
    INCLUDEPATH += /usr/include
    LIBS +=-L/usr/lib -lglog
}
