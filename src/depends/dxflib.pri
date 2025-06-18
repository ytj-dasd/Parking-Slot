win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = dxflib-3.26.4
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    CONFIG(debug, debug | release) {
#        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
#        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS  +=-L$$BAMBOO_LIB_PATH -ldxflib$$SUBFIX
}
unix {
    
}
