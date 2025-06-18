win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = libdxfrw-2.2.0
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    CONFIG(debug, debug | release) {
#        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
#        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS  +=-L$$BAMBOO_LIB_PATH -ldxfrw$$SUBFIX
}
unix {
    
}
