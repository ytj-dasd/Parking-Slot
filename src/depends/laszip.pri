win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = laszip
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = -d
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }

    LIBS  +=-L$$BAMBOO_LIB_PATH -llaszip$$SUBFIX
}
unix {
    ## mkdir build && cd build && cmake .. && sudo make install
    INCLUDEPATH += /usr/include/
    LIBS +=-L/usr/lib -llaszip
}
