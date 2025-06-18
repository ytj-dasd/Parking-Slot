win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = protobuf-3.11
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
    
    CONFIG(debug, debug | release) {
        SUBFIX = d
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Debug
    } else {
        SUBFIX =
        BAMBOO_LIB_PATH = $$BAMBOO_LIB_PATH/Release
    }
    
    LIBS +=-L$$BAMBOO_LIB_PATH -llibprotobuf$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -llibprotobuf-lite$$SUBFIX
    LIBS +=-L$$BAMBOO_LIB_PATH -llibprotoc$$SUBFIX
}
unix{
    INCLUDEPATH += /usr/include
    LIBS +=-L/usr/lib -lprotobuf
    LIBS +=-L/usr/lib -lprotobuf-lite
}
