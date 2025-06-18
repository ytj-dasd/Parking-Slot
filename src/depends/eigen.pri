win32 {
    BAMBOO_DEPENDS_PATH = C:/welkin/depends
    BAMBOO_LIB_NAME = eigen-3.4.0
    
    BAMBOO_LIB_PATH = $$BAMBOO_DEPENDS_PATH/$$BAMBOO_LIB_NAME
    INCLUDEPATH += $$BAMBOO_LIB_PATH/include
}
unix {
    INCLUDEPATH += $$/usr/include/eigen3
}
