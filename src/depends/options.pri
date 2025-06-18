CONFIG += c++14

win32 {
    QMAKE_CXXFLAGS += /openmp
    QMAKE_CFLAGS += /openmp
}

unix {
    QMAKE_CXXFLAGS += -fopenmp
    QMAKE_CFLAGS += -fopenmp
    LIBS += -fopenmp
}
## LIBS += -fopenmp
win32 {
    DLLDESTDIR += C:/welkin/bin
}
unix {
    DLLDESTDIR += /welkin/panda/app
}
