## message("PWD: $$PWD")
## message("OUT_PWD: $$OUT_PWD")

win32 {
   INCLUDEPATH +=$$PWD/../
   CONFIG(debug, debug | release) {
   LIBS  +=-L$$OUT_PWD/../../libs/bamboo/debug -lbamboo_d
   } else {
   LIBS  +=-L$$OUT_PWD/../../libs/bamboo/release -lbamboo
   }
}
unix {
   INCLUDEPATH +=$$PWD/../
   LIBS  +=-L$$OUT_PWD/../../libs/bamboo -lbamboo

   message("OUT_PWD: $$OUT_PWD/../../libs/bamboo")
}
