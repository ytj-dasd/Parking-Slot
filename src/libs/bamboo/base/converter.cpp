#include "converter.h"
namespace welkin::bamboo {
// cv::Mat ImageConverter::FromQt(const QImage& image) {
//
// }
QImage ImageConverter::ToQt(const cv::Mat& mat) {
    QImage image;
    switch (mat.type()) {
    case CV_8UC1: {
        image = QImage((const uint8_t*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        return image;
    }
    case CV_8UC3: {
        image = QImage((const uint8_t*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    }
    case CV_8UC4:
        image = QImage((const uint8_t*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGBA8888);
        return image.rgbSwapped();
    default: {
        UError(QObject::tr("Unknown image type: %1").arg(mat.type()));
        return image;
    }
    }
}
}
