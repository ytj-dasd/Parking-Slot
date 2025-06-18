#pragma once
#include "bamboo/base/macro.h"

#include <QString>
#include <QList>
#include <QLineF>
#include <QPolygonF>
#include <QImage>
#include <QObject>
#include <opencv2/opencv.hpp>
#include <uface/logger/ulogger.h>
#include <uface/base/uconverter.h>
#include <common/geometry/line2.h>
#include "common/geometry/polygon2.h"

namespace welkin::bamboo {
class BAMBOO_EXPORT ImageConverter {
public:
    ImageConverter() = default;
    virtual ~ImageConverter() = default;

    // static cv::Mat FromQt(const QImage& image);
    static QImage ToQt(const cv::Mat& mat);
};
}