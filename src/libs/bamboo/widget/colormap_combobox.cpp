#include "colormap_combobox.h"
#include <opencv2/opencv.hpp>
namespace welkin::bamboo {
ColorMapComboBox::ColorMapComboBox(QWidget *parent)
        : QComboBox(parent) {
    this->addItem(tr("NONE"), QVariant(0));
    this->addItem(tr("HOT"), QVariant(cv::COLORMAP_HOT));
    this->addItem(tr("BONE"), QVariant(cv::COLORMAP_BONE));
    //this->addItem(tr("DEEPGREEN"), QVariant(cv::COLORMAP_DEEPGREEN));
    this->addItem(tr("MAGMA"), QVariant(cv::COLORMAP_MAGMA));
    //this->addItem(tr("INFERNO"), QVariant(cv::COLORMAP_INFERNO));
    //this->addItem(tr("RAINBOW"), QVariant(cv::COLORMAP_RAINBOW));
    this->addItem(tr("OCEAN"), QVariant(cv::COLORMAP_OCEAN));
    this->addItem(tr("COOL"), QVariant(cv::COLORMAP_COOL));
    this->setCurrentIndex(1);
    // COLORMAP_AUTUMN
    // COLORMAP_BONE
    // COLORMAP_JET
    // COLORMAP_WINTER
    // COLORMAP_RAINBOW
    // COLORMAP_OCEAN
    // COLORMAP_SUMMER
    // COLORMAP_SPRING
    // COLORMAP_COOL
    // COLORMAP_HSV
    // COLORMAP_PINK
    // COLORMAP_HOT
    //
    connect(this, SIGNAL(currentIndexChanged(int)),
        this, SLOT(onCurrentIndex(int)));
}

ColorMapComboBox::~ColorMapComboBox() {

}

int ColorMapComboBox::getCurrentType() const {
    return this->currentData().toInt();
}
void ColorMapComboBox::onCurrentIndex(int /*index*/) {
    emit currentTypeChanged(getCurrentType());
}
}