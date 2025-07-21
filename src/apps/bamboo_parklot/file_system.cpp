#include "file_system.h"

namespace welkin::bamboo {
ParklotFileSystem::ParklotFileSystem() {}
ParklotFileSystem::~ParklotFileSystem() {}
const std::string& ParklotFileSystem::getRootDir() const {
    return _root_dir;
}
void ParklotFileSystem::setRootDir(const std::string& root_path) {
    _root_dir = root_path;
}
std::string ParklotFileSystem::getParklotDir() const {
    return _root_dir + "/parklot";
}
std::string ParklotFileSystem::getPreprocessDir() const {
    return getParklotDir() + "/preprocess";
}
std::string ParklotFileSystem::getCornerDir() const {
    return getParklotDir() + "/corner";
}
std::string ParklotFileSystem::getSlotDir() const {
    return getParklotDir() + "/slot";
}
std::string ParklotFileSystem::getMatchDir() const {
    return getParklotDir() + "/match";
}
std::string ParklotFileSystem::getVectorizationDir() const {
    return getParklotDir() + "/vectorization";
}
std::string ParklotFileSystem::getPatchImagesDir() const {
    return getPreprocessDir() + "/patch_img";
}
std::string ParklotFileSystem::getIntensityImagePath() const {
    return getPreprocessDir() + "/intensity_image.png";
}
std::string ParklotFileSystem::getBinaryIntensityImagePath() const {
    return getPreprocessDir() + "/binary_intensity_image.png";
}
std::string ParklotFileSystem::getHeightDiffImagePath() const {
    return getPreprocessDir() + "/height_diff.png";
}
std::string ParklotFileSystem::getRgbImagePath() const {
    return getPreprocessDir() + "/rgb.png";
}

std::string ParklotFileSystem::getPredictionsDir() const {
    return getCornerDir() + "/predictions";
}
std::string ParklotFileSystem::getKeypointsPath() const {
    return getCornerDir() + "/keypoints.txt";
}
std::string ParklotFileSystem::getKeypointsImagePath() const {
    return getCornerDir() + "/keypoints_image.png";
}

std::string ParklotFileSystem::getSlotsPath() const {
    return getSlotDir() + "/slots.txt";
}
std::string ParklotFileSystem::getUnPairedPointsPath() const {
    return getSlotDir() + "/unpaired_points.txt";
}
std::string ParklotFileSystem::getSlotsImagePath() const {
    return getSlotDir() + "/slots_image.png";
}

std::string ParklotFileSystem::getOCRDir() const {
    return getMatchDir() + "/ocr";
}

std::string ParklotFileSystem::getRgbTemplatesDir() const {
    return getMatchDir() + "/rgb_templates";
}
std::string ParklotFileSystem::getIntensityTemplatesDir() const {
    return getMatchDir() + "/intensity_templates";
}
std::string ParklotFileSystem::getEdgeIntersectImagePath() const {
    return getMatchDir() + "/edge_intersect.png";
}
std::string ParklotFileSystem::getMatchedSlotsImagePath() const {
    return getMatchDir() + "/matched_slots.png";
}
std::string ParklotFileSystem::getMatchedEdgeIntersectImagePath() const {
    return getMatchDir() + "/matched_edge_intersect.png";
}
std::string ParklotFileSystem::getRevisedMatchedSlotsImagePath() const {
    return getMatchDir() + "/revised_matched_slots.png";
}
std::string ParklotFileSystem::getOCRImagePath() const {
    return getMatchDir() + "/OCR.png";
}
std::string ParklotFileSystem::getFinalParkingSlotsPath() const {
    return getMatchDir() + "/final_parking_slot.txt";
}
std::string ParklotFileSystem::getCoarseParkingSlotsPath() const {
    return getVectorizationDir() + "/coarse_parking_slot.txt";
}
std::string ParklotFileSystem::getFineParkingSlotsPath() const {
    return getVectorizationDir() + "/fine_parking_slot.txt";
}
std::string ParklotFileSystem::getFineEdgeParkingSlotsPath() const {
    return getVectorizationDir() + "/fine_edge_parking_slot.txt";
}
std::string ParklotFileSystem::getFineCornerParkingSlotsPath() const {
    return getVectorizationDir() + "/fine_corner_parking_slot.txt";
}
std::string ParklotFileSystem::getReliabiltyPath() const {
    return getVectorizationDir() + "/reliability.txt";
}
}