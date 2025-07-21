#pragma once
#include <memory>
#include <string>
namespace welkin::bamboo {
class ParklotFileSystem {
public:
    using Ptr = std::shared_ptr<ParklotFileSystem>;
    ParklotFileSystem();
    virtual ~ParklotFileSystem();
    const std::string& getRootDir() const;
    void setRootDir(const std::string& root_dir);

    std::string getParklotDir() const;
    std::string getPreprocessDir() const;
    std::string getCornerDir() const;
    std::string getSlotDir() const;
    std::string getMatchDir() const;
    std::string getVectorizationDir() const;
    
    std::string getPatchImagesDir() const;
    std::string getIntensityImagePath() const;
    std::string getBinaryIntensityImagePath() const;
    std::string getHeightDiffImagePath() const;
    std::string getRgbImagePath() const;

    std::string getPredictionsDir() const;
    std::string getKeypointsPath() const;
    std::string getKeypointsImagePath() const;

    std::string getSlotsPath() const;
    std::string getUnPairedPointsPath() const;
    std::string getSlotsImagePath() const;
    
    std::string getOCRDir() const;
    std::string getRgbTemplatesDir() const;
    std::string getIntensityTemplatesDir() const;
    std::string getEdgeIntersectImagePath() const;
    std::string getMatchedSlotsImagePath() const;
    std::string getMatchedEdgeIntersectImagePath() const;
    std::string getRevisedMatchedSlotsImagePath() const;
    std::string getOCRImagePath() const;
    std::string getFinalParkingSlotsPath() const;
    std::string getCoarseParkingSlotsPath() const;
    std::string getFineParkingSlotsPath() const;
    std::string getFineEdgeParkingSlotsPath() const;
    std::string getFineCornerParkingSlotsPath() const;
    std::string getReliabiltyPath() const;
private:
    std::string _root_dir;
};
}