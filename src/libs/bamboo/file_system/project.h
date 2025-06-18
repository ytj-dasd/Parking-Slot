#pragma once
#include <common/file_system/project.h>

#include <bamboo/file_system/raw_data_folder.h>
#include <bamboo/file_system/origin_data_folder.h>
#include <bamboo/file_system/building_folder.h>
#include <bamboo/file_system/terrain_folder.h>

namespace welkin::bamboo {

using BaseProject =common::file_system::TypeProject<
        ITEM_BAMBOO_PROJECT,
        common::file_system::TypeFilerFinder<>,
        common::file_system::TypeFolderFinder<
        RawDataFolder,
        OriginDataFolder,
        BuildingListFolder,
        TerrainFeatureListFolder
        >>;

class BAMBOO_EXPORT Project : public BaseProject {
public:
    Project();
    virtual ~Project();

    OriginDataLocalPoseFiler* getLocalPoseFiler() {
        return this->getOriginDataFolder()->getLocalPoseFiler();
    }
    OriginDataGlobalPoseFiler* getGlobalPoseFiler() {
        return this->getOriginDataFolder()->getGlobalPoseFiler();
    }
    OriginDataBirdViewFolder* getBirdViewFolder() {
        return this->getOriginDataFolder()->getBirdViewFolder();
    }
    OriginDataFrontViewFolder* getFrontViewFolder() {
        return this->getOriginDataFolder()->getFrontViewFolder();
    }
    OriginDataFullCloudFolder* getFullCloudFolder() {
        return this->getOriginDataFolder()->getFullCloudFolder();
    }
    OriginDataDOMFolder* getDOMFolder() {
        return this->getOriginDataFolder()->getDOMFolder();
    }
    OriginDataCloudListFolder* getCloudListFolder() {
        return this->getOriginDataFolder()->getCloudListFolder();
    }
    OriginDataModelListFolder* getModelListFolder() {
        return this->getOriginDataFolder()->getModelListFolder();
    }

    //////////
    RawDataFolder* getRawDataFolder() {
        return this->template getTypeFolder<0>();
    }
    const RawDataFolder* getRawDataFolder() const {
        return this->template getTypeFolder<0>();
    }
    OriginDataFolder* getOriginDataFolder() {
        return this->template getTypeFolder<1>();
    }
    const OriginDataFolder* getOriginDataFolder() const {
        return this->template getTypeFolder<1>();
    }
    BuildingListFolder* getBuildingsFolder() {
        return this->template getTypeFolder<2>();
    }
    const BuildingListFolder* getBuildingsFolder() const {
        return this->template getTypeFolder<2>();
    }
    TerrainFeatureListFolder* getFeatureListFolder() {
        return this->template getTypeFolder<3>();
    }
    const TerrainFeatureListFolder* getFeatureListFolder() const {
        return this->template getTypeFolder<3>();
    }
};
}
