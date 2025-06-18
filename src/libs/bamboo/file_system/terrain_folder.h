#pragma once
#include <bamboo/file_system/types.h>
#include <uface/base/uconverter.h>
#include <common/file_system/filer/proto_filer.h>
#include <common/file_system/group.h>

namespace welkin::bamboo {
using TerrainFeatureLayerFiler = common::file_system::TypeLayerFiler<
    ITEM_TERRAIN_LAYER_FILER>;
using BaseTerrainFeatureLayerListFolder = common::file_system::TypeGroup<
    ITEM_TERRAIN_LAYER_LIST_FOLDER, TerrainFeatureLayerFiler>;
class BAMBOO_EXPORT TerrainFeatureLayerListFolder : public BaseTerrainFeatureLayerListFolder {
public:
    TerrainFeatureLayerListFolder(const std::string& name, Folder* parent = nullptr)
            : BaseTerrainFeatureLayerListFolder(name, parent) {
    }
    virtual ~TerrainFeatureLayerListFolder() {}

    std::string getFileFilter() const override {
        return std::string(".layer");
    }
};
using BaseTerrainFeatureListFolder = common::file_system::TypeOnlyFolderFolder<
    ITEM_TERRAIN_FEATURE_LIST_FOLDER,
    TerrainFeatureLayerListFolder, TerrainFeatureLayerListFolder,
    TerrainFeatureLayerListFolder, TerrainFeatureLayerListFolder,
    TerrainFeatureLayerListFolder, TerrainFeatureLayerListFolder>;
class BAMBOO_EXPORT TerrainFeatureListFolder : public BaseTerrainFeatureListFolder {
public:
    TerrainFeatureListFolder(const std::string& name, Folder* parent = nullptr)
            : BaseTerrainFeatureListFolder(name, parent) {
        this->initFolders(
            UFQ("一般地物"), UFQ("建筑物"), UFQ("构筑物"),
            UFQ("点状地物"), UFQ("线状地物"), UFQ("面状地物"));
    }
    virtual ~TerrainFeatureListFolder() {}
    // 一般地物
    TerrainFeatureLayerListFolder* getGeneralFeatureListFolder() {
        return this->template getTypeFolder<0>();
    }
    const TerrainFeatureLayerListFolder* getGeneralFeatureListFolder() const {
        return this->template getTypeFolder<0>();
    }
    // 建筑物
    TerrainFeatureLayerListFolder* getBuildingFeatureListFolder() {
        return this->template getTypeFolder<1>();
    }
    const TerrainFeatureLayerListFolder* getBuildingFeatureListFolder() const {
        return this->template getTypeFolder<1>();
    }
    // 构筑物
    TerrainFeatureLayerListFolder* getStructureFeatureListFolder() {
        return this->template getTypeFolder<2>();
    }
    const TerrainFeatureLayerListFolder* getStructureFeatureListFolder() const {
        return this->template getTypeFolder<2>();
    }
    // 点状地物
    TerrainFeatureLayerListFolder* getPointFeatureListFolder() {
        return this->template getTypeFolder<3>();
    }
    const TerrainFeatureLayerListFolder* getPointFeatureListFolder() const {
        return this->template getTypeFolder<3>();
    }
    // 线状地物
    TerrainFeatureLayerListFolder* getPolylineFeatureListFolder() {
        return this->template getTypeFolder<4>();
    }
    const TerrainFeatureLayerListFolder* getPolylineFeatureListFolder() const {
        return this->template getTypeFolder<4>();
    }
    // 面状地物
    TerrainFeatureLayerListFolder* getPolygonFeatureListFolder() {
        return this->template getTypeFolder<5>();
    }
    const TerrainFeatureLayerListFolder* getPolygonFeatureListFolder() const {
        return this->template getTypeFolder<5>();
    }
    bool removeAllLayers() {
        bool flag = true;
        flag &= getGeneralFeatureListFolder()->removeAllChildren();
        flag &= getBuildingFeatureListFolder()->removeAllChildren();
        flag &= getStructureFeatureListFolder()->removeAllChildren();
        flag &= getPointFeatureListFolder()->removeAllChildren();
        flag &= getPolylineFeatureListFolder()->removeAllChildren();
        flag &= getPolygonFeatureListFolder()->removeAllChildren();
        return flag;
    }
};
}
