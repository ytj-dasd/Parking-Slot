#pragma once
#include <qmath.h>
#include <iostream>
#include "bamboo/base/macro.h"

#include <osgGA/OrbitManipulator>

namespace welkin::bamboo {
class BAMBOO_EXPORT DriveManipulator : public osgGA::OrbitManipulator {
public:
    explicit DriveManipulator(int flags = DEFAULT_SETTINGS);
    explicit DriveManipulator(const DriveManipulator& tm,
        const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY);
    META_Object(welkin::bamboo, DriveManipulator);

    void fitInView(const osg::Vec3d& up_dir, const osg::Vec3d& eye_dir);

protected:
    void resetDriveView();
    void reverseDriveView();
    void moveCameraX(const double dx);
    void moveCameraY(const double dy);
    void moveCameraZ(const double dz);
    void rotateCameraRoll(const double angle);
    void rotateCameraPitch(const double angle);
    void rotateCameraYaw(const double angle);
protected:
    bool handleKeyDown(const osgGA::GUIEventAdapter& ea,
        osgGA::GUIActionAdapter& us) override;

private:
    double _move_dist = 1.0;
    double _move_angle = 1.0 * M_PI / 180;
};
}
