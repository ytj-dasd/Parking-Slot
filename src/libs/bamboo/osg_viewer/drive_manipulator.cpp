#include "drive_manipulator.h"

namespace welkin::bamboo {
DriveManipulator::DriveManipulator(int flags)
        : osgGA::OrbitManipulator(flags) {
}
DriveManipulator::DriveManipulator(
    const DriveManipulator& tm,
    const osg::CopyOp& copyOp)
    : osg::Object(tm, copyOp),
      osg::Callback(tm, copyOp),
      osgGA::OrbitManipulator(tm, copyOp) {}

void DriveManipulator::resetDriveView() {
    osg::Vec3d eye_pos, cnt_pos, up_dir;
    this->getTransformation(eye_pos, cnt_pos, up_dir);
    up_dir = osg::Vec3d(0, 0, 1);
    cnt_pos[2] = 1.5;
    eye_pos[2] = 1.5;
    if ((cnt_pos - eye_pos).length() < 1.0) {
        cnt_pos[0] = eye_pos[0] + 100;
    }
    this->setTransformation(eye_pos, cnt_pos, up_dir);
}
void DriveManipulator::reverseDriveView() {
    osg::Vec3d eye_pos, cnt_pos, up_dir;
    this->getTransformation(eye_pos, cnt_pos, up_dir);
    osg::Vec3d eye_dir = cnt_pos - eye_pos;
    cnt_pos = eye_pos - eye_dir;
    this->setTransformation(eye_pos, cnt_pos, up_dir);
}
void DriveManipulator::moveCameraX(const double dx) {
    osg::Vec3d eye_pos, cnt_pos, up_dir;
    this->getTransformation(eye_pos, cnt_pos, up_dir);
    osg::Vec3d eye_dir = cnt_pos - eye_pos;
    if (std::hypot(eye_dir[0], eye_dir[1]) < 1e-2) {
        eye_dir[0] = up_dir[0];
        eye_dir[1] = up_dir[1];
    }
    eye_dir[2] = 0; eye_dir.normalize();
    eye_pos += eye_dir * dx;
    cnt_pos += eye_dir * dx;
    this->setTransformation(eye_pos, cnt_pos, up_dir);
}
void DriveManipulator::moveCameraY(const double dy) {
    osg::Vec3d eye_pos, cnt_pos, up_dir;
    this->getTransformation(eye_pos, cnt_pos, up_dir);
    osg::Vec3d eye_dir = cnt_pos - eye_pos;
    eye_dir[2] = eye_dir[1]; eye_dir[1] = eye_dir[0];
    eye_dir[0] = -eye_dir[2]; eye_dir[2] = 0;
    if (std::hypot(eye_dir[0], eye_dir[1]) < 1e-2) {
        eye_dir[0] = -up_dir[1];
        eye_dir[1] = up_dir[0];
    }
    eye_dir.normalize();
    eye_pos += eye_dir * dy;
    cnt_pos += eye_dir * dy;
    this->setTransformation(eye_pos, cnt_pos, up_dir);
}
void DriveManipulator::moveCameraZ(const double dz) {
    osg::Vec3d eye_pos, cnt_pos, up_dir;
    this->getTransformation(eye_pos, cnt_pos, up_dir);
    eye_pos[2] += dz;
    cnt_pos[2] += dz;
    this->setTransformation(eye_pos, cnt_pos, up_dir);
}
void DriveManipulator::rotateCameraRoll(const double angle) {

}
void DriveManipulator::rotateCameraPitch(const double angle) {
    osg::Vec3d eye_pos, cnt_pos, up_dir;
    this->getTransformation(eye_pos, cnt_pos, up_dir);
    osg::Vec3d eye_dir = cnt_pos - eye_pos;
    double dxy = std::max(std::hypot(eye_dir[0], eye_dir[1]), 1e-1);
    double rot_ang = std::atan2(eye_dir[2], dxy) + angle;
    rot_ang = std::max(rot_ang, -80 * M_PI / 180.0);
    rot_ang = std::min(rot_ang, 80 * M_PI / 180.0);
    eye_dir[2] = dxy * std::tan(std::atan2(eye_dir[2], dxy) + angle);
    cnt_pos = eye_pos + eye_dir;
    this->setTransformation(eye_pos, cnt_pos, up_dir);
}
void DriveManipulator::rotateCameraYaw(const double angle) {
    osg::Vec3d eye_pos, cnt_pos, up_dir;
    this->getTransformation(eye_pos, cnt_pos, up_dir);
    osg::Vec3d eye_dir = cnt_pos - eye_pos;
    double cx = eye_dir[0] * std::cos(angle) - eye_dir[1] * std::sin(angle);
    double cy = eye_dir[0] * std::sin(angle) + eye_dir[1] * std::cos(angle);
    eye_dir[0] = cx;
    eye_dir[1] = cy;
    cnt_pos = eye_pos + eye_dir;
    this->setTransformation(eye_pos, cnt_pos, up_dir);
}
bool DriveManipulator::handleKeyDown(const osgGA::GUIEventAdapter& ea,
        osgGA::GUIActionAdapter& us) {
    // std::cout << "key: " << ea.getKey() << std::endl;
    switch (ea.getKey()) {
    case osgGA::GUIEventAdapter::KEY_Up:
        this->moveCameraX(_move_dist);
        return true;
    case osgGA::GUIEventAdapter::KEY_Down:
        this->moveCameraX(-_move_dist);
        return true;
    case osgGA::GUIEventAdapter::KEY_Left:
        this->moveCameraY(_move_dist);
        return true;
    case osgGA::GUIEventAdapter::KEY_Right:
        this->moveCameraY(-_move_dist);
        return true;
    case osgGA::GUIEventAdapter::KEY_Page_Up:
        this->moveCameraZ(_move_dist);
        return true;
    case osgGA::GUIEventAdapter::KEY_Page_Down:
        this->moveCameraZ(-_move_dist);
        return true;
    case osgGA::GUIEventAdapter::KEY_A:
        this->rotateCameraYaw(_move_angle);
        return true;
    case osgGA::GUIEventAdapter::KEY_D:
        this->rotateCameraYaw(-_move_angle);
        return true;
    case osgGA::GUIEventAdapter::KEY_U:
        this->rotateCameraPitch(_move_angle);
        return true;
    case osgGA::GUIEventAdapter::KEY_I:
        this->rotateCameraPitch(-_move_angle);
        return true;
    case osgGA::GUIEventAdapter::KEY_R:
        this->resetDriveView();
        return true;
    case osgGA::GUIEventAdapter::KEY_G:
        this->reverseDriveView();
        return true;
    default:
        return false;
    }
}
void DriveManipulator::fitInView(const osg::Vec3d& up_dir,
        const osg::Vec3d& eye_dir) {
    double radius = 100;
    osg::Vec3d center(0, 0, 0);
    auto node = this->getNode();
    if (node) {
        auto& bound = node->getBound();
        radius = bound.radius();
        center = bound.center();
    }
    double view_dist = radius * 2;
    osg::Vec3d eye_pos = center - eye_dir * view_dist;
    this->setTransformation(eye_pos, center, up_dir);
}
}
