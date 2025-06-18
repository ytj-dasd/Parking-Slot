#include "sam_predict.h"
#include "sam/interface/sam.h"
#include <common/file_system/folder/rect_image_folder.h>
namespace welkin::bamboo {
bool PromptPredict(OriginDataDOMFolder* dom_folder, 
        const common::Point2d& prompt_pt, cv::Mat& mask, common::Rectd& rect) {
    using namespace welkin::bamboo::dl::sam;
    Sam sam;
    // 读取sam配置文件
    auto conf_path = IMainWindow::Instance()->getConfPath();
    auto filename = QString("%1/dl/sam/sam.conf").arg(conf_path);
    if (!sam.init(UFQ(filename))) {
        UWarn(QObject::tr("Init ebd model failed!"));
        return false;
    }
    auto child_size = dom_folder->getChildSize();
    for (int i = 0; i < child_size; ++i) {
        auto sub_folder = dom_folder->getChild(i);
        auto image_rect = sub_folder->getRect();
        if (image_rect.isIn(prompt_pt)) {
            auto image_mat = sub_folder->getImage();
            auto ebd_path = sub_folder->getPath() + "/_.ebd";
            if (!sam.loadEbd(ebd_path, image_mat.rows, image_mat.cols)) {
                UWarn(QObject::tr("Load ebd file failed!"));
                return false;
            }
            auto res_x = image_rect.getLengthX() / image_mat.cols;
            auto res_y = image_rect.getLengthY() / image_mat.rows;
            common::Ortho2d ortho2d;
            ortho2d.setOrigin(image_rect.getMinX(), image_rect.getMinY());
            ortho2d.setRes(res_x, res_y);
            ortho2d.setSize(image_mat.cols, image_mat.rows);
            cv::Point cv_point;
            cv_point.x = ortho2d.getCol(prompt_pt.x);
            cv_point.y = ortho2d.getRow(prompt_pt.y);
            auto prompt_mask = sam.predict(cv_point);
            prompt_mask.binary_mat().copyTo(mask);
            rect = image_rect;
            break;
        }
    }
    return true;
}

bool PromptPredict(OriginDataDOMFolder* dom_folder, 
        const std::vector<common::Point2d>& prompt_pts, std::vector<RectMask>& rect_masks) {
    std::unordered_map<int, std::vector<int>> index_map;
    auto child_size = dom_folder->getChildSize();
    for (int i = 0; i < prompt_pts.size(); ++i) {
        auto prompt_pt = prompt_pts[i];
        for (int j = 0; j < child_size; ++j) {
            auto sub_folder = dom_folder->getChild(j);
            auto image_rect = sub_folder->getRect();
            if (image_rect.isIn(prompt_pt)) {
                index_map[j].push_back(i);
                break;
            }
        }
    }
    using namespace welkin::bamboo::dl::sam;
    Sam sam;
    // 读取sam配置文件
    auto conf_path = IMainWindow::Instance()->getConfPath();
    auto filename = QString("%1/dl/sam/sam.conf").arg(conf_path);
    if (!sam.init(UFQ(filename))) {
        UWarn(QObject::tr("Init ebd model failed!"));
        return false;
    }
    rect_masks.resize(index_map.size());
    int index = 0;
    for (auto& elem : index_map) {
        auto sub_folder = dom_folder->getChild(elem.first);
        auto image_rect = sub_folder->getRect();
        auto image_mat = sub_folder->getImage();
        auto res_x = image_rect.getLengthX() / image_mat.cols;
        auto res_y = image_rect.getLengthY() / image_mat.rows;
        common::Ortho2d ortho2d;
        ortho2d.setOrigin(image_rect.getMinX(), image_rect.getMinY());
        ortho2d.setRes(res_x, res_y);
        ortho2d.setSize(image_mat.cols, image_mat.rows);
        auto indexs = elem.second;
        std::vector<cv::Point> cv_points;
        for (auto& index : indexs) {
            cv::Point cv_point;
            cv_point.x = ortho2d.getCol(prompt_pts[index].x);
            cv_point.y = ortho2d.getRow(prompt_pts[index].y);
            cv_points.push_back(cv_point);
        }
        auto ebd_path = sub_folder->getPath() + "/_.ebd";
        sam.reset();
        if (!sam.loadEbd(ebd_path, image_mat.rows, image_mat.cols)) {
            UWarn(QObject::tr("Load ebd file failed!"));
            return false;
        }
        RectMask rect_mask;
        auto prompt_mask = sam.predict(cv_points);
        prompt_mask.binary_mat().copyTo(rect_mask.mask);
        rect_mask.rect = image_rect;
        rect_masks.push_back(rect_mask);
        std::string result_path = "/home/scene/data/test/";
        cv::imwrite(result_path + std::to_string(index) + "_origin.jpg", image_mat);
        cv::imwrite(result_path + std::to_string(index) + "_mask.jpg", rect_mask.mask);
        index++;
    }
    return true;
}

bool RectMask::computeContours(std::vector<common::Point2d>& points) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {return false;}

    points.clear();
    common::Ortho2d ortho2d;
    ortho2d.setOrigin(rect.getMinX(), rect.getMinY());
    auto res_x = rect.getLengthX() / mask.cols;
    auto res_y = rect.getLengthY() / mask.rows;
    ortho2d.setRes(res_x, res_y);
    ortho2d.setSize(mask.cols, mask.rows);
    for (int i = 0; i < contours.size(); ++i) {
        for (int j = 0; j < contours[i].size(); ++j) {
            auto& pt = contours[i][j];
            double x = ortho2d.getX(pt.x);
            double y = ortho2d.getY(pt.y);
            points.push_back(common::Point2d(x, y));
        }
    }
    return true;
}
}