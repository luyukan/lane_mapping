//
// Created by yukan on 25-1-23.
//
#include "visualization/pangolin_drawer.h"

namespace mono_lane_mapping {
PangolinDrawer& PangolinDrawer::GetInstance() {
  static PangolinDrawer instance;
  return instance;
}
PangolinDrawer::PangolinDrawer() {}

void PangolinDrawer::Init() { init_color_map(); }

void PangolinDrawer::Draw() {
  auto& variables = VisualizationVariable::GetInstance();
  auto tracking_lanes = variables.GetTrackingLanes();
  auto latest_pose = variables.GetVehiclePose();
  if (shift_set_ == false) {
    shift_.x() = latest_pose.twb.x();
    shift_.y() = latest_pose.twb.y();
    shift_set_ = true;
  }
  draw_vehicle(latest_pose);
  // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " <<
  // latest_pose.twb.transpose() << std::endl;
  draw_tracking_lanes(tracking_lanes);
  // draw_coordinate_system();
}

void PangolinDrawer::draw_tracking_lanes(
    const std::map<int, LaneLandmark>& tracking_lanes) {
  for (auto it = tracking_lanes.begin(); it != tracking_lanes.end(); ++it) {
    Eigen::Vector3d color = color_map_.at(it->first);
    glColor3f(color.x(), color.y(), color.z());
    glLineWidth(2.0);
    glBegin(GL_LINE_STRIP);
    auto lane_points = it->second.GetLanePoints();
    for (size_t i = 0; i < lane_points.size(); ++i) {
      glVertex3d(lane_points.at(i).position.x() - shift_.x(),
                 lane_points.at(i).position.y() - shift_.y(),
                 lane_points.at(i).position.z());
    }
    glEnd();
    for (size_t i = 0; i < lane_points.size(); ++i) {
      Eigen::Vector3d position = lane_points.at(i).position -
                                 Eigen::Vector3d(shift_.x(), shift_.y(), 0);
      pangolin::glDrawCross(position, 0.5);
    }
  }
}
void PangolinDrawer::draw_vehicle(const Odometry& pose) {
  if (shift_set_ == false) {
    return;
  }
  Eigen::Vector3d twb = pose.twb;
  twb.x() -= shift_.x();
  twb.y() -= shift_.y();
  Eigen::Quaterniond qwb = pose.qwb;

  Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
  Twb.topLeftCorner(3, 3) = qwb.toRotationMatrix();
  Twb.topRightCorner(3, 1) = twb;
  glPushMatrix();
  glMultMatrixd(Twb.data());

  glLineWidth(5.0);
  glColor3f(0.5, 0.0, 0.5);
  glBegin(GL_LINE_STRIP);
  glVertex3f(Vehicle_forward, 0.5 * Vehicle_width, 0);
  glVertex3f(Vehicle_forward, -0.5 * Vehicle_width, 0);
  glVertex3f(Vehicle_forward, -0.5 * Vehicle_width, Vehicle_height);
  glVertex3f(Vehicle_forward, 0.5 * Vehicle_width, Vehicle_height);
  glVertex3f(Vehicle_forward, 0.5 * Vehicle_width, 0);
  glEnd();

  glBegin(GL_LINE_STRIP);
  glVertex3f(Vehicle_backward, 0.5 * Vehicle_width, 0);
  glVertex3f(Vehicle_backward, -0.5 * Vehicle_width, 0);
  glVertex3f(Vehicle_backward, -0.5 * Vehicle_width, Vehicle_height);
  glVertex3f(Vehicle_backward, 0.5 * Vehicle_width, Vehicle_height);
  glVertex3f(Vehicle_backward, 0.5 * Vehicle_width, 0);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(Vehicle_forward, 0.5 * Vehicle_width, 0);
  glVertex3f(Vehicle_backward, 0.5 * Vehicle_width, 0);
  glVertex3f(Vehicle_forward, -0.5 * Vehicle_width, 0);
  glVertex3f(Vehicle_backward, -0.5 * Vehicle_width, 0);
  glVertex3f(Vehicle_forward, -0.5 * Vehicle_width, Vehicle_height);
  glVertex3f(Vehicle_backward, -0.5 * Vehicle_width, Vehicle_height);
  glVertex3f(Vehicle_forward, 0.5 * Vehicle_width, Vehicle_height);
  glVertex3f(Vehicle_backward, 0.5 * Vehicle_width, Vehicle_height);
  glEnd();

  glPopMatrix();
}

void PangolinDrawer::init_color_map() {
  double bright_factor = 0.8;
  const int num_colors = 50;
  for (int i = 0; i < num_colors; ++i) {
    double r = (i % 2 == 0) ? bright_factor : 1.0 - bright_factor;
    double g = (i % 3 == 0) ? bright_factor : 1.0 - bright_factor;
    double b = (i % 5 == 0) ? bright_factor : 1.0 - bright_factor;

    Eigen::Vector3d color(r, g, b);

    color = color.normalized();
    color_map_[i] = color;
  }
}

void PangolinDrawer::draw_coordinate_system() { pangolin::glDrawAxis(10.0); }
}  // namespace mono_lane_mapping