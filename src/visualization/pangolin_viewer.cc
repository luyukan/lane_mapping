//
// Created by yukan on 25-1-23.
//
#include "visualization/pangolin_viewer.h"

namespace mono_lane_mapping {
PangolinViewer &PangolinViewer::GetInstance() {
  static PangolinViewer instance;
  return instance;
}
PangolinViewer::PangolinViewer() {}

void PangolinViewer::Init() {
  pango_drawer_ = std::make_shared<PangolinDrawer>();
}

void PangolinViewer::run() {
  pangolin::CreateWindowAndBind("LaneMapping Viewer",1024,768);
  glEnable(GL_DEPTH_TEST);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
      pangolin::ModelViewLookAt(0,0,-1,0,0,0,0.0,-1.0, 0.0)
  );

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
      .SetHandler(new pangolin::Handler3D(s_cam));


  while(!pangolin::ShouldQuit())
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f,1.0f,1.0f,1.0f);
    pango_drawer_->Draw();
    pangolin::FinishFrame();
  }
}
void PangolinViewer::Start() {
  std::thread th(&PangolinViewer::run, this);
  th.detach();
}
}  // namespace mono_lane_mapping