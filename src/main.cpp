#include "igl/read_triangle_mesh.h"
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuizmoWidget.h"
#include "GLFW/glfw3.h"

//#define TEST
//
//Eigen::MatrixXd V1,V2;
//Eigen::MatrixXi F1,F2;

//bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
//{
//  std::cout<<"Key: "<<key<<" "<<(unsigned int)key<<std::endl;
//  if (key == '1')
//  {
//    // Clear should be called before drawing the mesh
//    viewer.data().clear();
//    // Draw_mesh creates or updates the vertices and faces of the displayed mesh.
//    // If a mesh is already displayed, draw_mesh returns an error if the given V and
//    // F have size different than the current ones
//    viewer.data().set_mesh(V1, F1);
//    viewer.core().align_camera_center(V1,F1);
//  }
//  else if (key == '2')
//  {
//    viewer.data().clear();
//    viewer.data().set_mesh(V2, F2);
//    viewer.core().align_camera_center(V2,F2);
//  }
//
//  return false;
//}

int main(int argc, char *argv[])
{
#ifdef TEST
  igl::readOFF("./bunny.off", V1, F1);
  igl::readOFF("./cow.off", V2, F2);
  std::cout<<R"(
1 Switch to bump mesh
2 Switch to fertility mesh
    )";

  igl::opengl::glfw::Viewer viewer;
  // Register a keyboard callback that allows to switch between
  // the two loaded meshes
  viewer.callback_key_down = &key_down;
  viewer.data().set_mesh(V1, F1);
  viewer.launch();

  return 0;
#else
  // Load a mesh from file
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh("./bunny.off",V,F);
  // Set up viewer
  igl::opengl::glfw::Viewer vr;
  vr.data().set_mesh(V,F);

  igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
  vr.plugins.push_back(&imgui_plugin);

  // Add a 3D gizmo plugin
  igl::opengl::glfw::imgui::ImGuizmoWidget gizmo;
  imgui_plugin.widgets.push_back(&gizmo);
  // Initialize ImGuizmo at mesh centroid
  gizmo.T.block(0,3,3,1) =
      0.5*(V.colwise().maxCoeff() + V.colwise().minCoeff()).transpose().cast<float>();
  // Update can be applied relative to this remembered initial transform
  const Eigen::Matrix4f T0 = gizmo.T;
  // Attach callback to apply imguizmo's transform to mesh
  gizmo.callback = [&](const Eigen::Matrix4f & T)
  {
    const Eigen::Matrix4d TT = (T*T0.inverse()).cast<double>().transpose();
    vr.data().set_vertices(
        (V.rowwise().homogeneous()*TT).rowwise().hnormalized());
    vr.data().compute_normals();
  };
  // Maya-style keyboard shortcuts for operation
  vr.callback_key_pressed = [&](decltype(vr) &,unsigned int key, int mod)
  {
    switch(key)
    {
      case ' ': gizmo.visible = !gizmo.visible; return true;
      case 'W': case 'w': gizmo.operation = ImGuizmo::TRANSLATE; return true;
      case 'E': case 'e': gizmo.operation = ImGuizmo::ROTATE;    return true;
      case 'R': case 'r': gizmo.operation = ImGuizmo::SCALE;     return true;
    }
    return false;
  };

  igl::opengl::glfw::imgui::ImGuiMenu menu;
  imgui_plugin.widgets.push_back(&menu);

  std::cout<<R"(
W,w  Switch to translate operation
E,e  Switch to rotate operation
R,r  Switch to scale operation
)";
  vr.launch();
#endif
}
