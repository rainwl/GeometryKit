#include "igl/read_triangle_mesh.h"
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/glfw/imgui/ImGuiPlugin.h"
#include "igl/opengl/glfw/imgui/ImGuiMenu.h"
#include "igl/opengl/glfw/imgui/ImGuizmoWidget.h"
#include "GLFW/glfw3.h"
#include "unproject_onto_mesh.h"

int main(int argc, char *argv[]) {
  // Load a mesh from file
  Eigen::MatrixXd V, C;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh("./am.off", V, F);
  C = Eigen::MatrixXd::Constant(F.rows(), 3, 1);
  // Set up viewer
  igl::opengl::glfw::Viewer vr;
  vr.callback_mouse_down =
      [&V, &F, &C](igl::opengl::glfw::Viewer &viewer, int, int) -> bool {
        int fid;
        Eigen::Vector3f bc;
        // Cast a ray in the view direction starting from the mouse position
        double x = viewer.current_mouse_x;
        double y = viewer.core().viewport(3) - viewer.current_mouse_y;
        if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core().view,
                                     viewer.core().proj, viewer.core().viewport, V, F, fid, bc)) {
          // paint hit red
          C.row(fid) << 1, 0, 0;
          viewer.data().set_colors(C);
          return true;
        }
        return false;
      };
  vr.data().set_mesh(V, F);
  vr.data().set_colors(C);
  vr.data().show_lines = false;

  igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
  vr.plugins.push_back(&imgui_plugin);

  // Add a 3D gizmo plugin
  igl::opengl::glfw::imgui::ImGuizmoWidget gizmo;
  imgui_plugin.widgets.push_back(&gizmo);
  // Initialize ImGuizmo at mesh centroid
  gizmo.T.block(0, 3, 3, 1) =
      0.5 * (V.colwise().maxCoeff() + V.colwise().minCoeff()).transpose().cast<float>();
  // Update can be applied relative to this remembered initial transform
  const Eigen::Matrix4f T0 = gizmo.T;
  // Attach callback to apply imguizmo's transform to mesh
  gizmo.callback = [&](const Eigen::Matrix4f &T) {
    const Eigen::Matrix4d TT = (T * T0.inverse()).cast<double>().transpose();
    vr.data().set_vertices(
        (V.rowwise().homogeneous() * TT).rowwise().hnormalized());
    vr.data().compute_normals();
  };
  // Maya-style keyboard shortcuts for operation
  vr.callback_key_pressed = [&](decltype(vr) &, unsigned int key, int mod) {
    switch (key) {
      case ' ': gizmo.visible = !gizmo.visible;
        return true;
      case 'W':
      case 'w': gizmo.operation = ImGuizmo::TRANSLATE;
        return true;
      case 'E':
      case 'e': gizmo.operation = ImGuizmo::ROTATE;
        return true;
      case 'R':
      case 'r': gizmo.operation = ImGuizmo::SCALE;
        return true;
    }
    return false;
  };

  igl::opengl::glfw::imgui::ImGuiMenu menu;
  imgui_plugin.widgets.push_back(&menu);

  std::cout << R"(
W,w  Switch to translate operation
E,e  Switch to rotate operation
R,r  Switch to scale operation
)";
  vr.launch();
}
