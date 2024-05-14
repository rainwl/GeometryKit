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
  igl::read_triangle_mesh("./couplingdown.off", V, F);
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


          std::cout << "Face index: " << fid << std::endl;

          // Retrieve the indices of the vertices that make up the face
          int v1 = F(fid, 0);
          int v2 = F(fid, 1);
          int v3 = F(fid, 2);

          // Output the coordinates of the vertices
          Eigen::Vector3d coord1 = V.row(v1);
          Eigen::Vector3d coord2 = V.row(v2);
          Eigen::Vector3d coord3 = V.row(v3);

// Check if x, y, or z coordinates are the same among the vertices
          if (coord1.x() == coord2.x() && coord2.x() == coord3.x()) {
            std::cout << "All vertices of the selected face have the same x value: " << coord1.x() << std::endl;
            double X = coord1.x();

            // Traverse all faces to find and color faces where all vertices have the same y coordinate equal to X
            for (int i = 0; i < F.rows(); ++i) {
              // Get vertex indices of the face
              int v1 = F(i, 0);
              int v2 = F(i, 1);
              int v3 = F(i, 2);

              // Check if all vertices of this face have the y coordinate equal to X
              if (V(v1, 0) == X && V(v2, 0) == X && V(v3, 0) == X) {
                // Set the color for the corresponding face
                std::cout << i << std::endl;
                C.row(i) << 1, 0, 0;
              }
            }
          } else if (coord1.y() == coord2.y() && coord2.y() == coord3.y()) {
            std::cout << "All vertices of the selected face have the same y value: " << coord1.y() << std::endl;
            double commonY = coord1.y();

            // Traverse all faces to find and color faces where all vertices have the same y coordinate equal to commonY
            for (int i = 0; i < F.rows(); ++i) {
              // Get vertex indices of the face
              int v1 = F(i, 0);
              int v2 = F(i, 1);
              int v3 = F(i, 2);

              // Check if all vertices of this face have the y coordinate equal to commonY
              if (V(v1, 1) == commonY && V(v2, 1) == commonY && V(v3, 1) == commonY) {
                // Set the color for the corresponding face
                std::cout << i << std::endl;
                C.row(i) << 1, 0, 0;
              }
            }
          } else if (coord1.z() == coord2.z() && coord2.z() == coord3.z()) {
            std::cout << "All vertices of the selected face have the same z value: " << coord1.z() << std::endl;
            double commonZ = coord1.z();

            // Traverse all faces to find and color faces where all vertices have the same y coordinate equal to commonZ
            for (int i = 0; i < F.rows(); ++i) {
              // Get vertex indices of the face
              int v1 = F(i, 0);
              int v2 = F(i, 1);
              int v3 = F(i, 2);

              // Check if all vertices of this face have the y coordinate equal to commonZ
              if (V(v1, 2) == commonZ && V(v2, 2) == commonZ && V(v3, 2) == commonZ) {
                // Set the color for the corresponding face
                std::cout << i << std::endl;
                C.row(i) << 1, 0, 0;
              }
            }
          }
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
