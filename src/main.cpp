#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <igl/unproject_onto_mesh.h>
#include "dmSimSvrUnity.pb.h"
#include "ecal/ecal.h"
#include "ecal/msg/protobuf/publisher.h"
#include "google/protobuf/any.pb.h"

std::string deformable_path = "./acl.off";
std::string bone_path       = "./bone.off";
std::string hook_path       = "./hook.off";

void eCALSend(Eigen::Vector3d translation, Eigen::Quaterniond quaternion, eCAL::CPublisher &pub, const int instrumentId) {
  auto *imSend = new dm::InstrumentMsg;
  imSend->set_insid(instrumentId);
  imSend->set_visible(true);
  auto *posSend = new dm::Vec3;
  posSend->set_x(static_cast<float>(translation.x()));
  posSend->set_y(static_cast<float>(translation.y()));
  posSend->set_z(static_cast<float>(translation.z()));
  auto *rotSend = new dm::Vec4;
  rotSend->set_x(static_cast<float>(quaternion.x()));
  rotSend->set_y(static_cast<float>(quaternion.y()));
  rotSend->set_z(static_cast<float>(quaternion.z()));
  rotSend->set_w(static_cast<float>(quaternion.w()));

  imSend->set_allocated_pos(posSend);
  imSend->set_allocated_rot(rotSend);

  google::protobuf::Any any;
  any.PackFrom(*imSend);
  std::size_t sendLen  = any.ByteSizeLong();
  auto *      sendData = new std::uint8_t[sendLen]{0};
  any.SerializePartialToArray((void *) (sendData), static_cast<int>(sendLen));
  pub.SetID(6);
  if (const std::size_t sendCode = pub.Send(sendData, sendLen); sendCode != sendLen) { std::cout << pub.GetTopicName() << " send failed\n"; }
  delete[] sendData;
  delete imSend;
}

int main(int argc, char *argv[]) {
  eCAL::Initialize(1, nullptr, "GeometryKit");
  eCAL::CPublisher pub_navi("simsvr-data");

  Eigen::MatrixXd V1, V2;
  Eigen::MatrixXi F1, F2;

  if (!igl::read_triangle_mesh(bone_path, V1, F1)) { return EXIT_FAILURE; }
  if (!igl::read_triangle_mesh(hook_path, V2, F2)) { return EXIT_FAILURE; }

  igl::opengl::glfw::Viewer viewer;

  viewer.append_mesh();
  viewer.data(0).set_mesh(V1, F1);
  viewer.data(0).set_colors(Eigen::RowVector3d(1, 1, 1));
  viewer.data(0).show_lines = false;

  viewer.append_mesh();
  viewer.data(1).set_mesh(V2, F2);
  viewer.data(1).set_colors(Eigen::RowVector3d(1, 1, 1));
  viewer.data(1).show_lines = false;

  igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
  viewer.plugins.push_back(&imgui_plugin);

  igl::opengl::glfw::imgui::ImGuizmoWidget gizmo1, gizmo2;
  imgui_plugin.widgets.push_back(&gizmo1);
  imgui_plugin.widgets.push_back(&gizmo2);

  gizmo1.visible = false;
  gizmo2.visible = false;

  gizmo1.T.block(0, 3, 3, 1) = 0.5 * (V1.colwise().maxCoeff() + V1.colwise().minCoeff()).transpose().cast<float>();
  gizmo2.T.block(0, 3, 3, 1) = 0.5 * (V2.colwise().maxCoeff() + V2.colwise().minCoeff()).transpose().cast<float>();

  Eigen::Matrix4f T0_1 = gizmo1.T;
  Eigen::Matrix4f T0_2 = gizmo2.T;

  gizmo1.callback = [&](const Eigen::Matrix4f &T) {
    const Eigen::Matrix4d TT = (T * T0_1.inverse()).cast<double>().transpose();
    viewer.data(0).set_vertices((V1.rowwise().homogeneous() * TT).rowwise().hnormalized());
    viewer.data(0).compute_normals();

    Eigen::Matrix4d          M              = (T * T0_1.inverse()).cast<double>();
    const Eigen::Vector3d    translation    = M.block<3, 1>(0, 3);
    const Eigen::Matrix3d    rotationMatrix = M.block<3, 3>(0, 0);
    const Eigen::Quaterniond quaternion(rotationMatrix);
    eCALSend(translation, quaternion, pub_navi, 302);
  };
  gizmo2.callback = [&](const Eigen::Matrix4f &T) {
    const Eigen::Matrix4d TT = (T * T0_2.inverse()).cast<double>().transpose();
    viewer.data(1).set_vertices((V2.rowwise().homogeneous() * TT).rowwise().hnormalized());
    viewer.data(1).compute_normals();

    Eigen::Matrix4d          M              = (T * T0_2.inverse()).cast<double>();
    const Eigen::Vector3d    translation    = M.block<3, 1>(0, 3);
    const Eigen::Matrix3d    rotationMatrix = M.block<3, 3>(0, 0);
    const Eigen::Quaterniond quaternion(rotationMatrix);
    eCALSend(translation, quaternion, pub_navi, 303);
  };
  viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer &v, const int button, int modifier) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      gizmo1.visible = false;
      gizmo2.visible = false;

      const double          x = viewer.current_mouse_x;
      const double          y = viewer.core().viewport(3) - static_cast<double>(viewer.current_mouse_y);
      const Eigen::Vector2f mouse_pos(x, y);

      int selected_mesh_index = -1;
      for (int i = 0; i < v.data_list.size(); ++i) {
        Eigen::Vector3f bc;
        if (int fid; igl::unproject_onto_mesh(mouse_pos, v.core().view, v.core().proj, v.core().viewport,
                                              v.data_list[i].V, v.data_list[i].F, fid, bc)) {
          selected_mesh_index = i;
          break;
        }
      }
      if (selected_mesh_index != -1) {
        gizmo1.visible = (selected_mesh_index == 0);
        gizmo2.visible = (selected_mesh_index == 1);
        return true;
      }
    }
    return false;
  };
  viewer.callback_key_pressed = [&](decltype(viewer) &, const unsigned int key, int mod) {
    switch (key) {
      case 'W':
      case 'w': gizmo1.operation = ImGuizmo::TRANSLATE;
        gizmo2.operation = ImGuizmo::TRANSLATE;
        return true;
      case 'E':
      case 'e': gizmo1.operation = ImGuizmo::ROTATE;
        gizmo2.operation = ImGuizmo::ROTATE;
        return true;
      case 'R':
      case 'r': gizmo1.operation = ImGuizmo::SCALE;
        gizmo2.operation = ImGuizmo::SCALE;
        return true;
      default: ;
    }
    return false;
  };

  // igl::opengl::glfw::imgui::ImGuiMenu menu;
  // imgui_plugin.widgets.push_back(&menu);
  viewer.launch();
  return EXIT_SUCCESS;
}