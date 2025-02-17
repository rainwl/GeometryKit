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
#include <mutex>

std::mutex      g_mesh_mutex;
Eigen::MatrixXf g_ecal_V;
Eigen::MatrixXi g_ecal_F;
Eigen::MatrixXf g_ecal_N;
bool            g_new_mesh_data_available = false;
int             g_ecal_mesh_index         = -1;

std::string bone_path = "./bone.obj";
std::string hook_path = "./hook.obj";

struct Model {
  Eigen::MatrixXd                           original_V;
  Eigen::MatrixXi                           F;
  int                                       meshIndex{};
  int                                       instrumentId{};
  igl::opengl::glfw::imgui::ImGuizmoWidget *gizmo{};
  Eigen::Matrix4f                           T0;
};

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

bool AddModel(const std::string &path, const int instrumentId, igl::opengl::glfw::Viewer &viewer, igl::opengl::glfw::imgui::ImGuiPlugin &imgui_plugin, Model &model) {
  if (!igl::read_triangle_mesh(path, model.original_V, model.F)) {
    std::cerr << "Unable to load model: " << path << std::endl;
    return false;
  }

  model.meshIndex = viewer.append_mesh();
  viewer.data(model.meshIndex).set_mesh(model.original_V, model.F);
  viewer.data(model.meshIndex).set_colors(Eigen::RowVector3d(1, 1, 1));
  viewer.data(model.meshIndex).show_lines = false;

  model.instrumentId = instrumentId;
  model.gizmo        = new igl::opengl::glfw::imgui::ImGuizmoWidget();
  imgui_plugin.widgets.push_back(model.gizmo);
  model.gizmo->visible = false;
  model.gizmo->T.setIdentity();
  model.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  model.T0                         = model.gizmo->T;

  return true;
}

void OnMeshDataReceived(const char *topic_name, const struct eCAL::SReceiveCallbackData *data) {
  std::lock_guard lock(g_mesh_mutex);
  const auto *    ptr = static_cast<const uint8_t *>(data->buf);

#pragma region I.ecal_id
  ptr += sizeof(int);
  // const int ecal_id = *reinterpret_cast<const int *>(ptr);
  ptr += sizeof(int);
#pragma endregion

#pragma region II.positions
  const int positions_bytes = *reinterpret_cast<const int *>(ptr);
  ptr += sizeof(int);
  const size_t num_vertices = positions_bytes / (3 * sizeof(float));
  if (num_vertices == 0) return;
  Eigen::MatrixXf    V(num_vertices, 3);
  std::vector<float> raw_positions(num_vertices * 3);
  memcpy(raw_positions.data(), ptr, positions_bytes);
  for (int i = 0; i < num_vertices; i++) {
    V(i, 0) = raw_positions[i * 3];
    V(i, 1) = raw_positions[i * 3 + 1];
    V(i, 2) = raw_positions[i * 3 + 2];
  }
  ptr += positions_bytes;
#pragma endregion

#pragma region III.normals
  const int normals_bytes = *reinterpret_cast<const int *>(ptr);
  ptr += sizeof(int);
  const size_t num_normals = normals_bytes / (3 * sizeof(float));
  if (num_normals == 0) return;
  Eigen::MatrixXf    N(num_normals, 3);
  std::vector<float> raw_normals(num_normals * 3);
  memcpy(raw_normals.data(), ptr, normals_bytes);
  for (int i = 0; i < num_normals; i++) {
    N(i, 0) = raw_normals[i * 3];
    N(i, 1) = raw_normals[i * 3 + 1];
    N(i, 2) = raw_normals[i * 3 + 2];
  }
  ptr += normals_bytes;
#pragma endregion

#pragma region IV.indices
  const int index_bytes = *reinterpret_cast<const int *>(ptr);
  ptr += sizeof(int);
  const size_t num_indices = index_bytes / sizeof(uint32_t);
  const size_t num_faces   = num_indices / 3;
  if (num_faces == 0) return;
  Eigen::MatrixXi       F(num_faces, 3);
  std::vector<uint32_t> raw_indices(num_indices);
  memcpy(raw_indices.data(), ptr, index_bytes);
  for (int i = 0; i < num_faces; i++) {
    F(i, 0) = static_cast<int>(raw_indices[i * 3]);
    F(i, 1) = static_cast<int>(raw_indices[i * 3 + 1]);
    F(i, 2) = static_cast<int>(raw_indices[i * 3 + 2]);
  }
  ptr += index_bytes;
#pragma endregion

#pragma region V.uvs
  const int uv_bytes = *reinterpret_cast<const int *>(ptr);
  ptr += sizeof(int);
  // std::vector<Eigen::Vector2f> uvs;
  // if (uv_bytes > 0) {
  //   size_t num_uvs = uv_bytes / (2 * sizeof(float));
  //   uvs.resize(num_uvs);
  //   memcpy(uvs.data(), ptr, uv_bytes);
  // }
  ptr += uv_bytes;
#pragma endregion

#pragma region VI.tangents
  const int tangent_bytes = *reinterpret_cast<const int *>(ptr);
  ptr += sizeof(int);
  // std::vector<Eigen::Vector4f> tangents;
  // if (tangent_bytes > 0) {
  //   size_t num_tangents = tangent_bytes / (4 * sizeof(float));
  //   tangents.resize(num_tangents);
  //   memcpy(tangents.data(), ptr, tangent_bytes);
  // }
  ptr += tangent_bytes;
#pragma endregion

#pragma region VII.surfaces
  const int surface_bytes = *reinterpret_cast<const int *>(ptr);
  ptr += sizeof(int);
  // std::vector<int> surface;
  // if (surface_bytes > 0) {
  //   size_t num_surface = surface_bytes / sizeof(int);
  //   surface.resize(num_surface);
  //   memcpy(surface.data(), ptr, surface_bytes);
  // }
  ptr += surface_bytes;
#pragma endregion

#pragma region VIII.tear_surfaces
  const int tear_surface_bytes = *reinterpret_cast<const int *>(ptr);
  ptr += sizeof(int);
  // std::vector<uint32_t> tear_surface;
  // if (tear_surface_bytes > 0) {
  //   size_t num_tear = tear_surface_bytes / sizeof(uint32_t);
  //   tear_surface.resize(num_tear);
  //   memcpy(tear_surface.data(), ptr, tear_surface_bytes);
  // }
  ptr += tear_surface_bytes;
#pragma endregion

  g_ecal_V                  = V;
  g_ecal_F                  = F;
  g_ecal_N                  = N;
  g_new_mesh_data_available = true;
}

int main(int argc, char *argv[]) {
  eCAL::Initialize(1, nullptr, "GeometryKit");
  eCAL::CSubscriber sub("Physics2Unity-SoftBody");
  eCAL::CPublisher  pub_navi("simsvr-data");
  sub.AddReceiveCallback([](auto &&argument_1, auto &&argument_2) { OnMeshDataReceived(std::forward<decltype(argument_1)>(argument_1), std::forward<decltype(argument_2)>(argument_2)); });

  igl::opengl::glfw::Viewer             viewer;
  igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
  viewer.plugins.push_back(&imgui_plugin);
  std::vector<Model> models;

  Model boneModel;
  if (!AddModel(bone_path, 302, viewer, imgui_plugin, boneModel)) return EXIT_FAILURE;
  models.push_back(boneModel);

  Model hookModel;
  if (!AddModel(hook_path, 303, viewer, imgui_plugin, hookModel)) return EXIT_FAILURE;
  models.push_back(hookModel);

  g_ecal_mesh_index = viewer.append_mesh();
  viewer.data(g_ecal_mesh_index).set_colors(Eigen::RowVector3d(0.6f * 0.75f, 0.8f * 0.75f, 1.0f * 0.75f));
  viewer.data(g_ecal_mesh_index).show_lines = false;

#pragma region CALLBACK
  viewer.callback_pre_draw = [](igl::opengl::glfw::Viewer &v) -> bool {
    std::lock_guard lock(g_mesh_mutex);
    if (g_ecal_mesh_index < 0) return false;
    if (g_new_mesh_data_available) {
      if (g_ecal_V.size() == 0 || g_ecal_F.size() == 0 || g_ecal_N.size() == 0) {
        g_new_mesh_data_available = false;
        return false;
      }
      v.data(g_ecal_mesh_index).set_mesh(g_ecal_V.cast<double>(), g_ecal_F);
      v.data(g_ecal_mesh_index).set_normals(g_ecal_N.cast<double>());
      v.data(g_ecal_mesh_index).set_colors(Eigen::RowVector3d(0.6f * 0.75f, 0.8f * 0.75f, 1.0f * 0.75f));
      g_new_mesh_data_available = false;
    }
    return false;
  };

  for (auto &model : models) {
    model.gizmo->callback = [meshIndex = model.meshIndex,
          instrumentId = model.instrumentId,
          T0 = model.T0,
          original_V = model.original_V,
          &viewer,
          &pub_navi](const Eigen::Matrix4f &T) mutable {
          Eigen::Matrix4d       M  = (T * T0.inverse()).cast<double>();
          const Eigen::Matrix4d TT = M.transpose();
          viewer.data(meshIndex).set_vertices((original_V.rowwise().homogeneous() * TT).rowwise().hnormalized());
          viewer.data(meshIndex).compute_normals();

          const Eigen::Vector3d    translation    = M.block<3, 1>(0, 3);
          const Eigen::Matrix3d    rotationMatrix = M.block<3, 3>(0, 0);
          const Eigen::Quaterniond quaternion(rotationMatrix);
          eCALSend(translation, quaternion, pub_navi, instrumentId);
        };
  }

  viewer.callback_mouse_down = [&models, &viewer](igl::opengl::glfw::Viewer &v, const int button, int modifier) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      // 隐藏所有gizmo
      for (auto &model : models) model.gizmo->visible = false;

      const double          x = viewer.current_mouse_x;
      const double          y = viewer.core().viewport(3) - static_cast<double>(viewer.current_mouse_y);
      const Eigen::Vector2f mouse_pos(x, y);

      int selected_mesh_index = -1;
      for (int i = 0; i < v.data_list.size(); ++i) {
        Eigen::Vector3f bc;
        int             fid;
        if (igl::unproject_onto_mesh(mouse_pos, v.core().view, v.core().proj,
                                     v.core().viewport, v.data_list[i].V, v.data_list[i].F, fid, bc)) {
          selected_mesh_index = i;
          break;
        }
      }
      if (selected_mesh_index != -1) {
        // 只显示选中模型的gizmo
        for (auto &model : models) { if (model.meshIndex == selected_mesh_index) model.gizmo->visible = true; }
        return true;
      }
    }
    return false;
  };
  viewer.callback_key_pressed = [&models](igl::opengl::glfw::Viewer &v, const unsigned int key, int mod) {
    ImGuizmo::OPERATION op = ImGuizmo::TRANSLATE;
    switch (key) {
      case 'W':
      case 'w': op = ImGuizmo::TRANSLATE;
        break;
      case 'E':
      case 'e': op = ImGuizmo::ROTATE;
        break;
      case 'R':
      case 'r': op = ImGuizmo::SCALE;
        break;
      case 'F':
      case 'f':
        //
        for (const auto &model : models) {
          model.gizmo->T.setIdentity();
          model.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
          model.gizmo->callback(model.gizmo->T);
        }
        break;
      default: return false;
    }
    for (const auto &model : models) model.gizmo->operation = op;
    return true;
  };

#pragma endregion

  hookModel.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(0.0f, 5.0f, 0.0f);
  hookModel.gizmo->callback(hookModel.gizmo->T);

#pragma region  EXIT
  // igl::opengl::glfw::imgui::ImGuiMenu menu;
  // imgui_plugin.widgets.push_back(&menu);
  viewer.launch();
  sub.RemReceiveCallback();
  eCAL::Finalize();
  return EXIT_SUCCESS;
#pragma endregion
}