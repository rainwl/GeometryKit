#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuizmoWidget.h>
#include <igl/unproject_onto_mesh.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <vector>
#include <memory>
#include "dmSimSvrUnity.pb.h"
#include "ecal/ecal.h"
#include "ecal/msg/protobuf/publisher.h"
#include "google/protobuf/any.pb.h"

#include <mutex>

#pragma region

std::string                       box0_path       = "./box0.obj";
std::string                       box1_path       = "./box1.obj";
std::string                       box2_path       = "./box2.obj";
std::string                       box3_path       = "./box3.obj";
std::string                       forceps_p1_path = "./forceps_p1.obj";
std::string                       forceps_p2_path = "./forceps_p2.obj";
std::string                       shin_path       = "./shin.obj";
std::string                       thigh_path      = "./thigh.obj";
std::string                       hook_path       = "./hook.obj";
std::string                       forceps_path    = "./forceps.obj";
std::string                       shaver_path     = "./shaver.obj";
std::unique_ptr<eCAL::CPublisher> pub_slave;
unsigned short                    forceps_value;

struct Model {
  Eigen::MatrixXd                                           original_V;
  Eigen::MatrixXi                                           F;
  int                                                       meshIndex{};
  int                                                       instrumentId{};
  std::shared_ptr<igl::opengl::glfw::imgui::ImGuizmoWidget> gizmo;
  Eigen::Matrix4f                                           T0;
};

enum SlaveDeviceSerialMessageType : unsigned short {
  SlaveDeviceSerialMessageType_Id = 0,
  SlaveDeviceSerialMessageType_AnaLog,
  SlaveDeviceSerialMessageType_FiveDirJoystick,

  SlaveDeviceSerialMessageType_HeartBeat,
  SlaveDeviceSerialMessageType_RawBytes,

  SlaveDeviceSerialMessageType_InstrumentData,

  // the message send only to the server not to the slave device
  SlaveDeviceSerialMessageType_NucleusOpenValue,
  SlaveDeviceSerialMessageType_NucleusCloseValue,
  SlaveDeviceSerialMessageType_AblationOpenValue,
  SlaveDeviceSerialMessageType_AblationCloseValue,
  SlaveDeviceSerialMessageType_BonePliersOpenValue,
  SlaveDeviceSerialMessageType_BonePliersCloseValue,

  SlaveDeviceSerialMessageType_OpenSerialPort,
  SlaveDeviceSerialMessageType_CloseSerialPort,

  SlaveDeviceSerialMessageType_ShowAnalogValues,
  SlaveDeviceSerialMessageType_HideAnalogValues,
};

std::vector<Model> models;

std::mutex g_mesh_mutex;

struct MeshData {
  Eigen::MatrixXf V;
  Eigen::MatrixXi F;
  Eigen::MatrixXf N;
  bool            new_data = false;
};

std::map<int, MeshData> g_mesh_data;

std::map<int, int> g_ecalIdToViewerMeshIndex;
int                ecal_id_acl     = -1;
int                ecal_id_pcl     = -1;
int                ecal_id_lcl     = -1;
int                ecal_id_mcl     = -1;
int                ecal_id_lm      = -1;
int                ecal_id_mm      = -1;
int                ecal_id_pl      = -1;
int                ecal_id_patella = -1;
int                ecal_id_jcap    = -1;

#pragma endregion

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
  const std::size_t sendLen  = any.ByteSizeLong();
  auto *            sendData = new std::uint8_t[sendLen]{0};
  any.SerializePartialToArray(sendData, static_cast<int>(sendLen));

  pub.SetID(6);
  if (const std::size_t sendCode = pub.Send(sendData, sendLen); sendCode != sendLen) { std::cout << pub.GetTopicName() << " send failed\n"; }
  delete[] sendData;
  delete imSend;
}

bool AddModel(const std::string &path, const int instrumentId, igl::opengl::glfw::Viewer &viewer, igl::opengl::glfw::imgui::ImGuiPlugin &imgui_plugin, Model &model, const Eigen::Vector4f &color, bool is_line, eCAL::CPublisher &pub_navi) {
  if (!igl::read_triangle_mesh(path, model.original_V, model.F)) {
    std::cerr << "Unable to load model: " << path << std::endl;
    return false;
  }

  model.meshIndex = viewer.append_mesh();
  viewer.data(model.meshIndex).set_mesh(model.original_V, model.F);
  const Eigen::RowVector3d face_color = color.head<3>().cast<double>();
  viewer.data(model.meshIndex).set_colors(face_color);
  viewer.data(model.meshIndex).show_lines = is_line;
  if (is_line) {
    viewer.data(model.meshIndex).line_color = color;
    viewer.data(model.meshIndex).show_faces = false;
  } else { viewer.data(model.meshIndex).show_faces = true; }

  model.instrumentId = instrumentId;
  model.gizmo        = std::make_shared<igl::opengl::glfw::imgui::ImGuizmoWidget>();
  imgui_plugin.widgets.push_back(model.gizmo.get());
  model.gizmo->visible = false;
  model.gizmo->T.setIdentity();
  model.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  model.T0                         = model.gizmo->T;

  auto *p_pub_navi      = &pub_navi;
  model.gizmo->callback = [meshIndex = model.meshIndex,
        instrumentId = model.instrumentId,
        T0 = model.T0,
        original_V = model.original_V,
        &viewer, p_pub_navi](const Eigen::Matrix4f &T) mutable {
        Eigen::Matrix4d       M  = (T * T0.inverse()).cast<double>();
        const Eigen::Matrix4d TT = M.transpose();
        viewer.data(meshIndex).set_vertices((original_V.rowwise().homogeneous() * TT).rowwise().hnormalized());
        viewer.data(meshIndex).compute_normals();

        const Eigen::Vector3d    translation    = M.block<3, 1>(0, 3);
        const Eigen::Matrix3d    rotationMatrix = M.block<3, 3>(0, 0);
        const Eigen::Quaterniond quaternion(rotationMatrix);

        eCALSend(translation, quaternion, *p_pub_navi, instrumentId);
      };

  models.push_back(model);
  return true;
}

class MyCustomWidget final : public igl::opengl::glfw::imgui::ImGuiWidget {
public:
  void draw() override {
    ImGui::SetNextWindowPos(ImVec2(50, 50), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(500, 50), ImGuiCond_Always);
    ImGuiWindowFlags window_flags = 0;
    window_flags |= ImGuiWindowFlags_NoTitleBar;
    window_flags |= ImGuiWindowFlags_NoResize;
    window_flags |= ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoCollapse;
    window_flags |= ImGuiWindowFlags_NoBackground;
    window_flags |= ImGuiWindowFlags_NoSavedSettings;

    ImGui::Begin("##My custom window", nullptr, window_flags);

    float v = static_cast<float>(forceps_value) / 1000;
    ImGui::SliderFloat("forceps", &v, 0.0f, 1.0f);
    forceps_value = static_cast<unsigned short>(v * 1000);

    constexpr size_t num     = 13;
    const auto       message = new unsigned short[num];
    message[0]               = SlaveDeviceSerialMessageType_InstrumentData;
    message[1]               = 0;
    message[2]               = forceps_value;
    message[3]               = 0;
    message[4]               = forceps_value;
    message[5]               = 0;
    message[6]               = forceps_value;
    message[7]               = 0;
    message[8]               = forceps_value;
    message[9]               = 0;
    message[10]              = forceps_value;
    message[11]              = 0;
    message[12]              = forceps_value;

    pub_slave->Send(reinterpret_cast<char *>(message), num * sizeof(short));
    delete[] message;
    ImGui::End();
  }
};

void OnMeshDataReceived(const char *topic_name, const struct eCAL::SReceiveCallbackData *data) {
  std::lock_guard lock(g_mesh_mutex);
  const auto *    ptr = static_cast<const uint8_t *>(data->buf);

#pragma region I.ecal_id
  ptr += sizeof(int);
  const int ecal_id = *reinterpret_cast<const int *>(ptr);
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

  g_mesh_data[ecal_id].V        = V;
  g_mesh_data[ecal_id].F        = F;
  g_mesh_data[ecal_id].N        = N;
  g_mesh_data[ecal_id].new_data = true;
}

int main(int argc, char *argv[]) {
  eCAL::Initialize(1, nullptr, "GeometryKit");
  eCAL::CPublisher pub_navi("simsvr-data");
  pub_slave = std::make_unique<eCAL::CPublisher>("dmSlaveDeviceNextData");
  eCAL::CSubscriber sub("Physics2Unity-SoftBody");
  sub.AddReceiveCallback([](auto &&argument_1, auto &&argument_2) { OnMeshDataReceived(std::forward<decltype(argument_1)>(argument_1), std::forward<decltype(argument_2)>(argument_2)); });

  igl::opengl::glfw::Viewer             viewer;
  igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
  viewer.plugins.push_back(&imgui_plugin);

  // Model box0, box1, forceps_p1;
  // AddModel(box0_path, 0, viewer, imgui_plugin, box0, Eigen::Vector4f(1, 1, 1, 1), false, pub_navi);
  // AddModel(box1_path, 1, viewer, imgui_plugin, box1, Eigen::Vector4f(1, 1, 1, 1), false, pub_navi);
  // AddModel(forceps_p1_path, 2, viewer, imgui_plugin, forceps_p1, Eigen::Vector4f(1, 1, 1, 1), false, pub_navi);

  Model shinModel;
  Model hookModel;
  Model thighModel;
  Model shaverModel;
  Model forcepsModel;

  AddModel(hook_path, 5000, viewer, imgui_plugin, hookModel, Eigen::Vector4f(1, 1, 1, 1), false, pub_navi);
  AddModel(shaver_path, 5002, viewer, imgui_plugin, shaverModel, Eigen::Vector4f(1, 1, 1, 1), false, pub_navi);
  AddModel(thigh_path, 2100, viewer, imgui_plugin, thighModel, Eigen::Vector4f(1, 0, 0, 1), true, pub_navi);
  AddModel(shin_path, 2101, viewer, imgui_plugin, shinModel, Eigen::Vector4f(1, 0, 0, 1), true, pub_navi);
  AddModel(forceps_path, 5001, viewer, imgui_plugin, forcepsModel, Eigen::Vector4f(1, 1, 1, 1), false, pub_navi);

  ecal_id_acl     = viewer.append_mesh();
  ecal_id_pcl     = viewer.append_mesh();
  ecal_id_lcl     = viewer.append_mesh();
  ecal_id_mcl     = viewer.append_mesh();
  ecal_id_lm      = viewer.append_mesh();
  ecal_id_mm      = viewer.append_mesh();
  ecal_id_pl      = viewer.append_mesh();
  ecal_id_patella = viewer.append_mesh();
  ecal_id_jcap    = viewer.append_mesh();

  g_ecalIdToViewerMeshIndex[1] = ecal_id_acl;
  g_ecalIdToViewerMeshIndex[2] = ecal_id_pcl;
  g_ecalIdToViewerMeshIndex[3] = ecal_id_lcl;
  g_ecalIdToViewerMeshIndex[4] = ecal_id_mcl;
  g_ecalIdToViewerMeshIndex[5] = ecal_id_lm;
  g_ecalIdToViewerMeshIndex[6] = ecal_id_mm;
  g_ecalIdToViewerMeshIndex[7] = ecal_id_pl;
  g_ecalIdToViewerMeshIndex[8] = ecal_id_patella;
  g_ecalIdToViewerMeshIndex[9] = ecal_id_jcap;

  viewer.callback_pre_draw = [](igl::opengl::glfw::Viewer &v) -> bool {
    std::lock_guard lock(g_mesh_mutex);
    for (auto &entry : g_mesh_data) {
      int       received_id = entry.first;
      MeshData &meshData    = entry.second;
      if (meshData.new_data) {
        auto it = g_ecalIdToViewerMeshIndex.find(received_id);
        if (it != g_ecalIdToViewerMeshIndex.end()) {
          int viewerMeshIndex = it->second;
          if (meshData.V.size() != 0 && meshData.F.size() != 0 && meshData.N.size() != 0) {
            v.data(viewerMeshIndex).set_mesh(meshData.V.cast<double>(), meshData.F);
            // v.data(viewerMeshIndex).compute_normals();
            v.data(viewerMeshIndex).set_normals(meshData.N.cast<double>());
            v.data(viewerMeshIndex).set_colors(Eigen::RowVector3d(1, 1, 1));
            v.data(viewerMeshIndex).show_lines = false;
          }
        }
        meshData.new_data = false;
      }
    }
    return false;
  };

  viewer.callback_mouse_down = [&viewer](igl::opengl::glfw::Viewer &v, const int button, int modifier) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      for (const auto &model : models) model.gizmo->visible = false;
      const double          x = viewer.current_mouse_x;
      const double          y = viewer.core().viewport(3) - static_cast<double>(viewer.current_mouse_y);
      const Eigen::Vector2f mouse_pos(x, y);
      int                   selected_mesh_index = -1;
      for (int i = 0; i < v.data_list.size(); ++i) {
        int             fid;
        Eigen::Vector3f bc;
        if (igl::unproject_onto_mesh(mouse_pos, v.core().view, v.core().proj,
                                     v.core().viewport, v.data_list[i].V, v.data_list[i].F, fid, bc)) {
          selected_mesh_index = i;
          break;
        }
      }
      if (selected_mesh_index != -1) {
        for (const auto &model : models) { if (model.meshIndex == selected_mesh_index) model.gizmo->visible = true; }
        return true;
      }
    }
    return false;
  };

  viewer.callback_key_pressed = [hookModel, shaverModel, forcepsModel,&viewer](igl::opengl::glfw::Viewer &v, const unsigned int key, int mod) -> bool {
    ImGuizmo::OPERATION op = ImGuizmo::TRANSLATE;
    if (key == 'W' || key == 'w') {
      op = ImGuizmo::TRANSLATE;
      for (auto &model : models) model.gizmo->operation = op;
      return true;
    } else if (key == 'E' || key == 'e') {
      op = ImGuizmo::ROTATE;
      for (auto &model : models) model.gizmo->operation = op;
      return true;
    } else if (key == 'R' || key == 'r') {
      for (auto &model : models) {
        model.gizmo->T.setIdentity();
        model.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        model.gizmo->callback(model.gizmo->T);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      hookModel.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(2.0f, -2.0f, 0.0f);
      hookModel.gizmo->callback(hookModel.gizmo->T);
      shaverModel.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(3.0f, -2.0f, 0.0f);
      shaverModel.gizmo->callback(shaverModel.gizmo->T);
      forcepsModel.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(1.0f, -2.0f, 0.0f);
      forcepsModel.gizmo->callback(forcepsModel.gizmo->T);
      return true;
    }
    return false;
  };

  auto *custom_widget = new MyCustomWidget();
  imgui_plugin.widgets.push_back(custom_widget);

  hookModel.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(2.0f, -2.0f, 0.0f);
  hookModel.gizmo->callback(hookModel.gizmo->T);
  shaverModel.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(3.0f, -2.0f, 0.0f);
  shaverModel.gizmo->callback(shaverModel.gizmo->T);
  forcepsModel.gizmo->T.block(0, 3, 3, 1) = Eigen::Vector3f(1.0f, -2.0f, 0.0f);
  forcepsModel.gizmo->callback(forcepsModel.gizmo->T);

  viewer.launch();
  eCAL::Finalize();
  return EXIT_SUCCESS;
}