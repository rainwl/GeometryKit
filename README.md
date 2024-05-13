# Geometry Kit

## Build

Use `CMake` to generate this project,you can choose `command line` or `vscode` or `CLion` etc.

After build the project,put the `bunny.off` on the root of the build directory.

## Screenshots

![](./doc/Group.png)

## Feature

- [ ] CMake to xmake
- [ ] Draw plane
- [ ] ...

## Case

### Draw Mesh

```c++
Eigen::MatrixXd V;
Eigen::MatrixXi F;
igl::readOFF("./bunny.off",V,F);
igl::opengl::glfw::Viewer viewer;
viewer.data().set_mesh(V,F);
viewer.launch();
```

### Bind callback function
```c++
bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
  std::cout<<"Key: "<<key<<" "<<(unsigned int)key<<std::endl;
  if (key == '1')
  {
    // Clear should be called before drawing the mesh
    viewer.data().clear();
    // Draw_mesh creates or updates the vertices and faces of the displayed mesh.
    // If a mesh is already displayed, draw_mesh returns an error if the given V and
    // F have size different than the current ones
    viewer.data().set_mesh(V1, F1);
    viewer.core().align_camera_center(V1,F1);
  }
  else if (key == '2')
  {
    viewer.data().clear();
    viewer.data().set_mesh(V2, F2);
    viewer.core().align_camera_center(V2,F2);
  }

  return false;
}

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
```

### Use vertex positions as colors
```c++
Eigen::MatrixXd V;
Eigen::MatrixXi F;
Eigen::MatrixXd C;

int main(int argc, char *argv[])
{
  // Load a mesh in OFF format
  igl::readOFF(TUTORIAL_SHARED_PATH "/screwdriver.off", V, F);

  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);

  // Use the (normalized) vertex positions as colors
  C =
    (V.rowwise()            - V.colwise().minCoeff()).array().rowwise()/
    (V.colwise().maxCoeff() - V.colwise().minCoeff()).array();

  // Add per-vertex colors
  viewer.data().set_colors(C);

  // Launch the viewer
  viewer.launch();
}
```
### Draw points,lines,bounding box,overlay,text labels,etc.
```c++
Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[])
{
  // Load a mesh in OFF format
  igl::readOFF(TUTORIAL_SHARED_PATH "/bunny.off", V, F);

  // Find the bounding box
  Eigen::Vector3d m = V.colwise().minCoeff();
  Eigen::Vector3d M = V.colwise().maxCoeff();

  // Corners of the bounding box
  Eigen::MatrixXd V_box(8,3);
  V_box <<
  m(0), m(1), m(2),
  M(0), m(1), m(2),
  M(0), M(1), m(2),
  m(0), M(1), m(2),
  m(0), m(1), M(2),
  M(0), m(1), M(2),
  M(0), M(1), M(2),
  m(0), M(1), M(2);

  // Edges of the bounding box
  Eigen::MatrixXi E_box(12,2);
  E_box <<
  0, 1,
  1, 2,
  2, 3,
  3, 0,
  4, 5,
  5, 6,
  6, 7,
  7, 4,
  0, 4,
  1, 5,
  2, 6,
  7 ,3;

  // Plot the mesh
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);

  // Plot the corners of the bounding box as points
  viewer.data().add_points(V_box,Eigen::RowVector3d(1,0,0));

  // Plot the edges of the bounding box
  for (unsigned i=0;i<E_box.rows(); ++i)
    viewer.data().add_edges
    (
      V_box.row(E_box(i,0)),
      V_box.row(E_box(i,1)),
      Eigen::RowVector3d(1,0,0)
    );

  // Plot labels with the coordinates of bounding box vertices
  std::stringstream l1;
  l1 << m(0) << ", " << m(1) << ", " << m(2);
  viewer.data().add_label(m+Eigen::Vector3d(-0.007, 0, 0),l1.str());
  std::stringstream l2;
  l2 << M(0) << ", " << M(1) << ", " << M(2);
  viewer.data().add_label(M+Eigen::Vector3d(0.007, 0, 0),l2.str());
  // activate label rendering
  viewer.data().show_custom_labels = true;

  // Rendering of text labels is handled by ImGui, so we need to enable the ImGui
  // plugin to show text labels.
  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  viewer.plugins.push_back(&plugin);
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  plugin.widgets.push_back(&menu);
  menu.callback_draw_viewer_window = [](){};

  // Launch the viewer
  viewer.launch();
}
```
### Draw ImGui group and window
```c++
int main(int argc, char *argv[])
{
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  // Load a mesh in OFF format
  igl::readOFF(TUTORIAL_SHARED_PATH "/bunny.off", V, F);

  // Init the viewer
  igl::opengl::glfw::Viewer viewer;

  // Attach a menu plugin
  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  viewer.plugins.push_back(&plugin);
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  plugin.widgets.push_back(&menu);

  // Customize the menu
  double doubleVariable = 0.1f; // Shared between two menus

  // Add content to the default menu window
  menu.callback_draw_viewer_menu = [&]()
  {
    // Draw parent menu content
    menu.draw_viewer_menu();

    // Add new group
    if (ImGui::CollapsingHeader("New Group", ImGuiTreeNodeFlags_DefaultOpen))
    {
      // Expose variable directly ...
      ImGui::InputDouble("double", &doubleVariable, 0, 0, "%.4f");

      // ... or using a custom callback
      static bool boolVariable = true;
      if (ImGui::Checkbox("bool", &boolVariable))
      {
        // do something
        std::cout << "boolVariable: " << std::boolalpha << boolVariable << std::endl;
      }

      // Expose an enumeration type
      enum Orientation { Up=0, Down, Left, Right };
      static Orientation dir = Up;
      ImGui::Combo("Direction", (int *)(&dir), "Up\0Down\0Left\0Right\0\0");

      // We can also use a std::vector<std::string> defined dynamically
      static int num_choices = 3;
      static std::vector<std::string> choices;
      static int idx_choice = 0;
      if (ImGui::InputInt("Num letters", &num_choices))
      {
        num_choices = std::max(1, std::min(26, num_choices));
      }
      if (num_choices != (int) choices.size())
      {
        choices.resize(num_choices);
        for (int i = 0; i < num_choices; ++i)
          choices[i] = std::string(1, 'A' + i);
        if (idx_choice >= num_choices)
          idx_choice = num_choices - 1;
      }
      ImGui::Combo("Letter", &idx_choice, choices);

      // Add a button
      if (ImGui::Button("Print Hello", ImVec2(-1,0)))
      {
        std::cout << "Hello\n";
      }
    }
  };

  // Draw additional windows
  menu.callback_draw_custom_window = [&]()
  {
    // Define next window position + size
    ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiCond_FirstUseEver);
    ImGui::Begin(
        "New Window", nullptr,
        ImGuiWindowFlags_NoSavedSettings
    );

    // Expose the same variable directly ...
    ImGui::PushItemWidth(-80);
    ImGui::DragScalar("double", ImGuiDataType_Double, &doubleVariable, 0.1, 0, 0, "%.4f");
    ImGui::PopItemWidth();

    static std::string str = "bunny";
    ImGui::InputText("Name", str);

    ImGui::End();
  };

  // Plot the mesh
  viewer.data().set_mesh(V, F);
  viewer.data().add_label(viewer.data().V.row(0) + viewer.data().V_normals.row(0).normalized()*0.005, "Hello World!");
  viewer.launch();
}
```
### Draw multiple meshes
```c++
int main(int argc, char * argv[])
{
  igl::opengl::glfw::Viewer viewer;
  const auto names =
    {"cube.obj","sphere.obj","xcylinder.obj","ycylinder.obj","zcylinder.obj"};
  std::map<int, Eigen::RowVector3d> colors;
  int last_selected = -1;
  for(const auto & name : names)
  {
    viewer.load_mesh_from_file(std::string(TUTORIAL_SHARED_PATH) + "/" + name);
    colors.emplace(viewer.data().id, 0.5*Eigen::RowVector3d::Random().array() + 0.5);
  }

  viewer.callback_key_down =
    [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
  {
    if(key == GLFW_KEY_BACKSPACE)
    {
      int old_id = viewer.data().id;
      if (viewer.erase_mesh(viewer.selected_data_index))
      {
        colors.erase(old_id);
        last_selected = -1;
      }
      return true;
    }
    return false;
  };

  // Refresh selected mesh colors
  viewer.callback_pre_draw =
    [&](igl::opengl::glfw::Viewer &)
  {
    if (last_selected != viewer.selected_data_index)
    {
      for (auto &data : viewer.data_list)
      {
        data.set_colors(colors[data.id]);
      }
      viewer.data_list[viewer.selected_data_index].set_colors(Eigen::RowVector3d(0.9,0.1,0.1));
      last_selected = viewer.selected_data_index;
    }
    return false;
  };

  viewer.launch();
  return EXIT_SUCCESS;
}
```
### Draw multiple views
```c++
int main(int argc, char * argv[])
{
  igl::opengl::glfw::Viewer viewer;

  viewer.load_mesh_from_file(std::string(TUTORIAL_SHARED_PATH) + "/cube.obj");
  viewer.load_mesh_from_file(std::string(TUTORIAL_SHARED_PATH) + "/sphere.obj");

  unsigned int left_view, right_view;
  int cube_id = viewer.data_list[0].id;
  int sphere_id = viewer.data_list[1].id;
  viewer.callback_init = [&](igl::opengl::glfw::Viewer &)
  {
    viewer.core().viewport = Eigen::Vector4f(0, 0, 640, 800);
    left_view = viewer.core_list[0].id;
    right_view = viewer.append_core(Eigen::Vector4f(640, 0, 640, 800));
    return false;
  };

  viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
  {
    if(key == GLFW_KEY_SPACE)
    {
      // By default, when a core is appended, all loaded meshes will be displayed in that core.
      // Displaying can be controlled by calling viewer.data().set_visible().
      viewer.data(cube_id).set_visible(false, left_view);
      viewer.data(sphere_id).set_visible(false, right_view);
    }
    return false;
  };

  viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer &v, int w, int h) {
    v.core( left_view).viewport = Eigen::Vector4f(0, 0, w / 2, h);
    v.core(right_view).viewport = Eigen::Vector4f(w / 2, 0, w - (w / 2), h);
    return true;
  };

  viewer.launch();
  return EXIT_SUCCESS;
}
```
### Draw 3D Gizmo
```c++
int main(int argc, char *argv[])
{
  // Load a mesh from file
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh(argc>1?argv[1]: TUTORIAL_SHARED_PATH "/cow.off",V,F);
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
}
```
### Mat cap
```c++
int main(int argc, char *argv[])
{
  igl::opengl::glfw::Viewer v;
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh(
    argc>1?argv[1]: TUTORIAL_SHARED_PATH "/armadillo.obj",V,F);
  Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> R,G,B,A;
  igl::stb::read_image(argc>2?argv[2]: TUTORIAL_SHARED_PATH "/jade.png",R,G,B,A);
  v.data().set_mesh(V,F);
  v.data().set_texture(R,G,B,A);
  v.data().use_matcap = true;
  v.data().show_lines = false;
  v.launch();
}
```
### Select mesh
```c++
int main(int argc, char *argv[])
{
  // Inline mesh of a cube
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh(
    argc>1?argv[1]: TUTORIAL_SHARED_PATH "/armadillo.obj",V,F);

  // Plot the mesh
  igl::opengl::glfw::Viewer vr;
  igl::opengl::glfw::imgui::ImGuiPlugin imgui_plugin;
  vr.plugins.push_back(&imgui_plugin);
  igl::opengl::glfw::imgui::SelectionWidget widget;
  imgui_plugin.widgets.push_back(&widget);

  Eigen::VectorXd W = Eigen::VectorXd::Zero(V.rows());
  Eigen::Array<double,Eigen::Dynamic,1> and_visible = 
    Eigen::Array<double,Eigen::Dynamic,1>::Zero(V.rows());
  const Eigen::MatrixXd CM = (Eigen::MatrixXd(2,3)<<
      0.3,0.3,0.5,                 
      255.0/255.0,228.0/255.0,58.0/255.0).finished();
  bool only_visible = false;
  const auto update = [&]()
  {
    const bool was_face_based = vr.data().face_based;
    Eigen::VectorXd S = W;
    if(only_visible) { S.array() *= and_visible; }
    vr.data().set_data(S,0,1,igl::COLOR_MAP_TYPE_PLASMA,2);
    vr.data().face_based = was_face_based;
    vr.data().set_colormap(CM);
  };
  igl::AABB<Eigen::MatrixXd, 3> tree;
  tree.init(V,F);
  widget.callback = [&]()
  {
    screen_space_selection(V,F,tree,vr.core().view,vr.core().proj,vr.core().viewport,widget.L,W,and_visible);
    update();
  };
  vr.callback_key_pressed = [&](decltype(vr) &,unsigned int key, int mod)
  {
    switch(key)
    {
      case ' ': only_visible = !only_visible; update(); return true;
      case 'D': case 'd': W.setZero(); update(); return true;
    }
    return false;
  };
  std::cout<<R"(
Usage:
  [space]  Toggle whether to take visibility into account
  D,d      Clear selection
)";
  vr.data().set_mesh(V,F);
  vr.data().set_face_based(true);
  vr.core().background_color.head(3) = CM.row(0).head(3).cast<float>();
  vr.data().line_color.head(3) = (CM.row(0).head(3)*0.5).cast<float>();
  vr.data().show_lines = F.rows() < 20000;
  update();
  vr.launch();
}
```
### Draw floor and shadow
```c++
void floor(const Eigen::MatrixXd & V, 
    Eigen::MatrixXd & fV,
    Eigen::MatrixXd & fU,
    Eigen::MatrixXi & fF)
{
  igl::triangulated_grid(2,2,fU,fF);
  fV = fU;
  fV.array() -= 0.5;
  fV.array() *= 2 * 2 * 
    (V.colwise().maxCoeff() - V.colwise().minCoeff()).norm();
  fV = (fV * (Eigen::Matrix<double,2,3>()<<1,0,0,0,0,-1).finished() ).eval();
  fV.col(0).array() += 0.5*(V.col(0).minCoeff()+ V.col(0).maxCoeff());
  fV.col(1).array() += V.col(1).minCoeff();
  fV.col(2).array() += 0.5*(V.col(2).minCoeff()+ V.col(2).maxCoeff());
}

void checker_texture(const int s, const int f,
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> & X,
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> & A)
{
  X.resize(s*f,s*f);
  A.resize(s*f,s*f);
  for(int i = 0;i<s*f;i++)
  {
    const double x = double(i)/double(s*f-1)*2-1;
    for(int j = 0;j<s*f;j++)
    {
      const int u = i/f;
      const int v = j/f;
      const double y = double(j)/double(s*f-1)*2-1;
      const double r1 = std::min(std::max( (1.0 - sqrt(x*x+y*y))*1.0 ,0.0),1.0);
      const double r3 = std::min(std::max( (1.0 - sqrt(x*x+y*y))*3.0 ,0.0),1.0);
      //const double a = 3*r*r - 2*r*r*r;
      const auto smooth_step = [](const double w)
      {
        return ((w * (w * 6.0 - 15.0) + 10.0) * w * w * w) ;
      };
      double a3 = smooth_step(r1);
      double a1 = smooth_step(r1);
      X(i,j) = (0.75+0.25*a1) * (u%2 == v%2 ? 245 : 235);
      A(i,j) = a3 * 255;
    }
  }
}

int main(int argc, char *argv[])
{
  igl::opengl::glfw::Viewer vr;
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh(
    argc>1?argv[1]: TUTORIAL_SHARED_PATH "/armadillo.obj",V,F);

  // Create a floor
  Eigen::MatrixXd fV, fU;
  Eigen::MatrixXi fF;
  floor(V,fV,fU,fF);
  const int s = 16;
  const int f = 100;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> X;
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A;
  checker_texture(s,f,X,A);
  vr.data().set_mesh(fV,fF);
  vr.data().set_uv(fU);
  vr.data().uniform_colors(Eigen::Vector3d(0.3,0.3,0.3),Eigen::Vector3d(0.8,0.8,0.8),Eigen::Vector3d(0,0,0));
  vr.data().set_texture(X,X,X,A);
  vr.data().show_texture = true;
  vr.data().show_lines = false;

  // Move the light a bit off center to cast a more visible shadow.
  vr.core().light_position << 1.0f, 2.0f, 0.0f;
  // For now, the default is a positional light with no shadows. Meanwhile,
  // shadows only support a directional light. To best match the appearance of
  // current lighting use this conversion when turning on shadows. In the
  // future, hopefully this will reduce to just 
  //     core().is_shadow_mapping = true
  vr.core().is_directional_light = true;
  vr.core().light_position = vr.core().light_position + vr.core().camera_eye;
  vr.core().is_shadow_mapping = true;

  // Send the main object to the viewer
  vr.append_mesh();
  vr.data().set_mesh(V,F);
  vr.data().show_lines = false;
  vr.data().set_face_based(true);

  // If a second argument is present read it as a matcap
  if(argc>2)
  {
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> R,G,B,A;
    igl::stb::read_image(argv[2],R,G,B,A);
    // If more args, read them as light direction
    if(argc>2+3)
    {
      Eigen::Vector3f D;
      D << std::atof(argv[3]), std::atof(argv[4]), std::atof(argv[5]);
      D.normalize();
      vr.core().light_position = D;
      Eigen::Vector3d Ka(0.14,0.14,0.14);
      if(argc>2+3+1 && std::atoi(argv[6]))
      {
        // Assume that color opposite D along umbra boundary is ambient color
        const double s = -D(2)*1.0/sqrt((D(0)*D(0)+D(1)*D(1))*(D(0)*D(0)+D(1)*D(1)+D(2)*D(2))); 
        const int i = ((D(0)*s)*0.5+0.5)*R.cols();
        const int j = ((D(1)*s)*0.5+0.5)*R.rows();
        Ka << double(R(i,j))/255.0, double(G(i,j))/255.0, double(B(i,j))/255.0;
      }
      std::cout<<"Ka : "<<Ka<<std::endl;
      // viewer only exposes ambient color through per-face and per-vertex
      // materials
      vr.data().V_material_ambient.col(0).setConstant( Ka(0) );
      vr.data().V_material_ambient.col(1).setConstant( Ka(1) );
      vr.data().V_material_ambient.col(2).setConstant( Ka(2) );
      vr.data().F_material_ambient.col(0).setConstant( Ka(0) );
      vr.data().F_material_ambient.col(1).setConstant( Ka(1) );
      vr.data().F_material_ambient.col(2).setConstant( Ka(2) );
    }
    vr.data().set_texture(R,G,B,A);
    vr.data().use_matcap = true;
  }
  vr.core().is_animating = true;
  vr.core().camera_zoom *= 1.5;
  vr.callback_pre_draw = [&](decltype(vr)&)
  {
    if(vr.core().is_animating)
    {
      vr.core().trackball_angle = Eigen::AngleAxisf( 
          sin(igl::get_seconds())*igl::PI*0.5,
          Eigen::Vector3f(0,1,0));
    }
    return false;
  };
  vr.launch();
}
```
### Draw Gaussian curvature
```c++
int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;
  MatrixXd V;
  MatrixXi F;
  igl::readOFF(TUTORIAL_SHARED_PATH "/bumpy.off",V,F);

  VectorXd K;
  // Compute integral of Gaussian curvature
  igl::gaussian_curvature(V,F,K);
  // Compute mass matrix
  SparseMatrix<double> M,Minv;
  igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_DEFAULT,M);
  igl::invert_diag(M,Minv);
  // Divide by area to get integral average
  K = (Minv*K).eval();

  // Plot the mesh with pseudocolors
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);
  viewer.data().set_data(K);
  viewer.launch();
}
```
### Draw Curvature Directions
```c++
Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[])
{
  using namespace Eigen;
  std::string filename = TUTORIAL_SHARED_PATH "/fertility.off";
  if(argc>1)
  {
    filename = argv[1];
  }
  // Load a mesh in OFF format
  igl::read_triangle_mesh(filename, V, F);

  // Alternative discrete mean curvature
  MatrixXd HN;
  SparseMatrix<double> L,M,Minv;
  igl::cotmatrix(V,F,L);
  igl::massmatrix(V,F,igl::MASSMATRIX_TYPE_VORONOI,M);
  igl::invert_diag(M,Minv);
  // Laplace-Beltrami of position
  HN = -Minv*(L*V);
  // Extract magnitude as mean curvature
  VectorXd H = HN.rowwise().norm();

  // Compute curvature directions via quadric fitting
  MatrixXd PD1,PD2;
  VectorXd PV1,PV2;
  igl::principal_curvature(V,F,PD1,PD2,PV1,PV2);
  // mean curvature
  H = 0.5*(PV1+PV2);

  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);

  viewer.data().set_data(H);

  // Average edge length for sizing
  const double avg = igl::avg_edge_length(V,F);

  // Draw a red segment parallel to the maximal curvature direction
  const RowVector3d red(0.8,0.2,0.2),blue(0.2,0.2,0.8);
  viewer.data().add_edges(V + PD1*avg, V - PD1*avg, red);

  // Draw a blue segment parallel to the minimal curvature direction
  viewer.data().add_edges(V + PD2*avg, V - PD2*avg, blue);

  // Hide wireframe
  viewer.data().show_lines = false;

  viewer.launch();
}
```
### Do Laplacian
```c++
Eigen::MatrixXd V,U;
Eigen::MatrixXi F;
Eigen::SparseMatrix<double> L;
igl::opengl::glfw::Viewer viewer;

int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;

  // Load a mesh in OFF format
  igl::readOFF(TUTORIAL_SHARED_PATH "/cow.off", V, F);

  // Compute Laplace-Beltrami operator: #V by #V
  igl::cotmatrix(V,F,L);

  // Alternative construction of same Laplacian
  SparseMatrix<double> G,K;
  // Gradient/Divergence
  igl::grad(V,F,G);
  // Diagonal per-triangle "mass matrix"
  VectorXd dblA;
  igl::doublearea(V,F,dblA);
  // Place areas along diagonal #dim times
  const auto & T = 1.*(dblA.replicate(3,1)*0.5).asDiagonal();
  // Laplacian K built as discrete divergence of gradient or equivalently
  // discrete Dirichelet energy Hessian
  K = -G.transpose() * T * G;
  cout<<"|K-L|: "<<(K-L).norm()<<endl;

  const auto &key_down = [](igl::opengl::glfw::Viewer &viewer,unsigned char key,int mod)->bool
  {
    switch(key)
    {
      case 'r':
      case 'R':
        U = V;
        break;
      case ' ':
      {
        // Recompute just mass matrix on each step
        SparseMatrix<double> M;
        igl::massmatrix(U,F,igl::MASSMATRIX_TYPE_BARYCENTRIC,M);
        // Solve (M-delta*L) U = M*U
        const auto & S = (M - 0.001*L);
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double > > solver(S);
        assert(solver.info() == Eigen::Success);
        U = solver.solve(M*U).eval();
        // Compute centroid and subtract (also important for numerics)
        VectorXd dblA;
        igl::doublearea(U,F,dblA);
        double area = 0.5*dblA.sum();
        MatrixXd BC;
        igl::barycenter(U,F,BC);
        RowVector3d centroid(0,0,0);
        for(int i = 0;i<BC.rows();i++)
        {
          centroid += 0.5*dblA(i)/area*BC.row(i);
        }
        U.rowwise() -= centroid;
        // Normalize to unit surface area (important for numerics)
        U.array() /= sqrt(area);
        break;
      }
      default:
        return false;
    }
    // Send new positions, update normals, recenter
    viewer.data().set_vertices(U);
    viewer.data().compute_normals();
    viewer.core().align_camera_center(U,F);
    return true;
  };


  // Use original normals as pseudo-colors
  MatrixXd N;
  igl::per_vertex_normals(V,F,N);
  MatrixXd C = N.rowwise().normalized().array()*0.5+0.5;

  // Initialize smoothing with base mesh
  U = V;
  viewer.data().set_mesh(U, F);
  viewer.data().set_colors(C);
  viewer.callback_key_down = key_down;

  cout<<"Press [space] to smooth."<<endl;;
  cout<<"Press [r] to reset."<<endl;;
  return viewer.launch();
}
```
### Select face Kelvinlets
```c++

namespace {
void ShowHelpMarker(const char* desc)
{
  ImGui::SameLine();
  ImGui::TextDisabled("(?)");
  if (ImGui::IsItemHovered()) {
    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(450.0f);
    ImGui::TextUnformatted(desc);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}
}

int main()
{
  Eigen::MatrixXd V1, OrigV;
  Eigen::MatrixXi F1, OrigF;

  igl::readOFF(TUTORIAL_SHARED_PATH "/bumpy.off", OrigV, OrigF);
  std::cout << "1 View original mesh\n";
  std::cout << "2 Switch to deformed mesh\n";
  V1 = OrigV;
  F1 = OrigF;

  igl::opengl::glfw::Viewer viewer;
  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  viewer.plugins.push_back(&plugin);
  igl::opengl::glfw::imgui::ImGuiMenu menu;
  plugin.widgets.push_back(&menu);

  auto brushRadius = 1.;
  auto brushType = igl::BrushType::GRAB;
  auto scale = 1;
  menu.callback_draw_custom_window = [&]() {
    ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(200, 160), ImGuiCond_FirstUseEver);
    ImGui::Begin(
      "Kelvinlet Brushes", nullptr, ImGuiWindowFlags_NoSavedSettings);

    ImGui::InputDouble("Brush Radius", &brushRadius, 0, 0, "%.4f");
    ImGui::Combo("Brush type",
                 reinterpret_cast<int*>(&brushType),
                 "Grab\0Scale\0Twist\0Pinch\0\0");
    ImGui::InputInt("Falloff", &scale);
    ShowHelpMarker("Defines how localized the stroke is {1,2,3}");
    ImGui::End();
  };

  Eigen::Vector3d posStart(0, 0, 0);
  Eigen::Vector3d posEnd;
  decltype(OrigV) result;
  auto min_point = V1.colwise().minCoeff();
  auto max_point = V1.colwise().maxCoeff();
  // to multiply brush force proportional to size of mesh
  auto brush_strength = (max_point - min_point).norm();
  Eigen::Matrix3d twist, pinch;
  twist << 0, 1, -1, -1, 0, 1, 1, -1, 0; // skew-symmetric
  pinch << 0, 1, 1, 1, 0, 1, 1, 1, 0;    // symmetric

  viewer.callback_key_down =
    [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int) {
      if (key == '1') {
        viewer.data().clear();
        viewer.data().set_mesh(OrigV, OrigF);
        viewer.core().align_camera_center(OrigV, OrigF);
      } else if (key == '2') {
        viewer.data().clear();
        viewer.data().set_mesh(V1, F1);
        viewer.core().align_camera_center(V1, F1);
      }
      return false;
    };

  viewer.callback_mouse_down =
    [&](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
    Eigen::Vector3f bc;
    int fid;
    auto x = viewer.current_mouse_x;
    auto y =
      viewer.core().viewport(3) - static_cast<float>(viewer.current_mouse_y);
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y),
                                 viewer.core().view,
                                 viewer.core().proj,
                                 viewer.core().viewport,
                                 V1,
                                 F1,
                                 fid,
                                 bc)) {
      posStart = igl::unproject(Eigen::Vector3f(x, y, viewer.down_mouse_z),
                                viewer.core().view,
                                viewer.core().proj,
                                viewer.core().viewport)
                   .template cast<double>();
      return true;
    }
    return false;
  };

  viewer.callback_mouse_move =
    [&](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
    if (!posStart.isZero() && !posStart.hasNaN()) {
      posEnd = igl::unproject(
                 Eigen::Vector3f(viewer.current_mouse_x,
                                 viewer.core().viewport[3] -
                                   static_cast<float>(viewer.current_mouse_y),
                                 viewer.down_mouse_z),
                 viewer.core().view,
                 viewer.core().proj,
                 viewer.core().viewport)
                 .template cast<double>();

      // exaggerate the force by a little bit
      Eigen::Vector3d forceVec = (posEnd - posStart) * brush_strength;

      int scaleFactor = forceVec.norm();
      if (posEnd.x() < posStart.x()) {
        // probably not the best way to determine direction.
        scaleFactor = -scaleFactor;
      }
      Eigen::Matrix3d mat;
      switch (brushType) {
        case igl::BrushType::GRAB:
          mat.setZero();
          break;
        case igl::BrushType::SCALE:
          mat = Eigen::Matrix3d::Identity() * scaleFactor;
          break;
        case igl::BrushType::TWIST:
          mat = twist * scaleFactor;
          break;
        case igl::BrushType::PINCH:
          mat = pinch * scaleFactor;
          break;
      }

      igl::kelvinlets(
        V1,
        posStart,
        forceVec,
        mat,
        igl::KelvinletParams<double>(brushRadius, scale, brushType),
        result);
      viewer.data().set_vertices(result);
      viewer.data().compute_normals();
      return true;
    }
    return false;
  };

  viewer.callback_mouse_up =
    [&](igl::opengl::glfw::Viewer& viewer, int, int) -> bool {
    if (!posStart.isZero()) {
      V1 = result;
      posStart.setZero();
      return true;
    }
    return false;
  };

  viewer.data().set_mesh(V1, F1);
  viewer.core().align_camera_center(V1, F1);
  viewer.launch();
}
```
### Pick face
```c++
int main(int argc, char *argv[])
{
  // Mesh with per-face color
  Eigen::MatrixXd V, C;
  Eigen::MatrixXi F;

  // Load a mesh in OFF format
  igl::readOFF(TUTORIAL_SHARED_PATH "/fertility.off", V, F);

  // Initialize white
  C = Eigen::MatrixXd::Constant(F.rows(),3,1);
  igl::opengl::glfw::Viewer viewer;
  viewer.callback_mouse_down =
    [&V,&F,&C](igl::opengl::glfw::Viewer& viewer, int, int)->bool
  {
    int fid;
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core().viewport(3) - viewer.current_mouse_y;
    if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core().view,
      viewer.core().proj, viewer.core().viewport, V, F, fid, bc))
    {
      // paint hit red
      C.row(fid)<<1,0,0;
      viewer.data().set_colors(C);
      return true;
    }
    return false;
  };
  std::cout<<R"(Usage:
  [click]  Pick face on shape

)";
  // Show mesh
  viewer.data().set_mesh(V, F);
  viewer.data().set_colors(C);
  viewer.data().show_lines = false;
  viewer.launch();
}
```

