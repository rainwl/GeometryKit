// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2022 Alec Jacobson <alecjacobson@gmail.com>
// Copyright (C) 2018 Jérémie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
////////////////////////////////////////////////////////////////////////////////
#include "ImGuiPlugin.h"
#include "ImGuiHelpers.h"
#include "../../../project.h"
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>
#include <imgui_fonts_droid_sans.h>
#include <GLFW/glfw3.h>
#include <iostream>

namespace igl
{
namespace opengl
{
namespace glfw
{
namespace imgui
{

IGL_INLINE void ImGuiPlugin::init(igl::opengl::glfw::Viewer *_viewer)
{
  ViewerPlugin::init(_viewer);
  // Setup ImGui binding
  if (_viewer)
  {
    IMGUI_CHECKVERSION();
    if (!context_)
    {
      // Single global context by default, but can be overridden by the user
      static ImGuiContext * __global_context = ImGui::CreateContext();
      context_ = __global_context;
    }
    const char* glsl_version = "#version 150";
    ImGui_ImplGlfw_InitForOpenGL(viewer->window, false);
    ImGui_ImplOpenGL3_Init(glsl_version);
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 5.0f;
    style.WindowRounding = 12;
    style.ChildRounding = 12;
    style.FrameRounding = 12;
    style.PopupRounding = 6;
    style.ScrollbarRounding = 8;
    style.GrabRounding = 12;
    style.TabRounding = 8;
    ImVec4 *colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 0.34f);
    colors[ImGuiCol_BorderShadow] = ImVec4(0.66f, 0.66f, 0.66f, 0.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.47f, 0.47f, 0.47f, 0.40f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.79f, 0.79f, 0.79f, 0.67f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.26f, 0.26f, 0.26f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.55f, 0.55f, 0.55f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.65f, 0.65f, 0.65f, 0.40f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.66f, 0.66f, 0.66f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.85f, 0.85f, 0.85f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.70f, 0.70f, 0.70f, 0.80f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.85f, 0.85f, 0.85f, 1.00f);
    colors[ImGuiCol_SeparatorHovered] = ImVec4(0.60f, 0.60f, 0.60f, 0.78f);
    colors[ImGuiCol_SeparatorActive] = ImVec4(0.75f, 0.75f, 0.75f, 1.00f);
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.25f, 0.25f, 0.25f, 0.20f);
    colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.36f, 0.36f, 0.36f, 0.67f);
    colors[ImGuiCol_ResizeGripActive] = ImVec4(0.74f, 0.74f, 0.74f, 0.95f);
    colors[ImGuiCol_Tab] = ImVec4(0.64f, 0.64f, 0.64f, 0.86f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.24f, 0.24f, 0.24f, 0.80f);
    colors[ImGuiCol_TabActive] = ImVec4(0.81f, 0.81f, 0.81f, 1.00f);
    colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.66f, 0.66f, 0.66f, 1.00f);
    colors[ImGuiCol_TextSelectedBg] = ImVec4(0.71f, 0.71f, 0.71f, 0.35f);
    colors[ImGuiCol_NavHighlight] = ImVec4(0.52f, 0.52f, 0.52f, 1.00f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.52f, 0.52f, 0.52f, 0.54f);
    colors[ImGuiCol_Header] = ImVec4(0.67f, 0.67f, 0.67f, 0.31f);
    colors[ImGuiCol_TableHeaderBg] = ImVec4(0.38f, 0.38f, 0.38f, 1.00f);
    colors[ImGuiCol_DragDropTarget] = ImVec4(0.64f, 1.00f, 0.85f, 0.95f);




    reload_font();
  }
  init_widgets();
}

IGL_INLINE void ImGuiPlugin::init_widgets()
{
  // Init all widgets
  for(auto & widget : widgets) { widget->init(viewer, this); }
}
IGL_INLINE void ImGuiPlugin::reload_font(int font_size)
{
  hidpi_scaling_ = hidpi_scaling();
  pixel_ratio_ = pixel_ratio();
  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->Clear();
  io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data,
    droid_sans_compressed_size, font_size * hidpi_scaling_);
  io.FontGlobalScale = 1.0 / pixel_ratio_;
}

IGL_INLINE void ImGuiPlugin::shutdown()
{
  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  // User is responsible for destroying context if a custom context is given
  // ImGui::DestroyContext(*context_);
}

IGL_INLINE bool ImGuiPlugin::pre_draw()
{
  glfwPollEvents();

  // Check whether window dpi has changed
  float scaling = hidpi_scaling();
  if (std::abs(scaling - hidpi_scaling_) > 1e-5)
  {
    reload_font();
    ImGui_ImplOpenGL3_DestroyDeviceObjects();
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  return false;
}

IGL_INLINE bool ImGuiPlugin::post_draw()
{
  for(auto & widget : widgets){ widget->draw(); }
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  return false;
}

IGL_INLINE void ImGuiPlugin::post_resize(int width, int height)
{
  if (context_)
  {
    ImGui::GetIO().DisplaySize.x = float(width);
    ImGui::GetIO().DisplaySize.y = float(height);
  }
}

// Mouse IO
IGL_INLINE bool ImGuiPlugin::mouse_down(int button, int modifier)
{
  ImGui_ImplGlfw_MouseButtonCallback(viewer->window, button, GLFW_PRESS, modifier);
  if(ImGui::GetIO().WantCaptureMouse){ return true; }
  for( auto & widget : widgets)
  { 
    if(widget->mouse_down(button, modifier)) { return true; }
  }
  return false;
}

IGL_INLINE bool ImGuiPlugin::mouse_up(int button, int modifier)
{
  //return ImGui::GetIO().WantCaptureMouse;
  // !! Should not steal mouse up
  for( auto & widget : widgets)
  { 
    widget->mouse_up(button, modifier);
  }
  return false;
}

IGL_INLINE bool ImGuiPlugin::mouse_move(int mouse_x, int mouse_y)
{
  if(ImGui::GetIO().WantCaptureMouse){ return true; }
  for( auto & widget : widgets)
  { 
    if(widget->mouse_move(mouse_x, mouse_y)) { return true; }
  }
  return false;
}

IGL_INLINE bool ImGuiPlugin::mouse_scroll(float delta_y)
{
  ImGui_ImplGlfw_ScrollCallback(viewer->window, 0.f, delta_y);
  return ImGui::GetIO().WantCaptureMouse;
}

// Keyboard IO
IGL_INLINE bool ImGuiPlugin::key_pressed(unsigned int key, int modifiers)
{
  ImGui_ImplGlfw_CharCallback(nullptr, key);
  if(ImGui::GetIO().WantCaptureKeyboard) { return true; }
  for(auto & widget : widgets)
  { 
    if(widget->key_pressed(key,modifiers)) {return true; }
  }
  return false;
}

IGL_INLINE bool ImGuiPlugin::key_down(int key, int modifiers)
{
  ImGui_ImplGlfw_KeyCallback(viewer->window, key, 0, GLFW_PRESS, modifiers);
  if(ImGui::GetIO().WantCaptureKeyboard) { return true; }
  for(auto & widget : widgets)
  { 
    if(widget->key_down(key,modifiers)) {return true; }
  }
  return false;
}

IGL_INLINE bool ImGuiPlugin::key_up(int key, int modifiers)
{
  ImGui_ImplGlfw_KeyCallback(viewer->window, key, 0, GLFW_RELEASE, modifiers);
  if(ImGui::GetIO().WantCaptureKeyboard) { return true; }
  for(auto & widget : widgets)
  { 
    if(widget->key_up(key,modifiers)) { return true; }
  }
  return false;
}

IGL_INLINE float ImGuiPlugin::pixel_ratio()
{
  // Computes pixel ratio for hidpi devices
  int buf_size[2];
  int win_size[2];
  GLFWwindow* window = glfwGetCurrentContext();
  glfwGetFramebufferSize(window, &buf_size[0], &buf_size[1]);
  glfwGetWindowSize(window, &win_size[0], &win_size[1]);
  return (float) buf_size[0] / (float) win_size[0];
}

IGL_INLINE float ImGuiPlugin::hidpi_scaling()
{
  // Computes scaling factor for hidpi devices
  float xscale, yscale;
  GLFWwindow* window = glfwGetCurrentContext();
  glfwGetWindowContentScale(window, &xscale, &yscale);
  return 0.5 * (xscale + yscale);
}

} // end namespace
} // end namespace
} // end namespace
} // end namespace

