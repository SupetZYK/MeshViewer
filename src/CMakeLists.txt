cmake_minimum_required(VERSION 2.8)
project(MeshViewer)

aux_source_directory(. src_list)

if(WIN32 AND NOT MINGW)
      message("WIN32 编译设置")
	  link_directories("../glut_lib_for_win32")
endif()
add_executable(MeshViewer ${src_list})