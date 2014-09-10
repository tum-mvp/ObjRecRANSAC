project(BasicTools)
cmake_minimum_required(VERSION 2.6)

find_package(VTK)

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)


FILE(GLOB _AUX_         Aux/*.cpp)
FILE(GLOB _CG_ALGS_     ComputationalGeometry/Algorithms/*.cpp)
FILE(GLOB _CG_DS_       ComputationalGeometry/DataStructures/*.cpp)
FILE(GLOB _CG_DS_BSP_   ComputationalGeometry/DataStructures/BSPTree/*.cpp)
FILE(GLOB _CG_DS_KD_    ComputationalGeometry/DataStructures/KdTree/*.cpp)
FILE(GLOB _CG_DS_OCT_   ComputationalGeometry/DataStructures/Octree/*.cpp)
FILE(GLOB _ORR_DS_      DataStructures/*.cpp)
FILE(GLOB _GRAPH_       Graph/*.cpp)
FILE(GLOB _LA_          LinearAlgebra/*.cpp)
FILE(GLOB _STOCH_       Stochastics/*.cpp)
FILE(GLOB _MYVTK_       Vtk/*.cpp)

include_directories(
	${VTK_INCLUDE_DIRS}
	..
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

add_library(BasicTools SHARED
	@_AUX_@
	@_CG_ALGS_@
	@_CG_DS_@
	@_CG_DS_BSP_@
	@_CG_DS_KD_@
	@_CG_DS_OCT_@
  @_ORR_DS_@
	@_GRAPH_@
	@_LA_@
	@_STOCH_@
	@_MYVTK_@
)
target_link_libraries(BasicTools vtkRendering)

add_library(BasicToolsStatic STATIC
	@_AUX_@
	@_CG_ALGS_@
	@_CG_DS_@
	@_CG_DS_BSP_@
	@_CG_DS_KD_@
	@_CG_DS_OCT_@
  @_ORR_DS_@
	@_GRAPH_@
	@_LA_@
	@_STOCH_@
	@_MYVTK_@
)
set_target_properties(BasicToolsStatic PROPERTIES OUTPUT_NAME BasicTools)

install(TARGETS BasicTools BasicToolsStatic 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

