project(VtkBasics)
cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0053 OLD)
find_package(VTK)

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)

FILE(GLOB _VTK_         *.cpp)

include_directories(
	${VTK_INCLUDE_DIRS}
	..
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

add_library(VtkBasics SHARED
	@_VTK_@
)
add_library(VtkBasicsStatic STATIC
  @_VTK_@
)

target_link_libraries(VtkBasics  ${VTK_LIBRARIES})
target_link_libraries(VtkBasicsStatic  ${VTK_LIBRARIES})

find_package(PythonLibs)
if(PYTHONLIBS_FOUND)
    target_link_libraries(VtkBasics  ${PYTHON_LIBRARIES})
    target_link_libraries(VtkBasicsStatic  ${PYTHON_LIBRARIES})
endif()

set_target_properties(VtkBasicsStatic PROPERTIES OUTPUT_NAME VtkBasics)

install(TARGETS VtkBasics VtkBasicsStatic 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
