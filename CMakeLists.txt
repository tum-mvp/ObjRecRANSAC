project(ObjRecRANSACRun)
cmake_minimum_required(VERSION 2.6)

IF(OBJREC_USE_CUDA)
  add_definitions(-DUSE_CUDA)
  add_definitions(-DCUDA_DEVICE_ID=1 )
ENDIF(OBJREC_USE_CUDA)

find_package(VTK)

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)

find_package(OpenCV)

IF(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found. OpenCV is not installed or cmake can not find it. Install OpenCV first and then try again.
If OpenCV is installed but cmake can not find it, set the OPENCV_DIR entry to the folder which contains the file OpenCVConfig.cmake")
ENDIF(NOT OpenCV_FOUND)

add_subdirectory(src/BasicToolsL1)
add_subdirectory(src/BasicTools)
add_subdirectory(src/ObjRecRANSAC)
add_subdirectory(src/VtkBasics)

include_directories(
	${VTK_INCLUDE_DIRS}
	/usr/local/cuda/include
	${OpenCV_INCLUDE_DIRS}
	./src
)

link_directories(
	${VTK_LIBRARY_DIRS}
	/usr/local/cuda/lib
	${OpenCV_LIB_DIR}
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

SET(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}../FindCuda" ${CMAKE_MODULE_PATH})
FIND_PACKAGE(CUDA)

link_libraries(ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics vtkIO vtkCommon vtkRendering ${OpenCV_LIBS} rt)

if(OBJREC_USE_CUDA)
  CUDA_ADD_EXECUTABLE(ObjRecRANSACRun
    src/ObjRecRANSAC/AcceptHypothesisAlgo.cu
    main.cpp
    )
else()
  add_executable(ObjRecRANSACRun main.cpp)
endif()

install(TARGETS ObjRecRANSACRun
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
