
cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE "Release")
project(ObjRecRANSACRun)

IF(OBJREC_USE_CUDA)
  set(OBJREC_CUDA_DEVICE_ID 1 CACHE STRING "Which GPU device to use when running")
  add_definitions(-DUSE_CUDA)
  add_definitions(-DCUDA_DEVICE_ID=${OBJREC_CUDA_DEVICE_ID})

  set(CUDA_TOOLKIT_ROOT_DIR $ENV{CUDA_TOOLKIT_ROOT_DIR})
  FIND_PACKAGE(CUDA REQUIRED)

  add_definitions(-DOBJ_REC_RANSAC_PROFILE)

  #add_definitions(-DOBJ_REC_RANSAC_VERBOSE)
  #add_definitions(-DOBJ_REC_RANSAC_VERBOSE_1)
  #add_definitions(-DOBJ_REC_RANSAC_VERBOSE_2)
ENDIF(OBJREC_USE_CUDA)

find_package(Boost COMPONENTS thread system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
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

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

IF(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message (STATUS "Found OpenMP")
ENDIF(OPENMP_FOUND)

add_subdirectory(src/BasicToolsL1)
add_subdirectory(src/BasicTools)
add_subdirectory(src/ObjRecRANSAC)
add_subdirectory(src/VtkBasics)
#add_subdirectory(src/Greedy)

include_directories(
	${VTK_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
	/usr/local/cuda/include
	${OpenCV_INCLUDE_DIRS}
	./src
)

link_directories(
	${VTK_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
	/usr/local/cuda/lib
	${OpenCV_LIB_DIR}
)

if(OBJREC_USE_CUDA)
  CUDA_ADD_EXECUTABLE(ObjRecRANSACRun main.cpp)
else()
  add_executable(ObjRecRANSACRun main.cpp)
endif()

target_link_libraries(ObjRecRANSACRun ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics vtkIO vtkCommon vtkRendering ${OpenCV_LIBS} rt)

#add_library(Utility utility.h utility.cpp) 
#target_link_libraries(Utility ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_library(Greedy src/Greedy/utility.h src/Greedy/utility.cpp src/Greedy/seg.h src/Greedy/seg.cpp) 
target_link_libraries(Greedy ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(SegRansac main_seg.cpp)
target_link_libraries(SegRansac ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics vtkIO vtkCommon vtkRendering Greedy ${OpenCV_LIBS} ${PCL_LIBRARIES} rt)

add_executable(poseTest main_test_pose.cpp)
target_link_libraries(poseTest ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics vtkIO vtkCommon vtkRendering Greedy ${OpenCV_LIBS} ${PCL_LIBRARIES} rt)

add_executable(segTest main_test_seg.cpp)
target_link_libraries(segTest ObjRecRANSAC BasicToolsL1 BasicTools VtkBasics vtkIO vtkCommon vtkRendering Greedy ${OpenCV_LIBS} ${PCL_LIBRARIES} rt)

install(TARGETS ObjRecRANSACRun
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY src/
  DESTINATION include
  PATTERN "*.cpp" EXCLUDE
  PATTERN "*.cu" EXCLUDE
  PATTERN "*.c" EXCLUDE
  )

install(DIRECTORY data
  DESTINATION share/ObjRecRANSAC)

install(FILES package.xml
  DESTINATION share/ObjRecRANSAC)
