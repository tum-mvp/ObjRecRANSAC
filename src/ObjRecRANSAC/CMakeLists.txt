project(ObjRecRANSAC)
cmake_minimum_required(VERSION 2.6)

find_package(VTK)

IF(NOT VTK_FOUND)
  message(FATAL_ERROR "VTK not found. VTK is not installed or cmake can not find it. Install VTK first and then try again.
If VTK is installed but cmake can not find it, set the VTK_DIR entry to the folder which contains the file VTKConfig.cmake")
ENDIF(NOT VTK_FOUND)

FIND_PACKAGE(CUDA)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#To print profiling info:
if(OBJREC_PROFILE)
  add_definitions(-DOBJ_REC_RANSAC_PROFILE)
endif()

SET(_ORR_CUR_     ModelDatabase.cpp ObjRecRANSAC.cpp)
FILE(GLOB _ORR_ALGS_    Algorithms/*.cpp)
FILE(GLOB _ORR_DS_      DataStructures/*.cpp)
FILE(GLOB _ORR_DS_HTBS_ DataStructures/HashTableBoxStruct/*.cpp)
FILE(GLOB _ORR_DS_HTKT_ DataStructures/HashTableKdTree/*.cpp)
FILE(GLOB _ORR_DS_OKD_  DataStructures/ORRKdTree/*.cpp)
FILE(GLOB _ORR_DS_ORRO_ DataStructures/ORROctree/*.cpp)
FILE(GLOB _ORR_DS_RI_   DataStructures/RangeImage/*.cpp)
FILE(GLOB _ORR_SH_      Shapes/*.cpp)

IF(OBJREC_USE_CUDA)
  LIST(APPEND _ORR_CUR_ )
ENDIF(OBJREC_USE_CUDA)


include_directories(
	${VTK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	/usr/local/cuda/include
	..
)

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

IF(OBJREC_USE_CUDA)
  set(CUDA_NVCC_FLAGS "-Xptxas -v")

  #cuda_compile_ptx(
    #cuda_ptx_files
    #AcceptHypothesisAlgo.cu
    #)
  #add_custom_target(ptx ALL
    #DEPENDS ${cuda_ptx_files} AcceptHypothesisAlgo.cu
    #SOURCES AcceptHypothesisAlgo.cu)

  CUDA_ADD_LIBRARY(ObjRecRANSAC
    SHARED
    AcceptHypothesisAlgo.cpp
    AcceptHypothesisAlgo.cu
    @_ORR_CUR_@
    @_ORR_ALGS_@
    @_ORR_DS_@
    @_ORR_DS_HTBS_@
    @_ORR_DS_HTKT_@
    @_ORR_DS_OKD_@
    @_ORR_DS_ORRO_@
    @_ORR_DS_RI_@
    @_ORR_SH_@
  )
target_link_libraries(ObjRecRANSAC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} BasicTools BasicToolsL1)

  add_library(ObjRecRANSACStatic
    STATIC
    AcceptHypothesisAlgo.cpp
    AcceptHypothesisAlgo.cu
    @_ORR_CUR_@
    @_ORR_ALGS_@
    @_ORR_DS_@
    @_ORR_DS_HTBS_@
    @_ORR_DS_HTKT_@
    @_ORR_DS_OKD_@
    @_ORR_DS_ORRO_@
    @_ORR_DS_RI_@
    @_ORR_SH_@
  )
  set_target_properties(ObjRecRANSACStatic PROPERTIES OUTPUT_NAME ObjRecRANSAC)
else()
  add_library(ObjRecRANSAC SHARED
    @_ORR_CUR_@
    @_ORR_ALGS_@
    @_ORR_DS_@
    @_ORR_DS_HTBS_@
    @_ORR_DS_HTKT_@
    @_ORR_DS_OKD_@
    @_ORR_DS_ORRO_@
    @_ORR_DS_RI_@
    @_ORR_SH_@
  )
  target_link_libraries(ObjRecRANSAC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} BasicTools BasicToolsL1)

  add_library(ObjRecRANSACStatic STATIC
    @_ORR_CUR_@
    @_ORR_ALGS_@
    @_ORR_DS_@
    @_ORR_DS_HTBS_@
    @_ORR_DS_HTKT_@
    @_ORR_DS_OKD_@
    @_ORR_DS_ORRO_@
    @_ORR_DS_RI_@
    @_ORR_SH_@
  )
  set_target_properties(ObjRecRANSACStatic PROPERTIES OUTPUT_NAME ObjRecRANSAC)
endif()

install(TARGETS ObjRecRANSAC ObjRecRANSACStatic
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

