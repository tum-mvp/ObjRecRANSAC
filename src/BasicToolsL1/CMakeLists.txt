project(BasicToolsL1)
cmake_minimum_required(VERSION 2.6)

FILE(GLOB _BTL1_CUR_     *.cpp)
FILE(GLOB _BTL1_EIG3x3_  eigen3x3/*.cpp)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

add_library(BasicToolsL1 SHARED
	@_BTL1_CUR_@
	@_BTL1_EIG3x3_@
)
add_library(BasicToolsL1Static STATIC
	@_BTL1_CUR_@
	@_BTL1_EIG3x3_@
)
set_target_properties(BasicToolsL1Static PROPERTIES OUTPUT_NAME BasicToolsL1)

install(TARGETS BasicToolsL1 BasicToolsL1Static
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)
