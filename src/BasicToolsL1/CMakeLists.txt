project(BasicToolsL1)
cmake_minimum_required(VERSION 2.6)

FILE(GLOB _BTL1_CUR_     *.cpp)
FILE(GLOB _BTL1_EIG3x3_  eigen3x3/*.cpp)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wno-deprecated -mfpmath=sse -msse -msse2 -msse3 -mssse3 -msse4.1 -O3")
endif()

add_library(BasicToolsL1 STATIC
	@_BTL1_CUR_@
	@_BTL1_EIG3x3_@
)

