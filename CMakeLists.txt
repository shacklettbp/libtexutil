cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
cmake_policy(VERSION 3.10)

set(NAME texcompress)

IF (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
        "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
        "MinSizeRel" "RelWithDebInfo")
ENDIF()

project(${NAME} LANGUAGES C CXX)

string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)

add_compile_options(-Wall -Wextra -Wshadow)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

add_library(basisu STATIC
    basisu/encoder/apg_bmp.c
    basisu/encoder/apg_bmp.h
    basisu/encoder/basisu_astc_decomp.cpp
    basisu/encoder/basisu_astc_decomp.h
    basisu/encoder/basisu_backend.cpp
    basisu/encoder/basisu_backend.h
    basisu/encoder/basisu_basis_file.cpp
    basisu/encoder/basisu_basis_file.h
    basisu/encoder/basisu_bc7enc.cpp
    basisu/encoder/basisu_bc7enc.h
    basisu/encoder/basisu_enc.cpp
    basisu/encoder/basisu_enc.h
    basisu/encoder/basisu_etc.cpp
    basisu/encoder/basisu_etc.h
    basisu/encoder/basisu_frontend.cpp
    basisu/encoder/basisu_frontend.h
    basisu/encoder/basisu_global_selector_palette_helpers.cpp
    basisu/encoder/basisu_global_selector_palette_helpers.h
    basisu/encoder/basisu_gpu_texture.cpp
    basisu/encoder/basisu_gpu_texture.h
    basisu/encoder/basisu_kernels_declares.h
    basisu/encoder/basisu_kernels_imp.h
    basisu/encoder/basisu_kernels_sse.cpp
    basisu/encoder/basisu_miniz.h
    basisu/encoder/basisu_pvrtc1_4.cpp
    basisu/encoder/basisu_pvrtc1_4.h
    basisu/encoder/basisu_resample_filters.cpp
    basisu/encoder/basisu_resampler_filters.h
    basisu/encoder/basisu_resampler.cpp
    basisu/encoder/basisu_resampler.h
    basisu/encoder/basisu_ssim.cpp
    basisu/encoder/basisu_ssim.h
    basisu/encoder/basisu_uastc_enc.cpp
    basisu/encoder/basisu_uastc_enc.h
    basisu/encoder/cppspmd_flow.h
    basisu/encoder/cppspmd_math.h
    basisu/encoder/cppspmd_math_declares.h
    basisu/encoder/cppspmd_sse.h
    basisu/encoder/cppspmd_type_aliases.h
    basisu/encoder/jpgd.cpp
    basisu/encoder/jpgd.h
    basisu/encoder/lodepng.cpp
    basisu/encoder/lodepng.h)
target_compile_options(basisu PRIVATE -no-pie)

target_include_directories(basisu PUBLIC SYSTEM
    basisu)

add_subdirectory(bc7enc_rdo)

add_library(texutil
    texutil.cpp texutil.hpp)

target_include_directories(texutil
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(texutil PRIVATE
    bc7enc basisu OpenMP::OpenMP_CXX OpenImageIO)
target_compile_definitions(texutil PRIVATE SUPPORT_BC7E)
target_compile_options(texutil PRIVATE -no-pie)
target_link_options(texutil PRIVATE -no-pie)
