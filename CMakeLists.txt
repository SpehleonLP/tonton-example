cmake_minimum_required(VERSION 3.15)
project(tonton-example VERSION 1.0.0 LANGUAGES CXX C)

# Set C++ and C standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find GLM
find_package(glm QUIET)
if(NOT glm_FOUND)
    message(STATUS "GLM not found via find_package, assuming it's in include path")
endif()

# ============================================================================
# Add submodules
# ============================================================================

# fx-gltf (MIT) - GLTF loading library
set(FX_GLTF_BUILD_TESTS OFF CACHE BOOL "Disable fx-gltf tests" FORCE)
set(FX_GLTF_INSTALL OFF CACHE BOOL "Disable fx-gltf install" FORCE)
add_subdirectory(modules/fx-gltf)

# rintintin (GPL-2.0) - 3D volumetric analysis
add_subdirectory(modules/rintintin)

# tonton (Apache-2.0) - Biomechanical analysis (includes dodeedum submodule)
add_subdirectory(modules/tonton)

# ============================================================================
# Create symlink for old "thirdparty" path references
# This allows source files to use #include "../thirdparty/..." paths
# ============================================================================
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/thirdparty")
    if(UNIX)
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E create_symlink
                "${CMAKE_SOURCE_DIR}/modules"
                "${CMAKE_SOURCE_DIR}/thirdparty"
        )
        message(STATUS "Created symlink: thirdparty -> modules")
    else()
        message(WARNING "Cannot create symlink on non-Unix platform. You may need to update include paths.")
    endif()
endif()

# ============================================================================
# Gather source files for bridge libraries
# ============================================================================

# Common source files used by both executables
set(COMMON_SOURCES
    src/primitives.cpp
    src/lf_rintintin.cpp
)

# rintintin executable specific sources
set(RINTINTIN_SOURCES
    src/rintintin_main.cpp
    src/gltf_rintintin_bridge.cpp
    src/visualize_tensors.cpp
    ${COMMON_SOURCES}
)

# tonton executable specific sources
set(TONTON_SOURCES
    src/tonton_main.cpp
    src/get_armatures_from_files.cpp
    src/tonton_fxgltf_memo.cpp
    src/gltf_rintintin_bridge.cpp
    ${COMMON_SOURCES}
)

# ============================================================================
# rintintin executable
# ============================================================================
add_executable(rintintin ${RINTINTIN_SOURCES})

target_include_directories(rintintin PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/modules/fx-gltf/include
    ${CMAKE_SOURCE_DIR}/modules/rintintin/include
)

target_link_libraries(rintintin PRIVATE
    rintintin-lib
    fx-gltf
)

if(glm_FOUND)
    target_link_libraries(rintintin PRIVATE glm::glm)
endif()

# Add threading support
find_package(Threads REQUIRED)
target_link_libraries(rintintin PRIVATE Threads::Threads)

# ============================================================================
# tonton executable
# ============================================================================
add_executable(tonton ${TONTON_SOURCES})

target_include_directories(tonton PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/modules/fx-gltf/include
    ${CMAKE_SOURCE_DIR}/modules/tonton/include
    ${CMAKE_SOURCE_DIR}/modules/rintintin/include
)

target_link_libraries(tonton PRIVATE
    tonton
    fx-gltf
    rintintin-lib
)

if(glm_FOUND)
    target_link_libraries(tonton PRIVATE glm::glm)
endif()

# Add threading support
target_link_libraries(tonton PRIVATE Threads::Threads)

# ============================================================================
# Installation (optional)
# ============================================================================
install(TARGETS rintintin tonton
    RUNTIME DESTINATION bin
)

# ============================================================================
# Print build configuration
# ============================================================================
message(STATUS "")
message(STATUS "==================== TonTon Example Build Configuration ====================")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "")
message(STATUS "Submodules:")
message(STATUS "  - fx-gltf (MIT)")
message(STATUS "  - rintintin (GPL-2.0)")
message(STATUS "  - tonton (Apache-2.0)")
message(STATUS "    - dodeedum (MIT)")
message(STATUS "")
message(STATUS "Executables to build:")
message(STATUS "  - rintintin: Volumetric analysis + debug visualization")
message(STATUS "  - tonton: Biomechanical creature analysis")
message(STATUS "")
message(STATUS "NOTE: This project is GPL-2.0 licensed due to rintintin dependency")
message(STATUS "=============================================================================")
message(STATUS "")
