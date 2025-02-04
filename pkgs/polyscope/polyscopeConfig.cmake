get_filename_component(polyscope_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)


find_library(polyscope_LIBRARY NAMES polyscope PATHS "${opennurbs_ROOT_DIR}/lib")
find_path(polyscope_INCLUDE_DIR NAMES opennurbs.h PATHS "${opennurbs_ROOT_DIR}/include/polyscope")

message(STATUS "Found opennurbs: ${polyscope_ROOT_DIR}")


list(APPEND polyscope_INCLUDE_DIR 
    "${opennurbs_ROOT_DIR}/include/polyscope"
    "${opennurbs_ROOT_DIR}/include/polyscope/render"
    "${opennurbs_ROOT_DIR}/include/polyscope/render/mock_opengl"
    "${opennurbs_ROOT_DIR}/include/polyscope/render/opengl"
    "${opennurbs_ROOT_DIR}/include/polyscope/render/opengl/shaders"
)


if (polyscope_LIBRARY AND polyscope_INCLUDE_DIR)
    add_library(polyscope STATIC IMPORTED)
    set_target_properties(polyscope PROPERTIES
        IMPORTED_LOCATION "${opennurbsStatic_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${opennurbsStatic_INCLUDE_DIR}"
    )
endif()

if("${POLYSCOPE_BACKEND_OPENGL3_GLFW}")

    ## Glad
    if(NOT TARGET glad)
        add_subdirectory(glad)
    endif()

    ## GLFW
    if(NOT TARGET glfw)
        set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
        add_subdirectory(glfw)
    endif()
endif()

## glm
if(NOT TARGET glm::glm)
    # add_subdirectory(glm)
    add_library(glm INTERFACE)
    target_compile_definitions(glm INTERFACE GLM_ENABLE_EXPERIMENTAL)
    set_target_properties(glm PROPERTIES LINKER_LANGUAGE CXX)
endif()

## Imgui
if(NOT TARGET imgui)
    # add_subdirectory(imgui)
    add_library(imgui INTERFACE)
endif()

## Json
if(NOT TARGET nlohmann_json::nlohmann_json)
    # add_subdirectory(json)
    add_library(nlohmann_json INTERFACE)
endif()

## MarchingCube
if(NOT TARGET MarchingCube::MarchingCube)
    # add_subdirectory(MarchingCubeCpp)
    add_library(MarchingCube INTERFACE)
endif()

## stb
if(NOT TARGET stb)
    # add_subdirectory(stb)
    add_library(stb INTERFACE)
endif()

