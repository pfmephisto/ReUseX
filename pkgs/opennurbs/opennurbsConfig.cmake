get_filename_component(opennurbs_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

message(STATUS "Found opennurbs: ${opennurbs_ROOT_DIR}")


find_library(opennurbsStatic_LIBRARY NAMES opennurbsStatic PATHS "${opennurbs_ROOT_DIR}/lib")
find_path(opennurbsStatic_INCLUDE_DIR NAMES opennurbs.h PATHS "${opennurbs_ROOT_DIR}/include/opennurbsStatic")

if (opennurbsStatic_LIBRARY AND opennurbsStatic_INCLUDE_DIR)
    add_library(opennurbsStatic STATIC IMPORTED)
    set_target_properties(opennurbsStatic PROPERTIES
        IMPORTED_LOCATION "${opennurbsStatic_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${opennurbsStatic_INCLUDE_DIR}"
    )
    set(OPENNURBS_STATIC_PUBLIC_INSTALL_DIR "${opennurbs_ROOT_DIR}/include/opennurbsStatic")
    # add_compile_definitions(OPENNURBS_PUBLIC_INSTALL_DIR="${opennurbs_ROOT_DIR}/include")
endif()

find_library(OpenNURBS_LIBRARY NAMES OpenNURBS PATHS "${opennurbs_ROOT_DIR}/lib")
find_path(OpenNURBS_INCLUDE_DIR NAMES opennurbs.h PATHS "${opennurbs_ROOT_DIR}/include/OpenNURBS")

if (OpenNURBS_LIBRARY AND OpenNURBS_INCLUDE_DIR)
    add_library(OpenNURBS SHARED IMPORTED)
    set_target_properties(OpenNURBS PROPERTIES
        IMPORTED_LOCATION "${OpenNURBS_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${OpenNURBS_INCLUDE_DIR}"
    )
    set(OPENNURBS_SHARED_PUBLIC_INSTALL_DIR "${opennurbs_ROOT_DIR}/include/OpenNURBS")
    # add_compile_definitions(OPENNURBS_PUBLIC_INSTALL_DIR="${opennurbs_ROOT_DIR}/include")
endif()

add_compile_definitions(OPENNURBS_PUBLIC_INSTALL_DIR="${opennurbs_ROOT_DIR}/include")


set(OPENNURBS_IMPORTS)
