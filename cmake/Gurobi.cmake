find_package(GUROBI REQUIRED)
if (GUROBI_FOUND)
    message(NOTICE "Gurobi found")
    message(STATUS "GUROBI Version: ${GUROBI_VERSION}")
    message(STATUS "GUROBI_LIBRARY: ${GUROBI_LIBRARY}")
    message(STATUS "GUROBI_INCLUDE_DIRS: ${GUROBI_INCLUDE_DIRS}")
    
    add_compile_definitions(CGAL_USE_GUROBI=1)
    include_directories(${GUROBI_INCLUDE_DIRS})
else()
    message(WARNING "Gurobi not found")
endif()