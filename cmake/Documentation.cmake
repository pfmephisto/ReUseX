# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Documentation with Doxygen
# ===============================================

option(BUILD_DOCUMENTATION "Build API documentation with Doxygen" ON)

if(BUILD_DOCUMENTATION)
    find_package(Doxygen OPTIONAL_COMPONENTS dot)
    
    if(DOXYGEN_FOUND)
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
        
        # Configure the Doxyfile to use build directory
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        
        message(STATUS "Doxygen found - documentation can be built with 'make doc' or 'cmake --build . --target doc'")
        
        # Add custom target for building documentation
        add_custom_target(doc
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )
        
        # Optionally install documentation
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
            install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/doc/html
                    DESTINATION ${CMAKE_INSTALL_DOCDIR}
                    OPTIONAL)
        endif()
    else()
        message(STATUS "Doxygen not found - documentation will not be built")
    endif()
else()
    message(STATUS "Documentation build disabled")
endif()
