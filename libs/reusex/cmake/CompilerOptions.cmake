# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

# ===============================================
# Compiler and Linker Options
# ===============================================

# Sanitizer options
option(LIN_ENABLE_ASAN "if true, enables clang/MSVC address sanitizer" OFF)
option(LIN_ENABLE_MSAN "if true, enables clang/MSVC memory sanitizer" OFF)
option(LIN_ENABLE_UBSAN "if true, enables clang/MSVC undefined behaviour sanitizer" OFF)
option(LIN_ENABLE_TSAN "if true, enables clang/MSVC thread sanitizer" OFF)

# Validate sanitizer options
if (LIN_ENABLE_ASAN AND LIN_ENABLE_TSAN)
    message(FATAL_ERROR "Can only enable one of TSan or ASan at a time")
endif()
if (LIN_ENABLE_ASAN AND LIN_ENABLE_MSAN)
    message(FATAL_ERROR "Can only enable one of ASan or MSan at a time")
endif()

option(LIN_ENABLE_WERROR "if true, enables -Werror, /WX" OFF)

# -march=native was previously applied unconditionally to every Release C/C++
# TU. Inside the Nix dev shell — the only supported build environment — the
# compiler wrapper sets NIX_ENFORCE_NO_NATIVE and strips the flag again, so it
# never reached the compiler and instead emitted one "Skipping impure flag"
# warning per TU (93 of them). It is also at odds with the reproducibility
# requirement in docs/DIRECTION.md and STANDARDS §6, since it bakes the build
# host's ISA into the artifacts. Now opt-in, and off by default: enabling it
# changes codegen only outside Nix, because inside Nix it was already inert.
option(ENABLE_NATIVE_ARCH
       "if true, adds -march=native to Release C/C++ builds (non-portable, \
stripped anyway inside the Nix dev shell)" OFF)

# ===============================================
# Compiler and linker flags
# ===============================================

set(COMMON_COMPILER_FLAGS "")
set(COMMON_LINKER_FLAGS "")

# ===============================================
# Optimization flags for Release builds
# ===============================================

if (CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    if (MSVC)
        # MSVC optimization flags
        add_compile_options(/O2 /Ob2 /Oi /Ot /GL)
        add_link_options(/LTCG /OPT:REF /OPT:ICF)
    else()
        # GCC/Clang optimization flags (only for C/C++, not CUDA)
        # CUDA uses its own optimization flags via CMAKE_CUDA_FLAGS
        add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-O3>)
        add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-DNDEBUG>)
        add_compile_options($<$<COMPILE_LANGUAGE:C>:-O3>)
        add_compile_options($<$<COMPILE_LANGUAGE:C>:-DNDEBUG>)
        if (ENABLE_NATIVE_ARCH)
            add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-march=native>)
            add_compile_options($<$<COMPILE_LANGUAGE:C>:-march=native>)
            message(STATUS "ENABLE_NATIVE_ARCH: -march=native requested "
                           "(ignored inside the Nix dev shell)")
        endif()
        if (CMAKE_BUILD_TYPE STREQUAL "Release")
            # Strip symbols in pure Release mode
            add_link_options(-s)
        endif()
    endif()
    message(STATUS "Optimization flags enabled for ${CMAKE_BUILD_TYPE} build")
endif()

if (MSVC)
    list(APPEND COMMON_COMPILER_FLAGS /MP)
    list(APPEND COMMON_COMPILER_FLAGS /openmp)

    if (LIN_ENABLE_WERROR)
        list(APPEND COMMON_COMPILER_FLAGS /WX)
    endif()
else()
    list(APPEND COMMON_COMPILER_FLAGS -Wall -Wextra)
    
    # Add support for OpenMP
    list(APPEND COMMON_COMPILER_FLAGS -fopenmp)
    list(APPEND COMMON_LINKER_FLAGS -lgomp)

    if (LIN_ENABLE_WERROR)
        # This branch used to comment out -Werror and add blanket
        # -Wno-unused-variable -Wno-unused-function instead, so the option did
        # the opposite of its name: asking for "warnings are errors" quietly
        # suppressed two whole warning classes for everyone who set it. The
        # unused-* findings it was hiding are fixed (#318), so the flag can
        # mean what it says. Default is still OFF.
        list(APPEND COMMON_COMPILER_FLAGS -Werror)
    endif()

    # Sanitizer support
    if (LIN_ENABLE_ASAN OR LIN_ENABLE_TSAN OR LIN_ENABLE_MSAN OR LIN_ENABLE_UBSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fno-omit-frame-pointer -g)
        list(APPEND COMMON_LINKER_FLAGS -fno-omit-frame-pointer -g)
    endif()

    if (LIN_ENABLE_ASAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=address)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=address)
    endif()

    if (LIN_ENABLE_TSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=thread)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=thread)
    endif()

    if (LIN_ENABLE_MSAN)
        list(APPEND COMMON_COMPILER_FLAGS -fsanitize=memory)
        list(APPEND COMMON_LINKER_FLAGS -fsanitize=memory)
    endif()

    if (LIN_ENABLE_UBSAN)
        list(APPEND COMMON_COMPILER_FLAGS
            -fsanitize=undefined
            -fno-sanitize-recover=all
            -fno-sanitize=alignment,vptr
        )
        list(APPEND COMMON_LINKER_FLAGS
            -fsanitize=undefined
            -fno-sanitize-recover=all
            -fno-sanitize=alignment,vptr
        )
    endif()
endif()

# Export to parent scope (only if not at top level)
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(COMMON_COMPILER_FLAGS ${COMMON_COMPILER_FLAGS} PARENT_SCOPE)
    set(COMMON_LINKER_FLAGS ${COMMON_LINKER_FLAGS} PARENT_SCOPE)
endif()
