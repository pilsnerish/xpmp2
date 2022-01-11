# XPMP2 - Set up to be used in the provided docker environment to build lin and mac
#         Set up to be used in a Visual Studio environment to build win (File > Open > Folder, then VS recognized the CMAKE configuration)

cmake_minimum_required(VERSION 3.16)

project(XPMP2
        VERSION 2.3
        DESCRIPTION "Multiplayer library for X-Plane 11")

################################################################################
# Target Systems
################################################################################

# Windows: Target Windows 7.0 and later
if (WIN32)
    add_compile_definitions(_WIN32_WINNT=0x0601 WIN32_LEAN_AND_MEAN)
    if (NOT DEFINED ENV{platform})
        set(ENV{platform} "win")
    endif()
elseif(APPLE)
    add_compile_options(-mmacosx-version-min=10.11)
    add_link_options(-mmacosx-version-min=10.11)
endif()

################################################################################
# C++ Standard required
################################################################################

set(CMAKE_CXX_STANDARD 17)
set_property(GLOBAL PROPERTY CXX_STANDARD_REQUIRED 17)
set_property(GLOBAL PROPERTY CXX_STANDARD 17)

################################################################################
# Compile Options
################################################################################

# Enable all X-Plane SDK APIs up to the newest version.
add_compile_definitions(XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM303=1)

# Define platform macros.
add_compile_definitions(APL=$<BOOL:${APPLE}> IBM=$<BOOL:${WIN32}> LIN=$<AND:$<BOOL:${UNIX}>,$<NOT:$<BOOL:${APPLE}>>>)

# Enable stricter warnings and then disable some we are not interested in.
# For XPMP2 compile, we don't need to be warned about our self-defined depreciations
if (MSVC)
    add_compile_options(/wd4996 /wd4068)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(-Wall -Wshadow -Wfloat-equal -Wextra -Wno-deprecated-declarations)
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 9.0 AND NOT APPLE)
        add_compile_options(-Wno-stringop-truncation)
    endif()

    # Force-enable exception support. This is most likely redundant, although for C
    # code the default is the opposite. Since we are mixing C++ and C libraries,
    # safer to set it on?
    add_compile_options(-fexceptions)

    # Makes symbols non-exported by default.
    add_compile_options(-fvisibility=hidden)
endif()


# Debug vs Release build
if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_definitions(DEBUG=1)
    if (MSVC)
        add_compile_options(/Zi)
    else()
        add_compile_options(-O0 -g)
    endif()
else()
    add_compile_definitions(NDEBUG=1)
    if(MSVC)
        # Use highest optimization level in Release builds
        add_compile_options(/GL)
    elseif(APPLE)
        add_compile_options(-O3 -fPIC)
    elseif (UNIX OR MINGW)
        # Use position-independent code and highest optimization level (FPS!).
        add_compile_options(-O3 -fPIC)
        # Strip symbols during linking
        add_link_options(-s)
    endif()
endif()

################################################################################
# XPMP2 Library
################################################################################

# Source list
add_library(XPMP2 STATIC
    inc/XPCAircraft.h
    inc/XPMPAircraft.h
    inc/XPMPMultiplayer.h
    inc/XPMPRemote.h
    inc/XPMPPlaneRenderer.h
    src/2D.h
    src/2D.cpp
    src/AIMultiplayer.h
    src/AIMultiplayer.cpp
    src/Aircraft.h
    src/Aircraft.cpp
    src/CSLCopy.cpp
    src/CSLModels.h
    src/CSLModels.cpp
    src/Map.h
    src/Map.cpp
    src/Network.h
    src/Network.cpp
    src/RelatedDoc8643.h
    src/RelatedDoc8643.cpp
    src/Remote.h
    src/Remote.cpp
    src/Utilities.h
    src/Utilities.cpp
    src/XPMP2.h
    src/XPMPMultiplayer.cpp
)

# Define pre-compiled header
target_precompile_headers(XPMP2 PUBLIC src/XPMP2.h)

# Header include directories
target_include_directories(XPMP2
	PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    PRIVATE
        ${ADDITIONAL_INCLUDES}
		${CMAKE_CURRENT_SOURCE_DIR}/XPMP2-Sample/SDK/CHeaders/XPLM
		${CMAKE_CURRENT_SOURCE_DIR}/src
)

if (WIN32)
    set_property(TARGET XPMP2 PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
endif ()