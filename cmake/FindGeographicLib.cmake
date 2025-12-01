# FindGeographicLib.cmake
# Finds the GeographicLib library using pkg-config
#
# This will define:
#   GeographicLib_FOUND - System has GeographicLib
#   GeographicLib_INCLUDE_DIRS - The GeographicLib include directories
#   GeographicLib_LIBRARIES - The libraries needed to use GeographicLib

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(PC_GeographicLib QUIET geographiclib)
endif()

find_path(GeographicLib_INCLUDE_DIR
    NAMES GeographicLib/Config.h
    HINTS ${PC_GeographicLib_INCLUDE_DIRS}
    PATHS /usr/include /usr/local/include
)

find_library(GeographicLib_LIBRARY
    NAMES Geographic
    HINTS ${PC_GeographicLib_LIBRARY_DIRS}
    PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GeographicLib
    REQUIRED_VARS GeographicLib_LIBRARY GeographicLib_INCLUDE_DIR
)

if(GeographicLib_FOUND)
    set(GeographicLib_LIBRARIES ${GeographicLib_LIBRARY})
    set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
    
    if(NOT TARGET GeographicLib::GeographicLib)
        add_library(GeographicLib::GeographicLib UNKNOWN IMPORTED)
        set_target_properties(GeographicLib::GeographicLib PROPERTIES
            IMPORTED_LOCATION "${GeographicLib_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${GeographicLib_INCLUDE_DIR}"
        )
    endif()
endif()

mark_as_advanced(GeographicLib_INCLUDE_DIR GeographicLib_LIBRARY)
