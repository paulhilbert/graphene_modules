###############################################################################
# Find Magellan
#
# This sets the following variables:
# MAGELLAN_FOUND - True if Magellan was found.
# MAGELLAN_INCLUDE_DIRS - Directories containing the Magellan include files.
# MAGELLAN_LIBRARY_DIRS - Directories containing the Magellan library.
# MAGELLAN_LIBRARIES - Magellan library files.

if (WIN32)
    find_path(MAGELLAN_INCLUDE_DIR magellan
    PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATH)
else()
    find_path(MAGELLAN_INCLUDE_DIR magellan
    PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATH)
endif()

if (WIN32)
    find_library(MAGELLAN_LIBRARY_PATH magellan PATHS "/usr/x86_64-w64-mingw32/bin" NO_DEFAULT_PATH)
else()
    find_library(MAGELLAN_LIBRARY_PATH magellan PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATH)
endif()

if(EXISTS ${MAGELLAN_LIBRARY_PATH})
    get_filename_component(MAGELLAN_LIBRARY ${MAGELLAN_LIBRARY_PATH} NAME)
    if(WIN32)
        find_path(MAGELLAN_LIBRARY_DIR ${MAGELLAN_LIBRARY} HINTS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib/")
    else()
        find_path(MAGELLAN_LIBRARY_DIR ${MAGELLAN_LIBRARY} HINTS "/usr/lib" "/usr/local/lib")
    endif()
endif()

set(MAGELLAN_INCLUDE_DIRS ${MAGELLAN_INCLUDE_DIR})
set(MAGELLAN_LIBRARY_DIRS ${MAGELLAN_LIBRARY_DIR})
set(MAGELLAN_LIBRARIES ${MAGELLAN_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Magellan DEFAULT_MSG MAGELLAN_INCLUDE_DIR MAGELLAN_LIBRARY MAGELLAN_LIBRARY_DIR)

mark_as_advanced(MAGELLAN_INCLUDE_DIR)
mark_as_advanced(MAGELLAN_LIBRARY_DIR)
mark_as_advanced(MAGELLAN_LIBRARY)
mark_as_advanced(MAGELLAN_LIBRARY_PATH)
