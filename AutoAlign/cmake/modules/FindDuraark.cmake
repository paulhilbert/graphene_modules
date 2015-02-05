###############################################################################
# Find Duraark
#
# This sets the following variables:
# DURAARK_FOUND - True if Duraark was found.
# DURAARK_INCLUDE_DIRS - Directories containing the Duraark include files.
# DURAARK_LIBRARY_DIRS - Directories containing the Duraark library.
# DURAARK_LIBRARIES - Duraark library files.

find_path(DURAARK_INCLUDE_DIR duraark
    HINTS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}")

if (WIN32)
    find_library(DURAARK_LIBRARY_PATH duraark HINTS "/usr/x86_64-w64-mingw32/lib")
else()
    find_library(DURAARK_LIBRARY_PATH duraark HINTS "/usr/lib" "/usr/local/lib")
endif()

if(EXISTS ${DURAARK_LIBRARY_PATH})
get_filename_component(DURAARK_LIBRARY ${DURAARK_LIBRARY_PATH} NAME)
find_path(DURAARK_LIBRARY_DIR ${DURAARK_LIBRARY} HINTS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib/")
endif()

set(DURAARK_INCLUDE_DIRS ${DURAARK_INCLUDE_DIR})
set(DURAARK_LIBRARY_DIRS ${DURAARK_LIBRARY_DIR})
set(DURAARK_LIBRARIES ${DURAARK_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Duraark DEFAULT_MSG DURAARK_INCLUDE_DIR DURAARK_LIBRARY DURAARK_LIBRARY_DIR)

mark_as_advanced(DURAARK_INCLUDE_DIR)
mark_as_advanced(DURAARK_LIBRARY_DIR)
mark_as_advanced(DURAARK_LIBRARY)
mark_as_advanced(DURAARK_LIBRARY_PATH)
