###############################################################################
# Find Harmont
#
# This sets the following variables:
# HARMONT_FOUND - True if Harmont was found.
# HARMONT_INCLUDE_DIRS - Directories containing the Harmont include files.
# HARMONT_LIBRARY_DIRS - Directories containing the Harmont library.
# HARMONT_LIBRARIES - Harmont library files.

find_path(HARMONT_INCLUDE_DIR harmont
    PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATH)

find_library(HARMONT_LIBRARY_PATH harmont HINTS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib/")

if(EXISTS ${HARMONT_LIBRARY_PATH})
get_filename_component(HARMONT_LIBRARY ${HARMONT_LIBRARY_PATH} NAME)
find_path(HARMONT_LIBRARY_DIR ${HARMONT_LIBRARY} HINTS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib/")
endif()

set(HARMONT_INCLUDE_DIRS ${HARMONT_INCLUDE_DIR})
set(HARMONT_LIBRARY_DIRS ${HARMONT_LIBRARY_DIR})
set(HARMONT_LIBRARIES ${HARMONT_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Harmont DEFAULT_MSG HARMONT_INCLUDE_DIR HARMONT_LIBRARY HARMONT_LIBRARY_DIR)

mark_as_advanced(HARMONT_INCLUDE_DIR)
mark_as_advanced(HARMONT_LIBRARY_DIR)
mark_as_advanced(HARMONT_LIBRARY)
mark_as_advanced(HARMONT_LIBRARY_PATH)
