###############################################################################
# Find Graphene
#
# This sets the following variables:
# GRAPHENE_FOUND - True if Graphene was found.
# GRAPHENE_INCLUDE_DIRS - Directories containing the Graphene include files.
# GRAPHENE_LIBRARY_DIRS - Directories containing the Graphene library.
# GRAPHENE_LIBRARIES - Graphene library files.

find_path(GRAPHENE_INCLUDE_DIR graphene
    PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATH)

if (WIN32)
    find_library(GRAPHENE_LIBRARY_PATH Graphene HINTS "/usr/x86_64-w64-mingw32/lib")
else()
    find_library(GRAPHENE_LIBRARY_PATH Graphene HINTS "/usr/lib" "/usr/local/lib")
endif()

if(EXISTS ${GRAPHENE_LIBRARY_PATH})
get_filename_component(GRAPHENE_LIBRARY ${GRAPHENE_LIBRARY_PATH} NAME)
find_path(GRAPHENE_LIBRARY_DIR ${GRAPHENE_LIBRARY} HINTS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib/")
endif()

set(GRAPHENE_INCLUDE_DIRS ${GRAPHENE_INCLUDE_DIR} ${GRAPHENE_INCLUDE_DIR}/graphene)
set(GRAPHENE_LIBRARY_DIRS ${GRAPHENE_LIBRARY_DIR})
set(GRAPHENE_LIBRARIES ${GRAPHENE_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Graphene DEFAULT_MSG GRAPHENE_INCLUDE_DIR GRAPHENE_LIBRARY GRAPHENE_LIBRARY_DIR)

mark_as_advanced(GRAPHENE_INCLUDE_DIR)
mark_as_advanced(GRAPHENE_LIBRARY_DIR)
mark_as_advanced(GRAPHENE_LIBRARY)
mark_as_advanced(GRAPHENE_LIBRARY_PATH)
