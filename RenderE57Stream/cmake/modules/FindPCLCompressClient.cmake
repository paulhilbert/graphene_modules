###############################################################################
# Find PCLCompressClient
#
# This sets the following variables:
# PCLCOMPRESSCLIENT_FOUND - True if PCLCompressClient was found.
# PCLCOMPRESSCLIENT_INCLUDE_DIRS - Directories containing the PCLCompressClient include files.
# PCLCOMPRESSCLIENT_LIBRARY_DIRS - Directories containing the PCLCompressClient library.
# PCLCOMPRESSCLIENT_LIBRARIES - PCLCompressClient library files.

if(WIN32)
    find_path(PCLCOMPRESSCLIENT_INCLUDE_DIR pcl_compress_client PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(PCLCOMPRESSCLIENT_LIBRARY_PATH pcl_compress_client PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${PCLCOMPRESSCLIENT_LIBRARY_PATH})
        get_filename_component(PCLCOMPRESSCLIENT_LIBRARY ${PCLCOMPRESSCLIENT_LIBRARY_PATH} NAME)
        find_path(PCLCOMPRESSCLIENT_LIBRARY_DIR ${PCLCOMPRESSCLIENT_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(PCLCOMPRESSCLIENT_INCLUDE_DIR pcl_compress_client PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(PCLCOMPRESSCLIENT_LIBRARY_PATH pcl_compress_client PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${PCLCOMPRESSCLIENT_LIBRARY_PATH})
        get_filename_component(PCLCOMPRESSCLIENT_LIBRARY ${PCLCOMPRESSCLIENT_LIBRARY_PATH} NAME)
        find_path(PCLCOMPRESSCLIENT_LIBRARY_DIR ${PCLCOMPRESSCLIENT_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(PCLCOMPRESSCLIENT_INCLUDE_DIRS ${PCLCOMPRESSCLIENT_INCLUDE_DIR})
set(PCLCOMPRESSCLIENT_LIBRARY_DIRS ${PCLCOMPRESSCLIENT_LIBRARY_DIR})
set(PCLCOMPRESSCLIENT_LIBRARIES ${PCLCOMPRESSCLIENT_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCLCompressClient DEFAULT_MSG PCLCOMPRESSCLIENT_INCLUDE_DIR PCLCOMPRESSCLIENT_LIBRARY PCLCOMPRESSCLIENT_LIBRARY_DIR)

mark_as_advanced(PCLCOMPRESSCLIENT_INCLUDE_DIR)
mark_as_advanced(PCLCOMPRESSCLIENT_LIBRARY_DIR)
mark_as_advanced(PCLCOMPRESSCLIENT_LIBRARY)
mark_as_advanced(PCLCOMPRESSCLIENT_LIBRARY_PATH)
