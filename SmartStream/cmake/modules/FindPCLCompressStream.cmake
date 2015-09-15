###############################################################################
# Find PCLCompressStream
#
# This sets the following variables:
# PCLCOMPRESSSTREAM_FOUND - True if PCLCompressStream was found.
# PCLCOMPRESSSTREAM_INCLUDE_DIRS - Directories containing the PCLCompressStream include files.
# PCLCOMPRESSSTREAM_LIBRARY_DIRS - Directories containing the PCLCompressStream library.
# PCLCOMPRESSSTREAM_LIBRARIES - PCLCompressStream library files.

if(WIN32)
    find_path(PCLCOMPRESSSTREAM_INCLUDE_DIR pcl_compress_stream PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(PCLCOMPRESSSTREAM_LIBRARY_PATH pcl_compress_stream PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${PCLCOMPRESSSTREAM_LIBRARY_PATH})
        get_filename_component(PCLCOMPRESSSTREAM_LIBRARY ${PCLCOMPRESSSTREAM_LIBRARY_PATH} NAME)
        find_path(PCLCOMPRESSSTREAM_LIBRARY_DIR ${PCLCOMPRESSSTREAM_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(PCLCOMPRESSSTREAM_INCLUDE_DIR pcl_compress_stream PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(PCLCOMPRESSSTREAM_LIBRARY_PATH pcl_compress_stream PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${PCLCOMPRESSSTREAM_LIBRARY_PATH})
        get_filename_component(PCLCOMPRESSSTREAM_LIBRARY ${PCLCOMPRESSSTREAM_LIBRARY_PATH} NAME)
        find_path(PCLCOMPRESSSTREAM_LIBRARY_DIR ${PCLCOMPRESSSTREAM_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(PCLCOMPRESSSTREAM_INCLUDE_DIRS ${PCLCOMPRESSSTREAM_INCLUDE_DIR})
set(PCLCOMPRESSSTREAM_LIBRARY_DIRS ${PCLCOMPRESSSTREAM_LIBRARY_DIR})
set(PCLCOMPRESSSTREAM_LIBRARIES ${PCLCOMPRESSSTREAM_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCLCompressStream DEFAULT_MSG PCLCOMPRESSSTREAM_INCLUDE_DIR PCLCOMPRESSSTREAM_LIBRARY PCLCOMPRESSSTREAM_LIBRARY_DIR)

mark_as_advanced(PCLCOMPRESSSTREAM_INCLUDE_DIR)
mark_as_advanced(PCLCOMPRESSSTREAM_LIBRARY_DIR)
mark_as_advanced(PCLCOMPRESSSTREAM_LIBRARY)
mark_as_advanced(PCLCOMPRESSSTREAM_LIBRARY_PATH)
