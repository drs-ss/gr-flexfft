INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_FLEXFFT flexfft)

FIND_PATH(
    FLEXFFT_INCLUDE_DIRS
    NAMES flexfft/api.h
    HINTS $ENV{FLEXFFT_DIR}/include
        ${PC_FLEXFFT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    FLEXFFT_LIBRARIES
    NAMES gnuradio-flexfft
    HINTS $ENV{FLEXFFT_DIR}/lib
        ${PC_FLEXFFT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(FLEXFFT DEFAULT_MSG FLEXFFT_LIBRARIES FLEXFFT_INCLUDE_DIRS)
MARK_AS_ADVANCED(FLEXFFT_LIBRARIES FLEXFFT_INCLUDE_DIRS)

