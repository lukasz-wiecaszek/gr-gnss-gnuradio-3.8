INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_GNSS gnss)

FIND_PATH(
    GNSS_INCLUDE_DIRS
    NAMES gnss/api.h
    HINTS $ENV{GNSS_DIR}/include
        ${PC_GNSS_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GNSS_LIBRARIES
    NAMES gnuradio-gnss
    HINTS $ENV{GNSS_DIR}/lib
        ${PC_GNSS_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnssTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GNSS DEFAULT_MSG GNSS_LIBRARIES GNSS_INCLUDE_DIRS)
MARK_AS_ADVANCED(GNSS_LIBRARIES GNSS_INCLUDE_DIRS)
