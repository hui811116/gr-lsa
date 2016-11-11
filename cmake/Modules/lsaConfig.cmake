INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_LSA lsa)

FIND_PATH(
    LSA_INCLUDE_DIRS
    NAMES lsa/api.h
    HINTS $ENV{LSA_DIR}/include
        ${PC_LSA_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    LSA_LIBRARIES
    NAMES gnuradio-lsa
    HINTS $ENV{LSA_DIR}/lib
        ${PC_LSA_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LSA DEFAULT_MSG LSA_LIBRARIES LSA_INCLUDE_DIRS)
MARK_AS_ADVANCED(LSA_LIBRARIES LSA_INCLUDE_DIRS)

