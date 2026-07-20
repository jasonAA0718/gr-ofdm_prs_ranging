find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_OFDM_PRS_RANGING gnuradio-ofdm_prs_ranging)

FIND_PATH(
    GR_OFDM_PRS_RANGING_INCLUDE_DIRS
    NAMES gnuradio/ofdm_prs_ranging/api.h
    HINTS $ENV{OFDM_PRS_RANGING_DIR}/include
        ${PC_OFDM_PRS_RANGING_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_OFDM_PRS_RANGING_LIBRARIES
    NAMES gnuradio-ofdm_prs_ranging
    HINTS $ENV{OFDM_PRS_RANGING_DIR}/lib
        ${PC_OFDM_PRS_RANGING_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-ofdm_prs_rangingTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_OFDM_PRS_RANGING DEFAULT_MSG GR_OFDM_PRS_RANGING_LIBRARIES GR_OFDM_PRS_RANGING_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_OFDM_PRS_RANGING_LIBRARIES GR_OFDM_PRS_RANGING_INCLUDE_DIRS)
