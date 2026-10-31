# FindEtherCAT.cmake - locate the IgH EtherCAT Master userspace library.
#
# Search order: EtherCAT_ROOT, pkg-config (libethercat), /opt/etherlab, /usr/local, /usr.
#
# Result variables:
#   EtherCAT_FOUND, EtherCAT_INCLUDE_DIR, EtherCAT_LIBRARY, EtherCAT_VERSION (if known)
# Imported target:
#   EtherCAT::EtherCAT

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_ETHERCAT QUIET libethercat)
endif()

find_path(EtherCAT_INCLUDE_DIR
    NAMES ecrt.h
    HINTS ${EtherCAT_ROOT} $ENV{EtherCAT_ROOT} ${PC_ETHERCAT_INCLUDE_DIRS}
    PATHS /opt/etherlab /usr/local /usr
    PATH_SUFFIXES include)

find_library(EtherCAT_LIBRARY
    NAMES ethercat
    HINTS ${EtherCAT_ROOT} $ENV{EtherCAT_ROOT} ${PC_ETHERCAT_LIBRARY_DIRS}
    PATHS /opt/etherlab /usr/local /usr
    PATH_SUFFIXES lib lib64 lib/x86_64-linux-gnu)

if(PC_ETHERCAT_VERSION)
    set(EtherCAT_VERSION ${PC_ETHERCAT_VERSION})
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EtherCAT
    REQUIRED_VARS EtherCAT_LIBRARY EtherCAT_INCLUDE_DIR
    VERSION_VAR EtherCAT_VERSION)

if(EtherCAT_FOUND AND NOT TARGET EtherCAT::EtherCAT)
    add_library(EtherCAT::EtherCAT UNKNOWN IMPORTED)
    set_target_properties(EtherCAT::EtherCAT PROPERTIES
        IMPORTED_LOCATION "${EtherCAT_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${EtherCAT_INCLUDE_DIR}")
endif()

mark_as_advanced(EtherCAT_INCLUDE_DIR EtherCAT_LIBRARY)
