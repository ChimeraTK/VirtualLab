#
# cmake module for finding mtca4uVirtualLab
#
# returns:
#   mtca4uVirtualLab_FOUND        : true or false, depending on whether
#                                   the package was found
#   mtca4uVirtualLab_VERSION      : the package version
#   mtca4uVirtualLab_INCLUDE_DIRS : path to the include directory
#   mtca4uVirtualLab_LIBRARY_DIRS : path to the library directory
#   mtca4uVirtualLab_LIBRARY      : the provided libraries
#
# @author Martin Hierholzer, DESY
#

SET(mtca4uVirtualLab_FOUND 0)

#FIXME: the search path for the device config has to be extended/generalised/improved
FIND_PATH(mtca4uVirtualLab_DIR
    mtca4uVirtualLabConfig.cmake
    ${CMAKE_CURRENT_LIST_DIR}
    )

#Once we have found the config our job is done. Just load the config which provides the required 
#varaibles.
include(${mtca4uVirtualLab_DIR}/mtca4uVirtualLabConfig.cmake)

#use a macro provided by CMake to check if all the listed arguments are valid
#and set mtca4uVirtualLab_FOUND accordingly
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(mtca4uVirtualLab 
        REQUIRED_VARS mtca4uVirtualLab_LIBRARIES mtca4uVirtualLab_INCLUDE_DIRS
	VERSION_VAR mtca4uVirtualLab_VERSION )

