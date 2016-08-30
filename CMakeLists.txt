PROJECT(mtca4uVirtualLab)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 03)
set(${PROJECT_NAME}_PATCH_VERSION 05)
set(${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION})
set(${PROJECT_NAME}_SOVERSION ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION})
#This library depends on deviceaccess, which is likely to change often.
#We add a build version to the version number to be able to install
#the same software version of the library linked against different versions
#of deviceacces (or other dependencies)
set(${PROJECT_NAME}_BUILDVERSION $ENV{PROJECT_BUILDVERSION}$ENV{${PROJECT_NAME}_BUILDVERSION})
if( ${PROJECT_NAME}_BUILDVERSION )
  set(${PROJECT_NAME}_SOVERSION "${${PROJECT_NAME}_SOVERSION}${${PROJECT_NAME}_BUILDVERSION}")
endif( ${PROJECT_NAME}_BUILDVERSION )
message("${PROJECT_NAME}_SOVERSION ${${PROJECT_NAME}_SOVERSION}")
#We need the build name to be part of the libraries full version name
set(${PROJECT_NAME}_FULL_LIBRARY_VERSION ${${PROJECT_NAME}_SOVERSION}.${${PROJECT_NAME}_PATCH_VERSION})

#put the min version of deviceaccess into a variable. We need it for the debian packaging scripts
set(mtca4u-deviceaccess_MIN_VERSION 00.12)
FIND_PACKAGE(mtca4u-deviceaccess ${mtca4u-deviceaccess_MIN_VERSION} REQUIRED)
include_directories(SYSTEM ${mtca4u-deviceaccess_INCLUDE_DIRS})

FIND_PACKAGE(Boost COMPONENTS system REQUIRED)

#For some reason the system library has to be mentioned explicitly to the linker.
#This is strange because the thead library has it as a dependency.
FIND_PACKAGE(Boost COMPONENTS system unit_test_framework)
IF(Boost_UNIT_TEST_FRAMEWORK_FOUND)
  set(TESTING_IS_ENABLED "true")
  ENABLE_TESTING()
ELSE(Boost_UNIT_TEST_FRAMEWORK_FOUND)
  message("Boost unit_test_framework not found, disabling testing")
ENDIF(Boost_UNIT_TEST_FRAMEWORK_FOUND)

include_directories(${CMAKE_SOURCE_DIR}/include)
set(${PROJECT_NAME}_INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/)
aux_source_directory(${CMAKE_SOURCE_DIR}/src library_sources)

# Set the build type to Release if none is specified
# Force it into Release if "None" is specified (needed to overrule dkpg_buildpackage)
if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "None")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "None")

#Moderate version of the compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fPIC -Wextra -ansi -Wuninitialized ${mtca4u-deviceaccess_CXX_FLAGS} -O3")

#use -DCMAKE_BUILD_TYPE=Debug in your cmake command to turn on the coverage option
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 --coverage -fno-inline -fno-inline-small-functions -fno-default-inline")

# linker flags
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${mtca4u-deviceaccess_LINKER_FLAGS} -Wl,--export-dynamic")
set(CMAKE_EXE_LINKER_FLAGS "${mtca4u-deviceaccess_LINKER_FLAGS}")

MACRO( COPY_MAPPING_FILES )
  foreach( FILE_TO_COPY test.mapp dummies.dmap )
    FILE (COPY tests/${FILE_TO_COPY} DESTINATION ${PROJECT_BINARY_DIR})
  endforeach( FILE_TO_COPY )
ENDMACRO( COPY_MAPPING_FILES )

#Create the executables for automated unit testing.
#Currently we assume that they are all tests contained in one file, so
#each file gives a new executable. This section has to be adapted if this should change.
if(TESTING_IS_ENABLED)
  COPY_MAPPING_FILES()

  aux_source_directory(${CMAKE_SOURCE_DIR}/tests/executables_src testExecutables)
  foreach( testExecutableSrcFile ${testExecutables})
    #NAME_WE means the base name without path and (longest) extension
    get_filename_component(excutableName ${testExecutableSrcFile} NAME_WE)
    add_executable(${excutableName} ${testExecutableSrcFile})
    target_link_libraries(${excutableName} ${PROJECT_NAME} ${mtca4u-deviceaccess_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
    add_test(${excutableName} ${excutableName})
  endforeach( testExecutableSrcFile )

  aux_source_directory(${CMAKE_SOURCE_DIR}/performance_tests perftestExecutables)
  foreach( perftestExecutableSrcFile ${perftestExecutables})
    #NAME_WE means the base name without path and (longest) extension
    get_filename_component(excutableName ${perftestExecutableSrcFile} NAME_WE)
    add_executable(${excutableName} ${perftestExecutableSrcFile})
    target_link_libraries(${excutableName} ${PROJECT_NAME} ${mtca4u-deviceaccess_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
  endforeach( perftestExecutableSrcFile )

  #The make coverage command is only available in debug mode
  IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
    configure_file(cmake/make_coverage.sh.in
      ${PROJECT_BINARY_DIR}/make_coverage.sh @ONLY)
    add_custom_target(coverage
      ./make_coverage.sh
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating test coverage documentation" VERBATIM
      )
  ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

endif(TESTING_IS_ENABLED)

# C++ library
add_library(${PROJECT_NAME} SHARED ${library_sources} )
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION})
target_link_libraries(${PROJECT_NAME} ${mtca4u-deviceaccess_LIBRARIES} ${Boost_LIBRARIES})
# do not remove runtime path of the library  when installing
set_property(TARGET ${PROJECT_NAME} PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)

#configure a header file which contains the version information for use in C++
#configure_file(cmake/version.h.in "${PROJECT_BINARY_DIR}/version.h" @ONLY)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  add_custom_target(doc ALL
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
  install(DIRECTORY ${CMAKE_BINARY_DIR}/doc/ DESTINATION share/doc/${PROJECT_NAME}-${${PROJECT_NAME}_SOVERSION})
else(DOXYGEN_FOUND)
  message("Doxygen not found, documentation will not be build.")
endif(DOXYGEN_FOUND)


#Install the library and the executables
install(TARGETS ${PROJECT_NAME}
            RUNTIME DESTINATION bin
	    LIBRARY DESTINATION lib
  )


#all include files go into include/PROJECT_NAME
#The exclusion of ${PROJECT_NAME} prevents the recursive installation of the files just being installed.
#The original headers are in include/*.h, the installed ones in include/PROJECT_NAME/*.h.
install(DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRS} DESTINATION include/mtca4u/VirtualLab
          FILES_MATCHING PATTERN "*.h" 
	  PATTERN ".svn" EXCLUDE
	  PATTERN "${PROJECT_NAME}" EXCLUDE
	  )



#Create the config files by filling the correct variables into the template (*.cmake.in).
#All variables @VARIABLE@ are replaced with the current values, but they have to be set first....
#Only the reference header is needed. The other variables are for dependencies of this library,
#but there are none.
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include ${mtca4u-deviceaccess_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}")
set(${PROJECT_NAME}_LIBRARIES "${mtca4u-deviceaccess_LIBRARIES} ${Boost_LIBRARIES}")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib ${mtca4u-deviceaccess_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}")
set(${PROJECT_NAME}_CXX_FLAGS "${mtca4u-deviceaccess_CXX_FLAGS}")
set(${PROJECT_NAME}_LINKER_FLAGS "-Wl,-rpath=${CMAKE_INSTALL_PREFIX}/lib,--enable-new-dtags")
include(${CMAKE_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)

# The instructions to create a debian package are loaded from a separate file
include(${CMAKE_SOURCE_DIR}/cmake/prepare_debian_package.cmake)
