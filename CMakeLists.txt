PROJECT(mtca4u-VirtualLab)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 08)
set(${PROJECT_NAME}_PATCH_VERSION 01)
include(${CMAKE_SOURCE_DIR}/cmake/set_version_numbers.cmake)

include(${CMAKE_SOURCE_DIR}/cmake/add_dependency.cmake)
add_dependency(ChimeraTK-DeviceAccess 01.04 REQUIRED)

FIND_PACKAGE(Boost COMPONENTS system unit_test_framework REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
set(${PROJECT_NAME}_INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/)
aux_source_directory(${CMAKE_SOURCE_DIR}/src library_sources)


include(${CMAKE_SOURCE_DIR}/cmake/set_default_build_to_release.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/set_default_flags.cmake)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-inline -fno-inline-small-functions -fno-default-inline")
include(${CMAKE_SOURCE_DIR}/cmake/enable_code_coverage_report.cmake)


# linker flags
string(APPEND CMAKE_SHARED_LINKER_FLAGS " ${ChimeraTK-DeviceAccess_LINKER_FLAGS} -Wl,--export-dynamic ")
string(APPEND CMAKE_EXE_LINKER_FLAGS " ${ChimeraTK-DeviceAccess_LINKER_FLAGS} ")

#Create the executables for automated unit testing.
#Currently we assume that they are all tests contained in one file, so
#each file gives a new executable. This section has to be adapted if this should change.
ENABLE_TESTING()

aux_source_directory(${CMAKE_SOURCE_DIR}/tests/executables_src testExecutables)
foreach( testExecutableSrcFile ${testExecutables})
  #NAME_WE means the base name without path and (longest) extension
  get_filename_component(excutableName ${testExecutableSrcFile} NAME_WE)
  add_executable(${excutableName} ${testExecutableSrcFile})
  target_link_libraries(${excutableName} ${PROJECT_NAME} ${ChimeraTK-DeviceAccess_LIBRARIES} ${PYTHON_LIBRARIES})
  add_test(${excutableName} ${excutableName})
endforeach( testExecutableSrcFile )

aux_source_directory(${CMAKE_SOURCE_DIR}/performance_tests perftestExecutables)
foreach( perftestExecutableSrcFile ${perftestExecutables})
  #NAME_WE means the base name without path and (longest) extension
  get_filename_component(excutableName ${perftestExecutableSrcFile} NAME_WE)
  add_executable(${excutableName} ${perftestExecutableSrcFile})
  target_link_libraries(${excutableName} ${PROJECT_NAME} ${ChimeraTK-DeviceAccess_LIBRARIES} ${PYTHON_LIBRARIES})
endforeach( perftestExecutableSrcFile )

# copy the map files needed for the tests
foreach( FILE_TO_COPY test.mapp dummies.dmap )
  FILE (COPY tests/${FILE_TO_COPY} DESTINATION ${PROJECT_BINARY_DIR})
endforeach( FILE_TO_COPY )

# C++ library
add_library(${PROJECT_NAME} SHARED ${library_sources} )
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION} SOVERSION ${${PROJECT_NAME}_SOVERSION})
target_link_libraries(${PROJECT_NAME} ${ChimeraTK-DeviceAccess_LIBRARIES})
# do not remove runtime path of the library  when installing
set_property(TARGET ${PROJECT_NAME} PROPERTY INSTALL_RPATH_USE_LINK_PATH TRUE)

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
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin
                                LIBRARY DESTINATION lib )


#all include files go into include/PROJECT_NAME
#The exclusion of ${PROJECT_NAME} prevents the recursive installation of the files just being installed.
#The original headers are in include/*.h, the installed ones in include/PROJECT_NAME/*.h.
install(DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRS}
        DESTINATION include/ChimeraTK/VirtualLab
        FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE
                                     PATTERN "${PROJECT_NAME}" EXCLUDE )

# Compatibility...
install(DIRECTORY ${${PROJECT_NAME}_INCLUDE_DIRS}
       DESTINATION include/mtca4u/VirtualLab
       FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE
                                    PATTERN "${PROJECT_NAME}" EXCLUDE )
#Create the config files by filling the correct variables into the template (*.cmake.in).
#All variables @VARIABLE@ are replaced with the current values, but they have to be set first....
#Only the reference header is needed. The other variables are for dependencies of this library,
#but there are none.
set(${PROJECT_NAME}_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include ${ChimeraTK-DeviceAccess_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}")
set(${PROJECT_NAME}_LIBRARIES "${ChimeraTK-DeviceAccess_LIBRARIES}")
set(${PROJECT_NAME}_LIBRARY_DIRS "${CMAKE_INSTALL_PREFIX}/lib ${ChimeraTK-DeviceAccess_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS}")
set(${PROJECT_NAME}_CXX_FLAGS "${ChimeraTK-DeviceAccess_CXX_FLAGS}")
set(${PROJECT_NAME}_LINKER_FLAGS "-Wl,-rpath=${CMAKE_INSTALL_PREFIX}/lib,--enable-new-dtags ${ChimeraTK-DeviceAccess_LINKER_FLAGS}")
include(${CMAKE_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)

