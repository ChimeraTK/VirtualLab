project(mtca4u-VirtualLab)
cmake_minimum_required(VERSION 3.16)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(${PROJECT_NAME}_MAJOR_VERSION 00)
set(${PROJECT_NAME}_MINOR_VERSION 10)
set(${PROJECT_NAME}_PATCH_VERSION 00)
include(${CMAKE_SOURCE_DIR}/cmake/set_version_numbers.cmake)

find_package(ChimeraTK-DeviceAccess 03.09 REQUIRED)
find_package(Boost COMPONENTS system unit_test_framework REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)
aux_source_directory(${CMAKE_SOURCE_DIR}/src library_sources)

include(${CMAKE_SOURCE_DIR}/cmake/set_default_build_to_release.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/set_default_flags.cmake)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-inline -fno-inline-small-functions -fno-default-inline")
include(${CMAKE_SOURCE_DIR}/cmake/enable_code_coverage_report.cmake)

# Create the executables for automated unit testing.
# Currently we assume that they are all tests contained in one file, so
# each file gives a new executable. This section has to be adapted if this should change.
enable_testing()

aux_source_directory(${CMAKE_SOURCE_DIR}/tests/executables_src testExecutables)
foreach(testExecutableSrcFile ${testExecutables})
  #NAME_WE means the base name without path and (longest) extension
  get_filename_component(excutableName ${testExecutableSrcFile} NAME_WE)
  add_executable(${excutableName} ${testExecutableSrcFile})
  target_link_libraries(${excutableName} ${PROJECT_NAME})
  add_test(${excutableName} ${excutableName})
endforeach(testExecutableSrcFile)

aux_source_directory(${CMAKE_SOURCE_DIR}/performance_tests perftestExecutables)
foreach(perftestExecutableSrcFile ${perftestExecutables})
  #NAME_WE means the base name without path and (longest) extension
  get_filename_component(excutableName ${perftestExecutableSrcFile} NAME_WE)
  add_executable(${excutableName} ${perftestExecutableSrcFile})
  target_link_libraries(${excutableName} ${PROJECT_NAME})
endforeach(perftestExecutableSrcFile)

# copy the map files needed for the tests
foreach(FILE_TO_COPY test.mapp dummies.dmap)
  file(COPY tests/${FILE_TO_COPY} DESTINATION ${PROJECT_BINARY_DIR})
endforeach(FILE_TO_COPY)

# C++ library
add_library(${PROJECT_NAME} SHARED ${library_sources})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_FULL_LIBRARY_VERSION}
                                                 SOVERSION ${${PROJECT_NAME}_SOVERSION})
target_link_libraries(${PROJECT_NAME} PUBLIC ChimeraTK::ChimeraTK-DeviceAccess)

# add a target to generate API documentation with Doxygen
include(${CMAKE_SOURCE_DIR}/cmake/enable_doxygen_documentation.cmake)

# Install the library and the executables
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION bin
                                LIBRARY DESTINATION lib)

# all include files go into include/PROJECT_NAME
# The exclusion of ${PROJECT_NAME} prevents the recursive installation of the files just being installed.
# The original headers are in include/*.h, the installed ones in include/PROJECT_NAME/*.h.
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
        DESTINATION include/ChimeraTK/VirtualLab
        FILES_MATCHING PATTERN "*.h")

# Compatibility...
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
        DESTINATION include/mtca4u/VirtualLab
        FILES_MATCHING PATTERN "*.h")

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Export targets for downstream project
set(PROVIDES_EXPORTED_TARGETS 1)
list(APPEND ${PROJECT_NAME}_PUBLIC_DEPENDENCIES "ChimeraTK-DeviceAccess")
include(${CMAKE_SOURCE_DIR}/cmake/create_cmake_config_files.cmake)
