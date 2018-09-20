# Define library. Only source files here!
project(mba VERSION 0.1 LANGUAGES CXX)

add_library(${PROJECT_NAME}
    src/MBA.cpp
    src/MBAdata.cpp
    src/PointAccessUtils.cpp
    src/filterRestrict2D.cpp
    src/UCBsplines.cpp
    src/UCBsplineSurface.cpp
    src/UCButils.cpp
)

# Define headers for this library. PUBLIC headers are used for
# compiling the library, and will be added to consumers' build
# paths.
target_include_directories(${PROJECT_NAME} PUBLIC
	${Boost_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mba>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# If we have compiler requirements for this library, list them
# here
target_compile_features(${PROJECT_NAME}
    PUBLIC cxx_auto_type
    PRIVATE cxx_variadic_templates)

# Depend on a library that we defined in the top-level file
#target_link_libraries(${PROJECT_NAME} MyOtherLibrary)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS ${PROJECT_NAME} EXPORT MyLibraryConfig
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT MyLibraryConfig DESTINATION share/MyLibrary/cmake)

# This makes the project importable from the build directory
export(TARGETS ${PROJECT_NAME} FILE MyLibraryConfig.cmake)

#set (PROJECT_TEST_NAME testmba)

# Every library has unit tests, of course
#add_executable(${PROJECT_TEST_NAME}
#    test/testlib.cpp)

#target_link_libraries(${PROJECT_TEST_NAME}
#    ${PROJECT_NAME})

#add_test(${PROJECT_TEST_NAME} ${PROJECT_TEST_NAME})
