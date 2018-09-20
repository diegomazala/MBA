
project(simplest_mba VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} simplest_mba.cpp)

# Define the libraries this project depends upon
target_link_libraries(${PROJECT_NAME} mba)



find_package(OpenMesh)


project(mesh_mba VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} mesh_mba.cpp)

#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS})

# Define the libraries this project depends upon
if (MSVC)
	target_link_libraries(${PROJECT_NAME} mba ${OPENMESH_LIBRARIES})
else()
	target_link_libraries(${PROJECT_NAME} mba ${OPENMESH_LIBRARIES} stdc++fs)
endif()



