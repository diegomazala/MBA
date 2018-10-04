
project(simplest_mba VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} simplest_mba.cpp)

# Define the libraries this project depends upon
target_link_libraries(${PROJECT_NAME} mba)



find_package(OpenMesh)


project(mesh_mba VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} mesh_mba.cpp mesh_utils.h)

#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS})

# Define the libraries this project depends upon
if (MSVC)
	target_link_libraries(${PROJECT_NAME} mba ${OPENMESH_LIBRARIES})
else()
	target_link_libraries(${PROJECT_NAME} mba ${OPENMESH_LIBRARIES} stdc++fs)
endif()





project(mesh_normalize VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} mesh_normalize.cpp mesh_utils.h)

#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS})

# Define the libraries this project depends upon
if (MSVC)
	target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES})
else()
	target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES} stdc++fs)
endif()



project(mesh_uvz2xyz VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} mesh_uvz2xyz.cpp)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS})

# Define the libraries this project depends upon
target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES})





#[[
find_package(Eigen3)

project(mba_eigen VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} mba_eigen.cpp)

#set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Define the libraries this project depends upon
target_link_libraries(${PROJECT_NAME} mba ${OPENMESH_LIBRARIES})
]]



project(bspline_surface_app VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} bspline_surface_app.cpp bspline_surface.h)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Define the libraries this project depends upon
target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES})




project(mba_surface_app VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} mba_surface_app.cpp bspline_surface.h mba_surface.h)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Define the libraries this project depends upon
target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES})



find_package(Eigen3)
project(bezier_app VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} bezier_app.cpp bezier.h)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

target_include_directories (${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})

# Define the libraries this project depends upon
#target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES})









project(mesh_uv VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} mesh_uv.cpp)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS})

# Define the libraries this project depends upon
target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES})






project(mesh_cut VERSION 0.1 LANGUAGES CXX)

# Define an executable
add_executable(${PROJECT_NAME} mesh_cut.cpp)

target_include_directories (${PROJECT_NAME} PRIVATE ${OPENMESH_INCLUDE_DIRS} )

# Define the libraries this project depends upon
target_link_libraries(${PROJECT_NAME} ${OPENMESH_LIBRARIES})