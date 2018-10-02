#include <iostream>
#include <string>
#include "mesh_utils.h"


int main(int argc, char *argv[])
{
    if (argc < 4)
	{
		std::cout 
			<< "Usage: app <mesh_file> <input_uvz> <input_xyz> <output_xyz>\n"
			<< "Usage: app input_uvz.obj input_xyz.obj output_xyz.obj \n";
		return EXIT_FAILURE;
	}

	std::string input_path_uvz = argv[1];
    std::string input_path_xyz = argv[2];
    std::string output_path_xyz = argv[3];

    TriMesh mesh_uvz;
	if (!load_mesh(mesh_uvz, input_path_uvz))
	{
		std::cout << "Could not read " << input_path_uvz << std::endl;
		return EXIT_FAILURE;
	}

    TriMesh mesh_xyz;
	if (!load_mesh(mesh_xyz, input_path_xyz))
	{
		std::cout << "Could not read " << input_path_xyz << std::endl;
		return EXIT_FAILURE;
	}


    auto vi_uvz = mesh_uvz.vertices_begin();
    auto vi_xyz = mesh_xyz.vertices_begin();

	for (; vi_uvz != mesh_uvz.vertices_end(); ++vi_uvz, ++vi_xyz)
	{
		auto point_uvz = mesh_uvz.point(*vi_uvz);
        auto point_xyz = mesh_xyz.point(*vi_xyz);
		
        point_xyz[2] = point_uvz[2];
		mesh_xyz.set_point(*vi_xyz, point_xyz);
	}

	if (!save_mesh(mesh_xyz, output_path_xyz))
	{
		std::cout << "Could not save " << output_path_xyz << std::endl;
		return EXIT_FAILURE;
	}

    return 0;
}