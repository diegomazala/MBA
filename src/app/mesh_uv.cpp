#include <iostream>
#include <string>

#include "mesh_utils.h"


int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		std::cout << "Parameters invalid!" << std::endl;
		for (auto i = 0; i < argc; i++)
		{
			std::cout << "Param " << i << " -> " << argv[i] << std::endl;
		}
		return 0;
	}

	std::string input_path = argv[1];
	size_t index = input_path.find_last_of('.');
	std::string output_path = input_path.insert(index, "_uvz");

	OpenMesh::IO::Options OptionRead(OpenMesh::IO::Options::VertexNormal | OpenMesh::IO::Options::VertexTexCoord);
	OpenMesh::IO::Options OptionWrite(OpenMesh::IO::Options::VertexNormal | OpenMesh::IO::Options::VertexTexCoord);
	
	TriMesh mesh;
	mesh.request_vertex_texcoords2D();
	mesh.request_vertex_normals();
	
	if (!OpenMesh::IO::read_mesh(mesh, argv[1], OptionRead))
	{
		std::cout << "Read File fail!" << std::endl;
	}

	if (!OptionRead.check(OpenMesh::IO::Options::VertexNormal))
	{
		std::cout << "Mesh dose not have normal,now calculating..." << std::endl;
		mesh.request_face_normals();
		mesh.update_normals();
		mesh.release_face_normals();
	}

    if (!OptionRead.check(OpenMesh::IO::Options::VertexTexCoord))
	{
		std::cout << "Mesh dose not have texcoords,now calculating..." << std::endl;
		for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
        {
            TriMesh::TexCoord2D uv;
            auto point = mesh.point(*vi);
            uv[0] = point[0];
            uv[1] = point[1];
            mesh.set_texcoord2D(*vi, uv);
        }
	}

	// for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	// {
	// 	auto uv = mesh.texcoord2D(*vi);
	// 	auto point = mesh.point(*vi);
	// 	point[0] = uv[0];
	// 	point[1] = uv[1];
	// 	//point[2] = 0.0f;
	// 	mesh.set_point(*vi, point);
	// }

	if (OpenMesh::IO::write_mesh(mesh, output_path, OptionWrite))
	{
		std::cout << "File transformed! " << output_path << std::endl;
	}

    return 0;
}