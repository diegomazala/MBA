#include <iostream>
#include <string>

//OpenMesh needed

#define _USE_MATH_DEFINES

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using namespace std;

using mesh = OpenMesh::TriMesh_ArrayKernelT<>;

int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		cout << "Parameters invalid!" << endl;
		for (auto i = 0; i < argc; i++)
		{
			cout << "Param " << i << " -> " << argv[i] << endl;
		}
		return 0;
	}

	string input_path = argv[1];
	size_t index = input_path.find_last_of('.');
	string output_path = input_path.insert(index, "_uvz");

	OpenMesh::IO::Options OptionRead(OpenMesh::IO::Options::VertexTexCoord);
	OpenMesh::IO::Options OptionWrite(OpenMesh::IO::Options::VertexTexCoord);// | OpenMesh::IO::Options::VertexNormal);
	
	mesh mesh;
	mesh.request_vertex_texcoords2D();
	//mesh.request_vertex_normals();
	
	if (!OpenMesh::IO::read_mesh(mesh, argv[1], OptionRead))
	{
		cout << "Read File fail!" << endl;
	}

	// if (!OptionRead.check(OpenMesh::IO::Options::VertexNormal))
	// {
	// 	cout << "Mesh dose not have normal,now calculating..." << endl;
	// 	mesh.request_face_normals();
	// 	mesh.update_normals();
	// 	mesh.release_face_normals();
	// }

	for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		auto uv = mesh.texcoord2D(*vi);
		auto point = mesh.point(*vi);
		point[0] = uv[0];
		point[1] = uv[1];
		//point[2] = 0.0f;
		mesh.set_point(*vi, point);
	}

	if (OpenMesh::IO::write_mesh(mesh, output_path, OptionWrite))
	{
		cout << "File transformed! " << output_path << endl;
	}

    return 0;
}