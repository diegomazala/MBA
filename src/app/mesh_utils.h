#pragma once

#include <iostream>

#include <algorithm>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#define _USE_MATH_DEFINES
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using TriMesh = OpenMesh::TriMesh_ArrayKernelT<>;
OpenMesh::VPropHandleT<TriMesh::Normal> Normals;
OpenMesh::VPropHandleT<TriMesh::TexCoord2D> TexCoord;



template <typename T>
static std::string filename_append_before_extension(const std::string& filename_in, const T& append, const char separator = '_')
{
	auto ext_pos = filename_in.rfind('.', filename_in.length());
	if (ext_pos == std::string::npos)
		ext_pos = filename_in.length();
	std::stringstream filename_out;
	filename_out << filename_in.substr(0, ext_pos) << separator << append << filename_in.substr(ext_pos, filename_in.length() - ext_pos);
	return filename_out.str();
}


static bool load_mesh(TriMesh& mesh, const std::string& filename)
{
	mesh.request_vertex_texcoords2D();
	mesh.request_vertex_normals();
	try
	{
		std::cout << "-- Loading mesh " << filename << std::endl;
		OpenMesh::IO::Options OptionRead(OpenMesh::IO::Options::VertexTexCoord);
		return OpenMesh::IO::read_mesh(mesh, filename, OptionRead);
	}
	catch (const std::exception& ex)
	{
		std::cerr << ex.what() << std::endl;
		return false;
	}
}

static bool save_mesh(TriMesh& mesh, const std::string& filename)
{
	try
	{
		std::cout << "-- Saving mesh " << filename << std::endl;
		mesh.request_vertex_texcoords2D();
		mesh.request_vertex_normals();
		OpenMesh::IO::Options OptionWrite(OpenMesh::IO::Options::VertexTexCoord); // | OpenMesh::IO::Options::VertexNormal);
		return OpenMesh::IO::write_mesh(mesh, filename, OptionWrite);
	}
	catch (const std::exception& ex)
	{
		std::cerr << ex.what() << std::endl;
		return false;
	}
}


static std::tuple<OpenMesh::Vec3f, OpenMesh::Vec3f> mesh_bounding_box(const TriMesh& mesh)
{
    OpenMesh::Vec3f min_vert = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
	OpenMesh::Vec3f max_vert = { std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min() };

	for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		auto pt = mesh.point(*vi);
		for (auto i = 0; i < 3; ++i)
		{
			if (pt[i] < min_vert[i])	min_vert[i] = pt[i];
			if (pt[i] > max_vert[i])	max_vert[i] = pt[i];
		}
	}
	return std::make_tuple(min_vert, max_vert);
}

static OpenMesh::Vec3f center_of_bounding_box(const OpenMesh::Vec3f& min_corner, const OpenMesh::Vec3f& max_corner)
{
    return 
    {
        min_corner[0] + ((max_corner[0] - min_corner[0]) * 0.5f),
        min_corner[1] + ((max_corner[1] - min_corner[1]) * 0.5f),
        min_corner[2] + ((max_corner[2] - min_corner[2]) * 0.5f)
    };
}

static float max_range_of_bounding_box(const OpenMesh::Vec3f& min_corner, const OpenMesh::Vec3f& max_corner)
{
    std::vector<float> range = 
    {
        max_corner[0] - min_corner[0],
        max_corner[1] - min_corner[1],
        max_corner[2] - min_corner[2]
    };
    return *std::max_element(range.begin(), range.end());
}

static void normalize_mesh(TriMesh& mesh, const OpenMesh::Vec3f& center, float scale)
{
    for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		auto point = mesh.point(*vi);
        point -= center;
        point /= scale;
        point += OpenMesh::Vec3f(0.5f, 0.5f, 0.5f);
        mesh.set_point(*vi, point);
    }
}


static void normalize_mesh(TriMesh& mesh)
{
    auto [min_corner, max_corner] = mesh_bounding_box(mesh);
    auto scale = max_range_of_bounding_box(min_corner, max_corner);
    auto center = center_of_bounding_box(min_corner, max_corner);
    normalize_mesh(mesh, center, scale);
}

