#include <iostream>
#include <MBA.h>
#include <UCButils.h>
#include <PointAccessUtils.h>
typedef std::vector<double> dVec;

#include <algorithm>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#define _USE_MATH_DEFINES
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

using TriMesh = OpenMesh::TriMesh_ArrayKernelT<>;
OpenMesh::VPropHandleT<TriMesh::Normal> Normals;
OpenMesh::VPropHandleT<TriMesh::TexCoord2D> TexCoord;

std::string g_filename_in;
std::string g_filename_out;



template <typename T>
std::string filename_append_before_extension(const std::string& filename_in, const T& append, const char separator = '_')
{
	auto ext_pos = filename_in.rfind('.', filename_in.length());
	if (ext_pos == std::string::npos)
		ext_pos = filename_in.length();
	std::stringstream filename_out;
	filename_out << filename_in.substr(0, ext_pos) << separator << append << filename_in.substr(ext_pos, filename_in.length() - ext_pos);
	return filename_out.str();
}

std::string build_output_filename(int m0, int n0, int h)
{
	fs::path file_in(g_filename_in);
	std::stringstream out;
	out << file_in.parent_path().string() << '/' << file_in.stem().string() << '_' << m0 << 'x' << n0 << 'x' << h 
		//<< ".ply"
		<< file_in.extension().string();
	fs::path file_out(out.str());
	// std::cout << "Out: " << file_out.string() << std::endl;
	return file_out.string();
}


bool load_mesh_in(TriMesh& mesh)
{
	mesh.request_vertex_texcoords2D();
	mesh.request_vertex_normals();
	try
	{
		std::cout << "-- Loading mesh " << g_filename_in << std::endl;
		OpenMesh::IO::Options OptionRead(OpenMesh::IO::Options::VertexTexCoord);
		return OpenMesh::IO::read_mesh(mesh, g_filename_in, OptionRead);
	}
	catch (const std::exception& ex)
	{
		std::cerr << ex.what() << std::endl;
		return false;
	}
}

bool save_mesh_out(TriMesh& mesh)
{
	try
	{
		std::cout << "-- Saving mesh " << g_filename_out << std::endl;
		mesh.request_vertex_texcoords2D();
		mesh.request_vertex_normals();
		OpenMesh::IO::Options OptionWrite(OpenMesh::IO::Options::VertexTexCoord); // | OpenMesh::IO::Options::VertexNormal);
		return OpenMesh::IO::write_mesh(mesh, g_filename_out, OptionWrite);
	}
	catch (const std::exception& ex)
	{
		std::cerr << ex.what() << std::endl;
		return false;
	}
}


std::pair<OpenMesh::Vec3f, OpenMesh::Vec3f> mesh_min_max_vert(const TriMesh& mesh)
{
	OpenMesh::Vec3f min_vert = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
	OpenMesh::Vec3f max_vert = { std::numeric_limits<float>::min(), std::numeric_limits<float>::min(), std::numeric_limits<float>::min() };

	for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		//auto uv = mesh.texcoord2D(*vi);
		auto pt = mesh.point(*vi);

		for (auto i = 0; i < 3; ++i)
		{
			if (pt[i] < min_vert[i])	min_vert[i] = pt[i];
			if (pt[i] > max_vert[i])	max_vert[i] = pt[i];
		}
	}

	//std::cout << "min: " << min_vert << std::endl;
	//std::cout << "max: " << max_vert << std::endl;
	return std::make_pair(min_vert, max_vert);
}

template <typename T>
constexpr T ndc(T v, T min, T max) // normalize to [0, 1]
{
	return (v - min) / (max - min);
}

void mesh_to_vecs(TriMesh& mesh, const std::pair<OpenMesh::Vec3f, OpenMesh::Vec3f>& min_max_vert,
	std::shared_ptr<dVec> U, std::shared_ptr<dVec> V, std::shared_ptr<dVec> Z)
{
	for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		auto uv = mesh.texcoord2D(*vi);
		auto point = mesh.point(*vi);

		uv[0] = ndc(point[0], min_max_vert.first[0], min_max_vert.second[0]);
		uv[1] = ndc(point[1], min_max_vert.first[1], min_max_vert.second[1]);

		point[0] = ndc(point[0], min_max_vert.first[0], min_max_vert.second[0]);
		point[1] = ndc(point[1], min_max_vert.first[1], min_max_vert.second[1]);
		point[2] = ndc(point[2], min_max_vert.first[2], min_max_vert.second[2]);

		mesh.set_texcoord2D(*vi, uv);
		mesh.set_point(*vi, point);
	
		//if (point[2] > 0.5)
		{
			U->push_back(uv[0]);
			V->push_back(uv[1]);
			Z->push_back(point[2]);
		}
	}
}


void mba_to_mesh(UCBspl::SplineSurface& interp, TriMesh& mesh,
    const std::pair<OpenMesh::Vec3f, OpenMesh::Vec3f>& min_max_vert,
	std::shared_ptr<dVec> U, std::shared_ptr<dVec> V, int z_index)
{
	auto u = U->begin();
	auto v = V->begin();
	for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		//auto uv = mesh.texcoord2D(*vi);
		auto point = mesh.point(*vi);
		//if (point[2] > 0.5)
			//point[z_index] = static_cast<float>(interp.f(uv[0], uv[1]));
		//point[z_index] = static_cast<float>(interp.f(*u, *v));
        auto f = static_cast<float>(interp.f(point[0], point[2]));
        if (f > 0.5)
            point[1] = f;
        //point[1] = static_cast<float>(interp.f(point[0], point[2]));
		mesh.set_point(*vi, point);

		u++; v++;
	}
}


void create_grid(TriMesh& mesh_out, uint32_t rows, uint32_t cols, float width = 1.0f, float height = 1.0f)
{
    std::vector<TriMesh::VertexHandle> vhandle((rows + 1) * (cols + 1));
    std::vector<TriMesh::VertexHandle> fhandle(3);

    for (auto i = 0; i <= rows; ++i)
    {
        for (auto j = 0; j <= cols; ++j)
        {
            auto x = static_cast<float>(j) / cols * width;
            auto z = static_cast<float>(i) / rows * height;
            vhandle[i*(rows+1)+j] = mesh_out.add_vertex(TriMesh::Point(x, 0.0f, z));
            //std::cout << i*(rows+1)+j << std::endl;
        }
    }

    rows += 1;
    cols += 1;

    for (auto i = 0; i < rows - 1; ++i)
    {
        for (auto j = 1; j < cols; ++j)
        {

            //std::cout << i * rows + j << ' ' << (i + 1) * rows + j - 1 << ' ' << i * rows + j - 1 << std::endl;
            fhandle[0] = vhandle[i * rows + j];
            fhandle[2] = vhandle[(i + 1) * rows + j - 1];
            fhandle[1] = vhandle[i * rows + j - 1];
            mesh_out.add_face(fhandle);
            //std::cout << i * rows + j << ' ' << (i + 1) * rows + j << ' ' << (i + 1) * rows + j - 1 << std::endl;
            fhandle[0] = vhandle[i * rows + j];
            fhandle[2] = vhandle[(i + 1) * rows + j];
            fhandle[1] = vhandle[(i + 1) * rows + j - 1];
            mesh_out.add_face(fhandle);
            //std::cout << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
	if (argc != 6)
	{
		std::cout 
			<< "Usage: app <mesh_file> < m_n > < h > < y/z > <mesh_grid_size>\n"
			<< "Usage: app ../data/face.obj 5 3 z 128 \n";
		return EXIT_FAILURE;
	}
	
	g_filename_in = argv[1];
	const uint16_t m0 = atoi(argv[2]);
	const uint16_t n0 = m0;
	const uint16_t h = atoi(argv[3]);
	const char z_function = argv[4][0];
    const uint16_t mesh_grid_size = atoi(argv[5]);
	g_filename_out = build_output_filename(m0, n0, h);

	TriMesh mesh_in;
	if (!load_mesh_in(mesh_in))
	{
		std::cout << "Could not read " << g_filename_in << std::endl;
		return EXIT_FAILURE;
	}

	auto min_max_vert = mesh_min_max_vert(mesh_in);
	std::cout << "-- min: " << min_max_vert.first << std::endl;
	std::cout << "-- max: " << min_max_vert.second << std::endl;

#if 0
    std::vector<float> range = 
    {
        min_max_vert.second[0] - min_max_vert.first[0],
        min_max_vert.second[1] - min_max_vert.first[1],
        min_max_vert.second[2] - min_max_vert.first[2]
    };
    std::cout << "Range: " << range[0] << ' ' << range[1] << ' ' << range[2] << std::endl;

    std::vector<float>::const_iterator max_elem = std::max_element(range.begin(), range.end());
    min_max_vert.second[0] = min_max_vert.first[0] + range[0] / *max_elem * range[0];
    min_max_vert.second[1] = min_max_vert.first[1] + range[1] / *max_elem * range[1];
    min_max_vert.second[2] = min_max_vert.first[2] + range[2] / *max_elem * range[2];

    std::cout << "-- min: " << min_max_vert.first << std::endl;
	std::cout << "-- max: " << min_max_vert.second << std::endl;
#endif
    //return 0;
	
	std::shared_ptr<dVec> u_arr(new std::vector<double>);
	std::shared_ptr<dVec> v_arr(new std::vector<double>);
	std::shared_ptr<dVec> z_arr(new std::vector<double>);

	int z_index = 2;
	if (z_function == 'y')
	{
		z_index = 1;
		mesh_to_vecs(mesh_in, min_max_vert, u_arr, z_arr, v_arr);
	}
	else
	{
		z_index = 2;
		mesh_to_vecs(mesh_in, min_max_vert, u_arr, v_arr, z_arr);
	}

	//save_mesh_out(mesh_in);
	//return 0;

	// Algorithm setup.
	MBA mba(u_arr, v_arr, z_arr);
	mba.MBAalg(m0, n0, h);
	UCBspl::SplineSurface interp = mba.getSplineSurface();

	// save_mesh_out(mesh_in);
	// return 0;
	
	//TriMesh mesh_out = mesh_in;
    TriMesh mesh_out;
    create_grid(mesh_out, mesh_grid_size, mesh_grid_size);
	mba_to_mesh(interp, mesh_out, min_max_vert, u_arr, v_arr, z_index);


	if (!save_mesh_out(mesh_out))
	{
		std::cout << "Could not save " << g_filename_out << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}