#include <MBA.h>
#include <UCButils.h>
#include <PointAccessUtils.h>
typedef std::vector<double> dVec;

#include "mesh_utils.h"

std::string g_filename_in;
std::string g_filename_out;


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


void mesh_to_vecs(TriMesh& mesh, const std::pair<OpenMesh::Vec3f, OpenMesh::Vec3f>& min_max_vert,
	std::shared_ptr<dVec> U, std::shared_ptr<dVec> V, std::shared_ptr<dVec> Z)
{
	for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		//auto uv = mesh.texcoord2D(*vi);
		auto point = mesh.point(*vi);

		//uv[0] = ndc(point[0], min_max_vert.first[0], min_max_vert.second[0]);
		//uv[1] = ndc(point[1], min_max_vert.first[1], min_max_vert.second[1]);

		point[0] = ndc(point[0], min_max_vert.first[0], min_max_vert.second[0]);
		point[1] = ndc(point[1], min_max_vert.first[1], min_max_vert.second[1]);
		point[2] = ndc(point[2], min_max_vert.first[2], min_max_vert.second[2]);

		//mesh.set_texcoord2D(*vi, uv);
		mesh.set_point(*vi, point);
	
		//if (point[2] > 0.5)
		{
			U->push_back(point[0]);
			V->push_back(point[1]);
			Z->push_back(point[2]);
		}
	}
}



void mba_to_mesh(UCBspl::SplineSurface& interp, TriMesh& mesh,
    const std::pair<OpenMesh::Vec3f, OpenMesh::Vec3f>& min_max_vert,
	std::shared_ptr<dVec> U, std::shared_ptr<dVec> V, int z_index)
{
    std::vector<float> range = 
    {
        min_max_vert.second[0] - min_max_vert.first[0],
        min_max_vert.second[1] - min_max_vert.first[1],
        min_max_vert.second[2] - min_max_vert.first[2]
    };
    
    std::vector<float>::const_iterator max_elem = std::max_element(range.begin(), range.end());

	auto u = U->begin();
	auto v = V->begin();
	for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		//auto uv = mesh.texcoord2D(*vi);
		auto point = mesh.point(*vi);
		//if (point[2] > 0.5)
			//point[z_index] = static_cast<float>(interp.f(uv[0], uv[1]));
		//point[z_index] = static_cast<float>(interp.f(*u, *v));
        //auto f = static_cast<float>(interp.f(point[0], point[1]));
        auto f = static_cast<float>(interp.f(point[0], point[1]));
        //if (point[2] > 0.5)
            point[2] = f;
        
        // point[0] = min_max_vert.first[0] + point[0] * range[0];
        // point[1] = min_max_vert.first[1] + point[1] * range[1];
        // point[2] = min_max_vert.first[2] + point[2] * range[2];

		mesh.set_point(*vi, point);

		u++; v++;
	}
}



void mba_to_mesh(UCBspl::SplineSurface& interp, TriMesh& mesh, int z_index)
{
	for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
	{
		auto point = mesh.point(*vi);
        point[2] = static_cast<float>(interp.f(point[0], point[1]));
		mesh.set_point(*vi, point);
	}
}



int main(int argc, char* argv[])
{
	if (argc != 5)
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
    //const uint16_t mesh_grid_size = atoi(argv[5]);
	g_filename_out = build_output_filename(m0, n0, h);

	TriMesh mesh_in;
	if (!load_mesh(mesh_in, g_filename_in))
	{
		std::cout << "Could not read " << g_filename_in << std::endl;
		return EXIT_FAILURE;
	}

	std::shared_ptr<dVec> u_arr(new std::vector<double>);
	std::shared_ptr<dVec> v_arr(new std::vector<double>);
	std::shared_ptr<dVec> z_arr(new std::vector<double>);


    auto min_max_vert = mesh_min_max_vert(mesh_in);
	std::cout << "-- min: " << min_max_vert.first << std::endl;
	std::cout << "-- max: " << min_max_vert.second << std::endl;

    OpenMesh::Vec3f up;
	int z_index = 2;
	if (z_function == 'y')
	{
		z_index = 1;
        up = {0, 0, -1};
		mesh_to_vecs(mesh_in, min_max_vert, u_arr, z_arr, v_arr);
        //mesh_to_vecs(mesh_in, u_arr, z_arr, v_arr);
	}
	else
	{
		z_index = 2;
        up = {0, 1, 0};
		mesh_to_vecs(mesh_in, min_max_vert, u_arr, v_arr, z_arr);
        //mesh_to_vecs(mesh_in, u_arr, v_arr, z_arr);
	}

	// Algorithm setup.
	MBA mba(u_arr, v_arr, z_arr);
	mba.MBAalg(m0, n0, h);
	UCBspl::SplineSurface interp = mba.getSplineSurface();

    std::cout << std::fixed << "\ndelta\n";
    mba.delta_.print();
    std::cout << std::fixed << "\nomega\n";
    mba.omega_.print();
    std::cout << std::fixed << "\nphi\n";
    mba.PHI_->print();
  
    exit(0);

	
	TriMesh mesh_out = mesh_in;
    //TriMesh mesh_out;
    //create_grid_mesh(mesh_out, mesh_grid_size, mesh_grid_size, up);
    //std::cout << "create_grid_mesh\n" << std::endl;
	mba_to_mesh(interp, mesh_out, min_max_vert, u_arr, v_arr, z_index);
    //mba_to_mesh(interp, mesh_out, z_index);


	if (!save_mesh(mesh_out, g_filename_out))
	{
		std::cout << "Could not save " << g_filename_out << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}