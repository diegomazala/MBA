#include <iostream>
#include "mba_surface.h"
#include "mesh_utils.h"


int main(int argc, char* argv[])
{
    using decimal_t = double;

    if (argc < 4)
	{
		std::cout 
			<< "Usage: app <mesh_file> < m_n > < h >\n"
			<< "Usage: app ../data/face.obj 3 3 \n";
		return EXIT_FAILURE;
	}

    const std::string filename_in = argv[1];
	const uint32_t m = atoi(argv[2]);
	const uint32_t n = m;
    const uint32_t h = atoi(argv[3]);
    const std::string filename_out = 
        filename_append_before_extension(
            filename_append_before_extension(
                filename_append_before_extension(filename_in, argv[2]), 
                argv[3]), 
            "mba");


    TriMesh mesh;
	if (!load_mesh(mesh, filename_in))
	{
		std::cout << "Could not read " << filename_in << std::endl;
		return EXIT_FAILURE;
	}

    std::vector<decimal_t> x(mesh.n_vertices(), 0);
    std::vector<decimal_t> y(mesh.n_vertices(), 0);
    std::vector<decimal_t> z(mesh.n_vertices(), 0);
    mesh_uv_to_vecs(mesh, x, y, z);

    surface::multilevel_bspline<decimal_t> surf = { x.data(), y.data(), z.data(), mesh.n_vertices(), m, n, h };
    surf.compute();
    
#if 0   // Save each k-level
    int k = 0;
    for (const auto& surf_level : surf.get_surfaces())
    {
        for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
        {
            const auto& uv = mesh.texcoord2D(*vi);
            auto point = mesh.point(*vi);
            point[2] = surf_level(uv[0], uv[1]);
            mesh.set_point(*vi, point);
        }

        std::stringstream ss;
        ss << "_mn_" << m << "_k_" << k << "_mba";
        const std::string filename = filename_append_before_extension(filename_in, ss.str());

        if (!save_mesh(mesh, filename))
        {
            std::cout << "Could not save " << filename << std::endl;
            return EXIT_FAILURE;
        }
        k++;
    }
#endif

#if 0
    for (uint32_t k = 0; k <= h; ++k)
    {
        for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
        {
            const auto& uv = mesh.texcoord2D(*vi);
            auto point = mesh.point(*vi);
            point[2] = surf(uv[0], uv[1], k);
            mesh.set_point(*vi, point);
        }

        std::stringstream ss;
        ss << "_mn_" << m << "_kh_" << k << "_mba_ref1";
        const std::string filename = filename_append_before_extension(filename_in, ss.str());

        if (!save_mesh(mesh, filename))
        {
            std::cout << "Could not save " << filename << std::endl;
            return EXIT_FAILURE;
        }
    }
#endif

    for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
    {
        const auto& uv = mesh.texcoord2D(*vi);
        auto point = mesh.point(*vi);
        point[2] = surf(uv[0], uv[1]);
		//point[2] = surf(point[0], point[1]);
        mesh.set_point(*vi, point);
    }
    

    if (!save_mesh(mesh, filename_out))
	{
		std::cout << "Could not save " << filename_out << std::endl;
		return EXIT_FAILURE;
	}

    return EXIT_SUCCESS;
}

