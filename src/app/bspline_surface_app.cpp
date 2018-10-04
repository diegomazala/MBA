#include <iostream>
#include <fstream>
#include "bspline_surface.h"
#include "mesh_utils.h"


int main(int argc, char* argv[])
{
    using decimal_t = double;

    if (argc < 3)
	{
		std::cout 
			<< "Usage: app <mesh_file> < m_n >\n"
			<< "Usage: app ../data/face.obj 3 \n";
		return EXIT_FAILURE;
	}

    const std::string filename_in = argv[1];
	const uint32_t m = atoi(argv[2]);
	const uint32_t n = m;
    const std::string filename_out = filename_append_before_extension(filename_append_before_extension(filename_in, argv[2]), "bsp");


    TriMesh mesh;
	if (!load_mesh(mesh, filename_in))
	{
		std::cout << "Could not read " << filename_in << std::endl;
		return EXIT_FAILURE;
	}

    std::vector<decimal_t> x(mesh.n_vertices(), 0);
    std::vector<decimal_t> y(mesh.n_vertices(), 0);
    std::vector<decimal_t> z(mesh.n_vertices(), 0);
    mesh_to_vecs(mesh, x, y, z);
    //mesh_uv_to_vecs(mesh, x, y, z);

    surface::bspline_t<decimal_t> surf = { x.data(), y.data(), z.data(), mesh.n_vertices(), m, n };
    surf.compute();
    std::cout << "Error: " << surf.compute_error() << std::endl;


#if 0
    for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi, index++)
    {
        const auto& uv = mesh.texcoord2D(*vi);
        auto point = mesh.point(*vi);
        //point[2] = surf(uv[0], uv[1]);
        point[2] = surf(point[0], point[1]);
        mesh.set_point(*vi, point);
    }
#else
    for (auto index = 0; index < mesh.n_vertices(); ++index)
    {
        // if (index % 16 == 0)
        //     std::cout << std::endl;

        TriMesh::VertexHandle vi = mesh.vertex_handle(index);
        //auto uv = mesh.texcoord2D(vi);
        auto point = mesh.point(vi);
        point[2] = surf(point[0], point[1]);

        // compute 4 neighbors
        // xyz_uv[4]
        
        //std::cout << index << '\t' << std::fixed << point[0] << ' ' << point[1] << '\t' << x << ' ' << y << std::endl;

        mesh.set_point(vi, point);
    }

    // for (uint32_t i = 0; i < m + 3; ++i )
    // {
    //     for (uint32_t j = 0; j < n + 3; ++j )
    //     {
    //         auto& pt = surf.xyzuv[i][j];
    //         std::cout << std::fixed 
    //         << '[' << pt.x << ' ' << pt.y << ' ' 
    //         << surf(pt.x, pt.y)
    //         //<< pt.z 
    //         //<< "]\t[" << pt.u << ' ' << pt.v 
    //         //<< "]\t[" << pt.s << ' ' << pt.t 
    //         << "]\n";
    //     }
    //     std::cout << std::endl;
    // }

    // {
    //     std::ofstream out(filename_out, std::ios::out);
    //     for (uint32_t i = 0; i < m + 3; ++i )
    //     {
    //         for (uint32_t j = 0; j < n + 3; ++j )
    //         {
    //             auto& pt = surf.xyzuv[i][j];
    //             out << std::fixed 
    //             << "v "
    //             << pt.s << ' ' << pt.t << ' ' 
    //             //<< surf(pt.x, pt.y)
    //             << '0' 
    //             //<< "]\t[" << pt.u << ' ' << pt.v 
    //             //<< "]\t[" << pt.s << ' ' << pt.t 
    //             << '\n';
    //         }
    //         //std::cout << std::endl;
    //     }
    // }

#endif


    if (!save_mesh(mesh, filename_out))
	{
		std::cout << "Could not save " << filename_out << std::endl;
		return EXIT_FAILURE;
	}

    return EXIT_SUCCESS;
}

