#include <iostream>
#include "bspline_approximation.h"
#include "mesh_utils.h"


int main(int argc, char* argv[])
{
    using decimal_t = float;

    if (argc < 2)
	{
		std::cout 
			<< "Usage: app <mesh_file> < m_n >\n"
			<< "Usage: app ../data/face.obj 3 \n";
		return EXIT_FAILURE;
	}

    const std::string filename_in = argv[1];
    const std::string filename_out = filename_append_before_extension(filename_in, "_bsp");


    TriMesh mesh;
	if (!load_mesh(mesh, filename_in))
	{
		std::cout << "Could not read " << filename_in << std::endl;
		return EXIT_FAILURE;
	}


    // const TriMesh::Point* ptr = mesh.points();
    // for (auto i = 0; i < mesh.n_vertices(); ++i)
    // {
    //     std::cout << *ptr << std::endl;
    //     ptr = ptr + 1;
    // }
    // exit(0);




    bspline_approx<decimal_t, TriMesh::Point> bspline = { mesh.points(), mesh.n_vertices(), 1, 1 };
    bspline.init();

    std::cout << std::fixed << "Domain: " 
        << bspline.umin << ' ' << bspline.vmin << ' ' 
        << bspline.umax << ' ' << bspline.vmax << ' '
        << bspline.urange_inv << ' ' << bspline.vrange_inv << std::endl;

    std::cout << "offset: " << bspline.average_z << std::endl;

    exit(0);

    bspline.compute();
    

    std::cout << std::fixed << "delta\n";
    for (const auto& v1 : bspline.delta)
    {
        for (const auto& v2 : v1)
        {
            std::cout << v2 << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << "omega\n";
    for (const auto& v1 : bspline.omega)
    {
        for (const auto& v2 : v1)
        {
            std::cout << v2 << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << "phi\n";
    for (const auto& v1 : bspline.phi)
    {
        for (const auto& v2 : v1)
        {
            std::cout << v2 << ' ';
        }
        std::cout << std::endl;
    }

    exit(0);


    for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
    {
        auto point = mesh.point(*vi);

        point[2] = bspline.eval(point[0], point[1]);

        mesh.set_point(*vi, point);
    }

    if (!save_mesh(mesh, filename_out))
	{
		std::cout << "Could not save " << filename_out << std::endl;
		return EXIT_FAILURE;
	}

    return EXIT_SUCCESS;
}
