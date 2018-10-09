#include <iostream>
#include <flann/flann.hpp>
#include "mesh_utils.h"
#include "bezier.h"
#include <Eigen/Core>
#include "timer.h"


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

    timer tm_load_mesh;
    TriMesh mesh;
	if (!load_mesh(mesh, filename_in))
	{
		std::cout << "Could not read " << filename_in << std::endl;
		return EXIT_FAILURE;
	}
    tm_load_mesh.stop();



    //
    // build knn
    //
    constexpr int dimension = 2;
    const int neighbours_count = (argc > 3) ? atoi(argv[3]) : 16;
	const int kdtree_count = (argc > 4) ? atoi(argv[4]) : 4;
	const int knn_search_checks = (argc > 5) ? atoi(argv[5]) : 128;

    const size_t num_input = mesh.n_vertices();
    const size_t num_query = num_input;

    std::cout << "-- Copying uv array" << std::endl;
    timer tm_copy_array;

    std::vector<decimal_t> uv_array(mesh.n_vertices() * dimension);
    size_t uv_index = 0;
    for (auto index = 0; index < mesh.n_vertices(); ++index)
    {
        const TriMesh::VertexHandle vi = mesh.vertex_handle(index);
        uv_array[uv_index++] = mesh.texcoord2D(vi)[0];
        uv_array[uv_index++] = mesh.texcoord2D(vi)[1];
    }
    tm_copy_array.stop();

    std::cout << "-- Build kdtree" << std::endl;
    timer tm_kdtree_build;
    // construct dataset
	flann::Matrix<decimal_t> dataset(uv_array.data(), num_input, dimension);
    // construct an randomized kd-tree index using 4 kd-trees
	flann::Index<flann::L2<decimal_t> > index(dataset, flann::KDTreeIndexParams(kdtree_count));
	index.buildIndex();
    tm_kdtree_build.stop();

    
    TriMesh grid;
    create_grid_mesh(grid, m - 1, n - 1);


    //
    // build uv query array
    //
    timer tm_kdtree_build_query;
    std::vector<decimal_t> uv_query(grid.n_vertices() * dimension);
    uv_index = 0;
    for (auto vi = grid.vertices_begin(); vi != grid.vertices_end(); ++vi)
    {
        uv_query[uv_index++] = grid.texcoord2D(*vi)[0];
        uv_query[uv_index++] = grid.texcoord2D(*vi)[1];
    }
    //
    // build uv query flann
    //
    flann::Matrix<decimal_t> query(uv_query.data(), uv_query.size() / dimension, dimension);
    std::unique_ptr<int> indices_buffer(new int[query.rows * neighbours_count]);
    flann::Matrix<int> indices(indices_buffer.get(), query.rows, neighbours_count);
    std::unique_ptr<decimal_t> dists_buffer(new decimal_t[query.rows * neighbours_count]);
    flann::Matrix<decimal_t> dists(dists_buffer.get(), query.rows, neighbours_count);
    tm_kdtree_build_query.stop();


    //
    // do a knn search, using 128 checks
    //
    std::cout << "-- Knn search" << std::endl;
    timer tm_kdtree_search;
    index.knnSearch(query, indices, dists, neighbours_count, flann::SearchParams(knn_search_checks));	//flann::SearchParams(128));
    tm_kdtree_search.stop();


    timer tm_update_grid;
    for (auto i = 0; i < indices.rows; ++i)
    {
        const auto index_grid = i;
        const auto index_mesh = indices[i][0];
        const TriMesh::VertexHandle vi_closest_in_mesh = mesh.vertex_handle(index_mesh);
        const TriMesh::VertexHandle vi_grid = grid.vertex_handle(index_grid);
        
#if 0 // INTERPOLATION_BEZIER

        for (auto pi = 0; pi < 3; ++pi)
        {
            decimal_t p[4][4];

            int j = 0;
            for (auto s = 0; s < 4; ++s)
            {
                for (auto t = 0; t < 4; ++t)
                {
                    const auto ind_mesh = indices[i][j];
                    p[s][t] = mesh.point(mesh.vertex_handle(ind_mesh))[pi];
                    ++j;
                }
            }

            const auto& uv_grid = grid.texcoord2D(vi_grid);
            auto point_grid = grid.point(vi_grid);
            point_grid[pi] = surface::bezier::cubic<decimal_t>(uv_grid[0], uv_grid[1], p);
            grid.set_point(vi_grid, point_grid);
        }

#else   // CLOSEST_POINT

        grid.set_point(vi_grid, mesh.point(vi_closest_in_mesh));

#endif        
    }
    tm_update_grid.stop();



    timer tm_save_mesh;
    //if (!save_mesh(grid, filename_out))
    if (!save_points_obj(grid, filename_out))
	{
		std::cout << "Could not save " << filename_out << std::endl;
		return EXIT_FAILURE;
	}
    tm_save_mesh.stop();
    



    std::cout << std::fixed
		<< "[Times in seconds]  \n"
		<< "Loading Mesh     : " << tm_load_mesh.diff_sec() << '\n'
		<< "Copying uv array : " << tm_copy_array.diff_sec() << '\n'
		<< "Building KdTree  : " << tm_kdtree_build.diff_sec() << '\n'
        << "Building Query   : " << tm_kdtree_build_query.diff_sec() << '\n'
		<< "KdTree Search    : " << tm_kdtree_search.diff_sec() << '\n'
        << "Update Grid      : " << tm_update_grid.diff_sec() << '\n'
		<< "Saving Mesh      : " << tm_save_mesh.diff_sec() << '\n'
		<< std::endl;

    return EXIT_SUCCESS;
}