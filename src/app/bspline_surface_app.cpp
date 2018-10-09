#include "bspline_surface.h"
#include "mesh_utils.h"
#include "timer.h"
#include <flann/flann.hpp>
#include <fstream>
#include <iostream>
#include <memory>

template <typename decimal_t>
void create_3d_control_lattice(TriMesh &grid, int m, int n, int dim,
                               int neighbours_count, int knn_search_checks,
                               const flann::Index<flann::L2<decimal_t>> &index,
                               const surface::bspline_t<decimal_t>& surf)
{
    create_grid_mesh(grid, m + 3 - 1, n + 3 - 1);

    //
    // build uv query array
    //
    timer tm_kdtree_build_query;
    std::vector<decimal_t> uv_query(grid.n_vertices() * dim);
    size_t uv_index = 0;
    for (auto vi = grid.vertices_begin(); vi != grid.vertices_end(); ++vi)
    {
        uv_query[uv_index++] = grid.texcoord2D(*vi)[0];
        uv_query[uv_index++] = grid.texcoord2D(*vi)[1];
    }

    //
    // build uv query flann
    //
    flann::Matrix<decimal_t> query(uv_query.data(), uv_query.size() / dim, dim);
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
    index.knnSearch(
        query, indices, dists, neighbours_count,
        flann::SearchParams(knn_search_checks)); // flann::SearchParams(128));
    tm_kdtree_search.stop();

    //
    // update grid vertices
    //
    timer tm_update_grid;
    for (auto i = 0; i < indices.rows; ++i)
    {
        const auto index_grid = i;
        const auto index_mesh = indices[i][0];
        const TriMesh::VertexHandle vi_grid = grid.vertex_handle(index_grid);
        auto uv = grid.texcoord2D(vi_grid);
        auto point = grid.point(vi_grid);
        point[2] = surf(uv[0], uv[1]);
        grid.set_point(vi_grid, point);
    }
    tm_update_grid.stop();
}

int main(int argc, char *argv[])
{
    using decimal_t = double;

    if (argc < 3)
    {
        std::cout << "Usage: app <mesh_file> < m_n >\n"
                  << "Usage: app ../data/face.obj 3 \n";
        return EXIT_FAILURE;
    }

    const std::string filename_in = argv[1];
    const uint32_t m = atoi(argv[2]);
    const uint32_t n = m;
    const std::string filename_out = filename_append_before_extension(
        filename_append_before_extension(filename_in, argv[2]), "bsp");

    TriMesh mesh;
    timer tm_load_mesh;
    if (!load_mesh(mesh, filename_in))
    {
        std::cout << "Could not read " << filename_in << std::endl;
        return EXIT_FAILURE;
    }
    tm_load_mesh.stop();

    timer tm_copy_data_arrays;
    std::vector<decimal_t> x(mesh.n_vertices(), 0);
    std::vector<decimal_t> y(mesh.n_vertices(), 0);
    std::vector<decimal_t> z(mesh.n_vertices(), 0);
    std::vector<decimal_t> uv_array(mesh.n_vertices() * 2, 0);
    // mesh_to_vecs(mesh, x, y, z);
    mesh_uv_to_vecs(mesh, x, y, z, uv_array);
    tm_copy_data_arrays.stop();

    timer tm_surf_compute;
    surface::bspline_t<decimal_t> surf = {x.data(), y.data(), z.data(),
                                          mesh.n_vertices(), m, n};
    surf.compute();
    tm_surf_compute.stop();
    std::cout << "Error: " << surf.compute_error() << std::endl;

    //
    // build knn
    //
    constexpr int dimension = 2;
    const int neighbours_count = (argc > 3) ? atoi(argv[3]) : 16;
    const int kdtree_count = (argc > 4) ? atoi(argv[4]) : 4;
    const int knn_search_checks = (argc > 5) ? atoi(argv[5]) : 128;
    const size_t num_input = mesh.n_vertices();
    const size_t num_query = num_input;
    //
    std::cout << "-- Build kdtree" << std::endl;
    timer tm_kdtree_build;
    // construct dataset
    flann::Matrix<decimal_t> dataset(uv_array.data(), num_input, dimension);
    // construct an randomized kd-tree index using 4 kd-trees
    flann::Index<flann::L2<decimal_t>> index(
        dataset, flann::KDTreeIndexParams(kdtree_count));
    index.buildIndex();
    tm_kdtree_build.stop();

    //
    // build control lattice grid
    //
    TriMesh grid;
    create_3d_control_lattice<decimal_t>(grid, m + 3 - 1, n + 3 - 1, dimension,
                                         neighbours_count, knn_search_checks,
                                         index, surf);

    timer tm_save_control_lattice;
    {
        const std::string filename_pts = filename_append_before_extension(
        filename_append_before_extension(filename_in, argv[2]), "pts");
        if (!save_points_obj(grid, filename_pts))
        {
            std::cout << "Could not save control_lattice mesh " << filename_pts << std::endl;
            return EXIT_FAILURE;
        }
    }
    tm_save_control_lattice.stop();


    // e agora???



    for (auto index = 0; index < mesh.n_vertices(); ++index)
    {
        TriMesh::VertexHandle vi = mesh.vertex_handle(index);
        auto uv = mesh.texcoord2D(vi);
        auto point = mesh.point(vi);
        point[2] = surf(uv[0], uv[1]);

        mesh.set_point(vi, point);
    }

    timer tm_save_mesh;
    if (!save_mesh(mesh, filename_out))
    {
        std::cout << "Could not save " << filename_out << std::endl;
        return EXIT_FAILURE;
    }
    tm_save_mesh.stop();

    std::cout
        << std::fixed << "[Times in seconds]  \n"
        << "Loading Mesh     : " << tm_load_mesh.diff_sec() << '\n'
        << "Copying arrays   : " << tm_copy_data_arrays.diff_sec() << '\n'
        << "Surface Computing: " << tm_surf_compute.diff_sec() << '\n'
        << "Building KdTree  : " << tm_kdtree_build.diff_sec() << '\n'
        //<< "Building Query   : " << tm_kdtree_build_query.diff_sec() << '\n'
        //<< "KdTree Search    : " << tm_kdtree_search.diff_sec() << '\n'
        //<< "Update Grid      : " << tm_update_grid.diff_sec() << '\n'
        << "Saving Mesh      : " << tm_save_mesh.diff_sec() << '\n'
        << std::endl;

    return EXIT_SUCCESS;
}
