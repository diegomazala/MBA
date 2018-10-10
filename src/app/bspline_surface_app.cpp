#include "bspline_surface.h"
#include "mesh_utils.h"
#include "bezier.h"
#include "timer.h"
#include <flann/flann.hpp>
#include <fstream>
#include <iostream>
#include <memory>

template <typename decimal_t>
void create_3d_control_lattice(TriMesh &grid, int m, int n, int dim,
                               int neighbours_count, int knn_search_checks,
                               const flann::Index<flann::L2<decimal_t>> &index,
                               TriMesh &mesh)
{
    create_grid_mesh(grid, m, n);

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
        const TriMesh::VertexHandle vi_closest_in_mesh = mesh.vertex_handle(index_mesh);
        const TriMesh::VertexHandle vi_grid = grid.vertex_handle(index_grid);

        auto point_grid = grid.point(vi_grid);
        auto point_mesh = mesh.point(vi_closest_in_mesh);

        //const auto uv = grid.texcoord2D(vi_grid);
        // if (uv[0] >= surf.umin && uv[0] <= surf.umax &&
        //     uv[1] >= surf.vmin && uv[1] <= surf.vmax)
        {
            //point_grid[2] = surf(uv[0], uv[1]);
            //point_grid[2] = point_mesh[2];
            point_grid = point_mesh;
            grid.set_point(vi_grid, point_grid);
        }
    }
    tm_update_grid.stop();

    std::cout
        << std::fixed << "[Times in seconds]  \n"
        << "KdTree Query     : " << tm_kdtree_build_query.diff_sec() << '\n'
        << "KdTree Search    : " << tm_kdtree_search.diff_sec() << '\n'
        << "Update Ctrl Pts  : " << tm_update_grid.diff_sec() << '\n';
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
    std::vector<decimal_t> u_array(mesh.n_vertices(), 0);
    std::vector<decimal_t> v_array(mesh.n_vertices(), 0);
    std::vector<decimal_t> uv_array(mesh.n_vertices() * 2, 0);
    // mesh_to_vecs(mesh, x, y, z);
    mesh_uv_to_vecs(mesh, x, y, z, uv_array, u_array, v_array);
    tm_copy_data_arrays.stop();

#if 1
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
    timer tm_build_control_lattice;
    TriMesh grid;
    create_3d_control_lattice<decimal_t>(grid, m + 3 - 1, n + 3 - 1, dimension,
                                         neighbours_count, knn_search_checks,
                                         index, mesh);
    tm_build_control_lattice.stop();

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
#endif

    
    timer tm_surf_compute;
    // surface::bspline_t<decimal_t> surf_x = {u_array.data(), v_array.data(), x.data(),
    //                                       mesh.n_vertices(), m, n};
    // surface::bspline_t<decimal_t> surf_y = {u_array.data(), v_array.data(), y.data(),
    //                                       mesh.n_vertices(), m, n};                                       
    surface::bspline_t<decimal_t> surf_z = {u_array.data(), v_array.data(), z.data(),
                                          mesh.n_vertices(), m, n};
    //
    // initialize phi matrix
    //
    for (auto i = 0; i < m + 3; ++i)
    {
        for (auto j = 0; j < n + 3; ++j)
        {
            auto grid_ind = i * (m + 3) + j;
            //surf_x.phi[i][j] = grid.point(grid.vertex_handle(grid_ind))[0];
            //surf_y.phi[i][j] = grid.point(grid.vertex_handle(grid_ind))[1];
            surf_z.phi[i][j] = grid.point(grid.vertex_handle(grid_ind))[2];
        }
    }
    // surf_x.average_z = 0;
    // surf_y.average_z = 0;
    // surf_z.average_z = 0;
    //surf_x.compute();
    //surf_y.compute();
    surf_z.compute();
    tm_surf_compute.stop();
    //std::cout << "Error: " << surf_x.compute_error() << std::endl;
    //std::cout << "Error: " << surf_y.compute_error() << std::endl;
    std::cout << "Error: " << surf_z.compute_error() << std::endl;

    //
    // For each vertex, compute the surface value at uv
    // and interpolate (x,y)
    //
    timer tm_update_vertices;
    //const decimal_t interval_normalization_factor_u = m * surf.urange_inv;
    //const decimal_t interval_normalization_factor_v = n * surf.vrange_inv;
    for (auto index = 0; index < mesh.n_vertices(); ++index)
    {
        TriMesh::VertexHandle vi = mesh.vertex_handle(index);
        const auto uv = mesh.texcoord2D(vi);
        auto point = mesh.point(vi);

        //point[0] = surf_x(uv[0], uv[1]);
        //point[1] = surf_y(uv[0], uv[1]);
        point[2] = surf_z(uv[0], uv[1]);

#if 0 // BEZIER INTERPOLATION

        // Map to the half open domain Omega = [0,m) x [0,n)
        // The mapped u and v must be (strictly) less than m and n respectively
        decimal_t u = (uv[0] - surf.umin) * interval_normalization_factor_u;
        decimal_t v = (uv[1] - surf.vmin) * interval_normalization_factor_v;
        //
        // compute 4x4 neighborhood position
        //auto [i, j, s, t] = surf.compute_ijst(uv[0], uv[1]);
        auto [i, j, s, t] = surf.compute_ijst(u, v);
        //
        // interpolate (x,y)
        //
        //std::cout << std::fixed << '[' << i << ',' << j << "] [" << index << "]  v: [" << point << "] uv [" << uv << ']' << std::endl;
        for (auto pi = 0; pi < 2; ++pi)
        {
            decimal_t p[4][4];
            for (auto k = 0; k < 4; ++k)
            {
                for (auto l = 0; l < 4; ++l)
                {
                    //const auto grid_ind = (i + k) * (m + 3) + (j + l);
                    const auto grid_ind = (j + l) * (m + 3) + (i + k);
                    p[k][l] = grid.point(grid.vertex_handle(grid_ind))[pi];
                    //std::cout << p[k][l] << ' ';
                }
                //std::cout << std::endl;
            }
            //std::cout << "uv " << uv << "  bezier: " << surface::bezier::cubic<decimal_t>(uv[0], uv[1], p) << "\n\n";
            point[pi] = surface::bezier::cubic<decimal_t>(uv[0], uv[1], p);
        }
#endif
        mesh.set_point(vi, point);
    }
    tm_update_vertices.stop();

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
        << "Copying Arrays   : " << tm_copy_data_arrays.diff_sec() << '\n'
        << "Surface Computing: " << tm_surf_compute.diff_sec() << '\n'
        //        << "Building KdTree  : " << tm_kdtree_build.diff_sec() << '\n'
        //        << "Building Ctrl Pts: " << tm_build_control_lattice.diff_sec() << '\n'
        << "Update Vertices  : " << tm_update_vertices.diff_sec() << '\n'
        //        << "Saving Ctrl Pts  : " << tm_save_control_lattice.diff_sec() << '\n'
        << "Saving Mesh      : " << tm_save_mesh.diff_sec() << '\n'
        << std::endl;

    return EXIT_SUCCESS;
}
