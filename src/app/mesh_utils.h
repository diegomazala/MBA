#pragma once

#include <iostream>

#include <algorithm>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#define _USE_MATH_DEFINES
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

// Define my personal traits
template<typename T>
struct MeshTraits : OpenMesh::DefaultTraits
{
    /// The default coordinate type is OpenMesh::Vec3f.
    using Point = OpenMesh::VectorT<T,3>;

    /// The default normal type is OpenMesh::Vec3f.
    using Normal = OpenMesh::VectorT<T,3>;

    /// The default 1D texture coordinate type is float.
    using TexCoord2D = OpenMesh::VectorT<T,2>;

    /// The default texture index type
    using TextureIndex = uint32_t;

    /// The default color type is OpenMesh::Vec3uc.
    using Color = OpenMesh::Vec3uc;

    #ifndef DOXY_IGNORE_THIS
    VertexTraits    {};
    HalfedgeTraits  {};
    EdgeTraits      {};
    FaceTraits      {};
    #endif

    VertexAttributes(0);
    HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
    EdgeAttributes(0);
    FaceAttributes(0);
};

using TriMesh = OpenMesh::TriMesh_ArrayKernelT<MeshTraits<float>>;
OpenMesh::VPropHandleT<TriMesh::Normal> Normals;
OpenMesh::VPropHandleT<TriMesh::TexCoord2D> TexCoord;

template <typename T>
constexpr T ndc(T v, T min, T max) // normalize to [0, 1]
{
    return (v - min) / (max - min);
}

template <typename T>
static std::string
filename_append_before_extension(const std::string &filename_in,
                                 const T &append, const char separator = '_')
{
    auto ext_pos = filename_in.rfind('.', filename_in.length());
    if (ext_pos == std::string::npos)
        ext_pos = filename_in.length();
    std::stringstream filename_out;
    filename_out << filename_in.substr(0, ext_pos) << separator << append
                 << filename_in.substr(ext_pos, filename_in.length() - ext_pos);
    return filename_out.str();
}

static bool load_mesh(TriMesh &mesh, const std::string &filename)
{
    mesh.request_vertex_texcoords2D();
    mesh.request_vertex_normals();
    try
    {
        std::cout << "-- Loading mesh " << filename << std::endl;
        OpenMesh::IO::Options OptionRead(OpenMesh::IO::Options::VertexTexCoord);
        return OpenMesh::IO::read_mesh(mesh, filename, OptionRead);
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
        return false;
    }
}

static bool save_mesh(TriMesh &mesh, const std::string &filename)
{
    try
    {
        std::cout << "-- Saving mesh " << filename << std::endl;
        mesh.request_vertex_texcoords2D();
        mesh.request_vertex_normals();
        OpenMesh::IO::Options OptionWrite(
            OpenMesh::IO::Options::
                VertexTexCoord); // | OpenMesh::IO::Options::VertexNormal);
        return OpenMesh::IO::write_mesh(mesh, filename, OptionWrite);
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
        return false;
    }
}

static std::tuple<OpenMesh::Vec3f, OpenMesh::Vec3f>
mesh_bounding_box(const TriMesh &mesh)
{
    OpenMesh::Vec3f min_vert = {std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max()};
    OpenMesh::Vec3f max_vert = {std::numeric_limits<float>::min(),
                                std::numeric_limits<float>::min(),
                                std::numeric_limits<float>::min()};

    for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
    {
        auto pt = mesh.point(*vi);
        for (auto i = 0; i < 3; ++i)
        {
            if (pt[i] < min_vert[i])
                min_vert[i] = pt[i];
            if (pt[i] > max_vert[i])
                max_vert[i] = pt[i];
        }
    }
    return std::make_tuple(min_vert, max_vert);
}

static OpenMesh::Vec3f
center_of_bounding_box(const OpenMesh::Vec3f &min_corner,
                       const OpenMesh::Vec3f &max_corner)
{
    return {min_corner[0] + ((max_corner[0] - min_corner[0]) * 0.5f),
            min_corner[1] + ((max_corner[1] - min_corner[1]) * 0.5f),
            min_corner[2] + ((max_corner[2] - min_corner[2]) * 0.5f)};
}

static float max_range_of_bounding_box(const OpenMesh::Vec3f &min_corner,
                                       const OpenMesh::Vec3f &max_corner)
{
    std::vector<float> range = {max_corner[0] - min_corner[0],
                                max_corner[1] - min_corner[1],
                                max_corner[2] - min_corner[2]};
    return *std::max_element(range.begin(), range.end());
}

static void normalize_mesh(TriMesh &mesh, const OpenMesh::Vec3f &center,
                           float scale)
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

static void normalize_mesh(TriMesh &mesh)
{
    auto [min_corner, max_corner] = mesh_bounding_box(mesh);
    auto scale = max_range_of_bounding_box(min_corner, max_corner);
    auto center = center_of_bounding_box(min_corner, max_corner);
    normalize_mesh(mesh, center, scale);
}

static void create_grid_mesh(TriMesh &mesh, uint32_t rows, uint32_t cols,
                             const OpenMesh::Vec3f &up = {0, 1, 0},
                             float width = 1.0f, float height = 1.0f)
{
    std::vector<TriMesh::VertexHandle> vhandle((rows + 1) * (cols + 1));
    std::vector<TriMesh::VertexHandle> fhandle(3);

    for (auto i = 0; i <= rows; ++i)
    {
        for (auto j = 0; j <= cols; ++j)
        {
            auto x = static_cast<float>(j) / cols * width;
            auto z = static_cast<float>(i) / rows * height;
            vhandle[i * (rows + 1) + j] = mesh.add_vertex(TriMesh::Point(x, 0.0f, z));
            // vhandle[i*(rows+1)+j] = mesh.add_vertex(TriMesh::Point(x, z * up[2], z
            // * up[1]));
            // std::cout << i*(rows+1)+j << std::endl;
        }
    }

    rows += 1;
    cols += 1;

    for (auto i = 0; i < rows - 1; ++i)
    {
        for (auto j = 1; j < cols; ++j)
        {
            // std::cout << i * rows + j << ' ' << (i + 1) * rows + j - 1 << ' ' << i
            // * rows + j - 1 << std::endl;
            fhandle[0] = vhandle[i * rows + j];
            fhandle[2] = vhandle[(i + 1) * rows + j - 1];
            fhandle[1] = vhandle[i * rows + j - 1];
            mesh.add_face(fhandle);
            // std::cout << i * rows + j << ' ' << (i + 1) * rows + j << ' ' << (i +
            // 1) * rows + j - 1 << std::endl;
            fhandle[0] = vhandle[i * rows + j];
            fhandle[2] = vhandle[(i + 1) * rows + j];
            fhandle[1] = vhandle[(i + 1) * rows + j - 1];
            mesh.add_face(fhandle);
            // std::cout << std::endl;
        }
    }
}

template <typename decimal>
static void mesh_to_vecs(TriMesh &mesh, std::shared_ptr<std::vector<decimal>> U,
                         std::shared_ptr<std::vector<decimal>> V,
                         std::shared_ptr<std::vector<decimal>> Z)
{
    for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
    {
        auto uv = mesh.texcoord2D(*vi);
        auto point = mesh.point(*vi);

        // if (point[2] > 0.5)
        {
            U->push_back(uv[0]);
            V->push_back(uv[1]);
            Z->push_back(point[2]);
        }
    }
}