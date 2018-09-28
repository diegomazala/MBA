#include "mesh_utils.h"

#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

template <typename T>
constexpr bool logically_equal(const T &a, const T &b, const T error_factor = 1)
{
    return a == b || std::abs(a - b) < std::abs(std::min(a, b)) * std::numeric_limits<T>::epsilon() * error_factor;
}

void eigen_test()
{
    // https://en.wikipedia.org/wiki/B-spline

    constexpr auto Dim = 2;
    constexpr auto Deg = 3; // Curve Order = Deg + 1
    using Spline2d = Eigen::Spline<double, Dim, Deg>;

    Eigen::RowVectorXd knots(11);
    knots << -2, -2, -2, -2, -1, 0, 1, 2, 2, 2, 2;

    Eigen::MatrixXd ctrls(2, 7);
    ctrls << 0, 0, 0, 6, 0, 0, 0,
        0, 0, 0, 6, 0, 0, 0;

    assert(knots.cols() == ctrls.cols() + Deg + 1);

    auto spline = Spline2d(knots, ctrls);

    Eigen::RowVectorXd u(9);
    u << -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2.0;

    for (int i = 0; i < u.size(); ++i)
    {
        auto f = spline(u(i));
        std::cout << f.transpose() << std::endl;
    }
}

void eigen_test_2d()
{
    constexpr auto Dim = 2;
    constexpr auto Deg = 3; // Curve Order = Deg + 1
    using Spline2d = Eigen::Spline<double, Dim, Deg>;
    using PointType = Spline2d::PointType;
    using KnotVectorType = Spline2d::KnotVectorType;
    using ControlPointVectorType = Spline2d::ControlPointVectorType;

    ControlPointVectorType points = ControlPointVectorType::Random(Dim, 10);
    // Eigen::MatrixXd points(2, 7);
    // points << 0, 0, 0, 6, 0, 0, 0,
    //     0, 0, 0, 6, 0, 0, 0;

    KnotVectorType chord_lengths; // knot parameters
    Eigen::ChordLengths(points, chord_lengths);

    std::cout
        << chord_lengths.rows() << ' ' << chord_lengths.cols() << std::endl
        << points.rows() << ' ' << points.cols() << std::endl
        << std::endl;

    // interpolation without knot parameters
    {
        const Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points, Deg);

        for (Eigen::DenseIndex i = 0; i < points.cols(); ++i)
        {
            PointType pt = spline(chord_lengths(i));
            PointType ref = points.col(i);

            std::cout << std::fixed << (pt - ref).matrix().norm() << '\t' << pt.transpose() << '\t' << ref.transpose() << '\n';
            //VERIFY((pt - ref).matrix().norm() < 1e-14);
        }
    }

    std::cout << '\n';

    // interpolation with given knot parameters
    {
        const Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points, Deg, chord_lengths);

        for (Eigen::DenseIndex i = 0; i < points.cols(); ++i)
        {
            PointType pt = spline(chord_lengths(i));
            PointType ref = points.col(i);

            std::cout << std::fixed << (pt - ref).matrix().norm() << '\t' << pt.transpose() << '\t' << ref.transpose() << '\n';

            //VERIFY((pt - ref).matrix().norm() < 1e-14);
        }
    }
}

void eigen_test_3d()
{
    constexpr auto Dim = 3;
    constexpr auto Deg = 3; // Curve Order = Deg + 1
    using Spline3d = Eigen::Spline<double, Dim, Deg>;
    using PointType = Spline3d::PointType;
    using KnotVectorType = Spline3d::KnotVectorType;
    using ControlPointVectorType = Spline3d::ControlPointVectorType;

    ControlPointVectorType points = ControlPointVectorType::Random(Dim, 10);

    KnotVectorType chord_lengths; // knot parameters
    Eigen::ChordLengths(points, chord_lengths);

    std::cout
        << chord_lengths.rows() << ' ' << chord_lengths.cols() << std::endl
        << points.rows() << ' ' << points.cols() << std::endl
        << std::endl;

    for (auto i = 0; i < chord_lengths.cols(); ++i)
        std::cout << chord_lengths(0, i) << ' ';

    std::cout << std::endl
              << std::endl;

    // interpolation without knot parameters
    {
        const Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, Deg);

        for (Eigen::DenseIndex i = 0; i < points.cols(); ++i)
        {
            PointType pt = spline(chord_lengths(i));
            PointType ref = points.col(i);

            std::cout << std::fixed << (pt - ref).matrix().norm() << '\t' << pt.transpose() << '\t' << ref.transpose() << '\n';
            //VERIFY((pt - ref).matrix().norm() < 1e-14);
        }
    }

    std::cout << '\n';
}

//template <typename decimal_t, int dim, int deg>
void eigen_test_openmesh(Eigen::Matrix<double, 3, Eigen::Dynamic> &points)
{
    constexpr auto Dim = 3;
    constexpr auto Deg = 3; // Curve Order = Deg + 1
    using Spline3d = Eigen::Spline<double, Dim, Deg>;
    using PointType = Spline3d::PointType;
    using KnotVectorType = Spline3d::KnotVectorType;
    using ControlPointVectorType = Spline3d::ControlPointVectorType;

    //ControlPointVectorType points = ControlPointVectorType::Random(Dim, 10);

    KnotVectorType chord_lengths; // knot parameters
    Eigen::ChordLengths(points, chord_lengths);

    std::cout
        << chord_lengths.rows() << ' ' << chord_lengths.cols() << std::endl
        << points.rows() << ' ' << points.cols() << std::endl
        << std::endl;

    for (auto i = 0; i < chord_lengths.cols(); ++i)
        std::cout << chord_lengths(0, i) << ' ';

    // std::cout << std::endl
    //           << std::endl;

    float error = 0;
    Eigen::DenseIndex i = 0;

    // interpolation without knot parameters
    const Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(points, Deg);
#if 1
    {
        for (i = 0; i < points.cols(); ++i)
        {
            PointType pt = spline(chord_lengths(i));
            PointType ref = points.col(i);

            //std::cout << std::fixed << (pt - ref).matrix().norm() << '\t' << pt.transpose() << '\t' << ref.transpose() << '\n';
            //VERIFY((pt - ref).matrix().norm() < 1e-14);
            error += (pt - ref).matrix().norm();
        }
    }
    std::cout
        << std::fixed
        << "error: " << error << std::endl
        << std::endl;
#endif
}

template <typename decimal_t, int dim>
void eigen_test_openmesh_2(Eigen::Matrix<decimal_t, dim, Eigen::Dynamic> &points)
{
    constexpr auto Size = 5;
    constexpr auto Dim1 = 1;
    constexpr auto Dim2 = 2;
    constexpr auto Deg = 3;
    using Spline1d = Eigen::Spline<decimal_t, Dim1, Deg>;

    // Eigen::Matrix<decimal_t, Dim1, Size> x(Size), y(Size);
    // x << 0.0, 0.25, 0.5, 0.75, 1.0;
    // y << 0.0, 2.50, 5.0, 7.50, 10.0;

    auto spline_fit_xz = Eigen::SplineFitting<Spline1d>::Interpolate(points.row(2), Deg, points.row(0));
    auto spline_fit_xy = Eigen::SplineFitting<Spline1d>::Interpolate(points.row(1), Deg, points.row(0));
    //auto spline_fit_1d = Eigen::SplineFitting<Spline1d<Scalar>>::Interpolate(y, Deg);
    Spline1d spline_xz(spline_fit_xz);
    Spline1d spline_xy(spline_fit_xy);
}

template <typename decimal_t, int dim>
static void mesh_to_eigen_data(const TriMesh &mesh, Eigen::Matrix<decimal_t, dim, Eigen::Dynamic> &points)
{
    points = {dim, mesh.n_vertices()};

    std::cout << points.rows() << ' ' << points.cols() << std::endl;

    std::size_t i = 0;
    for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi, ++i)
    {
        auto point = mesh.point(*vi);
        points(0, i) = point[0];
        points(1, i) = point[1];
        points(2, i) = point[2];
    }
}

int main(int argc, char *argv[])
{
    //eigen_test();
    //eigen_test_2d();
    //eigen_test_3d();

    if (argc != 3)
    {
        std::cout << "Usage: ./mba_eigen ~/data/mba/face.obj 4 \n";
        return EXIT_FAILURE;
    }

    using decimal_t = double;
    constexpr int Dim = 3;
    constexpr int Deg = 3;         // Curve Order = Deg + 1
    constexpr int Order = Deg + 1; // Curve Order = Deg + 1

    const std::string filename_in = argv[1];
    const std::string filename_out = filename_append_before_extension(filename_append_before_extension(filename_in, "eigen"), argv[2]);
    const int ctrl_points_count = atoi(argv[2]);

    TriMesh mesh;
    if (!load_mesh(mesh, filename_in))
    {
        std::cout << "Could not read " << filename_in << std::endl;
        return EXIT_FAILURE;
    }

    normalize_mesh(mesh);

    using Spline3d = Eigen::Spline<double, Dim, Deg>;
    using PointType = Spline3d::PointType;
    using KnotVectorType = Spline3d::KnotVectorType;
    using ControlPointVectorType = Spline3d::ControlPointVectorType;

    Eigen::Matrix<decimal_t, Dim, Eigen::Dynamic> points;
    mesh_to_eigen_data<decimal_t, Dim>(mesh, points);

    {
        Eigen::Matrix<decimal_t, Dim, Eigen::Dynamic> ctrl_pts = {Dim, ctrl_points_count * ctrl_points_count};

        auto big_step = static_cast<int>(static_cast<double>(points.cols()) / static_cast<float>(ctrl_points_count));
        auto small_step = static_cast<int>(static_cast<double>(points.cols()) / static_cast<float>(ctrl_points_count * ctrl_points_count));

        //std::cout << "step: " << big_step << '\t' << small_step << std::endl;

        std::size_t n = 0;
        for (auto i = 0; i < ctrl_points_count; ++i)
        {
            //for (auto j = 0; j < ctrl_points_count; ++j)
            {
                //auto idx = i * big_step + j * small_step;
                auto idx = i * big_step;
                auto pt = mesh.point(mesh.vertex_handle(idx));
                ctrl_pts(0, n) = pt[0];
                ctrl_pts(1, n) = pt[1];
                ctrl_pts(2, n) = pt[2];

                //std::cout << n << '\t' << idx << '\t' << ctrl_pts.col(n).transpose() << std::endl;
                ++n;
            }
        }

        //std::cout << "ctrl_pts: \n" << ctrl_pts << std::endl << std::endl;

        KnotVectorType chord_lengths; // knot parameters
        Eigen::ChordLengths(ctrl_pts, chord_lengths);

        KnotVectorType knots(ctrl_points_count + Order);
        for (auto i = 0; i < Order; ++i)
        {
            knots(i) = 0;
            knots(knots.size() - i - 1) = 1;
        }
        std::cout << "knots: " << knots << std::endl
                  << std::endl;
        auto in_between_count = knots.size() - Order * 2 + 1;
        auto in_between_step = 1.0 / in_between_count;
        std::cout << in_between_count << ' ' << in_between_step << std::endl;
        for (auto i = Order; i < Order + in_between_count; ++i)
            knots(i) = knots(i - 1) + in_between_step;

        std::cout << "knots: " << knots << std::endl
                  << std::endl;

        std::cout
            << "chord_lenghts: " << chord_lengths.rows() << ' ' << chord_lengths.cols() << std::endl
            << "control_pts  : " << ctrl_pts.rows() << ' ' << ctrl_pts.cols() << std::endl
            << std::endl;

        // interpolation without knot parameters
        const Spline3d spline = Eigen::SplineFitting<Spline3d>::Interpolate(ctrl_pts, Deg);


        for (auto vi = mesh.vertices_begin(); vi != mesh.vertices_end(); ++vi)
        {
            //auto uv = mesh.texcoord2D(*vi);
            auto pt = mesh.point(*vi);
            auto fx = spline(pt[0] / 1.0);
            auto fz = spline(pt[2] / 1.0);

            //point[0] = fx.x();
            //point[1] = fx.x();
            //point[2] = fx.z();


            //std::cout << fx.transpose() << std::endl;

            std::cout << std::fixed << pt[0] / 1.0 << '\t' << fx[2] << '\t' << fz.transpose() << '\t' << fx.transpose() << std::endl;

            pt[1] = fx[2];

            mesh.set_point(*vi, pt);
        }
    }

    if (!save_mesh(mesh, filename_out))
    {
        std::cout << "Could not save " << filename_out << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
