#pragma once
#if _USE_EIGEN_
#include <Eigen/Core>
#endif
#include <cmath>
#include <functional>
#include <memory>
#include <vector>

//
// B-splines functions
//
template <typename decimal_t>
static std::function<decimal_t(decimal_t)> B[4] = {
    [](decimal_t t) { return std::pow((1 - t), 3) / 6; },
    [](decimal_t t) {
        return (3 * std::pow(t, 3) - 6 * std::pow(t, 2) + 4) / 6;
    },
    [](decimal_t t) {
        return (-3 * std::pow(t, 3) + 3 * std::pow(t, 2) + 3 * t + 1) / 6;
    },
    [](decimal_t t) { return std::pow(t, 3) / 6; }};

//
// Function to test doubles and floats considering decimals
//
template <typename T>
constexpr bool logically_equal(const T &a, const T &b,
                               const T error_factor = 1)
{
    return a == b ||
           std::abs(a - b) < std::abs(std::min(a, b)) *
                                 std::numeric_limits<T>::epsilon() * error_factor;
}

template <typename decimal_t, typename vec3_t>
class bspline_approx
{
  public:
#if _USE_EIGEN_
    using matrix_t = Eigen::Matrix<decimal_t, Eigen::Dynamic, Eigen::Dynamic>;
    using matrix4_t = Eigen::Matrix<decimal_t, 4, 4>;
    using vec3_t = Eigen::Matrix<decimal_t, 3, 1>;
    using vec2_t = Eigen::Matrix<decimal_t, 2, 1>;
#else
    using matrix_t = std::vector<std::vector<decimal_t>>;
    using matrix4_t = std::array<std::array<decimal_t, 4>, 4>;
    // using vec3_t = std::array<decimal_t, 3>;
    using vec2_t = std::array<decimal_t, 2>;
#endif
    bspline_approx(const vec3_t* _points, size_t _point_count, uint32_t _m, uint32_t _n)
        : points(_points), point_count(_point_count), m(_n), n(_n)
    {
    }

    bspline_approx() = default;

    // private:
    decimal_t umin, vmin, umax, vmax, urange_inv, vrange_inv, average_z;
    uint32_t m = 1;
    uint32_t n = 1;
    matrix_t phi;
    matrix_t delta;
    matrix_t omega;
    const size_t point_count;
    const vec3_t* points;
    std::vector<decimal_t> z_surface;

    //
    // Compute [i,j] indices and [s,t] params
    //
    std::tuple<int64_t, int64_t, decimal_t, decimal_t> compute_ijst(decimal_t x,
                                                                    decimal_t y)
    {
#if _USING_EIGEN_
        vec2_t floor = {std::floor(x), std::floor(y)};
        iint64_t i = floor.x() - 1;
        iint64_t j = floor.y() - 1;
        auto s = x - floor.x();
        auto t = y - floor.y();
#else
        vec2_t floor = {std::floor(x), std::floor(y)};
        int64_t i = floor[0] - 1;
        int64_t j = floor[1] - 1;
        decimal_t s = x - floor[0];
        decimal_t t = y - floor[1];
#endif

        // test if with uv bigger (0.1), this is still needed
        if (i == m - 1)
        {
            i--;
            s = 1;
        }
        if (j == n - 1)
        {
            j--;
            t = 1;
        }

        return {i, j, s, t};
        // return std::make_tuple<int64_t, int64_t, decimal_t, decimal_t>(i, j, s,
        // t);
    }

    //
    // Compute w_kl's and sum_sum w²_ab
    //
    std::tuple<matrix4_t, decimal_t>
    compute_wkl_and_sum_w_ab2(decimal_t s, decimal_t t) const
    {
        matrix4_t w = {0, 0};
        decimal_t sum_w_ab2 = 0;
        for (auto k = 0; k < 4; ++k)
        {
            for (auto l = 0; l < 4; ++l)
            {
#if _USING_EIGEN_
                w(k, l) = B<decimal_t>[k](s) * B<decimal_t>[l](t);
                sum_w_ab2 += w(k, l) * w(k, l);
#else
                w[k][l] = B<decimal_t>[k](s) * B<decimal_t>[l](t);
                sum_w_ab2 += w[k][l] * w[k][l];
#endif
            }
        }
        return {w, sum_w_ab2};
        // return std::make_tuple(w, sum_w_ab2);
    }

    void compute()
    {
#if _USE_EIGEN_
        delta.setZero();
        omega.setZero();
        phi.setZero();
#else
        this->delta = {(m + 3), std::vector<decimal_t>(n + 3, 0)};
        this->omega = {(m + 3), std::vector<decimal_t>(n + 3, 0)};
        this->phi   = {(m + 3), std::vector<decimal_t>(n + 3, 0)};
#endif

        auto points_ptr = points;
        for (auto index = 0; index < point_count; ++index, points_ptr++)
        {
            const decimal_t x = (*points_ptr)[0];
            const decimal_t y = (*points_ptr)[1];
            const decimal_t z = z_surface[index]; //p[2];   --> z_surface is offseted with average_z

            //
            // Compute [i,j] indices and [s,t] params
            //
            auto [i, j, s, t] = compute_ijst(x, y);

            //
            // Compute w_kl's and sum_sum w²_ab
            //
            auto [w, sum_w_ab2] = compute_wkl_and_sum_w_ab2(s, t);
            auto sum_w_ab2_inv = 1 / sum_w_ab2;

            //
            // Update delta and gamma matrices
            //
            for (auto k = 0; k < 4; ++k)
            {
                for (auto l = 0; l < 4; ++l)
                {

#if _USE_EIGEN_
                    // eq(3)
                    auto phi_kl = w(k, l) * z_c / sum_w_ab2_inv;
                    auto w2 = w(k, l) * w(k, l);
                    this->delta(i + k, j + l) = w2 * phi_kl;
                    this->omega(i + k, j + l) = w2;
#else
                    // eq(3)
                    auto phi_kl = w[k][l] * z / sum_w_ab2_inv;
                    auto w2 = w[k][l] * w[k][l];

                    auto ik = i + k + 1;    // +1 to go for positive indices
                    auto jl = j + l + 1;

                    this->delta[ik][jl] += w2 * phi_kl;
                    this->omega[ik][jl] += w2;
#endif
                }
            }


               std::cout << std::fixed << "delta\n";
    for (const auto& v1 : delta)
    {
        for (const auto& v2 : v1)
        {
            std::cout << v2 << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << "omega\n";
    for (const auto& v1 : omega)
    {
        for (const auto& v2 : v1)
        {
            std::cout << v2 << ' ';
        }
        std::cout << std::endl;
    }
    std::cout << "phi\n";
    for (const auto& v1 : phi)
    {
        for (const auto& v2 : v1)
        {
            std::cout << v2 << ' ';
        }
        std::cout << std::endl;
    }

    exit(0);


            for (auto i = 0; i < m + 3; ++i)
            {
                for (auto j = 0; j < n + 3; ++j)
                {
#if _USE_EIGEN_
                    if (!logically_equal(this->omega(i, j), static_cast<decimal_t>(0)))
                    {
                        this->phi(i, j) = this->delta(i, j) / this->omega(i, j);
                    }
                    else
                    {
                        this->phi(i, j) = 0;
                    }
#else
                    if (!logically_equal(this->omega[i][j], static_cast<decimal_t>(0)))
                    {
                        this->phi[i][j] = this->delta[i][j] / this->omega[i][j];
                    }
                    else
                    {
                        this->phi[i][j] = 0;
                    }
#endif
                }
            }
        }
    }

    decimal_t eval(decimal_t u, decimal_t v)
    {
        //
        // Compute [i,j] indices and [s,t] params
        //
        auto [i, j, s, t] = compute_ijst(u, v);

        //
        // Evaluate the function
        //
        decimal_t val = 0;
        for (auto k = 0; k < 4; ++k)
        {
            for (auto l = 0; l < 4; ++l)
            {
#if _USE_EIGEN_
                val +=
                    this->phi(i + k, j + l) * B<decimal_t>[k](s) * B<decimal_t>[l](t);
#else
                val +=
                    this->phi[i + k][j + l] * B<decimal_t>[k](s) * B<decimal_t>[l](t);
#endif
            }
        }
        return val;
    }


    void init() 
    {
        umin = std::numeric_limits<decimal_t>::max();
        vmin = std::numeric_limits<decimal_t>::max();
        umax = std::numeric_limits<decimal_t>::min();
        vmax = std::numeric_limits<decimal_t>::min();

        decimal_t sum_z = 0;
        
        auto points_ptr = points;
        for (auto index = 0; index < point_count; ++index, points_ptr++)
        {
            const decimal_t x = (*points_ptr)[0];
            const decimal_t y = (*points_ptr)[1];
            const decimal_t z = (*points_ptr)[2];

            if (x < umin) umin = x;
            if (y < vmin) vmin = y;
            if (x > umax) umax = x;
            if (y > vmax) vmax = y;

            sum_z += z;
        }

        urange_inv = 1 / (umax - umin);
        vrange_inv = 1 / (vmax - vmin);

        std::cout << point_count << " sum " << sum_z << std::endl;
        average_z = sum_z / std::distance(std::begin(*points),std::end(*points));

        std::cout << "average_z " << average_z << std::endl;
        apply_offset();
    }

    void apply_offset()
    {
        z_surface.resize(point_count);
        auto points_ptr = points;
        for (auto index = 0; index < point_count; ++index, points_ptr++)
        {
            z_surface[index] = (*points_ptr)[2] - average_z;
        }
    }

};
