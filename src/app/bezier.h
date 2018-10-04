#pragma once
#include <cmath>

// namespace curve starts here
namespace curve
{
// namespace bezier starts here
namespace bezier
{
template <typename decimal_t>
static decimal_t linear(decimal_t t, decimal_t p[2])
{
    return p[0] + (p[1] - p[0]) * t;
}

template <typename decimal_t>
static decimal_t quadratic(decimal_t t, decimal_t p[3])
{
    return std::pow(1 - t, 2) * p[0] + (1 - t) * 2 * t * p[1] + t * t * p[2];
}

template <typename decimal_t>
static decimal_t cubic(decimal_t t, decimal_t p[4])
{
    return std::pow((1 - t), 3) * p[0] + 3 * t * std::pow(1 - t, 2) * p[1] +
           3 * std::pow(t, 2) * (1 - t) * p[2] + std::pow(t, 3) * p[3];
}
} // namespace bezier
} // namespace curve

// namespace surface starts here
namespace surface
{
// namespace bezier starts here
namespace bezier
{

template <typename decimal_t>
static auto cubic(decimal_t u, decimal_t v, decimal_t px[16], decimal_t py[16])
{
    constexpr auto Pij = static_cast<decimal_t>(1) / static_cast<decimal_t>(15);

    decimal_t val = 0;
    for (auto i = 0; i < 4; ++i)
    {
        for (auto j = 0; j < 4; ++j)
        {
            val += Pij * 
                curve::bezier::cubic<decimal_t>(u, &px[i * 4]) * 
                curve::bezier::cubic<decimal_t>(v, &px[j * 4]);
        }
    }

    return val;
}

} // namespace bezier
} // namespace surface
