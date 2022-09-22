// This file is part of INSTINCT, the INS Toolkit for Integrated
// Navigation Concepts and Training by the Institute of Navigation of
// the University of Stuttgart, Germany.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "CubicSpline.hpp"

#include "util/Assert.h"

namespace NAV
{

CubicSpline::CubicSpline(const std::vector<double>& X, const std::vector<double>& Y,
                         BoundaryCondition leftBoundaryCondition, BoundaryCondition rightBoundaryCondition)
    : boundaryConditionLeft(leftBoundaryCondition), boundaryConditionRight(rightBoundaryCondition)
{
    setPoints(X, Y);
}

void CubicSpline::setBoundaries(BoundaryCondition leftBoundaryCondition, BoundaryCondition rightBoundaryCondition)
{
    boundaryConditionLeft = leftBoundaryCondition;
    boundaryConditionRight = rightBoundaryCondition;
}

void CubicSpline::setPoints(const std::vector<double>& x,
                            const std::vector<double>& y)
{
    vals_x = x;
    vals_y = y;
    size_t n = x.size();

    size_t n_upper = (boundaryConditionLeft.type == BoundaryCondition::NotAKnot) ? 2 : 1;
    size_t n_lower = (boundaryConditionRight.type == BoundaryCondition::NotAKnot) ? 2 : 1;
    BandMatrix A(n, n_upper, n_lower);
    std::vector<double> rhs(n);
    for (size_t i = 1; i < n - 1; i++)
    {
        A(i, i - 1) = 1.0 / 3.0 * (x[i] - x[i - 1]);
        A(i, i) = 2.0 / 3.0 * (x[i + 1] - x[i - 1]);
        A(i, i + 1) = 1.0 / 3.0 * (x[i + 1] - x[i]);
        rhs[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]) - (y[i] - y[i - 1]) / (x[i] - x[i - 1]);
    }
    // boundary conditions
    if (boundaryConditionLeft.type == BoundaryCondition::SecondDerivative)
    {
        // 2*c[0] = f''
        A(0, 0) = 2.0;
        A(0, 1) = 0.0;
        rhs[0] = boundaryConditionLeft.value;
    }
    else if (boundaryConditionLeft.type == BoundaryCondition::FirstDerivative)
    {
        // b[0] = f', needs to be re-expressed in terms of c:
        // (2c[0]+c[1])(x[1]-x[0]) = 3 ((y[1]-y[0])/(x[1]-x[0]) - f')
        A(0, 0) = 2.0 * (x[1] - x[0]);
        A(0, 1) = 1.0 * (x[1] - x[0]);
        rhs[0] = 3.0 * ((y[1] - y[0]) / (x[1] - x[0]) - boundaryConditionLeft.value);
    }
    else if (boundaryConditionLeft.type == BoundaryCondition::NotAKnot)
    {
        // f'''(x[1]) exists, i.e. d[0]=d[1], or re-expressed in c:
        // -h1*c[0] + (h0+h1)*c[1] - h0*c[2] = 0
        A(0, 0) = -(x[2] - x[1]);
        A(0, 1) = x[2] - x[0];
        A(0, 2) = -(x[1] - x[0]);
        rhs[0] = 0.0;
    }
    if (boundaryConditionRight.type == BoundaryCondition::SecondDerivative)
    {
        // 2*c[n-1] = f''
        A(n - 1, n - 1) = 2.0;
        A(n - 1, n - 2) = 0.0;
        rhs[n - 1] = boundaryConditionRight.value;
    }
    else if (boundaryConditionRight.type == BoundaryCondition::FirstDerivative)
    {
        // b[n-1] = f', needs to be re-expressed in terms of c:
        // (c[n-2]+2c[n-1])(x[n-1]-x[n-2])
        // = 3 (f' - (y[n-1]-y[n-2])/(x[n-1]-x[n-2]))
        A(n - 1, n - 1) = 2.0 * (x[n - 1] - x[n - 2]);
        A(n - 1, n - 2) = 1.0 * (x[n - 1] - x[n - 2]);
        rhs[n - 1] = 3.0 * (boundaryConditionRight.value - (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2]));
    }
    else if (boundaryConditionRight.type == BoundaryCondition::NotAKnot)
    {
        // f'''(x[n-2]) exists, i.e. d[n-3]=d[n-2], or re-expressed in c:
        // -h_{n-2}*c[n-3] + (h_{n-3}+h_{n-2})*c[n-2] - h_{n-3}*c[n-1] = 0
        A(n - 1, n - 3) = -(x[n - 1] - x[n - 2]);
        A(n - 1, n - 2) = x[n - 1] - x[n - 3];
        A(n - 1, n - 1) = -(x[n - 2] - x[n - 3]);
        rhs[0] = 0.0;
    }

    // solve the equation system to obtain the parameters c[]
    coef_c = A.lu_solve(rhs);

    // calculate parameters b[] and d[] based on c[]
    coef_d.resize(n);
    coef_b.resize(n);
    for (size_t i = 0; i < n - 1; i++)
    {
        coef_d[i] = 1.0 / 3.0 * (coef_c[i + 1] - coef_c[i]) / (x[i + 1] - x[i]);
        coef_b[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i])
                    - 1.0 / 3.0 * (2.0 * coef_c[i] + coef_c[i + 1]) * (x[i + 1] - x[i]);
    }
    // for the right extrapolation coefficients (zero cubic term)
    // f_{n-1}(x) = y_{n-1} + b*(x-x_{n-1}) + c*(x-x_{n-1})^2
    double h = x[n - 1] - x[n - 2];
    // coef_c[n-1] is determined by the boundary condition
    coef_d[n - 1] = 0.0;
    coef_b[n - 1] = 3.0 * coef_d[n - 2] * h * h + 2.0 * coef_c[n - 2] * h + coef_b[n - 2]; // = f'_{n-2}(x_{n-1})
    if (boundaryConditionRight.type == BoundaryCondition::FirstDerivative)
    {
        coef_c[n - 1] = 0.0; // force linear extrapolation
    }
    // for left extrapolation coefficients
    coef_c0 = (boundaryConditionLeft.type == BoundaryCondition::FirstDerivative) ? 0.0 : coef_c[0];
}

size_t CubicSpline::findClosestIdx(double x) const
{
    auto it = std::upper_bound(vals_x.begin(), vals_x.end(), x);
    return static_cast<size_t>(std::max(static_cast<int>(it - vals_x.begin()) - 1, 0));
}

double CubicSpline::operator()(double x) const
{
    size_t n = vals_x.size();
    size_t idx = findClosestIdx(x);

    double h = x - vals_x[idx];

    if (x < vals_x[0]) // extrapolation to the left
    {
        return (coef_c0 * h + coef_b[0]) * h + vals_y[0];
    }
    if (x > vals_x[n - 1]) // extrapolation to the right
    {
        return (coef_c[n - 1] * h + coef_b[n - 1]) * h + vals_y[n - 1];
    }
    // interpolation
    return ((coef_d[idx] * h + coef_c[idx]) * h + coef_b[idx]) * h + vals_y[idx];
}

double CubicSpline::derivative(size_t order, double x) const
{
    size_t n = vals_x.size();
    size_t idx = findClosestIdx(x);

    double h = x - vals_x[idx];
    if (x < vals_x[0]) // extrapolation to the left
    {
        switch (order)
        {
        case 1:
            return 2.0 * coef_c0 * h + coef_b[0];
        case 2:
            return 2.0 * coef_c0;
        default:
            return 0.0;
        }
    }
    else if (x > vals_x[n - 1]) // extrapolation to the right
    {
        switch (order)
        {
        case 1:
            return 2.0 * coef_c[n - 1] * h + coef_b[n - 1];
        case 2:
            return 2.0 * coef_c[n - 1];
        default:
            return 0.0;
        }
    }
    else // interpolation
    {
        switch (order)
        {
        case 1:
            return (3.0 * coef_d[idx] * h + 2.0 * coef_c[idx]) * h + coef_b[idx];
        case 2:
            return 6.0 * coef_d[idx] * h + 2.0 * coef_c[idx];
        case 3:
            return 6.0 * coef_d[idx];
        default:
            return 0.0;
        }
    }
}

// #########################################################################################################################################
//                                                               BandMatrix
// #########################################################################################################################################

CubicSpline::BandMatrix::BandMatrix(size_t dim, size_t nUpper, size_t nLower)
    : upperBand(nUpper + 1, std::vector<double>(dim)),
      lowerBand(nLower + 1, std::vector<double>(dim)) {}

double& CubicSpline::BandMatrix::operator()(size_t i, size_t j)
{
    return const_cast<double&>(static_cast<const CubicSpline::BandMatrix&>(*this)(i, j)); // NOLINT(cppcoreguidelines-pro-type-const-cast)
}
const double& CubicSpline::BandMatrix::operator()(size_t i, size_t j) const
{
    // j - i = 0 -> diagonal, > 0 upper right part, < 0 lower left part
    return j >= i ? upperBand[j - i][i] : lowerBand[i - j][i];
}

size_t CubicSpline::BandMatrix::dim() const
{
    return !upperBand.empty() ? upperBand[0].size() : 0;
}
size_t CubicSpline::BandMatrix::dimUpperBand() const
{
    return upperBand.size() - 1;
}
size_t CubicSpline::BandMatrix::dimLowerBand() const
{
    return lowerBand.size() - 1;
}

void CubicSpline::BandMatrix::lu_decompose()
{
    // preconditioning
    // normalize column i so that a_ii=1
    for (size_t i = 0; i < dim(); i++)
    {
        assert(this->operator()(i, i) != 0.0);
        lowerBand[0][i] = 1.0 / this->operator()(i, i);
        size_t j_min = i > dimLowerBand() ? i - dimLowerBand() : 0;
        size_t j_max = std::min(dim() - 1, i + dimUpperBand());
        for (size_t j = j_min; j <= j_max; j++)
        {
            this->operator()(i, j) *= lowerBand[0][i];
        }
        this->operator()(i, i) = 1.0; // prevents rounding errors
    }

    // Gauss LR-Decomposition
    for (size_t k = 0; k < dim(); k++)
    {
        size_t i_max = std::min(dim() - 1, k + dimLowerBand());
        for (size_t i = k + 1; i <= i_max; i++)
        {
            assert(this->operator()(k, k) != 0.0);

            double x = -this->operator()(i, k) / this->operator()(k, k);
            this->operator()(i, k) = -x; // assembly part of L
            size_t j_max = std::min(dim() - 1, k + dimUpperBand());
            for (size_t j = k + 1; j <= j_max; j++)
            {
                // assembly part of R
                this->operator()(i, j) = this->operator()(i, j) + x * this->operator()(k, j);
            }
        }
    }
}

std::vector<double> CubicSpline::BandMatrix::l_solve(const std::vector<double>& b) const
{
    INS_ASSERT_USER_ERROR(dim() == b.size(), "The BandMatrix dimension must be the same as the vector b in the equation Ax = b.");

    std::vector<double> x(dim());
    for (size_t i = 0; i < dim(); i++)
    {
        double sum = 0;
        size_t j_start = i > dimLowerBand() ? i - dimLowerBand() : 0;
        for (size_t j = j_start; j < i; j++)
        {
            sum += this->operator()(i, j) * x[j];
        }
        x[i] = (b[i] * lowerBand[0][i]) - sum;
    }
    return x;
}

std::vector<double> CubicSpline::BandMatrix::u_solve(const std::vector<double>& b) const
{
    INS_ASSERT_USER_ERROR(dim() == b.size(), "The BandMatrix dimension must be the same as the vector b in the equation Ax = b.");

    std::vector<double> x(dim());
    for (size_t i = dim(); i-- > 0;)
    {
        double sum = 0;
        size_t j_stop = std::min(dim() - 1, i + dimUpperBand());
        for (size_t j = i + 1; j <= j_stop; j++)
        {
            sum += this->operator()(i, j) * x[j];
        }
        x[i] = (b[i] - sum) / this->operator()(i, i);
    }
    return x;
}

std::vector<double> CubicSpline::BandMatrix::lu_solve(const std::vector<double>& b,
                                                      bool is_lu_decomposed)
{
    INS_ASSERT_USER_ERROR(dim() == b.size(), "The BandMatrix dimension must be the same as the vector b in the equation Ax = b.");

    if (!is_lu_decomposed)
    {
        lu_decompose();
    }
    auto y = l_solve(b);
    auto x = u_solve(y);
    return x;
}

} // namespace NAV