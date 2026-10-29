#pragma once

#include <cmath>
#include <vector>
#include <algorithm>

namespace gnss {

struct Vec3
{
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
};

// Small dense row-major matrix, sufficient for least-squares navigation.
class Matrix
{
public:
    Matrix(int r = 0, int c = 0, double v = 0.0) : rows_(r), cols_(c), d_(size_t(r) * c, v) {}
    int rows() const { return rows_; }
    int cols() const { return cols_; }
    double& operator()(int i, int j) { return d_[size_t(i) * cols_ + j]; }
    double operator()(int i, int j) const { return d_[size_t(i) * cols_ + j]; }

    Matrix transposed() const
    {
        Matrix t(cols_, rows_);
        for (int i = 0; i < rows_; ++i)
            for (int j = 0; j < cols_; ++j)
                t(j, i) = (*this)(i, j);
        return t;
    }

    Matrix operator*(const Matrix& b) const
    {
        Matrix r(rows_, b.cols_);
        for (int i = 0; i < rows_; ++i)
            for (int k = 0; k < cols_; ++k) {
                const double a = (*this)(i, k);
                if (a == 0.0) continue;
                for (int j = 0; j < b.cols_; ++j)
                    r(i, j) += a * b(k, j);
            }
        return r;
    }

    // Gauss-Jordan inversion with partial pivoting. Returns false if singular.
    bool inverse(Matrix* out) const
    {
        if (rows_ != cols_) return false;
        const int n = rows_;
        Matrix a(*this);
        Matrix inv(n, n);
        for (int i = 0; i < n; ++i) inv(i, i) = 1.0;
        for (int c = 0; c < n; ++c) {
            int p = c;
            for (int r = c + 1; r < n; ++r)
                if (std::fabs(a(r, c)) > std::fabs(a(p, c))) p = r;
            if (std::fabs(a(p, c)) < 1e-300) return false;
            if (p != c)
                for (int j = 0; j < n; ++j) { std::swap(a(p, j), a(c, j)); std::swap(inv(p, j), inv(c, j)); }
            const double piv = a(c, c);
            for (int j = 0; j < n; ++j) { a(c, j) /= piv; inv(c, j) /= piv; }
            for (int r = 0; r < n; ++r) {
                if (r == c) continue;
                const double f = a(r, c);
                if (f == 0.0) continue;
                for (int j = 0; j < n; ++j) { a(r, j) -= f * a(c, j); inv(r, j) -= f * inv(c, j); }
            }
        }
        *out = inv;
        return true;
    }

private:
    int rows_, cols_;
    std::vector<double> d_;
};

inline double wrapHalfWeek(double dt)
{
    if (dt > 302400.0) dt -= 604800.0;
    else if (dt < -302400.0) dt += 604800.0;
    return dt;
}

} // namespace gnss
