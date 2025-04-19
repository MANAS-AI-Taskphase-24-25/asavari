#ifndef AUTONAV_CPP_CUBIC_SPLINE_HPP
#define AUTONAV_CPP_CUBIC_SPLINE_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace autonav_cpp {

class CubicSpline1D {
public:
    std::vector<double> x, y, a, b, c, d, h;

    CubicSpline1D(const std::vector<double>& x_, const std::vector<double>& y_) : x(x_), y(y_) {
        size_t n = x.size();
        if (n < 2 || y.size() != n)
            throw std::invalid_argument("x and y must have same size and at least 2 points");

        h.resize(n - 1);
        for (size_t i = 0; i < n - 1; i++) {
            h[i] = x[i + 1] - x[i];
            if (h[i] <= 0)
                throw std::invalid_argument("x values must be strictly increasing");
        }

        a = y;

        // coeff matrix A and vector B
        std::vector<double> alpha(n - 1);
        for (size_t i = 1; i < n - 1; i++) {
            alpha[i] = (3.0 / h[i]) * (a[i + 1] - a[i]) - (3.0 / h[i - 1]) * (a[i] - a[i - 1]);
        }

        std::vector<double> l(n), mu(n), z(n);
        l[0] = 1.0;
        mu[0] = 0.0;
        z[0] = 0.0;

        for (size_t i = 1; i < n - 1; i++) {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n - 1] = 1.0;
        z[n - 1] = 0.0;
        c.resize(n, 0.0);
        b.resize(n - 1);
        d.resize(n - 1);

        for (size_t j = n - 2; (int)j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }
    }

    double evaluate(double t) const {
        size_t i = findSegment(t);
        double dx = t - x[i];
        return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
    }

    size_t findSegment(double t) const {
        if (t <= x[0]) return 0;
        if (t >= x.back()) return x.size() - 2;

        for (size_t i = 0; i < x.size() - 1; ++i) {
            if (t >= x[i] && t <= x[i + 1]) {
                return i;
            }
        }
        return x.size() - 2; // default fallback
    }
};

class CubicSpline2D {
public:    
    std::vector<double> s;
    CubicSpline1D sx, sy;

    CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y)
        : s(calcS(x, y)), sx(s, x), sy(s, y) {}

    double calcX(double t) const {
        return sx.evaluate(t);
    }

    double calcY(double t) const {
        return sy.evaluate(t);
    }

    double calcLength() const {
        return s.back();
    }

    std::pair<double, double> evaluate(double t) const {
        return { calcX(t), calcY(t) };
    }

private:   

    std::vector<double> calcS(const std::vector<double>& x, const std::vector<double>& y) {
        std::vector<double> s(x.size(), 0.0);
        for (size_t i = 1; i < x.size(); i++) {
            double dx = x[i] - x[i - 1];
            double dy = y[i] - y[i - 1];
            s[i] = s[i - 1] + std::hypot(dx, dy);
        }
        return s;
    }
};

}  

#endif  // AUTONAV_CPP_CUBIC_SPLINE_HPP

