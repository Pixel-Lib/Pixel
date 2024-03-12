#include <vector>
#include <utility>
#include "pxl/parametrics/regression.hpp"
#include "pxl/parametrics/coord.hpp"
#include "pxl/util.hpp"

namespace pxl {


        // Constructor for the Regression class
    Regression::Regression(std::vector<pxl::Coord> points) 
    : points(points) {}



        // Function to calculate the dot product of two vectors
        double Regression::dotProduct(const std::vector<double>& v1, const std::vector<double>& v2) {
            double product = 0.0;
            for (int i = 0; i < v1.size(); i++) {
                product += v1[i] * v2[i];
            }
            return product;
        }

        // Function to fit a function through the coordinates using ridge regression
std::pair<double, double> Regression::Ridge() {
    std::vector<double> x(this->points.size());
    std::vector<double> y(this->points.size());

    for (int i = 0; i < points.size(); i++) {
        x[i] = points[i].x;
        y[i] = points[i].y;
    }

    double xMean = pxl::avg(x);
    double yMean = pxl::avg(y);

    double xDotY = this->dotProduct(x, y);
    double xDotX = this->dotProduct(x, x);
    double n = this->points.size();

    // Calculate lambda as 1% of the sum of squares of x
    lambda = 0.01 * xDotX;

    double m = (xDotY - n * xMean * yMean) / (xDotX - n * xMean * xMean + lambda);
    double b = yMean - m * xMean;

    return std::make_pair(m, b);
}

        double Regression::predict(double x) {
            std::pair<double, double> coefficients = Ridge();
            double m = coefficients.first;
            double b = coefficients.second;
            return m * x + b;
        }

}