#include <vector>
#include <utility>
#include "pxl/parametrics/regression.hpp"
#include "pxl/util.hpp"

namespace pxl {

    // Define a pair of doubles as a Point
    typedef std::pair<double, double> Point;

        std::vector<Point> points;
        double lambda;

        // Constructor for the Regression class
    Regression::Regression(std::vector<Point>& points) 
    : points(points) {}



        // Function to calculate the dot product of two vectors
        double dotProduct(const std::vector<double>& v1, const std::vector<double>& v2) {
            double product = 0.0;
            for (int i = 0; i < v1.size(); i++) {
                product += v1[i] * v2[i];
            }
            return product;
        }

        // Function to fit a function through the coordinates using ridge regression
std::pair<double, double> Ridge() {
    std::vector<double> x(points.size());
    std::vector<double> y(points.size());

    for (int i = 0; i < points.size(); i++) {
        x[i] = points[i].first;
        y[i] = points[i].second;
    }

    double xMean = pxl::avg(x);
    double yMean = pxl::avg(y);

    double xDotY = dotProduct(x, y);
    double xDotX = dotProduct(x, x);
    double n = points.size();

    // Calculate lambda as 1% of the sum of squares of x
    lambda = 0.01 * xDotX;

    double m = (xDotY - n * xMean * yMean) / (xDotX - n * xMean * xMean + lambda);
    double b = yMean - m * xMean;

    return std::make_pair(m, b);
}

        double predict(double x) {
            std::pair<double, double> coefficients = Ridge();
            double m = coefficients.first;
            double b = coefficients.second;
            return m * x + b;
        }

}