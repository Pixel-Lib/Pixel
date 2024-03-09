#include <vector>
#include <utility>

class Regression {
public:
    // Define a pair of doubles as a Point
    typedef std::pair<double, double> Point;

    // Function to calculate the mean of a vector
    double mean(const std::vector<double>& v) {
        double sum = 0.0;
        for (double num : v) {
            sum += num;
        }
        return sum / v.size();
    }

    // Function to calculate the dot product of two vectors
    double dotProduct(const std::vector<double>& v1, const std::vector<double>& v2) {
        double product = 0.0;
        for (int i = 0; i < v1.size(); i++) {
            product += v1[i] * v2[i];
        }
        return product;
    }

    // Function to fit a function through the coordinates using ridge regression
    std::pair<double, double> functionFitter(const std::vector<Point>& points, double lambda) {
        std::vector<double> x(points.size());
        std::vector<double> y(points.size());

        for (int i = 0; i < points.size(); i++) {
            x[i] = points[i].first;
            y[i] = points[i].second;
        }

        double xMean = mean(x);
        double yMean = mean(y);

        double m = (dotProduct(x, y) - yMean * dotProduct(x, x)) / (dotProduct(x, x) - xMean * dotProduct(x, x) + lambda);
        double b = yMean - m * xMean;

        return std::make_pair(m, b);
    }
};