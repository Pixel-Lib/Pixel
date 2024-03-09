

#include <vector>
#include <utility>

namespace pxl {

    // Define a pair of doubles as a Point
    typedef std::pair<double, double> Point;

    // Define the Regression class
    class Regression {
    private:
        std::vector<Point> points;
        double lambda;

    public:
        // Constructor for the Regression class
        Regression(std::vector<Point>& points, double lambda);

        // Function to calculate the mean of a vector
        double mean(const std::vector<double>& v);

        // Function to calculate the dot product of two vectors
        double dotProduct(const std::vector<double>& v1, const std::vector<double>& v2);

        // Function to fit a function through the coordinates using ridge regression
        std::pair<double, double> Ridge();

        double predict(double x);
    };
}
