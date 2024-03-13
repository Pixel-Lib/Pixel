

#include <utility>
#include <vector>

#include "pxl/parametrics/coord.hpp"
namespace pxl {

/**
 * @class Regression
 * @brief A class that performs regression analysis on a set of points.
 * 
 * The Regression class calculates the best-fit line through a set of points using ridge regression.
 * It also provides a method to predict the y-value for a given x-value based on the fitted line.
 */
class Regression {
    private:
        std::vector<pxl::Coord> points; ///< The set of points used for regression analysis.
        double lambda; ///< The regularization parameter for ridge regression.

    public:
        /**
         * @brief Constructor for the Regression class.
         * @param points The set of points to perform regression analysis on.
         */
        Regression(std::vector<pxl::Coord> points);

        /**
         * @brief Calculates the dot product of two vectors.
         * @param v1 The first vector.
         * @param v2 The second vector.
         * @return The dot product of v1 and v2.
         */
        double dotProduct(const std::vector<double> &v1, const std::vector<double> &v2);

        /**
         * @brief Fits a function through the coordinates using ridge regression.
         * @return A pair of doubles representing the slope and y-intercept of the fitted line.
         */
        std::pair<double, double> Ridge();

        /**
         * @brief Predicts the y-value for a given x-value based on the fitted line.
         * @param x The x-value for which to predict the y-value.
         * @return The predicted y-value.
         */
        double predict(double x);
};
}  // namespace pxl
