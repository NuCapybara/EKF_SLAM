#ifndef EKF_SLAM_INCLUDE_GUARD_HPP
#define EKF_SLAM_INCLUDE_GUARD_HPP
/// \file
/// \brief Extended Kalman Filter (SLAM).

#include <iosfwd>
#include <cmath>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include <unordered_set>

namespace turtlelib{

    /// \brief Extended Kalman Filter (SLAM).
    class CircleFitting
    {
    private:
        /// \brief number of data points
        size_t n;
        /// \brief x list of n data points
        arma::colvec x_list{};
        /// \brief x list of n data points
        arma::colvec y_list{};
        /// \brief x list when centroid is at origin
        arma::colvec x_i{};
        /// \brief y list when centroid is at origin
        arma::colvec y_i{};
        /// \brief Z matrix from n data points
        arma::mat Z{};
        /// \brief M moment matrix from n data points
        arma::mat M{};
        /// \brief Contraint Matrix for hyperaccurate algebraic fit
        arma::mat H{};
        /// \brief inv Contraint Matrix for hyperaccurate algebraic fit
        arma::mat H_inv{};
        /// \brief U matrix
        arma::mat U{};
        /// \brief V matrix
        arma::mat V{};
        /// \brief Sigma matrix
        arma::vec sigma;
        /// \brief Y matrix
        arma::mat Y{};
        /// \brief Q matrix
        arma::mat Q{};
        /// \brief A* the smallest positive eigen vector of Q
        arma::mat A_star{};
        /// \brief A for calculate ab and R
        arma::mat A{};

        double R;
        double a;
        double b;

    public:
        // /// \brief Constructor initialize the EKF slam with origin and default uncertainty
        // CircleFitting();
        /// \brief Constructor initialize the EKF slam with robot initial pose and default uncertainty
        explicit CircleFitting(arma::colvec xList, arma::colvec yList);
        /// \brief initial guess of covariance matrix
        void calculateH_Hinv();
    };
        



}
#endif