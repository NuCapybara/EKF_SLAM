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
    /// \brief number of degrees of freedom of the robot
    constexpr int num_landmarks=3;
    /// \brief number of landmarks
    constexpr int num_rs = 3;
    /// \brief noise on robot motion
    constexpr double w = 0.0001;
    /// \brief noise on landmarks
    constexpr double R_noise = 0.0001;

    /// \brief Extended Kalman Filter (SLAM).
    class EKFSlam
    {
    private:
        /// \brief State vector of the robot at time t. q ∈ num_rs x 1. [theta x y]^T 
        arma::colvec q{};
        /// \brief State vector of the obstacles at time t. m ∈ 2*num_landmarks x 1. [m_x1 m_y1 m_x2 m_y2 ... m_xn m_yn]^T
        arma::colvec m{};
        /// \brief State vector of the system at time t. ξ ∈ (num_rs + 2*num_landmarks) x 1. [q ; m]
        arma::colvec Xi{};
        /// \brief Covariance matrix. Σ ∈ (num_rs + 2*num_landmarks) x (num_rs + 2*num_landmarks). the m matrix is with infinity on its diagonal because we don't know the initial position of the landmarks.
        arma::mat sigma{};
        arma::colvec u{num_rs,arma::fill::zeros};
        /// \brief Identity matrix. I ∈ (num_dof + 2*num_landmarks) x (num_dof + 2*num_landmarks). Used to calculate A.
        const arma::mat I{num_rs+2*num_landmarks,num_rs+2*num_landmarks,arma::fill::eye};
        /// \brief Linearized state transition matrix. A ∈ (num_dof + 2*num_landmarks) x (num_dof + 2*num_landmarks)
        arma::mat A{num_rs+2*num_landmarks, num_rs+2*num_landmarks,arma::fill::eye};
        /// \brief (22) noise matrix for linearized state transition model Q ∈ 3*3
        const arma::mat Q{arma::mat{num_rs,num_rs,arma::fill::eye}*w};

        /// \brief Previously seen landmark IDs
        std::unordered_set<int> seen_landmarks{};
        /// \brief Actual measurement. z_j ∈ 2 x 1. From fake sensor publisher. Relative r_j and phi_j bearing measurements of a landmarks. 
        arma::colvec z_j{2,arma::fill::zeros};
        /// \brief Estimate measurement. ˆz_j ∈ 2 x 1. Relative ˆr_j and ˆphi_j bearing predictions of a landmarks, based on pose prediction.
        arma::colvec z_j_hat{2,arma::fill::zeros};
        /// \brief (18) H matrix derivative respect to measurement model
        arma::mat H_i{2, num_rs+2*num_landmarks, arma::fill::zeros};
        /// \brief (26) Kalman gain
        arma::mat K_i{3+2*num_landmarks, 2, arma::fill::zeros};
        /// \brief noise in z, R ∈ 2*2
        arma::mat R{2,2,arma::fill::eye};
        /// \brief (28) G matrix
    
    public:
        /// \brief Constructor initialize the EKF slam with origin and default uncertainty
        EKFSlam();
        /// \brief Constructor initialize the EKF slam with robot initial pose and default uncertainty
        explicit EKFSlam(Transform2D robot_initial_pose);
        /// \brief initial guess of covariance matrix
        void initialize_covariance();
        /// \brief set the initial robot pose
        /// \param robot_initial_pose the initial pose of the robot
        void initialize_pose(Transform2D robot_initial_pose);
        /// \brief update the state vector
        void update_Xi();
        /// \brief step 25 and before
        void predict(Twist2D twist);
        /// \brief step 26 and after, the x y should be in Trc
        void correct(double x, double y, size_t j);

        /// \brief get the state q
        Transform2D get_robot_state() const;

        /// \brief get the obstacle map m
        arma::colvec get_map() const;

        /// \brief set the state vector Xi
        arma::colvec get_Xi() const;

        /// \brief set the sigma matrix
        arma::mat get_sigma() const;

        /// \brief get twist
        Twist2D get_twist() const;

        /// \brief set the A matrix
        arma::mat get_A() const;

        /// \brief get actual measurement zj
        arma::mat get_actual_measurement() const;

        /// \brief get predicted measurement zj_hat
        arma::mat get_predicted_measurement() const;
        /// \brief set the initial state of the robot
        arma::mat get_Hi() const;
        /// @brief update the pose and map
        void update_pose_and_map();
    };
        



}
#endif