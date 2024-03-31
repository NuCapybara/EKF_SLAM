#include <iostream>
#include <armadillo>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf_slam.hpp"
#include <cmath>

// ############################ Begin_Citation [1] ############################
// Code that is directly influence by the citation [1]
// ############################ End_Citation [1]  #############################
namespace turtlelib
{
    EKFSlam::EKFSlam()
    {
        q = arma::colvec{num_rs,arma::fill::zeros};
        m = arma::colvec{2*num_landmarks,arma::fill::zeros};
        Xi = arma::colvec{num_rs+2*num_landmarks,arma::fill::zeros};
        ///question: how about m matrix?
        sigma = arma::mat{num_rs+2*num_landmarks,num_rs+2*num_landmarks,arma::fill::eye};
        initialize_covariance();
    }

    EKFSlam::EKFSlam(Transform2D robot_initial_pose)
    {
        q = arma::colvec{3,arma::fill::zeros};
        m = arma::colvec{2*num_landmarks,arma::fill::zeros};
        Xi = arma::colvec{num_rs+2*num_landmarks,arma::fill::zeros};
        sigma = arma::mat{num_rs+2*num_landmarks,num_rs+2*num_landmarks,arma::fill::eye};
        initialize_pose(robot_initial_pose);
        initialize_covariance();
    }

    void EKFSlam::initialize_covariance()
    {
        arma::mat sigma_0_q = arma::zeros<arma::mat>(num_rs,num_rs);///initialized to zero
        arma::mat sigma_0_m = arma::eye(2*num_landmarks,2*num_landmarks)*1e6;///initialized to infinity
        arma::mat mat_0_1 = arma::zeros<arma::mat>(num_rs,2*num_landmarks);
        arma::mat mat_1_0 = arma::zeros<arma::mat>(2*num_landmarks,num_rs);
        sigma = arma::join_cols(
            arma::join_rows(sigma_0_q,mat_0_1),
            arma::join_rows(mat_1_0,sigma_0_m));
    }

    void EKFSlam::initialize_pose(Transform2D robot_initial_pose)
    {
        q(0) = robot_initial_pose.rotation();
        q(1) = robot_initial_pose.translation().x;
        q(2) = robot_initial_pose.translation().y;
        update_Xi();
    }

    void EKFSlam::update_Xi(){
        for(int i=0; i<num_rs; i++){
            Xi(i) = q(i);
        }
        for(int col_index=0; col_index<num_landmarks; col_index++){
            Xi(num_rs + col_index*2) = m(2*col_index);
            Xi(num_rs + col_index*2 + 1) = m(2*col_index + 1);
        }
    }


    void EKFSlam::update_pose_and_map()
    {
        // Populate pose vector
        for (int pose_index = 0; pose_index < num_rs; pose_index++)
        {
            q(pose_index) = Xi(pose_index);
        }
        // Populate map vector
        for (int landmark_index = 0; landmark_index < num_landmarks; landmark_index++)
        {
            m(2*landmark_index) = Xi(num_rs + 2*landmark_index); // X coordinate of landmark
            m(2*landmark_index + 1) = Xi(num_rs + 2*landmark_index + 1); // Y coordinate of landmark
        }
    }

    void EKFSlam::predict(Twist2D twist)
    {
        if(!almost_equal(twist.y ,0.0)){
            throw std::runtime_error("Twist2D.y is not zero, which is not allowed");
        }

        u(0) = twist.omega;
        u(1) = twist.x;
        u(2) = 0.0;
        ///Initialize the A matrix
        arma::mat mat_0_1= arma::zeros<arma::mat>(num_rs,2*num_landmarks);///3*2n
        arma::mat mat_1_0= arma::zeros<arma::mat>(2*num_landmarks,num_rs);///2n*3
        arma::mat mat_1_1 = arma::zeros<arma::mat>(2*num_landmarks,2*num_landmarks);///2n*2n
        arma::mat pose_mat = arma::zeros(num_rs,num_rs);///3*3

        
        ///calculate A matrix, two sceanrios: delta theta(u[0])is/not zero
        if(almost_equal(u(0),0.0)){
            pose_mat(1,0) = -u(1)*sin(q(0));
            pose_mat(2,0) = u(1)*cos(q(0));
        }
        else{
            pose_mat(1,0) = (-u(1) / u(0) * cos(q(0))) + (u(1) /u(0) * cos(normalize_angle(q(0) + u(0))));
            pose_mat(2,0) = (-u(1) / u(0) * sin(q(0))) + (u(1) /u(0) * sin(normalize_angle(q(0) + u(0))));
        }

        A = I + arma::join_cols(
            arma::join_rows(pose_mat,mat_0_1),
            arma::join_rows(mat_1_0, mat_1_1));


        ///generate Q bar

        arma::mat Q_bar = arma::join_cols(
            arma::join_rows(Q , mat_0_1),
            arma::join_rows(mat_1_0, mat_1_1));
        ///update the sigma matrix  
        sigma = A*sigma*A.t() + Q_bar;
        

        ///update the state vector ξ¯_t, robot state
        Transform2D Twb = Transform2D(Vector2D{q(1),q(2)},q(0));
        Transform2D Tbb_prime = integrate_twist(twist);
        Transform2D Twb_prime = Twb * Tbb_prime;
        ///q = [theta x y]^T
        q(0) = Twb_prime.rotation();
        q(1) = Twb_prime.translation().x;
        q(2) = Twb_prime.translation().y;
        update_Xi();

        //extra for debugggggg
        // Check covariance matrix (symmetric and positive semi-definite)
        // Check if symmetric
        // if (!(sigma.is_symmetric(1e-8)))
        // {
        //     std::cout << "Sigma in predict" << sigma << std::endl;
        //     throw std::runtime_error("Covariance is Assymetric!!!");
        // }
        // // Check if positive semi-definite
        // arma::vec eigvals = arma::eig_sym(sigma);
        // if (!(arma::all(eigvals >= 0)))
        // {
        //     throw std::runtime_error("Covariance is not Positive Semi-Definite!!!");
        // }

    }

    void EKFSlam::correct(double x, double y, size_t j){
        ///range and bearing calculate
        double r_j = sqrt(pow(x,2) + pow(y,2));
        double phi_j = std::atan2(y,x);

        /// if j is not in seen_landmarks, initialize the obstacle
        if(seen_landmarks.find(j) == seen_landmarks.end()){
            double m_x_j = q(1) + r_j * cos(phi_j + q(0));
            double m_y_j = q(2) + r_j * sin(phi_j + q(0));
            m(j*2-2) = m_x_j;
            m(j*2-1) = m_y_j;
            ///insert j into seen_landmarks
            seen_landmarks.insert(j);
            ///update the state vector ξ¯_t, robot state
            update_Xi();
        }
        // Actual Measurement of that landmark. Not a distribution.
        z_j(0) = r_j;
        z_j(1) = phi_j;

        ///calculate ˆz_j
        // Relative predictions of landmark position, as cartesian coordinates
        // δ_{x,j} = ˆm_{x,j} − ˆx_t
        // δ_{y,j} = ˆm_{y,j} − ˆy_t
        // d_j = δ_{x,j}^2 + δ_{y,j}^2

        double delta_x_j = m(j*2-2)-q(1); ///m(j*2-2) for m_j_x
        double delta_y_j = m(j*2-1)-q(2); ///m(j*2-1) for m_j_y
        double d_j = pow(delta_x_j,2) + pow(delta_y_j,2);

        z_j_hat(0) = sqrt(d_j);
        z_j_hat(1) = normalize_angle(std::atan2(delta_y_j,delta_x_j) - q(0));


        ///composite H matrix
        double dx_d_val = delta_x_j / std::sqrt(d_j);
        double dy_d_val = delta_y_j / std::sqrt(d_j);

        arma::mat zeros_first_mat{2, 2 * (j-1), arma::fill::zeros};
        arma::mat zeros_second_mat{2, 2 * (num_landmarks-j), arma::fill::zeros};
        arma::mat H_first_small_mat{2, num_rs, arma::fill::zeros};
        arma::mat H_second_small_mat{2, 2, arma::fill::zeros};

        H_first_small_mat(0, 0) = 0.0;
        H_first_small_mat(0, 1) = -dx_d_val;
        H_first_small_mat(0, 2) = -dy_d_val;
        // H_first_small_mat(0, 1) = -delta_x_j / std::sqrt(d_j);
        // H_first_small_mat(0, 2) = -delta_y_j / std::sqrt(d_j);
        H_first_small_mat(1, 0) = -1;
        H_first_small_mat(1, 1) = delta_y_j / d_j;
        H_first_small_mat(1, 2) = -delta_x_j/d_j;

        H_second_small_mat(0, 0) = dx_d_val;
        H_second_small_mat(0, 1) = dy_d_val;
        // H_second_small_mat(0, 0) = delta_x_j / std::sqrt(d_j);
        // H_second_small_mat(0, 1) = delta_y_j / std::sqrt(d_j);
        H_second_small_mat(1, 0) = -delta_y_j / d_j;
        H_second_small_mat(1, 1) = delta_x_j/d_j;

        H_i = arma::join_horiz(
            arma::join_horiz(H_first_small_mat, zeros_first_mat), 
            arma::join_horiz(H_second_small_mat, zeros_second_mat));

        ///calculate the Kalman gain
        // K_i = Σ¯_t H_{i}^{T} (H_i Σ¯_t H_{i}^{T} + R)^{-1}
        R = arma::mat{2, 2, arma::fill::eye} * R_noise;
        // K_i = sigma * H_i.t() *(H_i * sigma * H_i.t() + R).i();
        K_i = sigma * H_i.t() * (H_i * sigma * H_i.t() + R).i();
        // std::cout << "Ki" << K_i << std::endl;

        ///update the state vector ξ, robot state
        // Xi = Xi + K_i * (z_j - z_j_hat);
        arma::colvec z_j_diff{2, arma::fill::zeros};

        z_j_diff(0) = z_j(0) - z_j_hat(0);
        z_j_diff(1) = normalize_angle(z_j(1) - z_j_hat(1));

        Xi = Xi + K_i * (z_j_diff);  
        update_pose_and_map();
        // std::cout << "Xi" << Xi << std::endl;

        ///update the covariance matrix
        sigma = (I - K_i * H_i) * sigma;
        // std::cout << "Sigma in correct" << sigma << std::endl;
    }



 /// \brief set the initial state of the robot
    Transform2D EKFSlam::get_robot_state() const
    {
        return Transform2D{Vector2D{q(1), q(2)}, q(0)};
    }

    /// \brief set the initial state of the robot
    arma::colvec EKFSlam::get_map() const
    {
        return m;
    }

    /// \brief set the initial state of the robot
    arma::colvec EKFSlam::get_Xi() const
    {
        return Xi;
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::get_sigma() const
    {
        return sigma;
    }

    /// \brief set the initial state of the robot
    Twist2D EKFSlam::get_twist() const
    {
        return Twist2D{u(0), u(1), u(2)};
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::get_A() const
    {
        return A;
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::get_actual_measurement() const
    {
        return z_j;
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::get_predicted_measurement() const
    {
        return z_j_hat;
    }

    /// \brief set the initial state of the robot
    arma::mat EKFSlam::get_Hi() const
    {
        return H_i;
    }


}