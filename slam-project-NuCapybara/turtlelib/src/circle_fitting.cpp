#include <iostream>
#include <armadillo>
#include "turtlelib/circle_fitting.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf_slam.hpp"
#include <cmath>

namespace turtlelib
{
    CircleFitting::CircleFitting(arma::colvec xList, arma::colvec yList)
    {
        n = xList.n_elem;
        x_list = xList;
        y_list = yList;
        x_i = x_list - arma::mean(x_list);
        y_i = y_list - arma::mean(y_list);
        Z = arma::mat(n, 4, arma::fill::ones);
        H = arma::mat(4, 4, arma::fill::zeros);
        H_inv = arma::mat(4, 4, arma::fill::zeros);
        calculateH_Hinv();
    }

    void CircleFitting::calculateH_Hinv(){
        arma::colvec z_i = pow(x_i,2) + pow(y_i,2);
        auto z_mean = arma::mean(z_i);
        
        arma::colvec one_col = arma::ones<arma::colvec>(n);
        auto Z_left_mat = arma::join_rows(z_i, x_i);
        auto Z_right_mat = arma::join_rows(y_i, one_col);
        Z = arma::join_rows(Z_left_mat, Z_right_mat);

        H(0,0) = 8*z_mean;
        H(0,3) = 2;
        H(1,1) = 1;
        H(2,2) = 1;
        H(3,0) = 2;

        H_inv(0, 3) = 0.5;
        H_inv(1, 1) = 1;
        H_inv(2, 2) = 1;
        H_inv(3, 0) = 0.5;
        H_inv(3, 3) = -2*z_mean;

        arma::svd(U, sigma, V, Z);

        if(sigma(3) < 1e-12){
            A = V.col(3);
        }
        else{
            Y = V * sigma * V.t();
            Q = Y * H_inv * Y;
            // Declare variables to hold the eigenvalues and eigenvectors
            arma::cx_vec eigval{};
            arma::cx_mat eigvec{};
            
            // Compute the eigenvalues and eigenvectors
            arma::eig_gen(eigval, eigvec, Q);

            arma::mat real_eigval = arma::real(eigval); // Select real eigenvalues.
            arma::mat real_eigvec = arma::real(eigvec); // and eigenvectors

            // Initialize variables to find the smallest positive eigenvalue
            double smallestPositiveEigenvalue = std::numeric_limits<double>::max();
            arma::vec correspondingEigenvector;

            for(size_t i = 0; i < eigval.size(); i++){
                if(real_eigval[i] > 0 && real_eigval[i]< smallestPositiveEigenvalue){
                    smallestPositiveEigenvalue = real_eigval(i);
                    correspondingEigenvector = real_eigvec.col(i);
                }
            }

            arma::vec A_star = correspondingEigenvector;

            A = Y.i() * A_star;        
        }
                    

        a = -A(1)/(2*A(0));
        b = -A(2)/(2*A(0));
        double r_square = (pow(A(1),2) + pow(A(2),2) - 4*A(0)*A(3))/(4*pow(A(0),2));
        R = sqrt(r_square);

    }

}