#include "model.h"
#include <cmath>
#include <mujoco/mujoco.h>
#include <xtensor/xarray.hpp>
#include <xtensor/xshape.hpp>
#include <xtensor/xtensor_forward.hpp>

Model::Model(const mjModel* model, mjData* data): model(model), data(data) {
    g = model->opt.gravity[2];
    x_CoM = { data->subtree_com[1 * 3], 0.0 }; 
    y_CoM = { data->subtree_com[1 * 3 + 1], 0.0 };
    z_CoM = data->subtree_com[1 * 3 + 2];
    w = std::sqrt(g / z_CoM);
    px = data->subtree_com[1 * 3];
    py = data->subtree_com[1 * 3 + 1];
}

void Model::computeA(double t, xt::xarray<double>& A) {
    double cosh_wt = std::cosh(w * t);
    double sinh_wt = std::sinh(w * t);

    A = xt::xarray<double>({{cosh_wt, sinh_wt/w}, 
                            {w * sinh_wt, cosh_wt}});
}

void Model::computeB(double t, xt::xarray<double>& B) {
    double cosh_wt = std::cosh(w * t);
    double sinh_wt = std::sinh(w * t);
    
    B = xt::xarray<double>({{1 - cosh_wt, -w * sinh_wt}});
}

void Model::step(double dt) {
    //TODO: crear implementación
    
    //current time
    double t = data->time;

    //compute the matrices
    xt::xarray<double> A, B;
    computeA(t, A);
    computeB(t, B);

    //Get current CoM state
    xt::xarray<double> x_CoM_current = x_CoM;
    xt::xarray<double> y_CoM_current = y_CoM;

    //update CoM position
    xt::xarray<double> x_CoM_next = A * x_CoM_current + B * px;
    xt::xarray<double> y_CoM_next = A * y_CoM_current + B * py;

    //update internal states
    x_CoM = x_CoM_next;
    y_CoM = y_CoM_next;
}

xt::xarray<double> Model::getCoM() const {
    return { x_CoM[0], y_CoM[0] }; //TODO: check if it is correct
}


