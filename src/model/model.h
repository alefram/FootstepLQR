#ifndef MODEL_H
#define MODEL_H

#include <mujoco/mujoco.h>
#include <xtensor/xarray.hpp>

class Model {
public:
    //constructor
    Model(const mjModel* model, mjData* data);

    // compute the next center of mass coordinate
    void step(double dt);

    //get the current center of mass coordinate
    xt::xarray<double> getCoM() const;
    
private:
    const mjModel* model;
    mjData* data;
    double g;                   //gravity
    double z_CoM;         // center of mass in z
    double w;                   // natural frequency
    double px, py;              // zero moment point
    xt::xarray<double> x_CoM;   // Current x CoM
    xt::xarray<double> y_CoM;   // Current y CoM

    void computeA(double t, xt::xarray<double>& A);
    void computeB(double t, xt::xarray<double>& B);
};

#endif // MODEL_H
