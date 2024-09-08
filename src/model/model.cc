#include "model.h"
#include <mujoco/mujoco.h>

Model::Model(const mjModel* model, mjData* data, mjtNum epsilon, bool centered): 
    m_model(model), m_data(data), m_epsilon(epsilon), m_centered(centered) {}

void Model::computeTransitionMatrices() {
    //define dimensions
    int nv = m_model->nv; 
    int na = m_model->na; 
    int nu = m_model->nu; 
    int nsensordata = m_model->nsensordata; 
    
    //resize the matrices
    m_A.resize((2 * nv + na) * (2 * nv + na));
    m_B.resize((2 * nv + na) * nu);
    m_C.resize(nsensordata * (2 * nv + na));
    m_D.resize(nsensordata * nu);

    //update matrices
    mjd_transitionFD(m_model, m_data, 
                     m_epsilon, m_centered, 
                     m_A.data(), m_B.data(),
                     m_C.data(), m_D.data());
}

// Getter methods for constant reference of transition matrices
const std::vector<mjtNum>& Model::getA() const { return m_A; }
const std::vector<mjtNum>& Model::getB() const { return m_B; }  
const std::vector<mjtNum>& Model::getC() const { return m_C; }  
const std::vector<mjtNum>& Model::getD() const { return m_D; }  
