#ifndef MODEL_H
#define MODEL_H

#include <mujoco/mujoco.h>
#include <vector>

class Model {
public:
    //constructor
    Model(const mjModel* model, mjData* data, mjtNum epsilon, bool centered);

    // compute the transition matrices of the discrete model
    void computeTransitionMatrices();
    
    //get constant reference of the transition matrices
    const std::vector<mjtNum>& getA() const;
    const std::vector<mjtNum>& getB() const;
    const std::vector<mjtNum>& getC() const;
    const std::vector<mjtNum>& getD() const;

private:
    const mjModel* m_model;
    mjData* m_data;
    mjtNum m_epsilon;
    bool m_centered;

    std::vector<mjtNum> m_A;
    std::vector<mjtNum> m_B;
    std::vector<mjtNum> m_C;
    std::vector<mjtNum> m_D;
};

#endif // MODEL_H
