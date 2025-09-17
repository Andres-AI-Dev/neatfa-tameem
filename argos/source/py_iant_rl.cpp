#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <argos3/core/simulator/simulator.h>
#include "iAnt_loop_functions.h"

namespace py = pybind11;

class IAntRLEnv {
public:
    explicit IAntRLEnv(const std::string& xml_path, bool force_no_viz = false)
        : m_xml_path(xml_path), m_initialized(false), m_prev_fitness(0.0), m_force_no_viz(force_no_viz) {}

    std::vector<argos::Real> reset() {
        auto& sim = argos::CSimulator::GetInstance();
        if (!m_initialized) {
            sim.SetExperimentFileName(m_xml_path);
            sim.LoadExperiment(m_force_no_viz);
            m_initialized = true;
        } else {
            // Fast episode reset without reloading to avoid duplicating controllers
            sim.Reset();
        }

        auto& lf = static_cast<iAnt_loop_functions&>(sim.GetLoopFunctions());
        m_prev_fitness = lf.getFitness();
        return lf.RLGetObservation();
    }

    py::tuple step(argos::Real left_speed, argos::Real right_speed, bool lay_pheromone) {
        auto& sim = argos::CSimulator::GetInstance();
        auto& lf = static_cast<iAnt_loop_functions&>(sim.GetLoopFunctions());

        lf.RLSetAction(left_speed, right_speed, lay_pheromone);
        sim.UpdateSpace();

        auto obs = lf.RLGetObservation();
        argos::Real fitness = lf.getFitness();
        argos::Real reward = fitness - m_prev_fitness;
        m_prev_fitness = fitness;
        bool terminated = lf.RLTerminated();
        bool truncated = lf.RLTruncated();
        py::dict info;
        info["sim_time"] = lf.GetSimTime();
        info["max_sim_time"] = lf.GetMaxSimTime();
        info["food_left"] = lf.GetFoodLeft();
        info["fitness"] = fitness;
        return py::make_tuple(obs, reward, terminated, truncated, info);
    }

    void close() {
        auto& sim = argos::CSimulator::GetInstance();
        if (m_initialized) {
            sim.Destroy();
            m_initialized = false;
        }
    }

private:
    std::string m_xml_path;
    bool m_initialized;
    argos::Real m_prev_fitness;
    bool m_force_no_viz;
};

PYBIND11_MODULE(iant_rl, m) {
    m.doc() = "Pybind11 RL wrapper for iAnt ARGoS environment";
    py::class_<IAntRLEnv>(m, "IAntRLEnv")
        .def(py::init<const std::string&, bool>(), py::arg("xml_path"), py::arg("force_no_viz") = true)
        .def("reset", &IAntRLEnv::reset)
        .def("step", &IAntRLEnv::step,
             py::arg("left_speed"), py::arg("right_speed"), py::arg("lay_pheromone"))
        .def("close", &IAntRLEnv::close);
}


