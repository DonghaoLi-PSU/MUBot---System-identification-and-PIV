/* Plugin for Megnetic-electric Actuator & Spring
 *
 * Joints(0:s_num-3) contain active actuator and passive spring
 * Joint (s_num-2)   contain passive spring
 *
 * Created by Donghao Li
 * Update time: March 23rd 2021
 *
 * This file merges what used to be three near-duplicate plugin source
 * files -- actuator_2.cc, actuator_4.cc, actuator_6.cc -- differing only
 * in the number of actuators (NoA) and the "AN2"/"AN4"/"AN6" substring in
 * the voltage-table filename, into a single implementation. NoA is now
 * read at Load() time from an SDF <num_actuators> plugin parameter.
 *
 * Fixes applied relative to the original actuator_x.cc files:
 *  1. The per-tick voltage-table and spring-parameter CSV reads are gone.
 *     The original code re-opened "vol_AN{N}_high.csv" from the start and
 *     re-scanned forward to the current line on EVERY physics tick --
 *     since the target line grows every tick, this was O(T^2) total work
 *     over a simulation run, not O(T). Both files are now read once in
 *     Load() into in-memory tables and indexed directly (O(1) per tick).
 *  2. File opens are checked and CSV rows/columns are validated, with a
 *     clear std::runtime_error instead of silently running with whatever
 *     was left in a partially-populated array.
 *  3. Resulting joint torque is sanitized and clamped (scaled to each
 *     joint's own parent/child moment of inertia) before SetForce(),
 *     mirroring the wrench-clamp added to Hydro.cc. Previously only the
 *     input voltage was clamped, not the torque actually applied.
 *  4. Per-segment arrays are sized to s_num (a runtime value here) instead
 *     of a bare "[10]".
 *  5. NoA/hp_num-style triplication across three files is gone; NoA=2/4/6
 *     select validated CSV formats within one file.
 *
 * ACTION REQUIRED: as with Hydro.cc, the model's URDF/SDF <plugin> block
 * must be updated to point at this merged library and pass the actuator
 * count, e.g.:
 *
 *     <plugin name="actuator_plugin" filename="libActuatorPlugin.so">
 *         <num_actuators>4</num_actuators>
 *     </plugin>
 *
 * and CMakeLists.txt must build/install a single "ActuatorPlugin" target
 * instead of three. Neither file was present in the project files
 * available here, so those two changes are not made in this pass.
 */
#ifndef __GAZEBO_ACTUATOR_PLUGIN_HH__
#define __GAZEBO_ACTUATOR_PLUGIN_HH__

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#define PI 3.141592653589793238463

using std::endl;    using std::cout;            using std::fstream;     using std::FILE;        using std::ios;     using std::fscanf;
using std::string;  using std::stringstream;    using std::ifstream;    using std::to_string;   using std::getline; using std::max;     using std::min;

namespace gazebo
{
    class ActuatorPlugin : public ModelPlugin
    {
        public: ActuatorPlugin(){};

        /// Fix #5-equivalent: clamp a raw joint torque to a bound scaled by
        /// that joint's own parent/child moment of inertia, so a transient
        /// (e.g. from a spike in pos[]/vel[]) can't inject an unbounded
        /// torque into the ODE integrator. Non-finite input is sanitized
        /// to 0 rather than propagated. Same MAX_ANG_ACCEL bound used for
        /// the equivalent clamp in Hydro.cc, for consistency.
        private: static double ClampTorque(const physics::JointPtr& joint, double raw)
        {
            if (!std::isfinite(raw)) return 0.0;
            if (!joint) return 0.0;

            double izz_p = 0.0, izz_c = 0.0;
            if (auto parent = joint->GetParent()) {
                if (auto inert = parent->GetInertial()) izz_p = inert->IZZ();
            }
            if (auto child = joint->GetChild()) {
                if (auto inert = child->GetInertial()) izz_c = inert->IZZ();
            }
            const double izz_p_safe = std::isfinite(izz_p) ? izz_p : 1E12;
            const double izz_c_safe = std::isfinite(izz_c) ? izz_c : 1E12;
            double izz_eff = std::min(izz_p_safe, izz_c_safe);
            izz_eff = std::isfinite(izz_eff) ? std::max(izz_eff, 1E-09) : 1E-09;

            const double MAX_ANG_ACCEL = 500.0; // rad/s^2, generous safety backstop.
            const double lim = MAX_ANG_ACCEL * izz_eff;
            if (raw >  lim) return  lim;
            if (raw < -lim) return -lim;
            return raw;
        }

        /// NoA/s_num are runtime values (set once in Load() from the SDF
        /// parameter) instead of being baked in per source file.
        private: int NoA   = 4;
        private: int s_num = NoA + 2;

        /// Fix #1: joints are resolved from the model once at Load() time
        /// and cached here, instead of calling GetJoints()[i] again on
        /// every single physics tick inside OnUpdate().
        private: std::vector<physics::JointPtr> joint;

        /// Fix #1: the full recorded voltage waveform is loaded once here,
        /// one row per timestep of the recording, s_num-2 columns per row.
        private: std::vector<std::vector<double>> voltage_table;

        /// Fix #1: spring-stiffness ratios, loaded once (NoA+1 values).
        private: std::vector<double> stiff_co;

        public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;

            this->NoA = 4;
            if (_sdf && _sdf->HasElement("num_actuators")) {
                this->NoA = _sdf->Get<int>("num_actuators");
            }
            if (NoA != 2 && NoA != 4 && NoA != 6) {
                throw std::runtime_error("[Actuator] Unsupported num_actuators=" + to_string(NoA) +
                    " (only 2, 4, or 6 are currently supported).");
            }
            this->s_num = NoA + 2;

            // Cache joints once instead of re-fetching every tick.
            const auto model_joints = this->model->GetJoints();
            if (static_cast<int>(model_joints.size()) < s_num) {
                throw std::runtime_error("[Actuator] Model has " + to_string(model_joints.size()) +
                    " joints, but num_actuators=" + to_string(NoA) + " requires at least " +
                    to_string(s_num) + ".");
            }
            this->joint.assign(model_joints.begin(), model_joints.begin() + s_num);

            std::string voltage_path = "/home/donghao/result/Hydro_ID/target/vol_AN" + to_string(NoA) + "_high.csv";
            if (_sdf && _sdf->HasElement("voltage_path")) {
                voltage_path = _sdf->Get<std::string>("voltage_path");
            }
            Load_Voltage_Table(voltage_path);

            std::string spring_path = "/home/donghao/result/Hydro_ID/spr_param.csv";
            if (_sdf && _sdf->HasElement("spring_path")) {
                spring_path = _sdf->Get<std::string>("spring_path");
            }
            Load_Spring_Params(spring_path);

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ActuatorPlugin::OnUpdate, this));
        }
        public: virtual void Init(){};

        /// Fix #1/#2: reads the entire recorded voltage waveform once. Each
        /// row has s_num-2 columns (one per active actuator joint), same
        /// format as the original per-tick read, just done a single time.
        private: void Load_Voltage_Table(const std::string& path)
        {
            ifstream in(path, ios::in);
            if (!in.is_open()) {
                throw std::runtime_error("[Actuator] Cannot open voltage file: " + path);
            }
            const int num_active = s_num - 2;
            voltage_table.clear();
            string line;
            int line_no = 0;
            while (getline(in, line)) {
                ++line_no;
                if (line.empty()) continue;
                stringstream iss(line);
                std::vector<double> row(num_active, 0.0);
                string colo;
                for (int i = 0; i < num_active; i++) {
                    if (!getline(iss, colo, ',')) {
                        throw std::runtime_error("[Actuator] Voltage file row " + to_string(line_no) +
                            " has fewer than " + to_string(num_active) + " columns: " + path);
                    }
                    stringstream convertor(colo);
                    convertor >> row[i];
                    if (convertor.fail()) {
                        throw std::runtime_error("[Actuator] Could not parse voltage value '" + colo +
                            "' at row " + to_string(line_no) + " in: " + path);
                    }
                }
                voltage_table.push_back(std::move(row));
            }
            if (voltage_table.empty()) {
                throw std::runtime_error("[Actuator] Voltage file is empty: " + path);
            }
        }

        /// Fix #1/#2: reads the single-line spring-stiffness-ratio CSV once.
        private: void Load_Spring_Params(const std::string& path)
        {
            ifstream in(path, ios::in);
            if (!in.is_open()) {
                throw std::runtime_error("[Actuator] Cannot open spring parameter file: " + path);
            }
            string line;
            if (!getline(in, line) || line.empty()) {
                throw std::runtime_error("[Actuator] Spring parameter file is empty: " + path);
            }
            stringstream hss(line);
            stiff_co.assign(NoA + 1, 0.0);
            string colo;
            for (int i = 0; i < NoA + 1; i++) {
                if (!getline(hss, colo, ',')) {
                    throw std::runtime_error("[Actuator] Spring parameter file has fewer than " +
                        to_string(NoA + 1) + " columns: " + path);
                }
                stringstream convertor(colo);
                convertor >> stiff_co[i];
                if (convertor.fail()) {
                    throw std::runtime_error("[Actuator] Could not parse spring value '" + colo + "' in: " + path);
                }
            }
        }

        private: void OnUpdate()
        {
            common::Time currTime = this->model->GetWorld()->SimTime();
            common::Time stepTime = currTime - this->prevUpdateTime;
            this->prevUpdateTime = currTime;
            // Time and step setup
            const double simu_time = this->model->GetWorld()->SimTime().Double();
            const double voltage_frequncy = 50;
            const int period = 1000/voltage_frequncy;
            const int line_indicator = static_cast<int>(ceil( (round(simu_time*1000))/period ));
            ///////////////////////////////////////////// Variables Declarantion
            std::vector<double> pos(s_num, 0.0);                std::vector<double> vel(s_num, 0.0);                    // Joint state
            std::vector<double> voltage(s_num-2, 0.0);          std::vector<double> torque_actuator(s_num, 0.0);        // Actuator state
            std::vector<double> torque_spring(s_num, 0.0);      const double spring_stiff = 5.328E-03;     const double tail_stiff = 33.73E-03;   // Spring state
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////// Dynamic Process
            // Fix #1: no more per-tick file I/O -- index directly into the
            // table loaded once in Load(). If we've run past the end of the
            // recorded signal, hold at zero voltage (same fallback behavior
            // as the original EOF handling, without the string-building).
            if (line_indicator >= 0 && line_indicator < static_cast<int>(voltage_table.size())) {
                voltage = voltage_table[line_indicator];
            }

            /// Applying Torque for actuators
            for (int i=0;i<s_num-2;i++) {
                {
                    pos[i] = joint[i]->Position(0);
                    vel[i] = joint[i]->GetVelocity(0);
                    const double v = min(max(voltage[i],-20.0),20.0);
                    torque_actuator[i] = (v-0.00391*vel[i])/46*0.00391;
                    torque_spring[i] = -spring_stiff*(1+stiff_co[i])*pos[i];
                    const double torque_raw = torque_spring[i]+torque_actuator[i];
                    joint[i]->SetForce(0, ClampTorque(joint[i], torque_raw));
                }
            }           // End of loop
            /// Applying Torque for passive joint
            {
                pos[s_num-2] = joint[s_num-2]->Position(0);
                torque_spring[s_num-2] = - tail_stiff*(1+stiff_co[s_num-2])*pos[s_num-2];
                joint[s_num-2]->SetForce(0, ClampTorque(joint[s_num-2], torque_spring[s_num-2]));
            }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////
        }       // End of OnUpdate

        private: event::ConnectionPtr updateConnection;

        private: physics::ModelPtr model;

        private: common::Time prevUpdateTime;

    };
    GZ_REGISTER_MODEL_PLUGIN(ActuatorPlugin)
}
#endif
