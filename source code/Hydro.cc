/* Plugin for Hydro-dynamics Force
 * Contain Reactive Force and Resistant Force
 *
 * Segment(0)           is head with elliptic cylinder,
 * Segments(1:s_num-3) are body with elliptic cylinder,
 * Segment(s_num-2)     is peduncle with elliptic cylinder,
 * Segment(s_num-1)     is plate
 *
 * Created by Donghao Li
 * Update time: March 23rd 2021
 *
 *
 */
#ifndef __GAZEBO_HYDRO_PLUGIN_HH__
#define __GAZEBO_HYDRO_PLUGIN_HH__

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"
#include <algorithm>
#include <cstdio>
#include <iomanip>
#include <numeric>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

using std::endl;    using std::cout;  using std::fstream;   using std::FILE;    using std::ios; using std::to_string;   using std::stringstream;
using std::fscanf;  using ignition::math::Vector3d;     using ignition::math::Pose3d;   using std::ifstream;    using std::getline;     using std::string;

namespace gazebo
{
class Hydro : public ModelPlugin
  {
    public: Hydro(){};

    /// Fix #4: robust sign function. Returns 0.0 (instead of NaN/inf) when
    /// |v| is at/below eps, so callers never need to divide by v directly.
    private: static double Sgn(double v, double eps = 1E-08)
    {
        if (!std::isfinite(v) || std::fabs(v) <= eps) return 0.0;
        return (v > 0.0) ? 1.0 : -1.0;
    }
    private: static double ClampWrench(double raw, double scale)
    {
        if (!std::isfinite(raw)) return 0.0;
        const double lim = scale;
        if (raw >  lim) return  lim;
        if (raw < -lim) return -lim;
        return raw;
    }

    private: int NoA    = 4;
    private: int s_num  = NoA + 2;
    private: int hp_num = NoA + 6;

    private: std::vector<double> Param;

    private: std::vector<physics::LinkPtr> link;

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;

        this->NoA = 4;
        if (_sdf && _sdf->HasElement("num_actuators")) {
            this->NoA = _sdf->Get<int>("num_actuators");
        }
        if (NoA != 2 && NoA != 4 && NoA != 6) {
            throw std::runtime_error("[Hydro] Unsupported num_actuators=" + to_string(NoA) +
                " (only 2, 4, or 6 are currently supported).");
        }
        this->s_num  = NoA + 2;
        this->hp_num = NoA + 6;

        std::string param_path = "/home/donghao/result/Hydro_ID/hyd_param.csv";
        if (_sdf && _sdf->HasElement("param_path")) {
            param_path = _sdf->Get<std::string>("param_path");
        }
        ifstream Hydrofile(param_path, ios::in);
        if (!Hydrofile.is_open()) {
            throw std::runtime_error("[Hydro] Cannot open hydrodynamic parameter file: " + param_path);
        }
        string hydroline;
        if (!getline(Hydrofile, hydroline) || hydroline.empty()) {
            throw std::runtime_error("[Hydro] Hydrodynamic parameter file is empty: " + param_path);
        }
        stringstream iss(hydroline);
        string colo;
        this->Param.assign(hp_num, 0.0);
        for (int i = 0; i < hp_num; i++) {
            if (!getline(iss, colo, ',')) {
                throw std::runtime_error("[Hydro] Parameter file has fewer than " +
                    to_string(hp_num) + " columns: " + param_path);
            }
            stringstream convertor(colo);
            convertor >> this->Param[i];
            if (convertor.fail()) {
                throw std::runtime_error("[Hydro] Could not parse numeric value '" + colo +
                    "' (column " + to_string(i) + ") in: " + param_path);
            }
        }

        // Cache links once instead of re-fetching every tick.
        const auto model_links = this->model->GetLinks();
        if (static_cast<int>(model_links.size()) < s_num) {
            throw std::runtime_error("[Hydro] Model has " + to_string(model_links.size()) +
                " links, but num_actuators=" + to_string(NoA) + " requires at least " +
                to_string(s_num) + ".");
        }
        this->link.assign(model_links.begin(), model_links.begin() + s_num);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Hydro::OnUpdate, this));
    }

    public: virtual void Init(){};

    private: static constexpr double FIXED_CD_GROUP23 = 2.25;

    private: void GetSegmentCoeffs(int i, double &C_d, double &C_a, double &C_p) const
    {
        if (i == 0) {                                   // Head
            C_d = fabs(Param[0]); C_a = fabs(Param[1]); C_p = fabs(Param[2]);
        }
        else if (i == s_num - 1) {                       // Tail
            C_d = fabs(Param[3]); C_a = fabs(Param[4]); C_p = fabs(Param[5]);
        }
        else if (NoA == 2) {                              // NoA==2: single "body" group (not group 2/3 -- see note above)
            C_d = fabs(Param[6]); C_a = fabs(Param[7]); C_p = 0.0;
        }
        else if (i == 1) {                                // First actuator segment
            C_d = fabs(Param[6]); C_a = fabs(Param[7]); C_p = 0.0;
        }
        else if (NoA == 4) {                              // NoA==4: group 2/3 (body/peduncle), Cd fixed
            C_d = FIXED_CD_GROUP23; C_a = fabs(Param[9]); C_p = 0.0;
        }
        else if (i == 2 || i == 3) {                      // NoA==6: group 2, Cd fixed
            C_d = FIXED_CD_GROUP23; C_a = fabs(Param[9]); C_p = 0.0;
        }
        else {                                            // NoA==6: group 3 (remaining, incl. peduncle), Cd fixed
            C_d = FIXED_CD_GROUP23; C_a = fabs(Param[11]); C_p = 0.0;
        }
    }

    private: void OnUpdate()
    {
        common::Time currTime = this->model->GetWorld()->SimTime();
        common::Time stepTime = currTime - this->prevUpdateTime;
        this->prevUpdateTime = currTime;
        const double simu_time = this->model->GetWorld()->SimTime().Double();
        const double simu_step = simu_time*1000*10.0; const int record_step = 3*1000*10;
    /////////////////////////////////////////////////////////////////////// Variables Declarantion
    ////////////////////////////////////// Coefficient
    /// fluid information
        const double pho_f = 1000.0;
    /// partial of added mass added in urdf, 0.0 means all in plugin
        const double Seg_M_total_ratio = 0.75;

    /////////////////////////////////////////////////////////////////// CK: Coefficient
    /// coefficient for fluid pressure force
        double C_f;
        double C_d, C_a, C_p;
//        C_f = Param[8];
        C_f  = 0.06;

    ///////////////////////////////////////////////////////////////////
    //////////////////////////////// Added Mass Parameters
        const double Info_M_total[4] = {12.94E-03,12.10E-03,8.935E-03,12.67E-03};   // int(M_0*ds)
        const double Info_M_S[4]     = {3.593E-04,1.619E-04,8.823E-05,1.773E-04};   // int(M_0*s*ds)
        const double Info_M_T[4]     = {1.122E-05,2.886E-06,1.162E-06,3.310E-06};   // int(M_0*s*ds)
        const double Info_M_0        =  4.524E-01;                               // added mass per length
        const double Info_M_l        =  4.524E-01;                               // added mass per length
        const double Info_cog[4]     = {28.57E-03,11.70E-03,7.515E-03,14.00E-03};   // Relative to s=0
        const double Info_lt[4]      = {43.00E-03,26.75E-03,19.75E-03,28.00E-03};   // Segment length
        const double Info_h_0        =  24.00E-03;                               // Segment depth
        const double Info_IH0[4]     = {7.991E-04,6.420E-04,4.740E-04, 6.720E-04};  // Resistive force param
        const double Info_IH1[4]     = {2.025E-05,8.587E-06,4.681E-06, 9.408E-06};  // Resistive force param
        const double Info_IH2[4]     = {6.118E-07,1.531E-07,6.163E-08, 1.756E-07};  // Resistive force param
        const double Info_IH3[4]     = {2.015E-08,3.072E-09,9.129E-10, 3.688E-09};  // Resistive force param
        const double Info_IP[4]      = {2.400E-03,1.928E-03,1.424E-03, 1.383E-03};  // Resistive force param
        const double Info_ls[4]      = {  0.0E-03,  2.0E-03,  2.0E-03,   0.0E-03};    // Reactive  force start point
        const double record_point    =  25.00E-03;

        double Seg_M_total, Seg_M_S,  Seg_M_T,  Seg_cog, Seg_lt, Seg_IH0, Seg_IH1, Seg_IH2, Seg_IH3, Seg_IP, Seg_lc, Seg_ls;
        double Seg_M_0 = Info_M_0;
        double Seg_M_l = Info_M_l;
        double Seg_h_0 = Info_h_0;
        double Seg_h_1 = 5.367E-03;      double Seg_lm = 25.00E-03;        double Seg_K = (Seg_h_0 - Seg_h_1)/Seg_lm;
     ////////////////////////// Hydro-dynamic Force related Variables
     /// wrench variables
        std::vector<double> react_x(s_num,0.0);   std::vector<double> react_y(s_num,0.0);   std::vector<double> react_zz(s_num,0.0);
        std::vector<double> resis_x(s_num,0.0);   std::vector<double> resis_y(s_num,0.0);   std::vector<double> resis_t(s_num,0.0);
        double fluip_x[2] ={0.0};
     /// absolute kinematic in mobile frame
        std::vector<double> vel_x(s_num,0.0);  std::vector<double> vel_y(s_num,0.0); std::vector<double> vel_y_l(s_num,0.0);
        std::vector<double> acc_x(s_num,0.0);  std::vector<double> acc_y(s_num,0.0); std::vector<double> acc_y_ant(s_num,0.0);
        std::vector<double> vel_yaw(s_num,0.0);   std::vector<double> theta3(s_num,0.0);
    /// Kinematic Variable of Link
        std::vector<Pose3d>   WP(s_num);              // link position
        std::vector<Vector3d> WLV(s_num);           // pivot linear velocity in world frame
        Vector3d WV_record[1];     // linear velocity of record point in world frame
        std::vector<Vector3d> WLA(s_num);           // pivot linear acceleration in world frame
        std::vector<Vector3d> WAV(s_num);           // angular velocity in world frame
        std::vector<Vector3d> WAA(s_num);           // angular acceleration in world frame
    /// Variables for calibration
        Vector3d cali_WLA; // linear acceleration in world
        Vector3d cali_WAA; // linear acceleration in world
    /// Variables for recording
        std::vector<double> rcra_x(s_num,0.0);  std::vector<double> rcra_y(s_num,0.0);    std::vector<double> rcra_z(s_num,0.0);
        double  CoGVel_x  = 0.0;       double CoGVel_y   = 0.0;         Vector3d WLV_CoG[1];
    ////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////// Dynamic Process
    ////////////////////// Calibration for redundant acceleration
        for (int i=0;i<s_num;i++) {
            GetSegmentCoeffs(i, C_d, C_a, C_p);

            if (i==0){                                              // Head
                Seg_M_total = Info_M_total[0];
                Seg_M_S     = Info_M_S[0];
                Seg_M_T     = Info_M_T[0]*(0.5-C_a);
                Seg_cog     = Info_cog[0]-Info_ls[0];
                Seg_lt      = Info_lt[0];
                Seg_IH0     = Info_IH0[0];
                Seg_IH1     = Info_IH1[0];
                Seg_IH2     = Info_IH2[0];
                Seg_IH3     = Info_IH3[0];
                Seg_IP      = Info_IP[0];
                Seg_ls      = Info_ls[0];
//                Seg_M_0     = Info_M_0;
            }
            else if (i==s_num-1) {                                  // Tail
                Seg_M_total = Info_M_total[3];
                Seg_M_S     = Info_M_S[3];
                Seg_M_T     = Info_M_T[3]*(0.5-C_a);
                Seg_cog     = Info_cog[3]-Info_ls[3];
                Seg_lt      = Info_lt[3];
                Seg_IH0     = Info_IH0[3];
                Seg_IH1     = Info_IH1[3];
                Seg_IH2     = Info_IH2[3];
                Seg_IH3     = Info_IH3[3];
                Seg_IP      = Info_IP[3];
                Seg_ls      = Info_ls[3];
            }
            else if (i==s_num-2) {                                  // Peduncle
                Seg_M_total = Info_M_total[2];
                Seg_M_S     = Info_M_S[2];
                Seg_M_T     = Info_M_T[2]*(0.5-C_a);
                Seg_cog     = Info_cog[2]-Info_ls[2];
                Seg_lt      = Info_lt[2];
                Seg_IH0     = Info_IH0[2];
                Seg_IH1     = Info_IH1[2];
                Seg_IH2     = Info_IH2[2];
                Seg_IH3     = Info_IH3[2];
                Seg_IP      = Info_IP[2];
                Seg_ls      = Info_ls[2];
            }
            else {                                                  // Body
                Seg_M_total = Info_M_total[1];
                Seg_M_S     = Info_M_S[1];
                Seg_M_T     = Info_M_T[1]*(0.5-C_a);
                Seg_cog     = Info_cog[1]-Info_ls[1];
                Seg_lt      = Info_lt[1];
                Seg_IH0     = Info_IH0[1];
                Seg_IH1     = Info_IH1[1];
                Seg_IH2     = Info_IH2[1];
                Seg_IH3     = Info_IH3[1];
                Seg_IP      = Info_IP[1];
                Seg_ls      = Info_ls[1];
            }
            {
                // get Rotation angle
                WP[i]        = link[i]->WorldPose();
                theta3[i]    = WP[i].Rot().Yaw();
                // linear velocity transfer
                WLV[i]       = link[i]->WorldLinearVel(Vector3d(Seg_ls, 0, 0));         // Velocity of Anterior Point in Global frame
                vel_x[i]     = WLV[i].X()*cos(theta3[i])+WLV[i].Y()*sin(theta3[i]);     // Velocity of Anterior Point in Mobile frame
                vel_y[i]     = WLV[i].Y()*cos(theta3[i])-WLV[i].X()*sin(theta3[i]);     // Velocity of Anterior Point in Mobile frame
                WLA[i]       = link[i]->WorldLinearAccel();                             // Acceleration of CoG in Global frame
                acc_x[i]     = WLA[i].X()*cos(theta3[i])+WLA[i].Y()*sin(theta3[i]);     // Acceleration of CoG in Mobile frame
                acc_y[i]     =-WLA[i].X()*sin(theta3[i])+WLA[i].Y()*cos(theta3[i]);     // Acceleration of CoG in Mobile frame
                // angular velocity transfer
                WAV[i]       = link[i]->WorldAngularVel();                              // Angular Velocity Vector
                vel_yaw[i]   = WAV[i].Z();                                              // Yaw Angular Velocity
                WAA[i]       = link[i]->WorldAngularAccel();                            // Yaw Angular Acceleration
                vel_y_l[i]   = (vel_y[i]+vel_yaw[i]*Seg_lt);                            // Velocity of Post Point in Mobile frame
                acc_y_ant[i] = acc_y[i]-Seg_cog*WAA[i].Z();                             // Velocity of Anterior Point in Mobile frame
                // reactive force
                react_x[i]   =  C_a * (
                                +Seg_M_S     * pow(vel_yaw[i],2)
                                +Seg_M_total * vel_y[i] * vel_yaw[i]
                                         )                                              // Reactive force
                                +Seg_M_total * Seg_M_total_ratio * acc_x[i];            // Mass transfer force
                react_y[i]   =  C_a * (
                                +Seg_M_total * vel_x[i] * vel_yaw[i]
                                -Seg_M_total * acc_y_ant[i]
                                -Seg_M_S     * WAA[i].Z()
                                -Seg_M_l     * vel_x[i] * vel_y_l[i]
                                +Seg_M_0     * vel_x[i] * vel_y[i]
                                         )
                                +Seg_M_total * Seg_M_total_ratio * acc_y[i];
                react_zz[i]  = -C_a * (
                                +Seg_M_l     * Seg_lt * vel_x[i] * vel_y_l[i]
                                +Seg_M_S     * acc_y_ant[i]
                                -Seg_M_S     * vel_x[i] * vel_yaw[i]
                                         )
                                +Seg_M_total * Seg_M_total_ratio * acc_y[i] * Seg_cog
                                +Seg_M_T * WAA[i].Z();


                if(i==0){
                    fluip_x[0]  = +0.5 * C_p * C_a * Seg_M_0 * pow(vel_y[0],2);
                    react_x[i] += fluip_x[0];
                }
                else if (i==s_num-1) {
                    fluip_x[1]  = -0.5 * C_p * C_a * Seg_M_l * pow(vel_y_l[s_num-1],2);
                    react_x[i] += fluip_x[1];
                }
                rcra_x[i] = C_a * (
                            +Seg_M_S     * pow(vel_yaw[i],2)
                            +Seg_M_total * vel_y[i] * vel_yaw[i]
                                     );
                rcra_y[i] = C_a * (
                            +Seg_M_total * vel_x[i] * vel_yaw[i]
                            -Seg_M_total * acc_y_ant[i]
                            -Seg_M_S     * WAA[i].Z()
                            -Seg_M_l     * vel_x[i] * vel_y_l[i]
                            +Seg_M_0     * vel_x[i] * vel_y[i]
                                     );
                rcra_z[i] =-C_a * (
                            +Seg_M_l     * Seg_lt * vel_x[i] * vel_y_l[i]
                            +Seg_M_S     * acc_y_ant[i]
                            -Seg_M_S     * vel_x[i] * vel_yaw[i]
                                     );

                // Resis Force
                resis_x[i] = 0.0;   resis_y[i] = 0.0;   resis_t[i] = 0.0;

                if(vel_yaw[i]>=1E-08 || vel_yaw[i]<=-1E-08){
                    Seg_lc = -vel_y[i]/vel_yaw[i];
                }
                else {
                    Seg_lc = 0.0;
                }
                const double sgn_vel_y = Sgn(vel_y[i]);
                // Friction Calculation
                resis_x[i] = (-0.5*pho_f*C_f)*fabs(vel_x[i])*vel_x[i]*Seg_IP;
                // Drag Calculation
                if(vel_yaw[i]<=1E-08 && vel_yaw[i]>=-1E-08){
                    resis_y[i] = (-0.5*pho_f*C_d)*fabs(vel_y[i])*vel_y[i]*Seg_IH0;
                    resis_t[i] = (-0.5*pho_f*C_d)*fabs(vel_y[i])*vel_y[i]*Seg_IH1;
                }
                else if(vel_y[i]<=1E-08 && vel_y[i]>=-1E-08){
                    resis_y[i] = (-0.5*pho_f*C_d)*fabs(vel_yaw[i])*vel_yaw[i]*Seg_IH2;
                    resis_t[i] = (-0.5*pho_f*C_d)*fabs(vel_yaw[i])*vel_yaw[i]*Seg_IH3;
                }
                else if (Seg_lc<0 || Seg_lc>=Seg_lt ) {
                    resis_y[i]= (-0.5*pho_f*C_d) * sgn_vel_y
                            *(
                               +pow(vel_y[i],2)           * Seg_IH0
                               +2*vel_yaw[i]*vel_y[i]     * Seg_IH1
                               +pow(vel_yaw[i],2)         * Seg_IH2
                              );
                    resis_t[i]= (-0.5*pho_f*C_d) * sgn_vel_y
                            *(
                               +pow(vel_y[i],2)           * Seg_IH1
                               +2*vel_yaw[i]*vel_y[i]     * Seg_IH2
                               +pow(vel_yaw[i],2)         * Seg_IH3
                              );
                }
                else {
                    if(i==0){                       // head depth changed from 0 to Seg_lm
                        if(Seg_lc < Seg_lm){
                            resis_y[i]= (-0.5*pho_f*C_d) * sgn_vel_y
                                    *(
                                        +pow(vel_y[i],2) * Seg_h_1                                                 * (2. *     Seg_lc    -     Seg_lm   )
                                        +1./2. * (2 * vel_y[i] * vel_yaw[i] * Seg_h_1 + pow(vel_y[i],2) * Seg_K)   * (2. * pow(Seg_lc,2) - pow(Seg_lm,2))
                                        +1./3. * (2 * vel_y[i] * vel_yaw[i] * Seg_K + pow(vel_yaw[i],2) * Seg_h_1) * (2. * pow(Seg_lc,3) - pow(Seg_lm,3))
                                        +1./4. * pow(vel_yaw[i],2) * Seg_K                                         * (2. * pow(Seg_lc,4) - pow(Seg_lm,4))
                                        +        Seg_h_0 * pow(vel_y[i],2)     * (    Seg_lt    -     Seg_lm   )
                                        +        Seg_h_0 * vel_y[i]*vel_yaw[i] * (pow(Seg_lt,2) - pow(Seg_lm,2))
                                        +1./3. * Seg_h_0 * pow(vel_yaw[i],2)   * (pow(Seg_lt,3) - pow(Seg_lm,3))
                                      );
                            resis_t[i]= (-0.5*pho_f*C_d) * sgn_vel_y
                                    *(
                                        +1./2. * pow(vel_y[i],2) * Seg_h_1                                         * (2. * pow(Seg_lc,2) - pow(Seg_lm,2))
                                        +1./3. * (2 * vel_y[i] * vel_yaw[i] * Seg_h_1 + pow(vel_y[i],2) * Seg_K)   * (2. * pow(Seg_lc,3) - pow(Seg_lm,3))
                                        +1./4. * (2 * vel_y[i] * vel_yaw[i] * Seg_K + pow(vel_yaw[i],2) * Seg_h_1) * (2. * pow(Seg_lc,4) - pow(Seg_lm,4))
                                        +1./5. * pow(vel_yaw[i],2) * Seg_K                                         * (2. * pow(Seg_lc,5) - pow(Seg_lm,5))
                                        +1./2. * Seg_h_0 * pow(vel_y[i],2)     * (pow(Seg_lt,2) - pow(Seg_lm,2))
                                        +1./3. * Seg_h_0 * vel_y[i]*vel_yaw[i] * (pow(Seg_lt,3) - pow(Seg_lm,3))
                                        +1./4. * Seg_h_0 * pow(vel_yaw[i],2)   * (pow(Seg_lt,4) - pow(Seg_lm,4))
                                      );
                        }
                        else {
                            resis_y[i]= (-0.5*pho_f*C_d) * sgn_vel_y
                                    *(
                                        +pow(vel_y[i],2) * Seg_h_1                                                 * (     Seg_lm    )
                                        +1./2. * (2 * vel_y[i] * vel_yaw[i] * Seg_h_1 + pow(vel_y[i],2) * Seg_K)   * ( pow(Seg_lm,2) )
                                        +1./3. * (2 * vel_y[i] * vel_yaw[i] * Seg_K + pow(vel_yaw[i],2) * Seg_h_1) * ( pow(Seg_lm,3) )
                                        +1./4. * pow(vel_yaw[i],2) * Seg_K                                         * ( pow(Seg_lm,4) )
                                        +        Seg_h_0 * pow(vel_y[i],2)     * (2. * Seg_lc        -     Seg_lt    -     Seg_lm   )
                                        +        Seg_h_0 * vel_y[i]*vel_yaw[i] * (2. * pow(Seg_lc,2) - pow(Seg_lt,2) - pow(Seg_lm,2))
                                        +1./3. * Seg_h_0 * pow(vel_yaw[i],2)   * (2. * pow(Seg_lc,3) - pow(Seg_lt,3) - pow(Seg_lm,3))
                                      );
                            resis_t[i]= (-0.5*pho_f*C_d) * sgn_vel_y
                                    *(
                                        +1./2. * pow(vel_y[i],2) * Seg_h_1                                         * ( pow(Seg_lm,2) )
                                        +1./3. * (2 * vel_y[i] * vel_yaw[i] * Seg_h_1 + pow(vel_y[i],2) * Seg_K)   * ( pow(Seg_lm,3) )
                                        +1./4. * (2 * vel_y[i] * vel_yaw[i] * Seg_K + pow(vel_yaw[i],2) * Seg_h_1) * ( pow(Seg_lm,4) )
                                        +1./5. * pow(vel_yaw[i],2) * Seg_K                                         * ( pow(Seg_lm,5) )
                                        +1./2. * Seg_h_0 * pow(vel_y[i],2)     * (2. * pow(Seg_lc,2) - pow(Seg_lt,2) - pow(Seg_lm,2))
                                        +1./3. * Seg_h_0 * vel_y[i]*vel_yaw[i] * (2. * pow(Seg_lc,3) - pow(Seg_lt,3) - pow(Seg_lm,3))
                                        +1./4. * Seg_h_0 * pow(vel_yaw[i],2)   * (2. * pow(Seg_lc,4) - pow(Seg_lt,4) - pow(Seg_lm,4))
                                      );
                        }
                    }
                    else {
                        resis_y[i]=(-0.5*pho_f*C_d*Seg_h_0) * sgn_vel_y
                                *(
                                   +pow(vel_y[i],2)     * (2.0*Seg_lc-Seg_lt)
                                   +vel_y[i]*vel_yaw[i] * (2.0*pow(Seg_lc,2)-pow(Seg_lt,2))
                                   +pow(vel_yaw[i],2)   * (2./3.*pow(Seg_lc,3)-1.0/3.0*pow(Seg_lt,3))
                                  );
                        resis_t[i]=(-0.5*pho_f*C_d*Seg_h_0)*sgn_vel_y
                                *(
                                   +pow(vel_y[i],2)     * (1.0*pow(Seg_lc,2)-0.5*pow(Seg_lt,2))
                                   +vel_y[i]*vel_yaw[i] * (4.0/3.0*pow(Seg_lc,3)-2.0/3.0*pow(Seg_lt,3))
                                   +pow(vel_yaw[i],2)   * (0.5*pow(Seg_lc,4)-1.0/4.0*pow(Seg_lt,4))
                                  );
                    }
                }
                {
                    const double fx_raw = react_x[i] + resis_x[i];
                    const double fy_raw = react_y[i] + resis_y[i];
                    const double tz_raw = react_zz[i] + resis_t[i];

                    double link_mass = 0.0, link_izz = 0.0;
                    if (auto inert = link[i]->GetInertial()) {
                        link_mass = inert->Mass();
                        link_izz  = inert->IZZ();
                    }
                    link_mass = std::isfinite(link_mass) ? std::max(link_mass, 1E-06) : 1E-06;
                    link_izz  = std::isfinite(link_izz)  ? std::max(link_izz,  1E-09) : 1E-09;

                    const double MAX_LIN_ACCEL = 50.0;
                    const double MAX_ANG_ACCEL = 500.0;

                    const double fx = ClampWrench(fx_raw, MAX_LIN_ACCEL * link_mass);
                    const double fy = ClampWrench(fy_raw, MAX_LIN_ACCEL * link_mass);
                    const double tz = ClampWrench(tz_raw, MAX_ANG_ACCEL * link_izz);

                    link[i]->AddLinkForce( Vector3d(fx, fy, 0.0), Vector3d(Seg_ls, 0,0) ) ;
                    link[i]->AddTorque( Vector3d(0.0,  0.0,  tz) );
                }
                }         // End of link[i]
            }           // End of i loop
    //////////////////////////////////////////////////////////////////////////////////// Recording Process
        std::vector<double> marker_y(s_num); double marker_length;
        fstream links_record;   fstream joint_record;            fstream links2_record;             fstream force_record;     fstream kinet_record;
        if (simu_step == record_step){
            links_record.open("/home/donghao/result/Hydro_ID/links_record.csv",ios::out);       links_record.close();

        }
        else if (simu_step>record_step) {
            links_record.open("/home/donghao/result/Hydro_ID/links_record.csv",ios::app);
            links_record<<simu_time<<','<<(WP[0].Pos().Y())*1.E3<<','<<(WP[0].Pos().Y()+42E-3*sin(WP[0].Rot().Yaw()))*1.E3;
            for(int i=1;i<s_num;i++) {
                if (i == s_num-1){
                    marker_length = 27.25E-3;
                }
                else if (i == s_num-2) {
                    marker_length = 19E-3;
                }
                else {
                    marker_length = 27.75E-3;
                }
                links_record<<','<<(WP[i].Pos().Y()+marker_length*sin(WP[i].Rot().Yaw()))  *1.E3;
//                cout<< i << ','<< WP[i].Pos().Y()<<','<< marker_length<<','<<WP[i].Rot().Yaw()<<','<< marker_length*sin(WP[i].Rot().Yaw())<<endl;
            }
            {
                WLV_CoG[0]= link[0]->WorldLinearVel(Vector3d(25E-3, 0, 0));
                CoGVel_x = WLV_CoG[0].X();
                CoGVel_y = WLV_CoG[0].Y();
            }
            links_record<<','<<(CoGVel_x)*1.E3<<endl;             links_record.close();
        }

                   // End of Recording
    ////////////////////////////////////////////////////////////////////////////////////
      }             // End of OnUpdate

    private: event::ConnectionPtr updateConnection;

    private: physics::ModelPtr model;

    private: common::Time prevUpdateTime;

  };
  GZ_REGISTER_MODEL_PLUGIN(Hydro)
}
#endif
