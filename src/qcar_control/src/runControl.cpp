#include "ros/ros.h"
#include <stdio.h>
#include <vector>
#include <cmath>
#include <iomanip>
#include <limits>
#include "classControl.h"

#define M_PI 3.14159265358979323846

const int Np = 40;
// ----------------- DYNAMICS -----------------
void qcarDynamics(const std::vector<double>& x,
                  const std::vector<double>& u,
                  double dt,
                  std::vector<double>& x_next)
{
    double L      = 0.256;
    double rw     = 0.033;
    double gr     = (13.0/70.0) * (19.0/37.0);
    double Rm     = 0.470;
    double Kt     = 0.0027;
    double Ke     = 0.0027;
    double V_max  = 11.1;
    double m      = 2.7;
    double c_drag = 0.5;

    double X   = x[0];
    double Y   = x[1];
    double psi = x[2];
    double v   = x[3];

    double delta    = u[0];
    double throttle = u[1];

    double V_mot  = throttle * V_max;
    double om_mot = (v / rw) * gr;
    double V_bemf = Ke * om_mot;
    double I_mot  = (V_mot - V_bemf) / Rm;
    double F_drv  = Kt * I_mot * gr / rw;
    double F_fric = c_drag * v;
    double a      = (F_drv - F_fric) / m;

    x_next[0] = X   + dt * v * cos(psi);
    x_next[1] = Y   + dt * v * sin(psi);
    x_next[2] = psi + dt * (v / L) * tan(delta);
    x_next[3] = v   + dt * a;
}

// ----------------- MPC -----------------
std::vector<double> runMPC(const std::vector<double>& x0,
                           const std::vector<std::vector<double>>& x_ref,
                           double dt,
                           const std::vector<double>& prev_u)
{
    

    // ---- WEIGHTS ----
    double w_pos   = 10000.0;
    double w_head  = 700.0;
    double w_vel   = 100.0;
    double w_input = 0.001; 
    double w_rate  = 500.0;
    
    
    double best_cost = std::numeric_limits<double>::max();
    std::vector<double> best_u(2, 0.0);

    // ---- SEARCH SPACE ----
    double max_delta = 35.0 * M_PI / 180.0; // ≈ 0.61 rad

    for(double delta = -max_delta; delta <= max_delta; delta += 0.1)
    {
        for(double throttle = 0.0; throttle <= 1; throttle += 0.1)
        {
            std::vector<double> x_pred = x0;
            double cost = 0.0;

            for(int k = 0; k < Np; k++)
                {
                    std::vector<double> u = {delta, throttle};

                    // 1. declare first
                    std::vector<double> x_next(4);

                    // 2. simulate
                    qcarDynamics(x_pred, u, dt, x_next);

                    // 3. compute acceleration
                    double v_current = x_pred[3];
                    double v_next    = x_next[3];
                    double a = (v_next - v_current) / dt;

                    // 4. constraint check
                    if(a > 3.0 || a < -3.0)
                    {
                        cost += 1e6;
                        break;  
                    }

                    // 5. update state
                    x_pred = x_next;

                    // 6. cost calculation
                    double dx = x_pred[0] - x_ref[0][k];
                    double dy = x_pred[1] - x_ref[1][k];
                    double pos_err = dx*dx + dy*dy;

                    double dpsi = atan2(sin(x_pred[2] - x_ref[2][k]),
                                        cos(x_pred[2] - x_ref[2][k]));
                    double head_err = dpsi*dpsi;

                    double vel_err = pow(x_pred[3] - x_ref[3][k], 2);

                    cost += w_pos*pos_err + w_head*head_err + w_vel*vel_err;

                    
                }
                    
                    double d_delta    = delta - prev_u[0];
                    double d_throttle = throttle - prev_u[1];
                    cost += w_rate * (d_delta*d_delta + d_throttle*d_throttle);

            // control effort penalty
            cost += w_input * (delta*delta + throttle*throttle);

            if(cost < best_cost)
            {
                best_cost = cost;
                best_u[0] = delta;
                best_u[1] = throttle;
            }
        }
    }

    return best_u;
}

// ----------------- MAIN -----------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    double dt = 0.01;
    double time = 0;

    classControl qcarController(&n);
    std::vector<double> prev_u = {0.0, 0.0};  
    
    int prevIndex = 0;
    const int window = 30;

    while(ros::ok())
    {
        if(qcarController.getWPT(0) != -1)
        {
            while(ros::ok())
            {
                // ---- CURRENT STATE ----
                States* state = qcarController.getStates();
                std::vector<double> x0 = {state->East, state->North, state->Psi, state->Vel}; // get current state of car

                int indexCurrent = qcarController.getNearestIndexForward(state->East, state->North, prevIndex, window);
                prevIndex = indexCurrent;

                // ---- BUILD REFERENCE ----
                std::vector<std::vector<double>> x_ref(4, std::vector<double>(Np));
                int wpSize = qcarController.getWPVec().size();

                double t_now = qcarController.getWPT(indexCurrent);

                for(int k = 0; k < Np; k++)
                {
                    double t_future = t_now + k * dt;   // ensure that guidance and controll working on the same time frame
                    int idx = qcarController.getIndex(t_future);

                    x_ref[0][k] = qcarController.getWPX(idx);
                    x_ref[1][k] = qcarController.getWPY(idx);

                    int next_idx = std::min(idx + 1, wpSize - 1);
                    x_ref[2][k] = atan2(
                        qcarController.getWPY(next_idx) - qcarController.getWPY(idx),
                        qcarController.getWPX(next_idx) - qcarController.getWPX(idx)
                    );

                    x_ref[3][k] = qcarController.getVel();
                }

                // ---- RUN MPC ----
                std::vector<double> u = runMPC(x0, x_ref, dt, prev_u);

                double delta = u[0];
                double throttle = u[1];
                prev_u = u;

                // ---- CONVERT THROTTLE TO WHEEL SPEED ----
                double omega = throttle * 10.0;

                qcarController.command(omega, delta);

                time += dt;

                ros::spinOnce();
                loop_rate.sleep();

                std::cout << "[t] " << time
                          << " | omega " << omega
                          << " | delta " << delta
                          << " | throttle " << throttle << "\n";

                if(time > qcarController.getWPVec().back() - 1)
                {
                    qcarController.command(0,0);
                    return 0;
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
