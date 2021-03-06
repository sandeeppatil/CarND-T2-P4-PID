#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
{
    dp[0] = 1;
    dp[1] = 1;
    dp[2] = 1;
}

PID::~PID() {}

void PID::Init(double f_Kp, double f_Ki, double f_Kd) {
    this->Kp = f_Kp;
    this->Ki = f_Ki;
    this->Kd = f_Kd;

    this->p_error = 0;
    this->i_error = 0;
    this->d_error = 0;
}

void PID::UpdateError(double cte) {
    this->d_error = cte - this->p_error;
    this->p_error = cte;
    this->i_error += cte;
}

double PID::TotalError() {
    return -Kp*p_error - Ki*i_error - Kd*d_error;
}

void PID::Twiddle(double f_tol, double err) {

    double *p[3];

    p[0] = &this->Kp;
    p[1] = &this->Ki;
    p[2] = &this->Kd;
    double best_error = err;
    while ((dp[0]+dp[1]+dp[2]) > f_tol)
    {
        for (int i=0; i<3; i++)
        {
            *p[i] += dp[i];
            if (err < best_error)
            {
                best_error = err;
                dp[i] *= 1.1;
            }
            else
            {
                *p[i] += dp[i];
                dp[i] *= 0.9;
            }
        }
    }
}