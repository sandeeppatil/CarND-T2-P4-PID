#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

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
    return this->Kp*this->p_error - this->Ki*this->i_error - this->Kd*this->d_error;
}

#if 0
void PID::Twiddle(double f_tol = 0.2) {
    double *p[3];
    double dp[3] = {1,1,1};

    p[0] = &this->p_error;
    p[1] = &this->i_error;
    p[2] = &this->d_error;

    while ((dp[0]+dp[1]+dp[2]) > f_tol)
    {
        for (int i=0; i<3; i++)
        {
            *p[i] += dp[i];

        }
    }
}
#endif
