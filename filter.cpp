#include "filter.h"
#include <data_mutex.h>

filter::filter()
{

}

double filter::tustin_derivate(double *u, double *y, double tau)
{

    return (-2*u[1]+2*u[0] - (T-2*tau)*y[1]) / (T+2*tau);

}

double filter::LPF1(double *u, double *y, double tau)
{
    return (T*(u[0]+u[1])+(2*tau-T)*y[1])/(2*tau+T);
}
