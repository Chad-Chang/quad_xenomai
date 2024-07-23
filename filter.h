#ifndef FILTER_H
#define FILTER_H
#include <data_mutex.h>

class filter
{
private:
    double output;
    double T = 0.001;

public:

    filter();
    double tustin_derivate(double *u, double *y, double tau);
    double LPF1(double *u, double *y, double tau);
};

#endif // FILTER_H
