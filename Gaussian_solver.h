#ifndef GAUSSIAN_SOLVER_H
#define GAUSSIAN_SOLVER_H

#include <math.h>
#include <string.h>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Find joint angle correction dq reducing the robot Cartesian position and orientation error
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 * Perform forward - elimination
 */
int forward_elimination(
    double upper[6][6],
    double rhs[6]);

/*
 * Perform back - substitution
 */
void back_substitution(
    const double upper[6][6],
    const double rhs[6],
    double x[6]);

/*
 * Find joint angle correction dq
 */
int solve_6x6(
    const double A[6][6],
    const double b[6],
    double x[6]);

#endif