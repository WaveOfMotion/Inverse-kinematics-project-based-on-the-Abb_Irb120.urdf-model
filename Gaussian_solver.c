#include <math.h>
#include <string.h>
#include "robot_constants.h"

/* ----------------- Find joint angle correction dq reducing the robot Cartesian position and orientation error ----------------- */
/* ------------------------------------------------------------------------------------------------------------------------------ */

/*
 * ----------------- Perform fowawrd - elimination -----------------
 */
int forward_elimination(
    double upper[6][6],
    double rhs[6])
{
    int pivotColumn; /* --> column currently being processed */
    int pivotRow;    /* --> row containing the best pivot candidate */
    int row;         /* --> row currently being inspected */
    int col;         /* --> column currently being updated */

    for (pivotColumn = 0; pivotColumn < 6; ++pivotColumn)
    {
        /* ----------------- Find the strongest pivot in the current column ----------------- */
        pivotRow = pivotColumn; /* --> Initial assumption */

        for (row = pivotColumn + 1; row < 6; ++row)
        {
            if (fabs(upper[row][pivotColumn]) > fabs(upper[pivotRow][pivotColumn]))
            {
                pivotRow = row;
            }
        }

        /* ----------------- Matrix is singular or nearly singular ----------------- */
        if (fabs(upper[pivotRow][pivotColumn]) < 1e-14)
            return 0;

        /* ----------------- Move the strongest pivot onto the diagonal ----------------- */
        if (pivotRow != pivotColumn)
        {
            for (col = 0; col < 6; ++col)
            {
                double temporary = upper[pivotColumn][col];

                upper[pivotColumn][col] = upper[pivotRow][col];

                upper[pivotRow][col] = temporary;
            }
            
            {
                double temporary = rhs[pivotColumn];

                rhs[pivotColumn] = rhs[pivotRow];
                rhs[pivotRow] = temporary;
            }
        }

        /* ----------------- Eliminate every element below the pivot ----------------- */
        for (row = pivotColumn + 1; row < 6; ++row)
        {
            double factor = upper[row][pivotColumn] / 
                            upper[pivotColumn][pivotColumn];

            upper[row][pivotColumn] = 0.0;

            for (col = pivotColumn + 1; col < 6; ++col)
            {
                upper[row][col] -= factor * upper[pivotColumn][col];
            }

            rhs[row] -= factor * rhs[pivotColumn];
        }
    }

    return 1;
}

/*
 * ----------------- Perform back - substitution -----------------
 */
void back_substitution(
    const double upper[6][6],
    const double rhs[6],
    double x[6])
{
    int row;
    int col;

    for (row = 6 - 1; row >= 0; --row)
    {
        double value = rhs[row];

        for (col = row + 1; col < 6; ++col)
        {
            value -= upper[row][col] * x[col];
        }

        x[row] = value / upper[row][row];
    }
}

/*
 * ----------------- Find joint angle correction dq -----------------
 */
int solve_6x6(
    const double A[6][6],
    const double b[6],
    double x[6])
{
    double upper[6][6];
    double rhs[6];

    memcpy(upper, A, sizeof upper);
    memcpy(rhs, b, sizeof rhs);

    if (!forward_elimination(upper, rhs))
        return 1;

    back_substitution(upper, rhs, x);

    return 0;
}