#include <string.h>
#include "mex.h"
#include "robot_constants.h"
#include "Rotation_math.h"
#include "Gaussian_solver.h"
#include "ABB_IRB120_forward_kinematics.h"
#include "ABB_IRB120_inverse_kinematics.h"

/* ----------------- Define input validation helper ----------------- */
static void require_real_double_vector(
    const mxArray *a,      /* --> one MATLAB input */
    mwSize expectedLength, /* --> required nmber of elements */
    const char *name)      /* --> name used in an error message */
{
    if (!mxIsDouble(a) || mxIsComplex(a) ||
        mxGetNumberOfElements(a) != expectedLength)
    {
        mexErrMsgIdAndTxt(
            "IRB120_IK:InvalidInput",
            "%s must be a real double vector with %u elements.",
            name,
            (unsigned)expectedLength);
    }
}

/* ----------------- Copy the 4x4 transformation matrix T */
static void copy_transform_to_matlab(
    const Matrix_4x4 *T,
    mxArray **output)
{
    int row, col;
    double *out;

    *output = mxCreateDoubleMatrix(4, 4, mxREAL);
    out = mxGetPr(*output);

    /* --> Here, MATLAB stores matrices column by column, for example:
     * T(1,1) -> out[0]
       T(2,1) -> out[1]
       T(3,1) -> out[2]
     */
    for (col = 0; col < 4; ++col)
        for (row = 0; row < 4; ++row)
            out[row + 4*col] = T->m[row][col];
}

void mexFunction(
    int nlhs,              /* Number of expected mxArray output arguments, specified as an integer */
    mxArray *plhs[],       /* Array of pointers to the expected mxArray output arguments */
    int nrhs,              /* Number of input arguments */
    const mxArray *prhs[]) /* Array of pointers to the mxArray input arguments */
{
    const double *q0;     /* Pointer to jointAngles data owned by MATLAB*/
    const double *target; /* Pointer to target = [x, y, z, roll, pitch, yaw] vector data owned by MATLAB*/
    double q[6];
    Pose targetPose;
    IK_Options options;
    IK_Result result;
    Matrix_4x4 T;
    int i;
    const char *fieldNames[] = {
        "converged",
        "iterations",
        "positionError",
        "orientationError"
    };

    if (nrhs < 2 || nrhs > 3) /* --> Allows only 2 inputs and max of 3 outputs (*3rd is optional*) */
    {
        mexErrMsgIdAndTxt(
            "IRB120_IK:ArgumentCount",
            "Usage: [q,T,info] = IRB120_IK(q0, targetPose, optionsVector)");
    }

    if (nlhs > 3)
        mexErrMsgIdAndTxt("IRB120_IK:OutputCount", "At most three outputs.");

    require_real_double_vector(prhs[0], 6, "q0");
    require_real_double_vector(prhs[1], 6, "targetPose");

    /* --> Here, mxGetPr() returns the address of the type double in a real MATLAB array */
    q0 = mxGetPr(prhs[0]);     
    target = mxGetPr(prhs[1]);

    /* --> Here, jointAngles is both:
     * 1) the initial guess entering the function
     * 2) the calculated angles leaving the function 
     * 
     * MEX wrapper must not let it modify the MATLAB input directly, so I make a private stack copy
     *
     * MATLAB q0 -> q0 pointer -> copy into local q -> solver modifies local q */

    for (i = 0; i < 6; ++i)
        q[i] = q0[i];

    /* --------------------------------------------------------------------*/

    targetPose.x = target[0];
    targetPose.y = target[1];
    targetPose.z = target[2];
    targetPose.roll = target[3];
    targetPose.pitch = target[4];
    targetPose.yaw = target[5];

    /* Here, all default options are initialized by default */
    ik_default_options(&options);

    /* If the third input - options, is used, then it overrides only the called - ones
     *
     * Optional options vector:
     * [maxIterations, positionTolerance, orientationTolerance,
     *  damping, numericalStep, maxCartesianStep,
     *  maxOrientationStep, maxJointStep,
     *  positionWeight, orientationWeight]
     */
    if (nrhs == 3)
    {
        const double *o;
        mwSize n;

        if (!mxIsDouble(prhs[2]) || mxIsComplex(prhs[2]))
            mexErrMsgIdAndTxt(
                "IRB120_IK:InvalidOptions",
                "optionsVector must be a real double vector.");

        o = mxGetPr(prhs[2]);
        n = mxGetNumberOfElements(prhs[2]);

        if (n > 0) options.maxIterations = (int)o[0];   /* itereation limit*/
        if (n > 1) options.positionTolerance = o[1];    /* metres*/
        if (n > 2) options.orientationTolerance = o[2]; /* radians */
        if (n > 3) options.damping = o[3];              /* Damped-Least-Squares damping lambda*/
        if (n > 4) options.numericalStep = o[4];        /* radians */
        if (n > 5) options.maxCartesianStep = o[5];     /* metres/iteration */
        if (n > 6) options.maxOrientationStep = o[6];   /* radians/iteration */
        if (n > 7) options.maxJointStep = o[7];         /* radians/iteration */
        if (n > 8) options.positionWeight = o[8];       /* Cartesian weighting */
        if (n > 9) options.orientationWeight = o[9];    /* Orientation weighting */
    }

    inverseKinematics(
        &targetPose,   /* --> Address of the requested pose */
        q,             /* --> Initial guess and resulting solution */
        &options,      /* --> Address of Solver settings */
        &result        /* --> Address of where convergence information is written */
    );

    forward_kinematics(
        q,             /* --> Initial guess and resulting solution */
        &T             /* --> Address of the requested pose */
    );

    if (nlhs >= 1)
    {
        plhs[0] = mxCreateDoubleMatrix(6, 1, mxREAL);  /* --> Allocate matrix of 6x1 real-double */
        memcpy(mxGetPr(plhs[0]), q, 6*sizeof(double)); /* --> Copy the 6x1 real-double C values into MATLAB-owned memory */
    }

    if (nlhs >= 2)
        copy_transform_to_matlab(&T, &plhs[1]);

    
    if (nlhs >= 3)
    {
        plhs[2] = mxCreateStructMatrix(
            1,           /* --> Desired number of rows */
            1,           /* --> Desired number of columns */
            4,           /* --> Number of fields */
            fieldNames); /* --> Supplied by fieldnames*/


        mxSetField(plhs[2], 0, "converged",
                   mxCreateLogicalScalar(result.converged != 0));
        mxSetField(plhs[2], 0, "iterations",
                   mxCreateDoubleScalar((double)result.iterations));
        mxSetField(plhs[2], 0, "positionError",
                   mxCreateDoubleScalar(result.positionError));
        mxSetField(plhs[2], 0, "orientationError",
                   mxCreateDoubleScalar(result.orientationError));
    }
}