/* Produced by CVXGEN, 2018-03-30 10:48:24 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;

int initialize_optimizer(void)
{
  set_defaults();
  setup_indexing();
}

int optimize(void)
{
  solve();
  // for(i=1;i<19;i++){
  // printf("%0.2f %0.2f %0.2f \n",vars.x[i][0],vars.x[i][3],vars.u[i][0]);
  // }
  // printf("%0.2f %0.2f %0.2f \n",vars.x[19][3],vars.x[19][4],vars.x[19][5]);
  return work.converged;
}

double* get_vel(void)
{
  return vars.a;
}

// double** get_control(void)
// {
//   return vars.u;
// }

void load_default_data(double MAcol0[], double MAcol1[], double MAcol2[], double MBcol0[], double MBcol1[],
                       double MBcol2[], double MCcol0[], double MCcol1[], double MCcol2[], double viM2[], double viM1[],
                       double v_max)
{
  params.MAcol0[0] = MAcol0[0];
  params.MAcol0[1] = MAcol0[1];
  params.MAcol0[2] = MAcol0[2];

  //
  params.MAcol1[0] = MAcol1[0];
  params.MAcol1[1] = MAcol1[1];
  params.MAcol1[2] = MAcol1[2];

  //
  params.MAcol2[0] = MAcol2[0];
  params.MAcol2[1] = MAcol2[1];
  params.MAcol2[2] = MAcol2[2];

  //
  params.MBcol0[0] = MBcol0[0];
  params.MBcol0[1] = MBcol0[1];
  params.MBcol0[2] = MBcol0[2];

  //
  params.MBcol1[0] = MBcol1[0];
  params.MBcol1[1] = MBcol1[1];
  params.MBcol1[2] = MBcol1[2];

  //
  params.MBcol2[0] = MBcol2[0];
  params.MBcol2[1] = MBcol2[1];
  params.MBcol2[2] = MBcol2[2];

  //
  params.MCcol0[0] = MCcol0[0];
  params.MCcol0[1] = MCcol0[1];
  params.MCcol0[2] = MCcol0[2];

  //
  params.MCcol1[0] = MCcol1[0];
  params.MCcol1[1] = MCcol1[1];
  params.MCcol1[2] = MCcol1[2];

  //
  params.MCcol2[0] = MCcol2[0];
  params.MCcol2[1] = MCcol2[1];
  params.MCcol2[2] = MCcol2[2];

  //
  params.viM1[0] = viM1[0];
  params.viM1[1] = viM1[1];
  params.viM1[2] = viM1[2];

  //
  params.viM2[0] = viM2[0];
  params.viM2[1] = viM2[1];
  params.viM2[2] = viM2[2];

  params.v_max[0] = v_max;
}
