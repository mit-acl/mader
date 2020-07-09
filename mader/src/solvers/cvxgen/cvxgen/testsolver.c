/* Produced by CVXGEN, 2020-06-13 17:15:52 -0400.  */
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
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.MAcol0[0] = 0.20319161029830202;
  params.MAcol0[1] = 0.8325912904724193;
  params.MAcol0[2] = -0.8363810443482227;
  params.viM2[0] = 0.04331042079065206;
  params.viM2[1] = 1.5717878173906188;
  params.viM2[2] = 1.5851723557337523;
  params.viM1[0] = -1.497658758144655;
  params.viM1[1] = -1.171028487447253;
  params.viM1[2] = -1.7941311867966805;
  params.v_max[0] = 0.8816196873012729;
  params.MAcol1[0] = -1.8804951564857322;
  params.MAcol1[1] = -0.17266710242115568;
  params.MAcol1[2] = 0.596576190459043;
  params.MAcol2[0] = -0.8860508694080989;
  params.MAcol2[1] = 0.7050196079205251;
  params.MAcol2[2] = 0.3634512696654033;
  params.MBcol0[0] = -1.9040724704913385;
  params.MBcol0[1] = 0.23541635196352795;
  params.MBcol0[2] = -0.9629902123701384;
  params.MBcol1[0] = -0.3395952119597214;
  params.MBcol1[1] = -0.865899672914725;
  params.MBcol1[2] = 0.7725516732519853;
  params.MBcol2[0] = -0.23818512931704205;
  params.MBcol2[1] = -1.372529046100147;
  params.MBcol2[2] = 0.17859607212737894;
  params.MCcol0[0] = 1.1212590580454682;
  params.MCcol0[1] = -0.774545870495281;
  params.MCcol0[2] = -1.1121684642712744;
  params.MCcol1[0] = -0.44811496977740495;
  params.MCcol1[1] = 1.7455345994417217;
  params.MCcol1[2] = 1.9039816898917352;
  params.MCcol2[0] = 0.6895347036512547;
  params.MCcol2[1] = 1.6113364341535923;
  params.MCcol2[2] = 1.383003485172717;
}
