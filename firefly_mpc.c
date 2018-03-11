/*
 * firefly_mpc.c
 */

#include "mpc.h"

int firefly_mpc(void)
{
real_t x[MPC_STATES]; /* current state of the system */
extern struct mpc_ctl ctl; /* already defined */
ctl.conf->in_iter = 10; /* number of iterations */
/* The current state */
x[0] = 0;
x[1] = 0;
/* Solve MPC problem and print the first element of input sequence */
mpc_ctl_solve_problem(&ctl, x); /* solve the MPC problem */
return 0;
}
