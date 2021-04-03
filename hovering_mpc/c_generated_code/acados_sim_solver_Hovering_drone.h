/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SIM_Hovering_drone_H_
#define ACADOS_SIM_Hovering_drone_H_

#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

int Hovering_drone_acados_sim_create();
int Hovering_drone_acados_sim_solve();
int Hovering_drone_acados_sim_free();
int Hovering_drone_acados_sim_update_params(double *value, int np);

sim_config  * Hovering_drone_acados_get_sim_config();
sim_in      * Hovering_drone_acados_get_sim_in();
sim_out     * Hovering_drone_acados_get_sim_out();
void        * Hovering_drone_acados_get_sim_dims();
sim_opts    * Hovering_drone_acados_get_sim_opts();
sim_solver  * Hovering_drone_acados_get_sim_solver();

// ** global data **
extern sim_config  * Hovering_drone_sim_config;
extern sim_in      * Hovering_drone_sim_in;
extern sim_out     * Hovering_drone_sim_out;
extern void        * Hovering_drone_sim_dims;
extern sim_opts    * Hovering_drone_sim_opts;
extern sim_solver  * Hovering_drone_sim_solver;

#ifdef __cplusplus
}
#endif


extern external_function_param_casadi * sim_forw_vde_casadi;
extern external_function_param_casadi * sim_expl_ode_fun_casadi;


#endif  // ACADOS_SIM_Hovering_drone_H_
