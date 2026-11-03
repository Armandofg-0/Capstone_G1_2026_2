# Definicion del NMPC del informe (ec. ocp) en CasADi y generacion del solver con acados.
#
# Estado aumentado x_tilde = [q; q_dot; u_{i-1}] (para el termino de suavidad), entrada u = q_ddot, dinamica
# discreta exacta del doble integrador. Costo NONLINEAR_LS con los residuos
#     y = [r(q, psi_hat, rho_hat); r_dot = R_q q_dot + R_psi psi_dot_hat; u; u - u_{i-1}]
# y costo terminal LINEAR_LS sobre [q; q_dot] con W_N = P_N (Riccati, en tiempo de ejecucion).
# Restricciones: torque normalizado por eje (zona plana de pull-out, roce de Coulomb suavizado
# con tanh), FOV blanda (octagono, con retranqueos b_{i,j}) y cotas de q, q_dot y u.
#
# Parametros por etapa (se cambian en cada ciclo sin regenerar):
#     0-1 psi_hat = [lambda, mu]     2-3 psi_dot_hat      4 rho_hat      5-12 b_{i,j}      13-14 J_pan, J_tilt
#     15 b               16 tau_c      17 m g l_cg   18 tau_max pan     19-22 f_u, f_v, c_u, c_v
#     23 l_x             24 r_eff      25 tau_max tilt
#
# tau_max,j = eta K_m I_p r_j: con la reduccion r_j (vueltas del motor por vuelta del eje) el torque
# disponible en el eje se multiplica por r_j. J, b y tau_c son los vistos desde el eje.
#
# El error de imagen se mide con el rumbo r = s* + F_pi (a_x - rho l_x, a_y), la proyeccion h del
# informe sin la division por a_z (codegen/geometria.py). Vale s* exactamente donde h = s*, asi
# que el optimo de seguimiento no cambia, y coincide con h cerca del centro (difieren en
# f (tan theta - sen theta) ~ f theta^3/2). Lejos del centro, la division por a_z acopla el tilt al error
# horizontal: con el blanco fuera de la imagen, el optimo usaba el tilt para achicar ese error
# por perspectiva y la aceleracion aplicada alternaba entre +-u_max. El rumbo es acotado, no
# tiene la singularidad a_z -> 0 y su componente horizontal no depende del tilt. La FOV se
# expresa en las mismas coordenadas, con r_eff convertido (src/control/nmpc.c). Se escribe en
# CasADi y se verifica contra sympy antes de generar.
#
# Uso (fuera del lazo, con el venv de acados; lo llama el CMake al configurar):
#     ACADOS_SOURCE_DIR=<acados> <acados>/venv/Scripts/python -I codegen/nmpc.py <salida>
import configparser
import math
import os
import random
import sys

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

# Parametros de config/torreta.ini: el periodo del lazo y la seccion [nmpc_solver].
_ini = configparser.ConfigParser(inline_comment_prefixes=(";", "#"))
_ini.optionxform = str
_ini.read(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "torreta.ini"),
          encoding="utf-8")
TS = 1.0 / _ini.getfloat("lazo", "fps")
N = _ini.getint("nmpc_solver", "N")
V_S = _ini.getfloat("nmpc_solver", "v_s")  # suavizado del signo del roce de Coulomb (rad/s)
ESCALA_COSTO = _ini.getfloat("nmpc_solver", "escala_costo")
NP = 26    # largo del vector de parametros por etapa (ver la cabecera)
M_FOV = 8  # lados del octagono de la FOV


def rumbo_casadi(q, psi, rho, fu, fv, cu, cv, lx):
    # Rumbo r = s* + F_pi (a_x - rho l_x, a_y), con a de la ecuacion del informe.
    lam, mu = psi[0], psi[1]
    d = lam - q[0]
    ax = -ca.cos(mu) * ca.sin(d)
    ay = ca.sin(q[1]) * ca.cos(mu) * ca.cos(d) - ca.cos(q[1]) * ca.sin(mu)
    return ca.vertcat(cu + fu * (ax - rho * lx), cv + fv * ay)


def verificar_contra_sympy():
    # El rumbo de CasADi debe coincidir con el de sympy de codegen/geometria.py.
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import geometria as g
    import sympy as sp

    r_sp = sp.lambdify((g.tp, g.tt, g.lam, g.mu, g.rho, g.fu, g.fv, g.cu, g.cv, g.lx), g.rb)
    qs, ps, rs = ca.SX.sym("q", 2), ca.SX.sym("p", 2), ca.SX.sym("r")
    f = ca.Function("r", [qs, ps, rs], [rumbo_casadi(qs, ps, rs, 500.0, 480.0, 320.0, 240.0, 0.02)])
    random.seed(3)
    for _ in range(200):
        q = [random.uniform(-2, 2), random.uniform(-1, 1)]
        p = [q[0] + random.uniform(-3.0, 3.0), q[1] + random.uniform(-1.0, 1.0)]  # tambien detras
        r = random.uniform(0.2, 3.0)
        a = np.array(f(q, p, r)).ravel()
        b = np.array(r_sp(q[0], q[1], p[0], p[1], r, 500.0, 480.0, 320.0, 240.0, 0.02), dtype=float).ravel()
        assert np.max(np.abs(a - b)) < 1e-9, "rumbo de CasADi distinto del de sympy"


def modelo():
    q = ca.SX.sym("q", 2)
    qd = ca.SX.sym("qd", 2)
    up = ca.SX.sym("u_prev", 2)
    u = ca.SX.sym("u", 2)
    p = ca.SX.sym("p", NP)
    x = ca.vertcat(q, qd, up)
    psi, psid, rho = p[0:2], p[2:4], p[4]
    b_fov = p[5:13]
    J = p[13:15]
    b_visc, tau_c, mgl = p[15], p[16], p[17]
    tau_max = ca.vertcat(p[18], p[25])
    fu, fv, cu, cv, lx, r_eff = p[19], p[20], p[21], p[22], p[23], p[24]

    s = rumbo_casadi(q, psi, rho, fu, fv, cu, cv, lx)
    sd = ca.jacobian(s, q) @ qd + ca.jacobian(s, psi) @ psid

    m = AcadosModel()
    m.name = "nmpc"
    m.x = x
    m.u = u
    m.p = p
    m.disc_dyn_expr = ca.vertcat(q + TS * qd + 0.5 * TS**2 * u, qd + TS * u, u)
    m.cost_y_expr = ca.vertcat(s, sd, u, u - up)
    m.cost_y_expr_e = ca.vertcat(q, qd)
    # Torque requerido por eje, normalizado por el disponible (informe, ec. del torque).
    tau_l = ca.vertcat(0.0, mgl * ca.cos(q[1]))
    tau = ca.vertcat(*[(J[j] * u[j] + b_visc * qd[j] + tau_c * ca.tanh(qd[j] / V_S) + tau_l[j]) / tau_max[j]
                       for j in range(2)])
    # FOV: a_j^T (s - s*) + b_j - r_eff <= 0, con a_j las normales del octagono.
    fov = [ca.cos(2 * math.pi * j / M_FOV) * (s[0] - cu) + ca.sin(2 * math.pi * j / M_FOV) * (s[1] - cv)
           + b_fov[j] - r_eff for j in range(M_FOV)]
    m.con_h_expr = ca.vertcat(tau, *fov)
    # En la etapa 0 el estado esta fijo (x0) y solo importa el torque de u0, que es el que se
    # aplica primero.
    m.con_h_expr_0 = tau
    return m


def ocp(salida):
    o = AcadosOcp()
    o.model = modelo()
    o.name = "nmpc"
    o.solver_options.N_horizon = N
    o.solver_options.tf = N * TS
    o.parameter_values = np.zeros(NP)

    ny = 8
    o.cost.cost_type = "NONLINEAR_LS"
    o.cost.cost_type_e = "NONLINEAR_LS"
    o.cost.W = np.eye(ny)
    o.cost.W_e = np.eye(4)
    o.cost.yref = np.zeros(ny)
    o.cost.yref_e = np.zeros(4)

    c = o.constraints
    c.x0 = np.zeros(6)
    c.idxbx = np.array([0, 1, 2, 3])
    c.lbx = np.array([-2.6, -1.2, -2.0, -2.0])
    c.ubx = np.array([2.6, 1.2, 2.0, 2.0])
    # Las mismas cotas duras en la etapa terminal (nmpc_iniciar las fija en 1..N).
    c.idxbx_e = np.array([0, 1, 2, 3])
    c.lbx_e = c.lbx.copy()
    c.ubx_e = c.ubx.copy()
    c.idxbu = np.array([0, 1])
    c.lbu = -100.0 * np.ones(2)
    c.ubu = 100.0 * np.ones(2)
    nh = 2 + M_FOV
    # La FOV es solo una cota superior: la inferior queda inalcanzable (|r - s*| <= f).
    c.lh = np.concatenate([-np.ones(2), -1e6 * np.ones(M_FOV)])
    c.uh = np.concatenate([np.ones(2), np.zeros(M_FOV)])
    # FOV blanda: penalizacion lineal exacta mas una cuadratica chica (QP estrictamente convexo).
    c.idxsh = np.arange(2, nh)
    c.lh_0 = -np.ones(2)
    c.uh_0 = np.ones(2)
    ns = M_FOV
    o.cost.zl = 1e4 * np.ones(ns)
    o.cost.zu = 1e4 * np.ones(ns)
    o.cost.Zl = 1.0 * np.ones(ns)
    o.cost.Zu = 1.0 * np.ones(ns)

    s = o.solver_options
    s.integrator_type = "DISCRETE"
    s.nlp_solver_type = "SQP_RTI"
    s.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    s.hessian_approx = "GAUSS_NEWTON"
    s.qp_solver_iter_max = 50
    # Costo como suma, igual que la ec. ocp del informe: acados multiplica por defecto el costo
    # de cada etapa por su paso de tiempo, pero no el terminal, con lo que P_N (Riccati sobre
    # los costos de etapa sin escalar) pesaria 1/T_s = 30 veces de mas. El factor comun
    # ESCALA_COSTO no cambia el optimo: lleva los multiplicadores (del orden de w_eps = 1e4) a la
    # escala del mu0 de HPIPM; sin el, el QP termina en paso minimo o NaN.
    s.cost_scaling = ESCALA_COSTO * np.ones(N + 1)
    o.code_gen_options.code_export_directory = salida
    o.code_gen_options.json_file = os.path.join(salida, "nmpc.json")
    return o


if __name__ == "__main__":
    verificar_contra_sympy()
    salida = os.path.abspath(sys.argv[1])
    AcadosOcpSolver.generate(ocp(salida))
