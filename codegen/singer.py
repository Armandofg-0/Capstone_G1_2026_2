# Transicion del modelo de Singer para un intervalo arbitrario Delta t (un eje).
#
# Modelo continuo [posicion, velocidad, aceleracion] con aceleracion de Ornstein-Uhlenbeck:
#     x_dot = A x + B w,  A = [[0,1,0],[0,0,1],[0,0,-alfa]],  B = [0;0;1],  alfa = 1/tau_m,
# con w blanco de densidad q_c = 2 alfa sigma_m^2 (asi la aceleracion estacionaria tiene desviacion sigma_m).
#     F(Delta t) = exp(A Delta t),   Q(Delta t) = int_0^Delta t exp(A s) B q_c B^T exp(A s)^T ds.
#
# La forma exacta resta numeros casi iguales cuando b = alfa Delta t es chico: el numerador de q11
# es O(b^5) y se arma con terminos O(1), asi que su error relativo es ~1e-16/b^5 (1e-5 con
# T_s = 1/30 y tau_m = 5). Por eso, para b < B_TAYLOR se usa la serie de Taylor en b, de un
# orden tal que el truncamiento quede bajo la precision del double. Genera
# src/estimacion/singer_gen.c.
#
# Uso (fuera del lazo):  python -I codegen/singer.py
import os

import sympy as sp

dt, al, sm = sp.symbols("dt alpha sigma_m", positive=True)
s = sp.symbols("s", positive=True)
b = sp.symbols("b", positive=True)  # b = alfa Delta t

A = sp.Matrix([[0, 1, 0], [0, 0, 1], [0, 0, -al]])
B = sp.Matrix([0, 0, 1])
qc = 2 * al * sm**2

F = sp.simplify((A * dt).exp())
Fs = (A * s).exp()
Q = (Fs * B * qc * B.T * Fs.T).applyfunc(lambda e: sp.integrate(sp.expand(e), (s, 0, dt)))
Q = sp.simplify(Q)

# Comprobaciones: F es la de Singer clasica y Q es simetrica.
assert sp.simplify(F[0, 2] - (al * dt - 1 + sp.exp(-al * dt)) / al**2) == 0
assert sp.simplify(F[2, 2] - sp.exp(-al * dt)) == 0
assert sp.simplify(Q - Q.T) == sp.zeros(3, 3)

# Serie de Taylor en b alrededor de 0 (alfa = b/Delta t): con b < 0.5 y orden 16, el primer termino
# omitido es ~0.5^16/16! < 1e-17 relativo.
ORDEN = 17
F_b = F.subs(al, b / dt)
Q_b = Q.subs(al, b / dt)
F_t = F_b.applyfunc(lambda e: sp.series(e, b, 0, ORDEN).removeO())
Q_t = Q_b.applyfunc(lambda e: sp.series(sp.simplify(e), b, 0, ORDEN).removeO())
# En el limite b -> 0 (tau_m -> inf) se recupera la aceleracion constante (CA).
assert sp.simplify(F_t.subs(b, 0) - sp.Matrix([[1, dt, dt**2 / 2], [0, 1, dt], [0, 0, 1]])) == sp.zeros(3, 3)

B_TAYLOR = "0.5"


def asignaciones(M, nombre, subs):
    pref = nombre.lower() + "t"  # temporales distintos para F y Q en un mismo bloque
    reps, red = sp.cse([M[i, j] for i in range(3) for j in range(3)], symbols=sp.numbered_symbols(pref))
    out = [f"        const double {x} = {sp.ccode(e.xreplace(subs))};" for x, e in reps]
    out += [f"        {nombre}[{k}] = {sp.ccode(e.xreplace(subs))};" for k, e in enumerate(red)]
    return "\n".join(out)


def generar():
    sd = {dt: sp.Symbol("dt"), al: sp.Symbol("a"), sm: sp.Symbol("sigma_m"), b: sp.Symbol("b")}
    return f"""// GENERADO por codegen/singer.py (sympy). No editar a mano: cambiar el script y volver
// a correrlo.
#include "singer_gen.h"

#include <math.h>

void singer_FQ(double dt, double tau_m, double sigma_m, double F[9], double Q[9])
{{
    const double a = 1.0 / tau_m;
    const double b = a * dt;
    if (b < {B_TAYLOR}) {{
        // Serie de Taylor en b = Delta t/tau_m (orden {ORDEN - 1}).
{asignaciones(F_t, "F", sd)}
{asignaciones(Q_t, "Q", sd)}
    }} else {{
{asignaciones(F, "F", sd)}
{asignaciones(Q, "Q", sd)}
    }}
}}
"""


if __name__ == "__main__":
    raiz = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ruta = os.path.join(raiz, "src", "estimacion", "singer_gen.c")
    with open(ruta, "w", encoding="utf-8", newline="\n") as f:
        f.write(generar())
    print("singer_gen.c generado:", ruta)
