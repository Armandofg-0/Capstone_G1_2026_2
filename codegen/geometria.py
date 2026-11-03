# Geometria de la camara y la torreta: se deriva con sympy, se verifica contra el informe
# (seccion "Modelo propuesto") y se exporta a C en src/modelo/geometria_gen.c.
# Uso: python -I codegen/geometria.py
import os
import random
import sympy as sp  # matematica simbolica

# Variables: angulos de la torreta (tp, tt), rumbo del blanco (lam, mu) e inverso de la distancia (rho).
tp, tt, lam, mu, rho = sp.symbols("theta_p theta_t lambda_ mu rho", real=True)
# Parametros: intrinsecos (fu, fv, cu, cv), offset de la camara (lx) y modelo del tamano (kw, bw).
fu, fv, cu, cv, lx, kw, bw = sp.symbols("fu fv cu cv lx kw bw", real=True)


# Rotacion en torno al eje x.
def Rx(a):
    return sp.Matrix([[1, 0, 0], [0, sp.cos(a), -sp.sin(a)], [0, sp.sin(a), sp.cos(a)]])


# Rotacion en torno al eje z.
def Rz(a):
    return sp.Matrix([[sp.cos(a), -sp.sin(a), 0], [sp.sin(a), sp.cos(a), 0], [0, 0, 1]])


Rc = sp.Matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])  # ejes de la camara respecto de la base
R = Rz(tp) * Rc * Rx(tt)  # lleva vectores del marco camara al marco mundo
u = sp.Matrix([sp.cos(mu) * sp.cos(lam), sp.cos(mu) * sp.sin(lam), sp.sin(mu)])  # direccion al blanco
a = sp.simplify(R.T * u)  # la misma direccion en el marco camara
ax, ay, az = a
delta = lam - tp

# Verificacion: a debe coincidir con la ecuacion del informe.
a_informe = sp.Matrix([
    -sp.cos(mu) * sp.sin(delta),
    sp.sin(tt) * sp.cos(mu) * sp.cos(delta) - sp.cos(tt) * sp.sin(mu),
    sp.sin(tt) * sp.sin(mu) + sp.cos(tt) * sp.cos(mu) * sp.cos(delta),
])
assert sp.simplify(sp.expand_trig(a - a_informe)) == sp.zeros(3, 1), "a no coincide con el informe"

# Proyeccion del blanco en la imagen, h(q, psi, rho), y sus jacobianos.
h = sp.Matrix([cu + fu * (ax - rho * lx) / az, cv + fv * ay / az])
q = sp.Matrix([tp, tt])
psi = sp.Matrix([lam, mu])
Hq = h.jacobian(q)
Hpsi = h.jacobian(psi)
Hrho = h.jacobian(sp.Matrix([rho]))
assert sp.simplify(Hq[:, 0] + Hpsi[:, 0]) == sp.zeros(2, 1), "dh/dtp debe ser -dh/dlam"
assert sp.simplify(Hrho - sp.Matrix([-fu * lx / az, 0])) == sp.zeros(2, 1), "dh/drho no coincide"

# Verificacion: Hq debe ser la L_eff(s, tt, Z) del modelo anterior, con 1/Z = rho/az.
x = (ax - rho * lx) / az
y = ay / az
lz = lx * rho / az
L_eff = sp.Matrix([
    [fu * ((1 + x**2) * sp.cos(tt) + y * sp.sin(tt) + lz * x * sp.cos(tt)), fu * x * y],
    [fv * (x + lz) * (y * sp.cos(tt) - sp.sin(tt)), fv * (1 + y**2)],
])


def planos(m):
    # Elementos de lo que devuelve lambdify como lista de escalares (con o sin numpy).
    return [e for fila in (m.tolist() if hasattr(m, "tolist") else m) for e in (fila if isinstance(fila, list) else [fila])]


# Se compara numericamente en 200 puntos al azar (simplificar Hq - L_eff es muy lento).
dif = sp.lambdify((tp, tt, lam, mu, rho, fu, fv, lx), Hq - L_eff)
random.seed(1)
for _ in range(200):
    v = (random.uniform(-2, 2), random.uniform(-1, 1), 0, random.uniform(-0.8, 0.8),
         random.uniform(0.2, 3), 500.0, 480.0, random.uniform(-0.03, 0.03))
    v = (v[0], v[1], v[0] + random.uniform(-0.4, 0.4)) + v[3:]
    assert max(abs(float(e)) for e in planos(dif(*v))) < 1e-6, "Hq no coincide con L_eff"

# Mapa de mirada g(psi, rho): angulos de la torreta que dejan al blanco en s* = (cu, cv).
k = rho * lx / sp.cos(mu)
g = sp.Matrix([lam + sp.asin(k), sp.atan(sp.tan(mu) / sp.sqrt(1 - k**2))])
dg = g.jacobian(psi)
h_en_g = sp.lambdify((lam, mu, rho, lx, fu, fv, cu, cv), h.subs({tp: g[0], tt: g[1]}))
for _ in range(200):
    v = (random.uniform(-3, 3), random.uniform(-1.2, 1.2), random.uniform(0.2, 3),
         random.uniform(-0.03, 0.03), 500.0, 480.0, 320.0, 240.0)
    s = planos(h_en_g(*v))
    assert abs(float(s[0]) - 320.0) < 1e-8 and abs(float(s[1]) - 240.0) < 1e-8, "h(g) no es s*"

# Tamano aparente del blanco y su gradiente.
hw = kw * rho / az + bw
dhw = sp.Matrix([hw]).jacobian(sp.Matrix([tp, tt, lam, mu, rho]))

# Rumbo del NMPC: h sin dividir por az. Vale s* donde h = s*, es acotado lejos del centro
# (sin la singularidad de az = 0) y su componente horizontal no depende del tilt.
rb = sp.Matrix([cu + fu * (ax - rho * lx), cv + fv * ay])
Rq = rb.jacobian(q)
Rpsi = rb.jacobian(psi)
Rrho = rb.jacobian(sp.Matrix([rho]))
en_g = sp.lambdify((lam, mu, rho, lx, fu, fv, cu, cv),
                   [rb.subs({tp: g[0], tt: g[1]}), (Rq - az * Hq).subs({tp: g[0], tt: g[1]}),
                    (Rpsi - az * Hpsi).subs({tp: g[0], tt: g[1]})])
for _ in range(200):
    v = (random.uniform(-3, 3), random.uniform(-1.2, 1.2), random.uniform(0.2, 3),
         random.uniform(-0.03, 0.03), 500.0, 480.0, 320.0, 240.0)
    r0, d1, d2 = en_g(*v)
    r0 = planos(r0)
    assert abs(float(r0[0]) - 320.0) < 1e-8 and abs(float(r0[1]) - 240.0) < 1e-8, "r(g) no es s*"
    assert max(abs(float(e)) for e in planos(d1) + planos(d2)) < 1e-8, "dr no es az*dh en la mirada"


# --- exportacion a C ---
# Nombre en C de cada simbolo.
NOMBRES = {tp: "q[0]", tt: "q[1]", lam: "psi[0]", mu: "psi[1]", rho: "rho",
           fu: "g->fu", fv: "g->fv", cu: "g->cu", cv: "g->cv", lx: "g->lx",
           kw: "kw", bw: "bw"}


def cuerpo(salidas):
    # Cuerpo de una funcion C, con subexpresiones comunes. salidas: lista de (lhs, expr).
    reps, red = sp.cse([e for _, e in salidas], symbols=sp.numbered_symbols("t"))
    lineas = []
    subs = {s: sp.Symbol(n) for s, n in NOMBRES.items()}
    for sim, e in reps:
        lineas.append(f"    const double {sim} = {sp.ccode(e.xreplace(subs))};")
    for (lhs, _), e in zip(salidas, red):
        lineas.append(f"    {lhs} = {sp.ccode(e.xreplace(subs))};")
    return "\n".join(lineas)


def matriz(nombre, M):
    # Asignaciones nombre[i] = M[fila, columna], por filas.
    return [(f"{nombre}[{i * M.cols + j}]", M[i, j]) for i in range(M.rows) for j in range(M.cols)]


# Firma de cada funcion C y lo que calcula (None: caso especial en generar).
FUNCIONES = [
    ("void geo_h(const Geometria *g, const double q[2], const double psi[2], double rho, double s[2])",
     matriz("s", h)),
    ("void geo_jacobianos(const Geometria *g, const double q[2], const double psi[2], double rho,\n"
     "                    double Hq[4], double Hpsi[4], double Hrho[2])",
     matriz("Hq", Hq) + matriz("Hpsi", Hpsi) + matriz("Hrho", Hrho)),
    ("double geo_az(const double q[2], const double psi[2])", None),
    ("void geo_mirada(const Geometria *g, const double psi[2], double rho, double qs[2])",
     matriz("qs", g)),
    ("void geo_mirada_jac(const Geometria *g, const double psi[2], double rho, double dg[4])",
     matriz("dg", dg)),
    ("double geo_tamano(const Geometria *g, const double q[2], const double psi[2], double rho,\n"
     "                  double kw, double bw, double grad[5])", None),
    ("void geo_rumbo(const Geometria *g, const double q[2], const double psi[2], double rho, double r[2])",
     matriz("r", rb)),
    ("void geo_rumbo_jac(const Geometria *g, const double q[2], const double psi[2], double rho,\n"
     "                   double Rq[4], double Rpsi[4], double Rrho[2])",
     matriz("Rq", Rq) + matriz("Rpsi", Rpsi) + matriz("Rrho", Rrho)),
]


def generar():
    # Texto completo de geometria_gen.c.
    partes = ["// GENERADO por codegen/geometria.py (sympy). No editar a mano: cambiar el script\n"
              "// y volver a correrlo. Notacion y ecuaciones: informe_control.tex, \"Modelo propuesto\".\n"
              "#include \"geometria.h\"\n\n#include <math.h>\n"]
    for firma, salidas in FUNCIONES:
        partes.append("//" + "-" * 75)
        if firma.startswith("double geo_az"):
            subs = {s: sp.Symbol(n) for s, n in NOMBRES.items()}
            partes.append(firma + "\n{\n    return " + sp.ccode(az.xreplace(subs)) + ";\n}")
        elif firma.startswith("double geo_tamano"):
            body = cuerpo(matriz("grad", dhw) + [("const double hw", hw)])
            partes.append(firma + "\n{\n" + body + "\n    return hw;\n}")
        else:
            partes.append(firma + "\n{\n" + cuerpo(salidas) + "\n}")
    return "\n".join(partes) + "\n"


if __name__ == "__main__":
    raiz = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ruta = os.path.join(raiz, "src", "modelo", "geometria_gen.c")
    with open(ruta, "w", encoding="utf-8", newline="\n") as f:
        f.write(generar())
    print("geometria_gen.c generado:", ruta)
