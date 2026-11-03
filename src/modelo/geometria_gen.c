// GENERADO por codegen/geometria.py (sympy). No editar a mano: cambiar el script
// y volver a correrlo. Notacion y ecuaciones: informe_control.tex, "Modelo propuesto".
#include "geometria.h"

#include <math.h>

//---------------------------------------------------------------------------
void geo_h(const Geometria *g, const double q[2], const double psi[2], double rho, double s[2])
{
    const double t0 = cos(psi[1]);
    const double t1 = psi[0] - q[0];
    const double t2 = sin(psi[1]);
    const double t3 = sin(q[1]);
    const double t4 = cos(q[1]);
    const double t5 = cos(t1);
    const double t6 = 1.0/(t0*t4*t5 + t2*t3);
    s[0] = g->cu + g->fu*t6*(-g->lx*rho - t0*sin(t1));
    s[1] = g->cv + g->fv*t6*(t0*t3*t5 - t2*t4);
}
//---------------------------------------------------------------------------
void geo_jacobianos(const Geometria *g, const double q[2], const double psi[2], double rho,
                    double Hq[4], double Hpsi[4], double Hrho[2])
{
    const double t0 = cos(psi[1]);
    const double t1 = psi[0] - q[0];
    const double t2 = cos(t1);
    const double t3 = sin(psi[1]);
    const double t4 = sin(q[1]);
    const double t5 = t3*t4;
    const double t6 = cos(q[1]);
    const double t7 = t0*t6;
    const double t8 = t2*t7 + t5;
    const double t9 = 1.0/t8;
    const double t10 = pow(t8, -2);
    const double t11 = sin(t1);
    const double t12 = t0*t11;
    const double t13 = g->fu*t10*(-g->lx*rho - t12);
    const double t14 = t12*t6;
    const double t15 = -g->fu*t0*t2*t9 + t13*t14;
    const double t16 = t3*t6;
    const double t17 = t0*t2*t4 - t16;
    const double t18 = g->fv*t9;
    const double t19 = g->fv*t10;
    const double t20 = t17*t19;
    const double t21 = t12*t18*t4 - t14*t20;
    const double t22 = g->fu*t9;
    const double t23 = -t0*t4 + t16*t2;
    Hq[0] = -t15;
    Hq[1] = t13*t17;
    Hq[2] = t21;
    Hq[3] = g->fv + pow(t17, 2)*t19;
    Hpsi[0] = t15;
    Hpsi[1] = t11*t22*t3 + t13*t23;
    Hpsi[2] = -t21;
    Hpsi[3] = t18*(-t2*t5 - t7) + t20*t23;
    Hrho[0] = -g->lx*t22;
    Hrho[1] = 0;
}
//---------------------------------------------------------------------------
double geo_az(const double q[2], const double psi[2])
{
    return sin(psi[1])*sin(q[1]) + cos(psi[1])*cos(q[1])*cos(psi[0] - q[0]);
}
//---------------------------------------------------------------------------
void geo_mirada(const Geometria *g, const double psi[2], double rho, double qs[2])
{
    const double t0 = cos(psi[1]);
    qs[0] = psi[0] + asin(g->lx*rho/t0);
    qs[1] = atan(tan(psi[1])/sqrt(-pow(g->lx, 2)*pow(rho, 2)/pow(t0, 2) + 1));
}
//---------------------------------------------------------------------------
void geo_mirada_jac(const Geometria *g, const double psi[2], double rho, double dg[4])
{
    const double t0 = sin(psi[1]);
    const double t1 = cos(psi[1]);
    const double t2 = pow(t1, -2);
    const double t3 = pow(g->lx, 2)*pow(rho, 2);
    const double t4 = -t2*t3 + 1;
    const double t5 = pow(t4, -1.0/2.0);
    const double t6 = tan(psi[1]);
    const double t7 = pow(t6, 2);
    dg[0] = 1;
    dg[1] = g->lx*rho*t0*t2*t5;
    dg[2] = 0;
    dg[3] = (t0*t3*t6/(pow(t1, 3)*pow(t4, 3.0/2.0)) + t5*(t7 + 1))/(1 + t7/t4);
}
//---------------------------------------------------------------------------
double geo_tamano(const Geometria *g, const double q[2], const double psi[2], double rho,
                  double kw, double bw, double grad[5])
{
    const double t0 = psi[0] - q[0];
    const double t1 = cos(q[1]);
    const double t2 = cos(psi[1]);
    const double t3 = t1*t2;
    const double t4 = sin(psi[1]);
    const double t5 = sin(q[1]);
    const double t6 = cos(t0);
    const double t7 = t3*t6 + t4*t5;
    const double t8 = kw*rho/pow(t7, 2);
    const double t9 = t3*t8*sin(t0);
    const double t10 = t1*t4;
    const double t11 = kw/t7;
    grad[0] = -t9;
    grad[1] = t8*(-t10 + t2*t5*t6);
    grad[2] = t9;
    grad[3] = t8*(t10*t6 - t2*t5);
    grad[4] = t11;
    const double hw = bw + rho*t11;
    return hw;
}
//---------------------------------------------------------------------------
void geo_rumbo(const Geometria *g, const double q[2], const double psi[2], double rho, double r[2])
{
    const double t0 = cos(psi[1]);
    const double t1 = psi[0] - q[0];
    r[0] = g->cu + g->fu*(-g->lx*rho - t0*sin(t1));
    r[1] = g->cv + g->fv*(t0*sin(q[1])*cos(t1) - sin(psi[1])*cos(q[1]));
}
//---------------------------------------------------------------------------
void geo_rumbo_jac(const Geometria *g, const double q[2], const double psi[2], double rho,
                   double Rq[4], double Rpsi[4], double Rrho[2])
{
    const double t0 = cos(psi[1]);
    const double t1 = psi[0] - q[0];
    const double t2 = cos(t1);
    const double t3 = g->fu*t0*t2;
    const double t4 = sin(q[1]);
    const double t5 = sin(t1);
    const double t6 = g->fv*t0*t4*t5;
    const double t7 = sin(psi[1]);
    const double t8 = t4*t7;
    const double t9 = t0*cos(q[1]);
    Rq[0] = t3;
    Rq[1] = 0;
    Rq[2] = t6;
    Rq[3] = g->fv*(t2*t9 + t8);
    Rpsi[0] = -t3;
    Rpsi[1] = g->fu*t5*t7;
    Rpsi[2] = -t6;
    Rpsi[3] = g->fv*(-t2*t8 - t9);
    Rrho[0] = -g->fu*g->lx;
    Rrho[1] = 0;
}
