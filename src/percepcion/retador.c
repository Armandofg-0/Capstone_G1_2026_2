#include "retador.h"

static float dist2(PuntoPx a, PuntoPx b)
{
    const float dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}
//---------------------------------------------------------------------------
ConfigBarrida barrida_por_defecto(void)
{
    ConfigBarrida c;
    c.habilitada = 1;
    c.periodo_cuadros = 15;
    c.n_confirmaciones = 3;
    c.umbral_movimiento = 15.0f;
    c.radio_exclusion = 40.0f;
    c.radio_asociacion = 60.0f;
    c.radio_primer_salto = 200.0f;
    return c;
}
//---------------------------------------------------------------------------
void retadores_iniciar(Retadores *rs, const ConfigBarrida *cfg)
{
    rs->cfg = *cfg;
    rs->n = 0;
}
//---------------------------------------------------------------------------
void retadores_vaciar(Retadores *rs)
{
    rs->n = 0;
}
//---------------------------------------------------------------------------
void retadores_trasladar(Retadores *rs, float dx, float dy)
{
    for (int i = 0; i < rs->n; i++)
    {
        Retador *r = &rs->r[i];
        r->inicio.x += dx; r->inicio.y += dy;
        r->previo.x += dx; r->previo.y += dy;
        r->ultimo.x += dx; r->ultimo.y += dy;
        r->blanco_inicio.x += dx; r->blanco_inicio.y += dy;
    }
}
//---------------------------------------------------------------------------
// 1. Se ignoran los candidatos cerca del blanco. 2. Cada retador toma el candidato libre mas
// cercano a su posicion prevista (velocidad constante entre barridas) dentro de
// radio_asociacion; en la primera asociacion, sin velocidad, se mide desde la ultima posicion
// con radio_primer_salto. El que no encuentra ninguno se elimina (la racha se corta).
// 3. Los candidatos libres crean retadores nuevos mientras haya cupo. 4. Gana, entre los
// confirmados que se movieron con el blanco quieto, el de mayor desplazamiento.
int retadores_barrida(Retadores *rs, PuntoPx blanco, const PuntoPx *cand, int n_cand, PuntoPx *nuevo)
{
    const ConfigBarrida *c = &rs->cfg;
    if (!c->habilitada)
        return 0;

    enum { CAND_MAX = 32 };
    PuntoPx libres[CAND_MAX];
    int usado[CAND_MAX], n = 0;
    const float excl2 = c->radio_exclusion * c->radio_exclusion;
    for (int i = 0; i < n_cand && n < CAND_MAX; i++)
        if (dist2(cand[i], blanco) >= excl2)
        {
            libres[n] = cand[i];
            usado[n++] = 0;
        }

    int vivos = 0;
    for (int i = 0; i < rs->n; i++)
    {
        Retador r = rs->r[i];
        const int con_velocidad = r.confirmaciones >= 2;
        const PuntoPx prevista = con_velocidad ? (PuntoPx){2 * r.ultimo.x - r.previo.x, 2 * r.ultimo.y - r.previo.y}
                                               : r.ultimo;
        const float radio = con_velocidad ? c->radio_asociacion : c->radio_primer_salto;
        int mejor = -1;
        float d_mejor = radio * radio;
        for (int j = 0; j < n; j++)
        {
            const float d = dist2(libres[j], prevista);
            if (!usado[j] && d <= d_mejor)
            {
                mejor = j;
                d_mejor = d;
            }
        }
        if (mejor < 0)
            continue;
        usado[mejor] = 1;
        r.previo = r.ultimo;
        r.ultimo = libres[mejor];
        r.confirmaciones++;
        rs->r[vivos++] = r;
    }
    rs->n = vivos;

    for (int j = 0; j < n && rs->n < RETADOR_MAX; j++)
        if (!usado[j])
        {
            Retador *r = &rs->r[rs->n++];
            r->inicio = r->previo = r->ultimo = libres[j];
            r->blanco_inicio = blanco;
            r->confirmaciones = 1;
        }

    const float mov2 = c->umbral_movimiento * c->umbral_movimiento;
    int ganador = -1;
    float d_ganador = 0.0f;
    for (int i = 0; i < rs->n; i++)
    {
        const Retador *r = &rs->r[i];
        const float d = dist2(r->ultimo, r->inicio);
        if (r->confirmaciones >= c->n_confirmaciones && d >= mov2 &&
            dist2(blanco, r->blanco_inicio) < mov2 && d > d_ganador)
        {
            ganador = i;
            d_ganador = d;
        }
    }
    if (ganador < 0)
        return 0;
    *nuevo = rs->r[ganador].ultimo;
    rs->n = 0;
    return 1;
}
