// Barrida global con retador.
//
// Mientras se sigue un blanco, cada periodo_cuadros el perfil busca en toda la imagen sin la
// pista del Kalman y le entrega aqui los candidatos. Los que estan lejos del blanco actual
// pasan a ser retadores: se asocian entre barridas y cuentan confirmaciones. Si un retador
// junta n_confirmaciones seguidas, se movio al menos umbral_movimiento y el blanco actual no
// se movio en ese tiempo, se cambia de blanco. Asi un falso positivo quieto no queda
// enganchado para siempre, y un blanco real quieto no se suelta si no hay un retador movil.
//
// Todas las posiciones son pixeles del cuadro actual: el perfil corre los puntos guardados
// con el movimiento propio de la torreta (retadores_trasladar).
#ifndef RETADOR_H
#define RETADOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define RETADOR_MAX 4 // retadores seguidos a la vez

typedef struct {
    int habilitada;          // 0 = sin barrida (comportamiento previo)
    int periodo_cuadros;     // cuadros entre barridas
    int n_confirmaciones;    // barridas seguidas que debe sumar un retador
    float umbral_movimiento; // desplazamiento minimo para considerar que algo se mueve (px)
    float radio_exclusion;   // candidatos mas cerca del blanco actual se ignoran (px)
    float radio_asociacion;  // distancia maxima a la posicion prevista del retador (px)
    float radio_primer_salto; // idem en la primera asociacion, sin velocidad todavia (px)
} ConfigBarrida;

typedef struct {
    float x, y;
} PuntoPx;

typedef struct {
    PuntoPx inicio, previo, ultimo; // primera, penultima y ultima posicion vista
    PuntoPx blanco_inicio;  // posicion del blanco cuando nacio el retador
    int confirmaciones;
} Retador;

typedef struct {
    ConfigBarrida cfg;
    Retador r[RETADOR_MAX];
    int n;
} Retadores;

// Valores iniciales (los de config/torreta.ini, seccion [barrida]).
ConfigBarrida barrida_por_defecto(void);

void retadores_iniciar(Retadores *rs, const ConfigBarrida *cfg);
void retadores_vaciar(Retadores *rs);

// Corre todos los puntos guardados (movimiento propio de la torreta entre cuadros).
void retadores_trasladar(Retadores *rs, float dx, float dy);

// Una barrida: blanco es la posicion actual del blanco y cand los candidatos globales,
// de mayor a menor confianza. Devuelve 1 y escribe *nuevo si hay que cambiar de blanco
// (y en ese caso vacia los retadores).
int retadores_barrida(Retadores *rs, PuntoPx blanco, const PuntoPx *cand, int n_cand, PuntoPx *nuevo);

#ifdef __cplusplus
}
#endif

#endif /* RETADOR_H */
