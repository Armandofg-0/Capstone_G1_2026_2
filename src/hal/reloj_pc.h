// Reloj monotonico del computador donde corre el codigo (PC o Pi), en milisegundos con
// decimales. Sirve para medir tiempos de computo; no es la base de tiempo del lazo.
#ifndef RELOJ_PC_H
#define RELOJ_PC_H

#ifdef __cplusplus
extern "C" {
#endif

double reloj_pc_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* RELOJ_PC_H */
