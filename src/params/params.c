// Todos los numeros del sistema (camara, planta, EKF, NMPC, comunicacion, limites...) viven en
// una sola estructura Params. Aqui estan sus valores por defecto y la lectura desde el INI, que
// los pisa. Asi ningun modulo tiene numeros escritos "a mano" en su codigo.
// Parametros centralizados (ver params.h).
#include "params.h"

#include <math.h>
#include <stdio.h>

// Valores por defecto. Los comentarios explican el porque de los que no son evidentes.
Params params_por_defecto(void)
{
    Params p;

    // Camara: 640x480, focal de 500 px y punto principal en el centro.
    p.cam.fu = 500.0;
    p.cam.fv = 500.0;
    p.cam.cu = 320.0;
    p.cam.cv = 240.0;
    p.cam.width = 640;
    p.cam.height = 480;

    // Camara: l_x <= 2 cm (informe); t_e = 10 ms y sigma_0 = 1.5 px (informe, tabla de
    // parametros del NMPC); lectura de las 480 filas en 22 ms (perdida.tex, cronologia).
    // Tamano aparente del avion: kw = f * envergadura, sin sesgo.
    p.camara.lx = 0.02;
    p.camara.t_e = 10e-3;
    p.camara.t_fila = 22e-3 / 480.0;
    p.camara.sigma0 = 1.5;
    p.camara.kw = 500.0 * 0.3;
    p.camara.bw = 0.0;
    p.camara.sigma_w = 2.0;

    // Enlace: UART a 921600 baudios.
    p.tiempos.baudios = 921600.0;
    // Telemetria (perdida.tex, tabla de mensajes): 35 bytes; a 500 Hz ocupa el 19 % de la UART.
    p.tiempos.f_telemetria = 500.0;

    // Planta (informe, tabla del motor 17HS4401): K_m = 0.166 N m/A con I_p = 1.2 A, N_r = 50,
    // T_d = 0.022 N m, J_r = 5.4e-6 kg m^2. Cargas, roces y m g l_cg son supuestos (CAD e
    // identificacion pendientes). Encoder magnetico de 12 bits.
    ParamsPlanta *pl = &p.planta;
    pl->J[0] = 5.4e-6 + 3e-4;
    pl->J[1] = 5.4e-6 + 1e-4;
    pl->b[0] = pl->b[1] = 1e-4;
    pl->tau_c[0] = pl->tau_c[1] = 5e-3;
    pl->Km_Ip = 0.166 * 1.2;
    pl->Nr = 50;
    pl->Td = 0.022;
    pl->mgl = 0.08 * 9.81 * 0.01;
    pl->micropasos[0] = 2;
    pl->micropasos[1] = 2;
    pl->relacion[0] = 3.0;
    pl->relacion[1] = 3.0;
    pl->cuentas_encoder = 4096;
    pl->dt_ejecutor = 1e-3;
    pl->tilt_activo = 1;
    pl->tilt_fijo = 0.0;

    p.lazo.Ts = 1.0 / 30.0;
    p.lazo.max_rechazos = 3;
    p.lazo.k_sigma_pista = 3.0;
    p.lazo.radio_min_pista = 20.0;

    // Limites: el pan puede girar unos +-150 grados y el tilt unos +-69 grados; FOV util de
    // 180 px de radio; la busqueda sin blanco acelera a 15 rad/s^2.
    p.lim.q_min[0] = -2.6;
    p.lim.q_min[1] = -1.2;
    p.lim.q_max[0] = 2.6;
    p.lim.q_max[1] = 1.2;
    p.lim.r_fov = 180.0;
    p.lim.qdd_max = 15.0;

    // Comunicacion (perdida.tex): Delta_fb >= T_fb + T_tx + margen = 2 + 1.4 + 1 ms; M = 8 nodos
    // optimos y N_ff = 20 (frenado a ~1 s sin paquetes).
    p.com.delta_fb = 4.5e-3;
    p.com.M = 8;
    p.com.N_ff = 20;

    // Ejecutor: a_lim derivado de la restriccion de torque (perdida.tex): eta K_m I_p / J_pan ~
    // 390 rad/s^2; se toma 120 rad/s^2, por sobre el u_max = 100 del NMPC, para que no actue en
    // operacion nominal ni por la cuantizacion de los
    // nodos. a_frenado = 10 rad/s^2.
    p.ejecutor.a_lim = 120.0;
    p.ejecutor.a_frenado = 10.0;
    p.ejecutor.margen_limite = 0.05;
    // Correccion proporcional del DDA desactivada (feedforward puro): con la resonancia del
    // stepper casi sin amortiguar (zeta ~ 0.001 con b = 1e-4) y el retardo del encoder, ninguna
    // correccion desde el encoder resulto robusta en el SIL (fase 5, hallazgo H13). Queda
    // parametrizada hasta identificar la resonancia y el amortiguamiento reales.
    p.ejecutor.kp = 0.0;
    p.ejecutor.tau_correccion = 0.0;
    p.ejecutor.alfa = 0.2;
    p.ejecutor.beta = 0.02;
    p.ejecutor.qd_max = 2.0;

    // NMPC (informe, tabla de parametros del NMPC): W_s = 1 px^-^2 fija la escala; w_p = 50
    // (duplicar la varianza cuesta como 7 px de error); W_u = 1e-2 y W_Delta = 1e-3 sobre la
    // correccion respecto del feedforward; eps_FOV = 1 % por lado (beta = 2.33); eta = 0.6;
    // u_max = 100 rad/s^2, bajo el limitador del ejecutor (ver ejecutor.a_lim).
    p.nmpc.W_s = 1.0;
    p.nmpc.w_p = 50.0;
    p.nmpc.W_u = 1e-2;
    p.nmpc.W_delta = 1e-3;
    p.nmpc.eps_P = 1e-6;
    p.nmpc.eps_fov = 0.01;
    p.nmpc.b_max = 60.0;
    p.nmpc.eta = 0.6;
    p.nmpc.u_max = 100.0;
    p.nmpc.iter_arranque = 3;
    p.nmpc.confianza_a = 1;
    p.nmpc.confianza_v = 2;
    // QP de HPIPM (fase 6, H32): con la FOV blanda activa, el modo por defecto del codigo generado
    // (balance, sin arranque en caliente y sin condensar) usaba una mediana de 36 iteraciones y
    // fijaba el p99 del ciclo. Con speed_abs, arranque en caliente de primales y duales y dos
    // bloques de condensacion, el p99 baja de 1.45 a 0.54 ms en el PC sin empeorar el RMS
    // medio. El tope de iteraciones se deja en 50: con 10 a 20 el seguimiento empeoraba.
    p.nmpc.qp_iter_max = 50;
    p.nmpc.qp_modo = QP_MODO_SPEED_ABS;
    p.nmpc.qp_arranque_caliente = 2;
    p.nmpc.qp_cond_N = 2;

    // Dashboard: apagado y solo en la maquina local; 30 cuadros de video y 10 Hz de paneles.
    p.dashboard.habilitado = 0;
    p.dashboard.puerto = 8080;
    snprintf(p.dashboard.escucha, sizeof(p.dashboard.escucha), "127.0.0.1");
    p.dashboard.clientes_max = 4;
    p.dashboard.formato = VIDEO_MJPEG;
    p.dashboard.calidad_jpeg = 80;
    p.dashboard.fps_max = 30.0;
    p.dashboard.hz_paneles = 10.0;
    snprintf(p.dashboard.ajustes, sizeof(p.dashboard.ajustes), "config/ajustes.ini");

    // Modo vivo: camara 640x480 a 30 fps, nivel 0, sin fin y a tiempo real.
    p.vivo.nivel = 0;
    p.vivo.ancho = 640;
    p.vivo.alto = 480;
    p.vivo.fps = 30.0;
    p.vivo.q0[0] = p.vivo.q0[1] = 0.0;
    p.vivo.latencia_nominal = 0.0;
    p.vivo.segundos = 0.0;
    p.vivo.tiempo_real = 1;
    snprintf(p.vivo.puerto, sizeof p.vivo.puerto, "/dev/serial0");
    p.vivo.archivo[0] = '\0'; // sin grabacion: la camara
    p.vivo.camara = CAMARA_AUTO;
    p.vivo.usb[0] = '\0';     // la primera camara USB que haya

    p.perfil_ekf = PERFIL_LASER_LENTO;
    params_ekf_por_perfil(&p);
    return p;
}
//---------------------------------------------------------------------------
// Nombres de los perfiles del EKF, en el mismo orden que el enum PerfilEkf: cada uno tiene su
// seccion [ekf_<nombre>] en el INI.
static const char *const nombres_perfil[PERFIL_N] = {"laser_lento", "laser_rapido", "laser_brusco",
                                                    "avion", "desconocido"};

const char *params_nombre_perfil_ekf(int perfil)
{
    return perfil >= 0 && perfil < PERFIL_N ? nombres_perfil[perfil] : "?";
}

const char *const PARAMS_CLAVES_AJUSTE[] = {
    "general.perfil", "ekf.perfil", "camara.fu", "camara.fv", "camara.cu", "camara.cv", "vivo.nivel", "laser.canal", "laser.umbral_adquisicion",
    "laser.umbral_seguimiento", "laser.radio_blob_max", "laser.confirmaciones", "laser.ambiguedad", "laser.contraste_min", "laser.camara", "objeto.prob_min",
    "objeto.camara", "objeto.margen_borde", "objeto.kcf_escala_max", "seleccion.camara", "barrida.habilitada",
    "usb.camara.fu", "usb.camara.fv", "usb.camara.cu", "usb.camara.cv", NULL};

// Claves de la seccion [ekf_<nombre>]: lo que cambia de un perfil a otro.
static void leer_ekf_perfil(Ini *ini, const char *nombre, ConfigEKF *e)
{
    char k[64];
#define CLAVE(c) (snprintf(k, sizeof(k), "ekf_%s.%s", nombre, c), k)
    static const char *const sels[3] = {"fijo", "nis", "imm"};
    static const char *const rhos[2] = {"considerado", "estimado"};
    ini_leer_opcion(ini, CLAVE("seleccion"), sels, 3, &e->seleccion);
    ini_leer_double(ini, CLAVE("tau_m"), &e->tau_m[0]);
    ini_leer_double(ini, CLAVE("sigma_m"), &e->sigma_m[0]);
    ini_leer_bool(ini, CLAVE("adaptativo"), &e->adaptativo);
    ini_leer_double(ini, CLAVE("imm_T_reposo"), &e->imm_T[0]);
    ini_leer_double(ini, CLAVE("imm_T_barrido"), &e->imm_T[1]);
    ini_leer_opcion(ini, CLAVE("rho"), rhos, 2, &e->tratamiento_rho);
    double d = 1.0 / e->rho0; // distancia inicial al blanco (m): la pared en el laser
    ini_leer_double(ini, CLAVE("distancia"), &d);
    e->rho0 = 1.0 / d;
    ini_leer_double(ini, CLAVE("sigma_rho0"), &e->sigma_rho0);
    ini_leer_double(ini, CLAVE("q_rho"), &e->q_rho);
    ini_leer_bool(ini, CLAVE("usar_tamano"), &e->usar_tamano);
#undef CLAVE
}
//---------------------------------------------------------------------------
// Lee del INI todas las secciones encima de los valores que ya tiene p. Cada clave es opcional:
// la que no esta conserva su valor por defecto.
void params_leer(Ini *ini, Params *p)
{
    ini_leer_double(ini, "camara.fu", &p->cam.fu);
    ini_leer_double(ini, "camara.fv", &p->cam.fv);
    ini_leer_double(ini, "camara.cu", &p->cam.cu);
    ini_leer_double(ini, "camara.cv", &p->cam.cv);
    ini_leer_int(ini, "camara.ancho", &p->cam.width);
    ini_leer_int(ini, "camara.alto", &p->cam.height);
    ini_leer_double(ini, "camara.lx", &p->camara.lx);
    ini_leer_double(ini, "camara.t_e", &p->camara.t_e);
    ini_leer_double(ini, "camara.t_fila", &p->camara.t_fila);
    ini_leer_double(ini, "camara.sigma0", &p->camara.sigma0);
    ini_leer_double(ini, "camara.kw", &p->camara.kw);
    ini_leer_double(ini, "camara.bw", &p->camara.bw);
    ini_leer_double(ini, "camara.sigma_w", &p->camara.sigma_w);

    ParamsPlanta *pl = &p->planta;
    ini_leer_int(ini, "planta.micropasos_pan", &pl->micropasos[0]);
    ini_leer_int(ini, "planta.micropasos_tilt", &pl->micropasos[1]);
    ini_leer_double(ini, "planta.relacion_pan", &pl->relacion[0]);
    ini_leer_double(ini, "planta.relacion_tilt", &pl->relacion[1]);
    ini_leer_double(ini, "planta.Km_Ip", &pl->Km_Ip);
    ini_leer_double(ini, "planta.J_pan", &pl->J[0]);
    ini_leer_double(ini, "planta.J_tilt", &pl->J[1]);
    ini_leer_double(ini, "planta.mgl", &pl->mgl);
    double b_visc = pl->b[0], tc = pl->tau_c[0];
    ini_leer_double(ini, "planta.b", &b_visc);
    ini_leer_double(ini, "planta.tau_c", &tc);
    pl->b[0] = pl->b[1] = b_visc;
    pl->tau_c[0] = pl->tau_c[1] = tc;
    ini_leer_int(ini, "planta.cuentas_encoder", &pl->cuentas_encoder);
    ini_leer_double(ini, "planta.dt_ejecutor", &pl->dt_ejecutor);
    ini_leer_bool(ini, "planta.tilt_activo", &pl->tilt_activo);
    ini_leer_double(ini, "planta.tilt_fijo", &pl->tilt_fijo);

    ParamsTiempos *ti = &p->tiempos;
    ini_leer_double(ini, "tiempos.baudios", &ti->baudios);
    ini_leer_double(ini, "tiempos.f_telemetria", &ti->f_telemetria);

    // En el INI se escribe la frecuencia, que es mas intuitiva; se guarda el periodo.
    double fps = 1.0 / p->lazo.Ts;
    ini_leer_double(ini, "lazo.fps", &fps);
    if (fps > 0.0)
        p->lazo.Ts = 1.0 / fps;
    else {
        fprintf(stderr, "lazo.fps tiene que ser positivo\n");
        ini->errores++;
    }
    ini_leer_int(ini, "lazo.max_rechazos", &p->lazo.max_rechazos);
    ini_leer_double(ini, "lazo.k_sigma_pista", &p->lazo.k_sigma_pista);
    ini_leer_double(ini, "lazo.radio_min_pista", &p->lazo.radio_min_pista);

    ParamsLimites *lm = &p->lim;
    ini_leer_double(ini, "limites.pan_min", &lm->q_min[0]);
    ini_leer_double(ini, "limites.pan_max", &lm->q_max[0]);
    ini_leer_double(ini, "limites.tilt_min", &lm->q_min[1]);
    ini_leer_double(ini, "limites.tilt_max", &lm->q_max[1]);
    ini_leer_double(ini, "limites.r_fov", &lm->r_fov);
    ini_leer_double(ini, "limites.qdd_max", &lm->qdd_max);

    ParamsNmpc *nm = &p->nmpc;
    ini_leer_double(ini, "nmpc.W_s", &nm->W_s);
    ini_leer_double(ini, "nmpc.w_p", &nm->w_p);
    ini_leer_double(ini, "nmpc.W_u", &nm->W_u);
    ini_leer_double(ini, "nmpc.W_delta", &nm->W_delta);
    ini_leer_double(ini, "nmpc.eps_P", &nm->eps_P);
    ini_leer_int(ini, "nmpc.iter_arranque", &nm->iter_arranque);
    ini_leer_double(ini, "nmpc.eps_fov", &nm->eps_fov);
    ini_leer_double(ini, "nmpc.b_max", &nm->b_max);
    ini_leer_double(ini, "nmpc.eta", &nm->eta);
    ini_leer_double(ini, "nmpc.u_max", &nm->u_max);
    ini_leer_bool(ini, "nmpc.confianza_a", &nm->confianza_a);
    ini_leer_int(ini, "nmpc.confianza_v", &nm->confianza_v);
    static const char *const modos[QP_MODO_N] = {"balance", "speed", "speed_abs", "robust"};
    ini_leer_int(ini, "nmpc.qp_iter_max", &nm->qp_iter_max);
    ini_leer_opcion(ini, "nmpc.qp_modo", modos, QP_MODO_N, &nm->qp_modo);
    ini_leer_int(ini, "nmpc.qp_arranque_caliente", &nm->qp_arranque_caliente);
    ini_leer_int(ini, "nmpc.qp_cond_N", &nm->qp_cond_N);

    ParamsComunicacion *co = &p->com;
    ini_leer_double(ini, "comunicacion.delta_fb", &co->delta_fb);
    ini_leer_int(ini, "comunicacion.M", &co->M);
    ini_leer_int(ini, "comunicacion.N_ff", &co->N_ff);
    ParamsEjecutor *ej = &p->ejecutor;
    ini_leer_double(ini, "ejecutor.a_lim", &ej->a_lim);
    ini_leer_double(ini, "ejecutor.a_frenado", &ej->a_frenado);
    ini_leer_double(ini, "ejecutor.margen_limite", &ej->margen_limite);
    ini_leer_double(ini, "ejecutor.kp", &ej->kp);
    ini_leer_double(ini, "ejecutor.tau_correccion", &ej->tau_correccion);
    ini_leer_double(ini, "ejecutor.alfa", &ej->alfa);
    ini_leer_double(ini, "ejecutor.beta", &ej->beta);
    ini_leer_double(ini, "ejecutor.qd_max", &ej->qd_max);

    // [ekf_<perfil>]: se leen todas las secciones de perfil (para validar sus claves) y se usa
    // la del perfil elegido. Despues, [ekf] con lo comun a todos los perfiles.
    ini_leer_opcion(ini, "ekf.perfil", nombres_perfil, PERFIL_N, &p->perfil_ekf);
    for (int i = 0; i < PERFIL_N; ++i) {
        Params t = *p;
        t.perfil_ekf = i;
        params_ekf_por_perfil(&t);
        leer_ekf_perfil(ini, nombres_perfil[i], &t.ekf);
        if (i == p->perfil_ekf)
            p->ekf = t.ekf;
    }
    ConfigEKF *e = &p->ekf;
    ini_leer_double(ini, "ekf.tau_m_barrido", &e->tau_m[1]);
    ini_leer_double(ini, "ekf.sigma_m_barrido", &e->sigma_m[1]);
    ini_leer_double(ini, "ekf.nis_subir", &e->nis_subir);
    ini_leer_double(ini, "ekf.nis_bajar", &e->nis_bajar);
    ini_leer_double(ini, "ekf.sigma_q", &e->sigma_q);
    ini_leer_double(ini, "ekf.sigma_perdido", &e->sigma_perdido);
    ini_leer_double(ini, "ekf.inflacion_fallo", &e->inflacion_fallo);
    ini_leer_double(ini, "ekf.tope_inflacion", &e->tope_inflacion);
    ini_leer_int(ini, "ekf.confirmar_maniobra", &e->confirmar_maniobra);
    ini_leer_double(ini, "ekf.nis_maniobra", &e->nis_maniobra);
    ini_leer_bool(ini, "ekf.histeresis_modo", &e->histeresis_modo);
    ini_leer_int(ini, "ekf.max_fallos", &e->max_fallos);

    // Secciones que no son de la Pi: las lee el firmware y la generacion del NMPC.
    ini_ignorar_seccion(ini, "esp32");
    ini_ignorar_seccion(ini, "nmpc_solver");

    ParamsDashboard *d = &p->dashboard;
    ini_leer_bool(ini, "dashboard.habilitado", &d->habilitado);
    ini_leer_int(ini, "dashboard.puerto", &d->puerto);
    ini_leer_texto(ini, "dashboard.escucha", d->escucha, sizeof(d->escucha));
    ini_leer_int(ini, "dashboard.clientes_max", &d->clientes_max);
    static const char *const formatos[2] = {"mjpeg", "h264"};
    int formato = (int)d->formato;
    ini_leer_opcion(ini, "dashboard.formato", formatos, 2, &formato);
    d->formato = (FormatoVideo)formato;
    ini_leer_int(ini, "dashboard.calidad_jpeg", &d->calidad_jpeg);
    ini_leer_double(ini, "dashboard.fps_max", &d->fps_max);
    ini_leer_double(ini, "dashboard.hz_paneles", &d->hz_paneles);

    ParamsVivo *v = &p->vivo;
    ini_leer_int(ini, "vivo.nivel", &v->nivel);
    if (v->nivel < 0 || v->nivel > 4) {
        fprintf(stderr, "vivo.nivel = %d: se esperaba un entero de 0 a 4\n", v->nivel);
        ini->errores++;
    }
    ini_leer_int(ini, "vivo.ancho", &v->ancho);
    ini_leer_int(ini, "vivo.alto", &v->alto);
    ini_leer_double(ini, "vivo.fps", &v->fps);
    ini_leer_double(ini, "vivo.q0_pan", &v->q0[0]);
    ini_leer_double(ini, "vivo.q0_tilt", &v->q0[1]);
    ini_leer_double(ini, "vivo.latencia_nominal", &v->latencia_nominal);
    ini_leer_double(ini, "vivo.segundos", &v->segundos);
    ini_leer_bool(ini, "vivo.tiempo_real", &v->tiempo_real);
    ini_leer_texto(ini, "vivo.puerto", v->puerto, sizeof(v->puerto));
    ini_leer_texto(ini, "vivo.archivo", v->archivo, sizeof(v->archivo));
    static const char *const camaras[] = {"auto", "picam", "usb"};
    ini_leer_opcion(ini, "vivo.camara", camaras, 3, &v->camara);
    ini_leer_texto(ini, "vivo.usb", v->usb, sizeof(v->usb));
}
//---------------------------------------------------------------------------
// Copia a la configuracion activa del EKF la del perfil elegido (laser_lento, avion, etc.).
void params_ekf_por_perfil(Params *p)
{
    ConfigEKF *e = &p->ekf;
    const int perfil = p->perfil_ekf;
    // Comunes: sigma_q = 0.5 mrad. El filtro usa la pose MEDIDA por el encoder (interpolada al
    // instante de captura), cuyo error es la cuantizacion, res/sqrt12 = 0.44 mrad con 4096
    // cuentas, mas la interpolacion; el angulo de carga (H8 de la fase 3) solo afecta a la
    // pose comandada. Ventana de 6 y 15 fallos (medio segundo).
    e->sigma_q = 0.5e-3;
    e->ventana = 6;
    e->phi_max = 1e3;
    e->inflacion_fallo = 4.0;
    // Tope de la inflacion por fallos y confirmacion de maniobra (fase 6, H27): sin tope, la
    // inflacion compuesta llegaba a cientos de rad^2/s^2; con un solo rechazo como maniobra, un
    // atipico aislado se convertia en una velocidad espuria.
    e->tope_inflacion = 10.0;
    e->confirmar_maniobra = 2;
    // Un rechazo con NIS > 15.2 (chi^22 al 99.95 %) ya es maniobra: de las falsas alarmas de la
    // puerta solo el 5 % lo supera (cola exponencial), y en el SIL separa los atipicos (p90 de
    // 12 a 15) de las maniobras del laser brusco y rapido (medianas de 27 y 16).
    e->nis_maniobra = 15.2;
    e->histeresis_modo = 1;
    e->max_fallos = 15;
    // Perdido cuando sigma de psi supera el semiangulo de la FOV (r_fov/f): la prediccion ya no
    // dice donde mirar.
    e->sigma_perdido = p->lim.r_fov / p->cam.fu;
    e->seleccion = SELECCION_FIJO;
    e->nis_subir = 4.0;
    e->nis_bajar = 2.0;
    e->tau_m[1] = 0.3;
    e->sigma_m[1] = 5.0;
    e->imm_T[0] = e->imm_T[1] = 1.0;
    e->tratamiento_rho = RHO_CONSIDERADO;
    e->q_rho = 0.0;
    e->rho_min = 1.0 / 3.5;
    e->rho_max = 1.0 / 0.4;
    e->sigma_rho_cota = 0.02;
    e->usar_tamano = 0;
    e->kw = p->camara.kw;
    e->bw = p->camara.bw;
    e->sigma_w = p->camara.sigma_w;
    e->adaptativo = 1;
    switch (perfil) {
    case PERFIL_LASER_LENTO:
    case PERFIL_LASER_RAPIDO:
    case PERFIL_LASER_BRUSCO:
        // Dos modos de Singer: reposo (temblor y deriva) y barrido (aceleraciones de varios
        // rad/s^2). rho considerado = 1/d_pared (pared a 2 m); su desviacion cubre
        // la variacion de r sobre la pared (r = 2.24 m en el borde frente a 2 m al centro).
        e->tau_m[0] = 0.5;
        e->sigma_m[0] = 0.05;
        e->seleccion = SELECCION_NIS;
        e->adaptativo = 0;
        e->rho0 = 1.0 / 2.0;
        e->sigma_rho0 = 0.05;
        // Permanencias medias del IMM (s), supuestas para un laser movido a mano: reposos de
        // 0.3 a 1.5 s; barridos de 0.85 m (distancia media en una pared de 2 x 1.2 m) a
        // 0.3 m/s (lento) o 1.5 m/s (rapido), o saltos de 0.15 s (brusco).
        e->imm_T[0] = 0.9;
        e->imm_T[1] = perfil == PERFIL_LASER_BRUSCO ? 0.15 : 0.8478 / (perfil == PERFIL_LASER_LENTO ? 0.3 : 1.5);
        // En el lento, IMM: sus barridos largos se siguen mezclando los modos, sin la cascada de
        // rechazos ni el timbre de la conmutacion (fase 6, H30: NMPC un 25-35 % mejor que con
        // conmutacion en semillas de validacion). En el brusco, la conmutacion con re-gating
        // sigue siendo mejor: el modo de barrido del IMM no esta listo al empezar cada salto
        // (H28).
        if (perfil == PERFIL_LASER_LENTO)
            e->seleccion = SELECCION_IMM;
        break;
    case PERFIL_AVION:
        // Maniobras suaves de ~1.4 rad/s^2; rho estimado con el tamano aparente.
        e->tau_m[0] = 1.0;
        e->sigma_m[0] = 1.5;
        e->tratamiento_rho = RHO_ESTIMADO;
        e->rho0 = 1.0 / 1.6;
        e->sigma_rho0 = 0.3;
        e->q_rho = 0.2;
        e->usar_tamano = 1;
        break;
    default: // desconocido: parametros conservadores y rho nominal del rango
        e->tau_m[0] = 1.0;
        e->sigma_m[0] = 2.0;
        e->rho0 = 0.5;
        e->sigma_rho0 = 0.3;
        break;
    }
}
