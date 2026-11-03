// Triple buffer sin bloqueo, para un escritor y un lector. El escritor (el hilo del lazo) nunca
// espera; el lector (el hilo del dashboard) se queda siempre con lo ultimo escrito, y lo
// intermedio que no alcanzo a leer se pierde.
//
// Protocolo. Tres ranuras: una es del escritor (escr_), otra del lector (lect_) y la tercera
// es la intermedia, cuyo indice vive en medio_ junto con el bit NUEVO.
//   - escribir: llena su ranura y la cambia por la intermedia con un exchange que deja
//     NUEVO en 1; la ranura que recibe pasa a ser la suya.
//   - leer: si NUEVO esta en 0, no hay nada. Si no, cambia su ranura por la intermedia con un
//     exchange que deja NUEVO en 0, y saca el contenido de la ranura recibida.
// Cada ranura tiene un solo dueno en cada momento, asi que nadie lee una ranura a medio
// escribir. Los exchange son acq_rel: lo escrito en una ranura antes de entregarla es visible
// para quien la recibe. Solo el lector borra NUEVO, asi que una escritura nunca se pierde
// para el lector sin que llegue otra mas nueva.
#ifndef TRIPLE_BUFFER_HPP
#define TRIPLE_BUFFER_HPP

#include <atomic>
#include <utility>

template <class T> class TripleBuffer {
public:
    TripleBuffer() : medio_(1), escr_(0), lect_(2) {}
    TripleBuffer(const TripleBuffer &) = delete;
    TripleBuffer &operator=(const TripleBuffer &) = delete;

    // Solo desde el hilo escritor. Nunca espera.
    void escribir(const T &v)
    {
        ranura_[escr_] = v;
        escr_ = medio_.exchange(escr_ | NUEVO, std::memory_order_acq_rel) & INDICE;
    }

    // Solo desde el hilo lector. true si hay algo nuevo desde la ultima lectura; entonces lo
    // mueve a v (la ranura queda vacia, y lo que v tenia antes se suelta en este hilo).
    bool leer(T &v)
    {
        if (!(medio_.load(std::memory_order_relaxed) & NUEVO)) return false;
        lect_ = medio_.exchange(lect_, std::memory_order_acq_rel) & INDICE;
        v = std::move(ranura_[lect_]);
        return true;
    }

private:
    static const unsigned INDICE = 3u, NUEVO = 4u;
    static_assert(ATOMIC_INT_LOCK_FREE == 2, "std::atomic<unsigned> debe ser sin bloqueo");

    T ranura_[3];
    std::atomic<unsigned> medio_;  // indice de la intermedia | NUEVO
    unsigned escr_, lect_;         // de uso exclusivo del escritor y del lector
};

#endif /* TRIPLE_BUFFER_HPP */
