// Historial con los ultimos cuadros, identificados por su indice. Lo comparten dos hilos:
// el hilo del lazo, que va agregando los cuadros a medida que llegan, y el hilo de la red,
// que los lee para detectar y para "poner al dia" un seguidor nuevo (ver perfil_objeto).
//
// Un detalle importante sobre la memoria: guardar un cv::Mat solo copia su "cabecera"
// (un puntero a los pixeles y algunos datos). Los pixeles en si no se copian: ambas
// cv::Mat apuntan a los mismos datos, y OpenCV lleva un contador de cuantas los estan
// usando (ese contador es atomico, asi que es seguro usarlo desde varios hilos). Esto
// hace que guardar un cuadro sea muy barato, pero tiene una consecuencia: quien entrega
// un cuadro no puede reescribirlo despues, porque cambiaria tambien la copia guardada.
// Por eso la captura tiene que leer cada cuadro en un buffer nuevo.

#pragma once

#include "percepcion.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <vector>

class Historial
{
public:
    // La capacidad es cuantos cuadros se recuerdan. Como minimo se guardan 2. "explicit"
    // evita que el compilador convierta un numero en un Historial sin que se note.
    explicit Historial(size_t capacidad) : anillo_(capacidad < 2 ? 2 : capacidad), ultimo_(-1) {}

    // Agrega un cuadro. Los cuadros se guardan en un "anillo" (buffer circular): el cuadro
    // de indice i va en la posicion i % capacidad, asi que cuando el anillo se llena, cada
    // cuadro nuevo pisa al mas viejo.
    void agregar(const Cuadro &c)
    {
        {
            std::lock_guard<std::mutex> lk(m_);
            anillo_[(size_t)(c.indice % (int64_t)anillo_.size())] = c;
            ultimo_ = c.indice;
        }
        // Se avisa a quien este esperando un cuadro (ver esperar). Se hace fuera del
        // bloque del mutex para que el hilo que despierte pueda tomarlo de inmediato.
        cv_.notify_all();
    }

    // false si el cuadro todavia no llego o ya salio del anillo.
    bool obtener(int64_t indice, Cuadro &c) const
    {
        std::lock_guard<std::mutex> lk(m_);
        return obtenerSinCandado(indice, c);
    }

    // Espera a que llegue el cuadro `indice` (lo usa el hilo de la red en modo
    // determinista). Devuelve false si se pidio abortar o si ya salio del anillo.
    bool esperar(int64_t indice, Cuadro &c, const std::atomic<bool> &abortar) const
    {
        std::unique_lock<std::mutex> lk(m_);
        while (ultimo_ < indice)
        {
            if (abortar)
                return false;
            // Se duerme como maximo 20 ms y se vuelve a revisar. El limite de tiempo
            // asegura que, aunque nadie avise, se revise cada tanto si hay que abortar.
            cv_.wait_for(lk, std::chrono::milliseconds(20));
        }
        return obtenerSinCandado(indice, c);
    }

    // Indice del ultimo cuadro agregado (-1 si todavia no hay ninguno).
    int64_t ultimo() const
    {
        std::lock_guard<std::mutex> lk(m_);
        return ultimo_;
    }

private:
    // Igual que obtener, pero supone que quien llama ya tomo el mutex.
    // Un indice no esta disponible si es negativo, si todavia no llego (mayor que el
    // ultimo) o si es tan viejo que ya fue pisado por uno nuevo (quedo a una capacidad o
    // mas de distancia del ultimo). La comparacion final c.indice == indice es una
    // proteccion extra de que la posicion del anillo tenga realmente ese cuadro.
    bool obtenerSinCandado(int64_t indice, Cuadro &c) const
    {
        if (indice < 0 || indice > ultimo_ || indice <= ultimo_ - (int64_t)anillo_.size())
            return false;
        c = anillo_[(size_t)(indice % (int64_t)anillo_.size())];
        return c.indice == indice;
    }

    // "mutable" permite modificar el mutex y la variable de condicion incluso dentro de
    // funciones const (como obtener): tomar un mutex no cambia el contenido del
    // historial, pero tecnicamente si modifica al objeto mutex.
    mutable std::mutex m_;
    mutable std::condition_variable cv_;
    std::vector<Cuadro> anillo_;
    int64_t ultimo_;
};
