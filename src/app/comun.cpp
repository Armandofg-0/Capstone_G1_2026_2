// Funciones compartidas por los distintos modos (ver modos.hpp).

#include "modos.hpp"

#include <algorithm>
#include <cstdio>

// Percentil p (de 0 a 100) con interpolacion lineal, igual que np.percentile. El vector
// se recibe por valor (es una copia), asi que se puede ordenar sin afectar el original.
// Se ubica la posicion fraccionaria p/100 * (n-1) en el vector ordenado y se interpola
// entre los dos valores vecinos (ver la misma funcion en sim_control.c).
double percentil(std::vector<double> v, double p)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    const double pos = p / 100.0 * (v.size() - 1);
    const size_t i = (size_t)pos;
    return i + 1 < v.size() ? v[i] + (pos - i) * (v[i + 1] - v[i]) : v[i];
}
//---------------------------------------------------------------------------
// Milisegundos (con decimales) transcurridos desde el instante t0.
double msDesde(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
