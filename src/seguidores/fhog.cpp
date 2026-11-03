// Implementacion del HOG de Felzenszwalb (ver fhog.hpp para la idea general y la
// licencia del codigo original de OpenCV).
//
// El calculo tiene tres etapas, y cada una tiene su propia funcion:
//  1) getFeatureMaps: calcula el gradiente de cada pixel (hacia donde cambia el brillo y
//     cuanto) y arma, en cada celda, un histograma de las direcciones de esos gradientes.
//     Quedan 27 canales por celda.
//  2) normalizeAndTruncate: normaliza cada celda respecto de sus vecinas, para que el
//     resultado no dependa del contraste de la zona, y recorta los valores muy altos.
//     Quedan 108 canales por celda.
//  3) pcaFeatureMaps: resume esos 108 canales en 31, sumando grupos de ellos.
//
// El codigo conserva el estilo de C del original (malloc, free y punteros "crudos"), que
// es el de la libreria de la que se tomo.

#include "fhog.hpp"

#include <opencv2/imgproc.hpp>
#include <cfloat>
#include <cmath>
#include <cstdlib>

// Todo lo de este namespace sin nombre solo es visible dentro de este archivo.
namespace {

// Cantidad de sectores de orientacion del histograma de gradientes. Con 9 sectores en
// media vuelta (180 grados), cada sector abarca 20 grados.
const int NUM_SECTOR = 9;

// Umbral de truncamiento que se aplica despues de normalizar: ningun valor puede superar
// 0.2. Esto evita que un solo borde muy fuerte domine toda la descripcion de la celda.
const float TRUNCATE_ALFA = 0.2f;

// Mapa rectangular de sizeX x sizeY celdas, donde cada celda guarda un vector de
// `numFeatures` componentes. El componente k de la celda (x, y) esta en
// map[(y * sizeX + x) * numFeatures + k], es decir, las celdas van por filas y, dentro de
// cada celda, sus componentes van seguidos.
struct FeatureMap
{
    int sizeX;
    int sizeY;
    int numFeatures;
    float *map;
};
//---------------------------------------------------------------------------
// Reserva un FeatureMap con todos sus valores en cero (calloc, a diferencia de malloc,
// deja la memoria reservada llena de ceros).
FeatureMap *allocFeatureMap(int sizeX, int sizeY, int numFeatures)
{
    FeatureMap *obj = new FeatureMap;
    obj->sizeX = sizeX;
    obj->sizeY = sizeY;
    obj->numFeatures = numFeatures;
    obj->map = (float *)calloc((size_t)sizeX * sizeY * numFeatures, sizeof(float));
    return obj;
}
//---------------------------------------------------------------------------
// Libera un FeatureMap (primero sus datos y luego la estructura).
void freeFeatureMap(FeatureMap *obj)
{
    free(obj->map);
    delete obj;
}
//---------------------------------------------------------------------------
// Construye los histogramas de orientacion: uno por celda, con asignacion suave e
// interpolacion bilineal. Salen 27 canales: 9 que no distinguen el sentido del contraste
// (claro a oscuro y oscuro a claro cuentan igual) y 18 que si lo distinguen.
//
// k es el tamano de la celda en pixeles.
FeatureMap *getFeatureMaps(const cv::Mat &image, const int k)
{
    const int height = image.rows;
    const int width = image.cols;
    const int numChannels = image.channels();

    // Diferencias centrales: el gradiente en x de un pixel es (vecino derecho - vecino
    // izquierdo), y lo mismo en y con los vecinos de abajo y de arriba. Se hace con el
    // kernel (-1, 0, 1) y el ancla centrada por defecto.
    const float kernel[3] = {-1.f, 0.f, 1.f};
    const cv::Mat kernel_dx(1, 3, CV_32F, (void *)kernel);
    const cv::Mat kernel_dy(3, 1, CV_32F, (void *)kernel);

    cv::Mat dx, dy; // Tipo Mat (matrices densas)
    // Funcion para convolucion con matrices:
    cv::filter2D(image, dx, CV_32F, kernel_dx);
    cv::filter2D(image, dy, CV_32F, kernel_dy);

    const int sizeX = width / k;         // celdas a lo ancho
    const int sizeY = height / k;        // celdas a lo alto
    const int p = 3 * NUM_SECTOR;        // 27 canales por celda
    const int stringSize = sizeX * p;    // cantidad de numeros en una fila completa de celdas

    FeatureMap *map = allocFeatureMap(sizeX, sizeY, p);

    // boundary_x[i], boundary_y[i] forman un vector unitario que apunta en la direccion
    // i * 20 grados. Se usan para ver a que sector se parece mas cada gradiente.
    float boundary_x[NUM_SECTOR + 1];
    float boundary_y[NUM_SECTOR + 1];
    for (int i = 0; i <= NUM_SECTOR; i++)
    {
        // Vectores unitarios de los bordes del sector repartidos en el semicirculo:
        const float arg = (float)i * ((float)CV_PI / (float)NUM_SECTOR);
        boundary_x[i] = cosf(arg);
        boundary_y[i] = sinf(arg);
    }

    // r: magnitud del gradiente de cada pixel. alfa: los dos sectores de orientacion en
    // los que cae (uno sin signo, de 0 a 8, y otro con signo, de 0 a 17).
    float *r = (float *)malloc(sizeof(float) * (width * height));
    int *alfa = (int *)malloc(sizeof(int) * (width * height * 2));

    // Se recorren todos los pixeles menos el borde (ahi el gradiente no esta bien definido).
    for (int j = 1; j < height - 1; j++)
    {
        const float *datadx = dx.ptr<float>(j);
        const float *datady = dy.ptr<float>(j);
        for (int i = 1; i < width - 1; i++)
        {
            // De todos los canales de color, se queda con el gradiente de mayor magnitud.
            // (En una imagen en color, cada pixel tiene tres gradientes, uno por canal.)
            float x = datadx[i * numChannels];
            float y = datady[i * numChannels];
            r[j * width + i] = sqrtf(x * x + y * y);

            for (int ch = 1; ch < numChannels; ch++)
            {
                const float tx = datadx[i * numChannels + ch];
                const float ty = datady[i * numChannels + ch];
                const float magnitude = sqrtf(tx * tx + ty * ty);
                if (magnitude > r[j * width + i])
                {
                    r[j * width + i] = magnitude;
                    x = tx;
                    y = ty;
                }
            }

            // Sector de orientacion mas cercano, con y sin signo.
            // El producto punto entre el gradiente y cada direccion de referencia es mas
            // grande cuanto mas alineados esten. Si el producto es muy negativo, el
            // gradiente apunta en la direccion contraria, y se usa el sector "opuesto"
            // (kk + 9). Asi, con solo 9 direcciones de referencia, se cubren los 18
            // sectores de la vuelta completa.
            float maxDot = boundary_x[0] * x + boundary_y[0] * y;
            int maxi = 0;
            for (int kk = 0; kk < NUM_SECTOR; kk++)
            {
                const float dotProd = boundary_x[kk] * x + boundary_y[kk] * y;
                if (dotProd > maxDot)
                {
                    maxDot = dotProd;
                    maxi = kk;
                }
                else if (-dotProd > maxDot)
                {
                    maxDot = -dotProd;
                    maxi = kk + NUM_SECTOR;
                }
            }
            alfa[j * width * 2 + i * 2] = maxi % NUM_SECTOR; // sin signo (0..8)
            alfa[j * width * 2 + i * 2 + 1] = maxi;          // con signo (0..17)
        }
    }

    // Pesos de la interpolacion bilineal de un pixel entre su propia celda y la vecina.
    //
    // En vez de que cada pixel vote solo en su celda, reparte su voto entre su celda y la
    // celda vecina mas cercana, segun que tan cerca este de cada centro. Esto evita que
    // un borde que se mueve un pixel cambie bruscamente de celda.
    // nearest[ii] dice hacia que lado esta la celda vecina mas cercana para un pixel en
    // la posicion ii dentro de su celda: -1 (la anterior) si esta en la primera mitad, y
    // +1 (la siguiente) si esta en la segunda mitad.
    int *nearest = (int *)malloc(sizeof(int) * k);
    float *w = (float *)malloc(sizeof(float) * (k * 2));

    for (int i = 0; i < k / 2; i++)
        nearest[i] = -1;
    for (int i = k / 2; i < k; i++)
        nearest[i] = 1;

    // w[2j] es el peso para la celda propia y w[2j+1] el peso para la vecina, para un
    // pixel en la posicion j. a_x y b_x son las distancias del pixel al centro de la celda
    // propia y al de la vecina. La formula (1/a) * (a b / (a + b)) se simplifica a
    // b / (a + b): cuanto mas cerca esta de un centro, mas peso le da a esa celda, y los
    // dos pesos siempre suman 1.
    for (int j = 0; j < k / 2; j++)
    {
        const float b_x = k / 2 + j + 0.5f;
        const float a_x = k / 2 - j - 0.5f;
        w[j * 2] = 1.0f / a_x * ((a_x * b_x) / (a_x + b_x));
        w[j * 2 + 1] = 1.0f / b_x * ((a_x * b_x) / (a_x + b_x));
    }
    for (int j = k / 2; j < k; j++)
    {
        const float a_x = j - k / 2 + 0.5f;
        const float b_x = -j + k / 2 - 0.5f + k;
        w[j * 2] = 1.0f / a_x * ((a_x * b_x) / (a_x + b_x));
        w[j * 2 + 1] = 1.0f / b_x * ((a_x * b_x) / (a_x + b_x));
    }

    // Se recorre cada celda (i, j) y cada pixel (ii, jj) dentro de ella. Cada pixel suma
    // la magnitud de su gradiente en dos canales de la celda: el de su orientacion sin
    // signo (a0, canales 0 a 8) y el de su orientacion con signo (a1, canales 9 a 26).
    // Ademas reparte ese voto con las celdas vecinas (la de arriba o abajo, la de la
    // izquierda o derecha, y la diagonal), segun los pesos calculados arriba.
    for (int i = 0; i < sizeY; i++)
    {
        for (int j = 0; j < sizeX; j++)
        {
            for (int ii = 0; ii < k; ii++)
            {
                for (int jj = 0; jj < k; jj++)
                {
                    // Los pixeles del borde de la imagen no tienen gradiente: se saltan.
                    if (!(i * k + ii > 0 && i * k + ii < height - 1 &&
                          j * k + jj > 0 && j * k + jj < width - 1))
                        continue;

                    const int d = (k * i + ii) * width + (j * k + jj); // indice del pixel
                    const int a0 = alfa[d * 2];
                    const int a1 = alfa[d * 2 + 1] + NUM_SECTOR;
                    const int ni = i + nearest[ii]; // fila de la celda vecina
                    const int nj = j + nearest[jj]; // columna de la celda vecina

                    // Voto en la celda propia:
                    map->map[i * stringSize + j * p + a0] += r[d] * w[ii * 2] * w[jj * 2];
                    map->map[i * stringSize + j * p + a1] += r[d] * w[ii * 2] * w[jj * 2];

                    // Voto en la vecina de arriba o de abajo (si existe):
                    if (ni >= 0 && ni <= sizeY - 1)
                    {
                        map->map[ni * stringSize + j * p + a0] += r[d] * w[ii * 2 + 1] * w[jj * 2];
                        map->map[ni * stringSize + j * p + a1] += r[d] * w[ii * 2 + 1] * w[jj * 2];
                    }
                    // Voto en la vecina de la izquierda o de la derecha (si existe):
                    if (nj >= 0 && nj <= sizeX - 1)
                    {
                        map->map[i * stringSize + nj * p + a0] += r[d] * w[ii * 2] * w[jj * 2 + 1];
                        map->map[i * stringSize + nj * p + a1] += r[d] * w[ii * 2] * w[jj * 2 + 1];
                    }
                    // Voto en la vecina en diagonal (si existe):
                    if (ni >= 0 && ni <= sizeY - 1 && nj >= 0 && nj <= sizeX - 1)
                    {
                        map->map[ni * stringSize + nj * p + a0] += r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
                        map->map[ni * stringSize + nj * p + a1] += r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
                    }
                }
            }
        }
    }

    free(w);
    free(nearest);
    free(r);
    free(alfa);

    return map;
}
//---------------------------------------------------------------------------
// Normaliza cada celda respecto de los cuatro bloques de 2x2 celdas que la rodean, y luego
// trunca. Descarta el borde de una celda, asi que el mapa se achica en 2 en cada dimension.
//
// Por que normalizar? Un mismo objeto con mas o menos luz produce gradientes mas o menos
// fuertes. Dividiendo por la "energia" de la zona, lo que queda describe la forma de los
// bordes y no su intensidad.
//
// Cada celda pertenece a cuatro bloques de 2x2 (el que se forma con sus vecinas hacia
// abajo y la derecha, hacia arriba y la derecha, hacia abajo y la izquierda, y hacia
// arriba y la izquierda). Se normaliza la celda por separado con cada uno de ellos, asi
// que sus 27 canales se convierten en 4 x 27 = 108.
void normalizeAndTruncate(FeatureMap *map, const float alfa)
{
    int sizeX = map->sizeX;
    int sizeY = map->sizeY;

    const int p = NUM_SECTOR;          // 9 canales sin signo
    const int xp = NUM_SECTOR * 3;     // 27 canales de entrada por celda
    const int pp = NUM_SECTOR * 12;    // 108 canales de salida por celda

    // Energia de cada celda: suma de los cuadrados de sus 9 canales sin signo.
    float *partOfNorm = (float *)malloc(sizeof(float) * (sizeX * sizeY));
    for (int i = 0; i < sizeX * sizeY; i++)
    {
        float valOfNorm = 0.0f;
        const int pos = i * map->numFeatures;
        for (int j = 0; j < p; j++)
            valOfNorm += map->map[pos + j] * map->map[pos + j];
        partOfNorm[i] = valOfNorm;
    }

    // El mapa de salida no tiene la primera ni la ultima fila y columna de celdas.
    sizeX -= 2;
    sizeY -= 2;

    float *newData = (float *)malloc(sizeof(float) * (sizeX * sizeY * pp));

    for (int i = 1; i <= sizeY; i++)
    {
        for (int j = 1; j <= sizeX; j++)
        {
            // pos1: celda (i, j) en el mapa de entrada (que tiene 2 celdas mas de ancho).
            // pos2: la misma celda en el mapa de salida (corrida en 1 por el borde quitado).
            const int pos1 = i * (sizeX + 2) * xp + j * xp;
            const int pos2 = (i - 1) * sizeX * pp + (j - 1) * pp;

            // Los cuatro bloques de 2x2 a los que pertenece esta celda: SE, NE, SW, NW
            // (sureste, noreste, suroeste y noroeste). dy y dx dicen hacia donde esta la
            // vecina de cada bloque. offLow y offHigh dicen donde se guarda, en la salida,
            // la parte sin signo (9 canales) y la parte con signo (18 canales) normalizada
            // con cada bloque.
            const int dy[4] = {+1, -1, +1, -1};
            const int dx[4] = {+1, +1, -1, -1};
            const int offLow[4] = {0, p, p * 2, p * 3};      // parte insensible al contraste
            const int offHigh[4] = {p * 4, p * 6, p * 8, p * 10}; // parte sensible al contraste

            for (int b = 0; b < 4; b++)
            {
                // Norma del bloque b: raiz de la energia de sus 4 celdas. FLT_EPSILON (un
                // numero diminuto) evita dividir por cero en zonas completamente lisas.
                const float valOfNorm = sqrtf(
                    partOfNorm[(i) * (sizeX + 2) + (j)] +
                    partOfNorm[(i) * (sizeX + 2) + (j + dx[b])] +
                    partOfNorm[(i + dy[b]) * (sizeX + 2) + (j)] +
                    partOfNorm[(i + dy[b]) * (sizeX + 2) + (j + dx[b])]) + FLT_EPSILON;

                for (int ii = 0; ii < p; ii++)
                    newData[pos2 + offLow[b] + ii] = map->map[pos1 + ii] / valOfNorm;
                for (int ii = 0; ii < 2 * p; ii++)
                    newData[pos2 + offHigh[b] + ii] = map->map[pos1 + p + ii] / valOfNorm;
            }
        }
    }

    // Truncamiento: ningun valor puede superar alfa (0.2).
    for (int i = 0; i < sizeX * sizeY * pp; i++)
        if (newData[i] > alfa)
            newData[i] = alfa;

    map->numFeatures = pp;
    map->sizeX = sizeX;
    map->sizeY = sizeY;

    free(map->map);
    free(partOfNorm);
    map->map = newData;
}
//---------------------------------------------------------------------------
// Reduce el mapa de 108 canales a 31, usando la proyeccion analitica del articulo
// original (sumas sobre bloques y sobre orientaciones) en vez de un PCA de verdad.
//
// Los 31 canales finales se arman asi:
//  - 18 canales: para cada orientacion con signo, la suma de sus 4 versiones (una por
//    bloque de normalizacion).
//  - 9 canales: lo mismo, para cada orientacion sin signo.
//  - 4 canales de "textura": para cada bloque de normalizacion, la suma de las 18
//    orientaciones con signo. Dicen cuanto borde hay en total, sin importar su direccion.
// nx y ny son factores de escala para que cada grupo quede de un tamano comparable.
void pcaFeatureMaps(FeatureMap *map)
{
    const int sizeX = map->sizeX;
    const int sizeY = map->sizeY;
    const int p = map->numFeatures;        // 108 canales de entrada
    const int pp = NUM_SECTOR * 3 + 4;     // 31 canales de salida
    const int yp = 4;                      // bloques de normalizacion
    const int xp = NUM_SECTOR;             // 9 orientaciones

    const float nx = 1.0f / sqrtf((float)(xp * 2));
    const float ny = 1.0f / sqrtf((float)(yp));

    float *newData = (float *)malloc(sizeof(float) * (sizeX * sizeY * pp));

    for (int i = 0; i < sizeY; i++)
    {
        for (int j = 0; j < sizeX; j++)
        {
            const int pos1 = (i * sizeX + j) * p;   // celda en la entrada
            const int pos2 = (i * sizeX + j) * pp;  // celda en la salida
            int k = 0;                              // proximo canal de salida a escribir

            // Recordar la organizacion de la entrada: los primeros 36 canales (4 bloques
            // x 9) son la parte sin signo, y desde el canal 36 (yp * xp) vienen los 72 de
            // la parte con signo (4 bloques x 18).

            // 18 orientaciones con signo, sumadas sobre los 4 bloques:
            for (int jj = 0; jj < xp * 2; jj++)
            {
                float val = 0;
                for (int ii = 0; ii < yp; ii++)
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                newData[pos2 + k++] = val * ny;
            }
            // 9 orientaciones sin signo, sumadas sobre los 4 bloques:
            for (int jj = 0; jj < xp; jj++)
            {
                float val = 0;
                for (int ii = 0; ii < yp; ii++)
                    val += map->map[pos1 + ii * xp + jj];
                newData[pos2 + k++] = val * ny;
            }
            // 4 bloques, cada uno sumado sobre sus 18 orientaciones con signo:
            for (int ii = 0; ii < yp; ii++)
            {
                float val = 0;
                for (int jj = 0; jj < 2 * xp; jj++)
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                newData[pos2 + k++] = val * nx;
            }
        }
    }

    map->numFeatures = pp;
    free(map->map);
    map->map = newData;
}

} // namespace

// Funcion publica: aplica las tres etapas y devuelve el resultado en el formato que
// necesita el KCF.
cv::Mat extractHOG(const cv::Mat &image, int cellSize, cv::Size &cells)
{
    FeatureMap *map = getFeatureMaps(image, cellSize);
    normalizeAndTruncate(map, TRUNCATE_ALFA);
    pcaFeatureMaps(map);

    cells = cv::Size(map->sizeX, map->sizeY);

    // map->map esta organizado como (celdas x canales); se transpone a (canales x
    // celdas) para que cada fila sea un canal de caracteristica. Primero se "envuelve"
    // la memoria en una cv::Mat sin copiarla, y t() (transponer) si hace una copia
    // completa, asi que el mapa se puede liberar justo despues.
    const cv::Mat wrapped(map->sizeX * map->sizeY, map->numFeatures, CV_32F, map->map);
    cv::Mat features = wrapped.t();

    freeFeatureMap(map);
    return features;
}
