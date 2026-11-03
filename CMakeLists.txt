# Torreta pan/tilt: modelo, estimacion y control (C), seguidores (KCF para el avion de juguete,
# CFAR para el laser), percepcion y dashboard, en un solo ejecutable: torreta (ver src/main.cpp).
#
# CMake no compila por si mismo: genera los archivos de compilacion (para Visual Studio,
# Make, Ninja, etc.) a partir de esta descripcion del proyecto. Se usa en dos pasos:
#
#   cmake -B build -DOpenCV_DIR=<...>  [-DPI4=ON]
#   cmake --build build --config Release
#
# El primer comando configura el proyecto en la carpeta build. OpenCV_DIR le dice donde
# esta instalado OpenCV; PI4=ON (opcional) optimiza para la Raspberry Pi 4. El segundo
# comando compila en modo Release (optimizado).
#
# El proyecto se organiza en varias bibliotecas estaticas (una por carpeta de src/), que
# al final se enlazan en el ejecutable torreta. Cada biblioteca declara de que otras
# depende, y CMake se encarga del orden y de pasar las rutas de los headers.

cmake_minimum_required(VERSION 3.16)
project(torreta LANGUAGES C CXX)

# Si no se eligio un tipo de compilacion, se usa Release (optimizado). La segunda
# condicion excluye a los generadores con varias configuraciones (como Visual Studio),
# donde el tipo se elige al compilar con --config.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Opciones que se pueden activar o desactivar al configurar, con -DNOMBRE=ON u OFF.
option(PI4 "Optimizar para Raspberry Pi 4B (Cortex-A72, OS de 64 bits)" OFF)

# Dependencias externas: OpenCV es obligatoria, y Threads es la biblioteca de hilos del
# sistema (la usa el hilo del adquisidor del perfil objeto).
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# El codigo en C usa el estandar C11 y el de C++ el estandar C++14.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release ya optimiza (/O2 en MSVC, -O3 en GCC). -ffast-math no: el Kalman y el MPC usan
# NaN para "sin dato".
# (-ffast-math permite al compilador suponer que nunca aparecen NaN ni infinitos, y
# entonces comprobaciones como isnan o isfinite podrian dejar de funcionar.)
# Para la Pi 4 se le pide al compilador que genere instrucciones especificas de su
# procesador (Cortex-A72); no aplica a MSVC, que es el compilador de Microsoft.
if(PI4 AND NOT MSVC)
    add_compile_options(-mcpu=cortex-a72)
endif()

# --- src/modelo: geometria de la camara y de la torreta (C) ----------------------------------
add_library(modelo STATIC src/modelo/model.c src/modelo/geometria.c src/modelo/geometria_gen.c)
# PUBLIC significa que quien use esta biblioteca tambien vera sus headers.
target_include_directories(modelo PUBLIC src/modelo)
# En Linux, las funciones matematicas (sin, cos, exp...) estan en una biblioteca aparte,
# "m", que hay que enlazar explicitamente. En Windows con MSVC no hace falta.
if(NOT MSVC)
    target_link_libraries(modelo PUBLIC m)
endif()

# --- src/estimacion: filtros (C) -------------------------------------------------------------
add_library(estimacion STATIC src/estimacion/singer_gen.c src/estimacion/ekf_pieza.c)
target_include_directories(estimacion PUBLIC src/estimacion)
target_link_libraries(estimacion PUBLIC modelo)

# --- src/firmware: nucleo portable del ejecutor de la ESP32 (C, sin dependencias) ----------
# Es a la vez el emulador del SIL y el codigo que se embarca (fase 7, sobre ESP-IDF).
add_library(firmware STATIC
    src/firmware/crc16.c
    src/firmware/mensajes.c
    src/firmware/trama.c
    src/firmware/ejecutor.c)
target_include_directories(firmware PUBLIC src/firmware)
if(NOT MSVC)
    target_link_libraries(firmware PUBLIC m)
endif()

# --- src/comms: comunicacion del lado de la Pi (C) -------------------------------------------
add_library(comms STATIC
    src/comms/sincronizacion.c
    src/comms/gilbert_elliott.c
    src/comms/quevedo.c
    src/comms/predictor.c)
target_include_directories(comms PUBLIC src/comms)
target_link_libraries(comms PUBLIC firmware)

# --- src/hal: interfaces de hardware, reloj del computador y ejecutor inerte (C) -----------
# El ejecutor inerte reemplaza a la ESP32 mientras no este el enlace serie (fase 7).
add_library(hal STATIC src/hal/reloj_pc.c src/hal/ejecutor_inerte.c src/hal/enlace_serie.c)
target_include_directories(hal PUBLIC src/hal)
target_link_libraries(hal PUBLIC firmware)

# --- src/control: ctl_programado, el controlador del ciclo de la Pi (C) -------------------
add_library(ctl STATIC src/control/ctl_programado.c)
target_include_directories(ctl PUBLIC src/control)
target_link_libraries(ctl PUBLIC estimacion comms hal params)

# --- src/dashboard: sockets portables, HTTP minimo y servidor del dashboard (C) ------------
add_library(dashboard STATIC src/dashboard/red.c src/dashboard/http.c
    src/dashboard/servidor.c src/dashboard/datos_cuadro.c src/dashboard/salud.c
    src/dashboard/ritmo.c)
target_include_directories(dashboard PUBLIC src/dashboard)
target_link_libraries(dashboard PUBLIC hal params)
if(WIN32)
    target_link_libraries(dashboard PUBLIC ws2_32 winmm)  # winmm: timeBeginPeriod (red.c)
endif()

# --- src/dashboard: triple buffer y publicador (C++, toca cv::Mat e hilos) ------------------
add_library(dashboard_cpp STATIC src/dashboard/publicador.cpp)
target_include_directories(dashboard_cpp PUBLIC src/dashboard ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dashboard_cpp PUBLIC dashboard Threads::Threads ${OpenCV_LIBS})

# --- src/params: valores por defecto y lectura de la configuracion (C) ----------------------
add_library(params STATIC src/params/params.c)
target_include_directories(params PUBLIC src/params)
target_link_libraries(params PUBLIC ini estimacion)

# --- src/params: lector INI y parametros centralizados --------------------------------------
add_library(ini STATIC src/params/ini.c)
target_include_directories(ini PUBLIC src/params)

# --- src/seguidores: KCF (objeto) y SpotTracker/CFAR (laser) ---------------------------------
add_library(seguidores STATIC
    src/seguidores/kcftracker.cpp
    src/seguidores/fhog.cpp
    src/seguidores/spottracker.cpp)
target_include_directories(seguidores PUBLIC src/seguidores ${OpenCV_INCLUDE_DIRS})
target_link_libraries(seguidores PUBLIC ${OpenCV_LIBS})

# --- src/percepcion: perfiles laser y objeto --------------------------------------------------
# Logica de la barrida global con retador, en C y sin OpenCV (se prueba sola).
add_library(retador STATIC src/percepcion/retador.c)
target_include_directories(retador PUBLIC src/percepcion)

add_library(percepcion STATIC
    src/percepcion/percepcion.cpp
    src/percepcion/perfil_laser.cpp
    src/percepcion/perfil_objeto.cpp
    src/percepcion/perfil_general.cpp
    src/percepcion/adquisidor_varianza.cpp
    src/percepcion/adquisidor_rojo.cpp)
target_include_directories(percepcion PUBLIC src/percepcion)
target_link_libraries(percepcion PUBLIC params seguidores retador Threads::Threads ${OpenCV_LIBS})

# --- src/app: funciones comunes a los modos (percentil, msDesde) -------------------------------
add_library(app_comun STATIC src/app/comun.cpp)
target_include_directories(app_comun PUBLIC src/app)
target_link_libraries(app_comun PUBLIC percepcion params)

# --- src/vivo: fuentes de cuadros y bucle del modo vivo (C++, toca cv::Mat) ------------------
# La pagina se embebe como pagina_embebida.c, generado en el directorio de build.
set(PAGINA_EMBEBIDA ${CMAKE_BINARY_DIR}/pagina_embebida.c)
add_custom_command(OUTPUT ${PAGINA_EMBEBIDA}
    COMMAND ${CMAKE_COMMAND} -DENTRADA=${CMAKE_SOURCE_DIR}/src/dashboard/pagina.html
            -DSALIDA=${PAGINA_EMBEBIDA} -P ${CMAKE_SOURCE_DIR}/cmake/embeber.cmake
    DEPENDS src/dashboard/pagina.html cmake/embeber.cmake
    COMMENT "Embebiendo pagina.html")
add_library(vivo STATIC
    ${PAGINA_EMBEBIDA}
    src/vivo/fuente.cpp
    src/vivo/cadena.cpp
    src/vivo/correr_vivo.cpp
    src/vivo/calibracion_camara.cpp)
target_include_directories(vivo PUBLIC src/vivo)
target_link_libraries(vivo PUBLIC app_comun dashboard_cpp ctl hal percepcion ${OpenCV_LIBS})

# --- torreta: el ejecutable unico -----------------------------------------------------------
add_executable(torreta
    src/main.cpp
    src/app/config.cpp
    src/app/modo_vivo.cpp)
target_include_directories(torreta PRIVATE src/app)
target_link_libraries(torreta PRIVATE app_comun vivo)
# Raiz del proyecto: ahi se busca config/ si no estan en el directorio actual.
# Se le pasa al codigo como la macro RUTA_PROYECTO (ver rutaProyecto en app/config.cpp).
target_compile_definitions(torreta PRIVATE RUTA_PROYECTO="${CMAKE_CURRENT_SOURCE_DIR}")

# --- NMPC: acados (opcional, ver cmake/acados.cmake) -----------------------------------------
include(cmake/acados.cmake)

# NMPC del informe (codegen/nmpc.py -> acados): el unico planificador, asi que acados es obligatorio.
if(NOT ACADOS_DIR)
    message(FATAL_ERROR "Falta -DACADOS_DIR=<acados>: el NMPC es el planificador del sistema")
endif()
if(ACADOS_DIR)
    acados_generar(nmpc_solver ${PROJECT_SOURCE_DIR}/codegen/nmpc.py nmpc)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/codegen/geometria.py
        ${PROJECT_SOURCE_DIR}/config/torreta.ini)
    add_library(nmpc STATIC src/control/nmpc.c)
    target_include_directories(nmpc PUBLIC src/control)
    target_link_libraries(nmpc PUBLIC nmpc_solver params)
    # El nivel 3 del modo vivo (src/vivo/cadena.hpp) corre el NMPC en sombra. PUBLIC: la
    # disposicion de Cadena depende de CON_NMPC.
    target_link_libraries(vivo PUBLIC nmpc)
    target_compile_definitions(vivo PUBLIC CON_NMPC)
endif()

# En Windows, el ejecutable necesita las DLL de OpenCV. Se copian junto a torreta.exe al compilar, para no tener que agregar la
# carpeta bin de OpenCV al PATH.
if(WIN32 AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.21)
    add_custom_command(TARGET torreta POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:torreta> $<TARGET_FILE_DIR:torreta>
        COMMAND_EXPAND_LISTS)
endif()
