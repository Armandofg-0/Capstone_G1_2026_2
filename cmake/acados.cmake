# acados (NMPC SQP-RTI con HPIPM y BLASFEO), opcional.
#
#   cmake -B build ... -DACADOS_DIR=C:/Users/fuent/acados
#                      [-DACADOS_PYTHON=C:/Users/fuent/acados/venv/Scripts/python.exe]
#
# ACADOS_DIR es la instalacion de acados (lib/ e include/). ACADOS_PYTHON es un Python con
# casadi y acados_template; por defecto, el venv dentro de ACADOS_DIR. Sin ACADOS_DIR se
# compila todo menos lo que depende del NMPC.
#
# acados_generar(<objetivo> <script.py> <nombre>) corre el script de codegen/ al configurar
# (y de nuevo cada vez que el script cambia) y arma una biblioteca estatica <objetivo> con el
# C generado en build/codegen/<nombre>.

set(ACADOS_DIR "" CACHE PATH "Instalacion de acados; vacio = sin NMPC")
set(ACADOS_PYTHON "" CACHE FILEPATH "Python con casadi y acados_template")

if(ACADOS_DIR)
    if(NOT ACADOS_PYTHON)
        set(ACADOS_PYTHON ${ACADOS_DIR}/venv/Scripts/python.exe)
    endif()
    add_library(acados_ext INTERFACE)
    target_include_directories(acados_ext INTERFACE
        ${ACADOS_DIR}/include ${ACADOS_DIR}/include/acados
        ${ACADOS_DIR}/include/blasfeo/include ${ACADOS_DIR}/include/hpipm/include)
    if(MSVC)
        target_link_libraries(acados_ext INTERFACE
            ${ACADOS_DIR}/lib/acados.lib ${ACADOS_DIR}/lib/hpipm.lib ${ACADOS_DIR}/lib/blasfeo.lib)
    else()
        # Si la instalacion solo trae bibliotecas compartidas (como la de la Pi), se usan las .so y
        # se guarda la ruta de busqueda en el ejecutable (-rpath), sin LD_LIBRARY_PATH.
        if(EXISTS ${ACADOS_DIR}/lib/libacados.a)
            target_link_libraries(acados_ext INTERFACE
                ${ACADOS_DIR}/lib/libacados.a ${ACADOS_DIR}/lib/libhpipm.a ${ACADOS_DIR}/lib/libblasfeo.a m)
        else()
            target_link_libraries(acados_ext INTERFACE
                ${ACADOS_DIR}/lib/libacados.so ${ACADOS_DIR}/lib/libhpipm.so ${ACADOS_DIR}/lib/libblasfeo.so m)
            target_link_options(acados_ext INTERFACE -Wl,--disable-new-dtags -Wl,-rpath,${ACADOS_DIR}/lib)
        endif()
    endif()
endif()

function(acados_generar objetivo script nombre)
    set(salida ${CMAKE_BINARY_DIR}/codegen/${nombre})
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${script})
    if(NOT EXISTS ${salida}/acados_solver_${nombre}.c OR ${script} IS_NEWER_THAN ${salida}/acados_solver_${nombre}.c)
        file(REMOVE_RECURSE ${salida})
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E env ACADOS_SOURCE_DIR=${ACADOS_DIR}
                    ${ACADOS_PYTHON} -I ${script} ${salida}
            RESULT_VARIABLE rc OUTPUT_QUIET ERROR_VARIABLE err)
        if(NOT rc EQUAL 0)
            message(FATAL_ERROR "codegen de ${nombre} fallo:\n${err}")
        endif()
    endif()
    file(GLOB_RECURSE fuentes ${salida}/*.c)
    list(FILTER fuentes EXCLUDE REGEX "/main_[^/]*\\.c$")
    add_library(${objetivo} STATIC ${fuentes})
    target_include_directories(${objetivo} PUBLIC ${salida})
    target_link_libraries(${objetivo} PUBLIC acados_ext)
endfunction()
