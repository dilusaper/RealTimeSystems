#include <stdio.h>

/*
Leer temperatura cada 100ms
Leer porcentaje del potenciómetro cada 30ms, usar semáforos y colas
Usar umbrales (thresholds) para cambiar el color del LED RGB, usar un intervalo de 50ms **
Crear tarea que lea el UART y permita cambiar el threshold de cualquiera de los colores

1. tarea que lea temperatura cada  100 ms
2 Una tarea que lea % de potenciometro cada  30 ms
)debe estar impplementado con semaforos y colas)
3 una tarea que controle el color del led basado en  thresholds para color verde rojo y azul  (leer cola de temperatura, leer cola de potenciometro, leer cola de cambio de tresholds)
4 Una tarea que lea el UART y que nos permita cambiar el threshold de cualquiera de los colores
*/

void app_main(void)
{

}
