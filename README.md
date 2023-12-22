# DIOS_TE_AMA

## FOLLOW LINE

# Índice

1. [Descripción del Proyecto](#descripción-del-proyecto)
2. [Video](#video)
3. [Componentes Necesarios](#componentes-necesarios)
   - [Ultrasonido](#ultrasonido)
   - [Sensor Infra-rojo](#sensor-infra-rojo)
   - [Motores](#motores)
   - [LED](#led)
4. [ESP32](#esp32)
   - [Arduino-IDE y librerías](#arduino-ide-y-librerías)
   - [Comprobar la conexión WiFi](#comprobar-la-conexión-wifi)
5. [Comunicación Serie](#comunicación-serie)
6. [Comunicación IoT](#comunicación-iot)
   - [MQTT](#mqtt)
7. [Opcionales](#opcionales)



### Descripción del Proyecto

Este proyecto utiliza un robot equipado con sensores infrarrojos para seguir una línea negra trazada en el suelo. El robot incluye un modelo ESP32 CAM, que nos permite comunicarnos a través de cualquier red WiFi. Una vez comunicados con la wifi podemos enviar los datos de los sensores a un servidor MQTT, que nos permitirá visualizar los datos en tiempo real.

### Video

https://github.com/jfisher2021/DIOS_TE_AMA/assets/113594937/28c11347-1fcc-430a-af50-1df26088617d

### Componentes Necesarios

Para este proyecto, necesitarás los siguientes componentes:

  - ELEGOO Smart Robot Car V4.0 with Camera 🏎️
  - Placa ESP32 🛜
  - Batería o fuente de alimentación para el coche y la ESP32 🔋
  - Cables para conectar los componentes (uno usb-C para la ESP32) 🔌

## PRIMEROS PASOS CON EL ROBOT

### SEGUIR LINEA 

Lo primero que hicimos fue probar y entender el codigo de prueba proporcionado en la wiki de la asignatura. Estuvimos tanteando con el sensor infrarrojo y viendo como funcionaba. Al ser analogico nos daba valores de entre 0 y 1023, siendo 0 cuando  más claro y 1023 cuando más oscuro.

```c++
#define PIN_ITR20001-LEFT   A2
#define PIN_ITR20001-MIDDLE A1
#define PIN_ITR20001-RIGHT  A0
```

###### PID 

Despues de bastantes codigos de prueba, nos dimos cuenta de que la mejor opción para que siguiera la linea sería emplear un PID. La idea principal era calcular el error entre el valor Left y el valor Rigth de los sensores. Una vez obtenido ese error ya podemos ir variando la velocidad de giro para seguir la linea. Entendido esto, nos pusimos a programar. 
En nuestro caso usamos un PD ya que la I no afecta mucho al no llegar al estado permanente el suficiente tiempo como para variar el giro.

```c++
    error = abs(left_val - right_val);
    derivativo = error - error_anterior;
    velocidad = KP * error + KD * derivativo;
    error_anterior = error;
```
Para hallar los valores de Kp y Kd y no morir en el intento, razonamos mas o menos los valores que nos debería de dar. El error oscilaba entre 700 y 800, por lo tanto los valores tendrian que estar entre 0.5 y 0.1 aprox ya que la velocidad optima esta entre 100 y 255.

Una vez razonado esto, nos pusimos a probar y a ajustar los valores hasta que el robot siguiera la linea. Despues de casi 1 hora de probar valores dimos con unos valores que funcionaban bastante bien. 

```c++
#define KP 0.245
#define KD 0.32
```

##### PROBLEMAS
Ya tenemos la velocidad de giro para seguir la linea, PERO, hay un GRAN PROBLEMA, cuando el robot pierde la linea gira muy poco o peor, se queda quieto. Pero ¿Por qué?
Esto se debe a que cuando se sale de la linea tanto el valor Right o Left nos da valores de entre 0 y 50 aprox por lo tanto el error es muy pequeño y la velocidad de giro tambien. Nosotros somos (futuros) ingenieros, a los ingenieros nos gusta solucionar problemas, por lo tanto, nos pusimos a pensar como solucionar este problema. 

##### SOLUCIONES

###### recovery()

La solucion fue muy secilla al igual que ingeniosa (o eso creemos). Lo que hicimos fue crear una funcion que se encargara de recuperar el robot cuando se saliera de la linea. Esta funcion lo que hace es girar el robot hasta que vuelva a encontrar la linea. Para ello lo que hicimos fue que cuando los 3 sensores dieran valores por debajo de un umbral se llamara a la funcion recovery(). 

- PRIMER INTENTO

Nos creamos una variable global para saber por que lado se salio de la linea y asi poder girar en el sentido contrario. (Si se sale por la derecha girar a la izquierda y viceversa). Parece que esta solucion es super buena y brillante pero cuando la probamos nos dimos cuenta de que no era tan buena como parecia. Nos dimos cuenta de que el robot no se perdia por girar mucho y por lo tanto girar en el sentido inverso como la loica podria indicar no era la mejor opcion. El robot se perdia por girar poco y por lo tanto girar en el sentido inverso no era la solucion.

- SEGUNDO INTENTO Y DEFINITIVO

Una vez entendido lo que pasaba simplemente cambiamos el sentido de giro y listo. Para ello ponemos una velocidad mayor que la de el PD para que gire mas rápido y encuentre la linea. 

```c++
void recovery() {

  if (anterior == 1) {
    motorControl(false, 0, true, i_show_speed_angular * 1.5);
  } else if (anterior == 2) {
    motorControl(true, i_show_speed_angular * 1.5, false, 0);
  }
}
```

### Esp32

Respecto a la ESP32, se ha implementado de tal manera que se comunicará con el Arduino mediante la 'SerialCommunication'. Se han proporcionado muchos códgios al respecto en la wiki de la asignatura, de los cuales hemos obtenido el 'esqueleto' de nuestra implementación. 
A la hora de comunicarse entre la ESP32 y Arduino Uno en nuestro código, en la función setup() nos encontramos con la inicialización de la comunicación entre sí:
```c++
Serial2.begin(9600,SERIAL_8N1, RXD2, TXD2);
```
Para que más tarde de forma periódica en la función loop(), se pueda definir que una vez que empiece la comunicación, se almacene la información en un buffer y en base a esta información almacenada, se puedan efectuar unas acciones u otras:
```c++
  if (Serial2.available()) {
    char c = Serial2.read();
    receive_buff += c;
  }
```
Para rellenar el buffer, nuestra manera de enviar mensajes y de recibirlos es introduciendo los mensajes entre corchetes.


Para el envío de mensajes y para la publicación de estos, se implementará WIFI y MQTT. También se han utilizado los códigos que encontramos en la wiki de la asignatura, por lo que para la wifi empleamos la librería <WiFi.h> y para el MQTT empleamos la librería de Adafruit.

Nuestro código en la ESP32 tiene la siguiente estructura:
Una vez conectado a la WIFI y a MQTT de forma correcta, el arduino mandará mensajes a la ESP, la cual leerá de forma iterada y según el mensaje recibido, actuará de una manera u otra: 
```c++
if(receive_buff == "{SP}"){
        start_lap_message();
}
```
Este snippet de código hace referencia a que dependiendo del mensaje que le llegue a la ESP, el código entrará en uno de los condicionales y realizará la acción que se le indique. Se utilizará una función distinta para cada tipo de mensaje. En el caso de recibir "{SP}", como en el snippet anterior, se llamrá a la función start_lap_message() que creará el mensaje de inicio de vuelta y llamará a la función 'publish()' para publicarlo:
```c++
void start_lap_message() {
  char message[256];
  sprintf(message, "{\n\t\"team_name\": \"DIOS_TE_AMA\",\n\t\"id\":\"%s\",\n\t\"action\":\"START_LAP\"\n}", id_equipo);
  publishData(message);
}
```

----------------------------------------------------------------

Además se han implementado Arduino Threads para simplificar algunas tareas en esta práctica. Para ello implementamos las librerías de <Thread.h>
y <ThreadController.h>. Definimos los threads de la siguiente manera: 
```c++
Thread temporary_check_thread = Thread();
Thread distance_thread = Thread();
ThreadController controller = ThreadController();
```
Se implementan 2 Threads y se utiliza un controlador. El Thread 'temporary_check_thread' es el que nos servirá de ayuda para el mensaje de 'PING', ya que el thread nos mide el tiempo cada 4 segundos, como se requiere en la práctica. El otro thread 'distance_thread' se emplea para controlar la distancia mediante el sensor de ultrasonidos de forma continuada. En la función setup() se inicializan de la siguiente manera: 
```c++
distance_thread.enabled = true;
 distance_thread.setInterval(100);
 distance_thread.onRun(distance_ping);
 controller.add(&distance_thread);

 temporary_check_thread.enabled = true;
 temporary_check_thread.setInterval(4000);
 temporary_check_thread.onRun(ping_time_check);
 controller.add(&temporary_check_thread);
```
### Opcionales

Además de los mensajes de Inicio de vuelta, Final de vuelta, Línea perdida y Obstaculo Detectado. Se han implementado los mensajes opcionales. Debido a que no todos ellos nos funcionaban y que el código no era lo suficientemente robusto, el que se ha utilizado el dia del examen ha sido una versión anterior donde nos asergurábamos de que el funcionamiento obligatorio, además de algunos mensajes opcionales, funcionaran sin problemas. 

##### Funcionamiento de search() (NO IMPLEMENTADO EN EL EXAMEN)
Como hemos implementado en el LostLine y FoundLine, creamos otro booleano que se activa cuando ha perdido la linea y se desactiva una vez llamado para que solo lo imprima una vez. 

```c++

// codigo en el arduino
void recovery()
{
  if (recovery_bool)
  {
    Serial.print("{RE}");
    recovery_bool = false;
  }
  // resto de codigo
}

//en el loop 
if (lost_line)
        {
            Serial.print("{LT}");
            lost_line = false;
            found_line = true;
            recovery_bool = true;
        }
```

```c++

// codigo en la esp32
void search()
{
  char message[256];
  sprintf(message, "{\n\t\"team_name\":\"DIOS_TE_AMA\",\n\t\"id\":\"%s\",\n\t\"action\":\"SEARCHING_LINE\"\n}", id_equipo);
  publishData(message);
}

// en el loop
if (receive_buff == "{SE}")
  {
    search();
  }

```

##### porcentaje de la linea (NO IMPLEMENTADO EN EL EXAMEN)

Esta es una solucion que se me ocurrio para sacar el porcentaje pero debido al problema del envio de mensajes, qeu aunque el codigo estaba bien a veces no se enviaban los mensajes, no lo implementamos en el examen. 

```c++
unsigned long timeOnLine = 0;

void loop()
{
    unsigned long loopStartTime = millis();
    
    if (right_val >= threshold || left_val >= threshold || mid_val >= threshold) {
        timeOnLine += millis() - loopStartTime;
    }

    // resto del codigo

    if (distance < 15) { // STOP
        // resto del codigo
        // Calcular el porcentaje de la linea
        float percentageOnLine = (float)timeOnLine / final_time * 100;
        Serial.print("{LP}");
        Serial.print(percentageOnLine);
        stop_motors();
    }
}
```

En este codigo se van sumando los milisegundos que esta en la linea y cuando se detecta un obstaculo se calcula el porcentaje de tiempo que ha estado en la linea y se envia el mensaje. Cabe recalcar que no hemos podido probar este codigo ya que no nos funcionaba el envio de mensajes. Por lo tanto ha sido una solucion puramente teorica.




