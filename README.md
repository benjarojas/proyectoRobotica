# proyectoRobotica
Proyecto Final ICI4150 - Robótica y Sistemas Autónomos

## Para ejecutar el código en el arduino:

1. Para encender el robot simplemente conectar el cable DC que está bajo el arduino
2. Abrir el archivo main.ino dentro de la carpeta main
3. Desconectar los cables de RX y TX del módulo bluetooth
4. Subir el archivo al arduino (Arduino Nano con ATMega328p (old bootloader))

## Para ejecutar la API que lee los datos mediante comunicación serial

1. Conectar el módulo bluetooth del robot al PC (Nombre: HeatBot, clave: 1234)
2. Verificar el puerto serial del módulo bluetooth y actualizarlo en el archivo api.py
3. Ejecutar api.py (se requiere Flask) y verificar los datos en los endpoints correspondientes (/gas, /temperature, /air_quality)