# Sistema de Control de Péndulo Dual

Descripción

Este proyecto implementa un sistema distribuido para el control de un péndulo dual utilizando un ESP32 como controlador principal y una aplicación de visión por computadora desarrollada en C++ con OpenCV. Ambos módulos se comunican mediante el protocolo UDP para transmitir en tiempo real el ángulo detectado por el sistema de visión hacia el controlador.

Componentes del proyecto

ESP32_Controller
- Control PID.
- Comunicación Wi-Fi mediante UDP.
- Lectura del sensor ADC.
- Generación de PWM para motores brushless.
- Modo seguro (Watchdog).
- Telemetría mediante FreeRTOS.

Vision_PC
- Captura de video desde una cámara IP.
- Detección de marcadores ArUco.
- Cálculo del ángulo de inclinación.
- Envío del Set Point al ESP32 mediante UDP.

Tecnologías utilizadas

- ESP32
- ESP-IDF
- PlatformIO
- FreeRTOS
- C++
- OpenCV
- UDP Sockets

Conclusión

Se desarrolló un sistema de control distribuido que integra una aplicación de visión por computadora con un controlador embebido basado en ESP32. La aplicación desarrollada en C++ permite detectar la inclinación de la estructura mediante marcadores ArUco y transmitir el ángulo calculado al controlador utilizando comunicación UDP. Por su parte, el ESP32 procesa esta información mediante un controlador PID ejecutado sobre FreeRTOS para generar las señales PWM necesarias para el accionamiento de los motores brushless.

La arquitectura implementada permitió separar el procesamiento de visión artificial del control en tiempo real, facilitando el desarrollo, la depuración y la escalabilidad del sistema. Además, la incorporación de mecanismos de seguridad, como el modo seguro ante pérdida de comunicación y la telemetría del sistema, incrementa la confiabilidad de la plataforma durante su operación.

En conjunto, el proyecto demuestra la integración de tecnologías de visión artificial, comunicaciones inalámbricas y sistemas embebidos para resolver un problema de control en tiempo real, proporcionando una base sólida para futuras mejoras y aplicaciones de mayor complejidad.
