# Trayectoria-por-maquina-de-estados

## Descripción del proyecto

Este repositorio contiene la implementación de un controlador para un robot diferencial (Pioneer P3-DX) en CoppeliaSim, que emplea una Máquina de Estados Finitos (FSM) y un controlador PID para planear localmente la trayectoria hacia una esfera verde. El robot reacciona a esferas de distintos colores:

- **Verde**: avanza y sigue la esfera en tiempo real.
- **Naranja**: gira a la derecha hasta volver a ver la esfera verde.
- **Azul**: gira a la izquierda hasta volver a ver la esfera verde.
- **Amarilla**: pausa durante 5 segundos, mostrando un temporizador.
- **Ausencia de esfera**: se detiene.

La detección de color se realiza con un VisionSensor de CoppeliaSim y OpenCV.

---

## Requisitos

- **CoppeliaSim Edu** (versión 4.4 o superior) con el plugin ZeroMQ Remote API activo.
- **Python 3.8+** con los siguientes paquetes:
  - `numpy`
  - `opencv-python`
  - `pyzmq`

---

## Instalación

1. Clona o descarga este repositorio:
   ```bash
   git clone https://github.com/tu_usuario/Trayectoria-por-maquina-de-estados.git
   cd Trayectoria-por-maquina-de-estados
