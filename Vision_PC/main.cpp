/**
 * Este programa captura video en tiempo real desde una cámara IP, 
 * procesa los frames para detectar marcadores fiduciales (ArUco), 
 * calcula el ángulo de inclinación de la estructura física y 
 * transmite el Set Point resultante de manera inalámbrica a un 
 *  microcontrolador ESP32  mediante Sockets UDP.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect.hpp> 
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <winsock2.h>
#include <ws2tcpip.h>

#define PI 3.14159265358979323846


// CLASES Y OBJETOS DE COMUNICACIÓN DE RED                                   

class UdpSender
{
private:
    SOCKET sock;              // Descriptor del socket 
    sockaddr_in destAddr;     // Estructura para almacenar la IP y el puerto destino

public:
    UdpSender(const std::string& ip, int port)
    {
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData); // Inicialización de librerías de red en Windows

        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // Protocolo UDP 

        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &destAddr.sin_addr);
    }

    void writeLine(const std::string& data)
    {
        if (sock != INVALID_SOCKET) {
            std::string msg = data;
            sendto(sock, msg.c_str(), static_cast<int>(msg.length()), 0, (sockaddr*)&destAddr, sizeof(destAddr));
        }
    }

    ~UdpSender()
    {
        if (sock != INVALID_SOCKET) {
            closesocket(sock);
        }
        WSACleanup();
    }
};

// BUCLE PRINCIPAL DE VISIÓN Y CÁLCULO DE ÁNGULO        

int main()
{
    // CONFIGURACIÓN DE LA CÁMARA IP 
    std::string url = "http://172.20.10.4:8080/video";
    cv::VideoCapture cap(url);
    
    // Reducir el tamaño del buffer obliga a descartar frames viejos, 
    // garantizando la latencia más baja posible 
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // INICIALIZACIÓN DEL ENLACE UDP 
    UdpSender udp("172.20.10.5", 3333);

    if (!cap.isOpened())
    {
        std::cout << "No se pudo abrir el stream de video\n";
        return -1;
    }

    // VARIABLES DE ESTADO Y CONTROL TEMPORAL 
    cv::Mat frame;
    double currentAngle = 0.0;
    double lastSentAngle = 0.0;

    int64 lastUpdateTime = cv::getTickCount();
    double updateInterval = 0.00001; // Forzar actualización lo más rápido posible
    int contador = 0;

    // VARIABLES DEL FILTRO PASA-BAJAS 
    // El filtro redcue el ruido introducido por la compresión de video de la cámara
    double smoothedAngle = 0.0;
    bool firstFrame = true;
    double alpha = 0.15; // Peso de la lectura actual vs histórico 

    // CONFIGURACIÓN DEL DETECTOR ARUCO 
    // Se utiliza el diccionario DICT_4X4_50 que coincide físicamente con los códigos impresos e instalados en los extremos del balancín maestro.
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters parameters = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, parameters);

    // BUCLE DE PROCESAMIENTO INFINITO 
    while (true)
    {
        int64 t0 = cv::getTickCount(); // Estampa de tiempo para perfilamiento de rendimiento
        cap >> frame;

        if (frame.empty())
            break;

        // Redimensionar reduce drásticamente la carga de CPU y mejora los FPS, manteniendo suficiente resolución para una detección sub-pixel precisa
        cv::resize(frame, frame, cv::Size(640, 480));

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

        // Detección de marcadores en el frame actual
        detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);

        // CONDICIÓN CRÍTICA: Deben verse ambos rotores (Izquierdo y Derecho)
        if (markerIds.size() >= 2)
        {
            // Contornos verdes en los marcadores hallados
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            // Calcular el centro geométrico del primer marcador (promedio de sus 4 esquinas)
            cv::Point2f c1(0.0f, 0.0f);
            for(int i = 0; i < 4; i++) c1 += markerCorners[0][i];
            c1 *= 0.25f;

            // Calcular el centro geométrico del segundo marcador
            cv::Point2f c2(0.0f, 0.0f);
            for(int i = 0; i < 4; i++) c2 += markerCorners[1][i];
            c2 *= 0.25f;

            // Identificar posicionalmente cuál es el motor izquierdo y el derecho
            cv::Point2f leftMarker, rightMarker;
            if (c1.x < c2.x) {
                leftMarker = c1;
                rightMarker = c2;
            } else {
                leftMarker = c2;
                rightMarker = c1;
            }

            // Dibujar telemetría visual de conexión (Barra central y nodos)
            cv::line(frame, leftMarker, rightMarker, cv::Scalar(0, 0, 255), 4);
            cv::circle(frame, leftMarker, 8, cv::Scalar(255, 0, 0), -1);
            cv::circle(frame, rightMarker, 8, cv::Scalar(0, 255, 0), -1);

            // CÁLCULO MATEMÁTICO DEL ÁNGULO 
            // Se utiliza arctan2 para obtener el ángulo del vector formado.
            // Se invierte 'dy' porque en OpenCV el eje 'Y' crece hacia abajo (pantalla).
            double dy = rightMarker.y - leftMarker.y;
            double dx = rightMarker.x - leftMarker.x;
            currentAngle = std::atan2(-dy, dx) * 180.0 / PI;

            // Saturación del ángulo al rango físico seguro del controlador PID
            if (currentAngle > 45.0) currentAngle = 45.0;
            if (currentAngle < -45.0) currentAngle = -45.0;

            int64 now = cv::getTickCount();
            double elapsed = static_cast<double>(now - lastUpdateTime) / cv::getTickFrequency();

            // Despachar comando únicamente si ha pasado el intervalo definido
            if (elapsed >= updateInterval)
            {
                // APLICACIÓN DEL FILTRO
                if (firstFrame) {
                    smoothedAngle = currentAngle;
                    firstFrame = false;
                } else {
                    smoothedAngle = (alpha * currentAngle) + ((1.0 - alpha) * smoothedAngle);
                }
                lastSentAngle = smoothedAngle;
                
                lastUpdateTime = now;

                // Empaquetamiento y envío del payload por UDP al ESP32
                std::string msg = "ANGLE:" + std::to_string(lastSentAngle);
                udp.writeLine(msg);

                std::cout << "Enviado UDP -> " << msg << std::endl;
            }
        }

        // INTERFAZ GRÁFICA DE USUARIO
        // Superposición del ángulo procesado y filtrado
        cv::putText(frame,
                    "Angulo: " + std::to_string(lastSentAngle) + " grados",
                    cv::Point(30, 50),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0, 0, 255),
                    2);

        // Perfilamiento de rendimiento del ciclo completo
        double tiempo = (cv::getTickCount() - t0) / cv::getTickFrequency();
        std::cout << "Tiempo por frame: " << tiempo * 1000 << " ms" << std::endl;
        
        contador++;

        // Optimización de interfaz: renderizar solo los frames impares
        if (contador % 2 != 0)
        {
            cv::imshow("Sistema de Vision - Pendulo", frame);

            if (cv::waitKey(1) == 27) // Código ASCII 27 = Tecla ESC
                break;
        }
    }

    // LIMPIEZA FINAL 
    cap.release(); // Libera el stream de la cámara web
    cv::destroyAllWindows(); // Destruye las ventanas de la memoria de video

    return 0;
}