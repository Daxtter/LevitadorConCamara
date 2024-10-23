# Pues cambie algunas cosas lo paso de una vez (PID raro, interfaz con nuevas cosas)
import sys
from time import sleep
import cv2
import numpy as np
import tkinter as tk
from tkinter import *
from PIL import Image, ImageTk
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import threading
import imutils
import datetime
from imutils.video import VideoStream
from scipy.spatial import distance as dist
from imutils import perspective
import serial.tools #Holaaaaa Alejandor 
import serial.tools.list_ports
import sys
# Variables globales
positions = []
times = []
pwms = []
max_time = 20
max_data_points = 40
width = 40
xB = 340  # Posición del punto de referencia donde empieza el tubo, asi ya no se pierde donde debe estar posicionada la camara
yB = 20
ser = serial.Serial()
ser.close()
com_names = []
setpoint = 0
start_time = time.time()
#Para saber si se puede enviar el PID
hayComunicacion = False
# variables PID
kp = 0
ki = 0  # constantes
kd = 0
error = 0
errorAcum = 0  # error
errorAnt = 0
Dist = 300  # distancia de pelota a techo
distD = 100  # distancia deseada
muestrasD = np.zeros(20)
count = 0
# Variable global para el frame actual
current_frame = None

#Para cerrar esta cochinada
def on_close():
    if ser.is_open:
        ser.write(0)
        time.sleep(1)
        ser.close()
    vs.stop()
    root.after(100,root.destroy)
    sys.exit()
def find_coms():
    com_names = []
    com_values = serial.tools.list_ports.comports()  # detectar puertos disponibles
    for com in com_values:
        com_names.append(com.name)
    com_list['values'] = com_names


def com_micro():
    global hayComunicacion
    # Configurar la comunicación serie con Arduino
    ser.port = com_list.get()
    ser.baudrate = 9600
    if ser.is_open:
        ser.close()
        hayComunicacion = False
        com_button.config(text="Comunicar con Micro")
        status_label.config(text="Estado: DES")
    else:
        ser.open()
        com_button.config(text="Descomunicar con Micro")
        hayComunicacion = True
        status_label.config(text="Estado: CON")

    time.sleep(2)


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

def change_bg():
    global firstFrame
    firstFrame = None #Cuando esta variable es None, dentro de procces_frame lo hace el background

def zoom_at(img, zoom, coord=None):
    """
    Simple image zooming without boundary checking.
    Centered at "coord", if given, else the image center.

    img: numpy.ndarray of shape (h,w,:)
    zoom: float
    coord: (float, float)
    """
    # Translate to zoomed coordinates
    h, w, _ = [zoom * i for i in img.shape]

    if coord is None:
        cx, cy = w / 2, h / 2
    else:
        cx, cy = [zoom * c for c in coord]

    img = cv2.resize(img, (0, 0), fx=zoom, fy=zoom)
    img = img[int(round(cy - h / zoom * .5)): int(round(cy + h / zoom * .5)),
          int(round(cx - w / zoom * .5)): int(round(cx + w / zoom * .5)),
    :]  # ROI

    return img


# Funciones para actualizar setpoint y PID, el setpoint es la distancia entre la pelota y el punto de referencia, entre mas alto el
# setpoint mas distancia hay entre la pelota y la referencia.
def set_pid_values():
    global kp, ki, kd
    kp = float(kp_entry.get())
    ki = float(ki_entry.get())
    kd = float(kd_entry.get())


def set_setpoint():
    global distD
    setpoint = int(setpoint_entry.get())
    line_ref.set_ydata([setpoint])
    distD = setpoint


def ingresarEnMuestras(dato):  # agregar a un areglo de valores distancia
    global count
    muestrasD[count] = dato
    if count == 19:
        count = 0
    count += 1


def promedio():  # sacar promedio de 20 valores distancia
    sum = 0
    for valor in muestrasD:
        sum = sum + valor
    return sum / 20


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def calcular_PID_funcion(Dist):
    global error, errorAnt, errorAcum, distD, distP, hayComunicacion
    #Si hay comunicacion quiere decir que ya hay comunicacion con el atmega
    #Si no hay entonces el programa todavia no se ah practicamente iniciado
    if hayComunicacion:
        ingresarEnMuestras(Dist)
        distP = promedio()
        error = distP - distD
        P = kp * error
        dt_error = (error - errorAnt)
        D = kd * dt_error
        errorAcum = errorAcum + error
        I = ki * errorAcum
        PID = P + I + D
        print("P: ", P)
        print("I: ", I)
        print("D: ", D)
        if PID < -65535:
            PID = -65535
        if PID > 65535:
            PID = 65535
        PID = round(map_range(PID, -65535, 65535, 0, 65535))
        print("PWM: ", PID)
        print("Error acumulado", errorAcum)
        BytesValue = PID.to_bytes(2, "little")  # se convierte en un byte (tipo de dato de 8 bits, abarca el rango del
        # PWM, que es 0-65535

        # Windeup
        if (errorAcum > 110000):
            errorAcum = 1100000
        else:
            if(errorAcum < -100000):
                errorAcum =-100000
            else:
                if (errorAcumAnt < (errorAcum + 1) & & errorAcumAnt > (errorAcum - 1)):
                    contador2 = contador2 + 1;
                    if (contador2 > 60):
                        errorAcum = 0
                else:
                contador2=0

        ser.write(BytesValue)



def calcular_PID_hilo():
    global error, errorAnt, errorAcum, distD, distP, Dist, hayComunicacion
    while True:
        ingresarEnMuestras(Dist)
        distP = promedio()
        error = distP - distD
        P = kp * error
        dt_error = (error - errorAnt)
        D = kd * dt_error
        errorAcum = errorAcum + error
        I = ki * errorAcum
        PID = P + I + D
        print("P: ", P)
        print("I: ", I)
        print("D: ", D)
        if PID < -600:
            PID = -600
        if PID > 600:
            PID = 600
        PID = round(map_range(PID, -600, 600, 0, 65535))
        print("PWM: ", PID)
        if hayComunicacion:
            BytesValue = PID.to_bytes(2,"little")  # se convierte en un byte (tipo de dato de 8 bits, abarca el rango del
            # PWM, que es 0-65535
            ser.write(BytesValue)

        errorAnt = error
        time.sleep(0.1)


# Crear la ventana principal (interfaz)

# Actualiza la grafica
def update_graph():
    global start_time
    current_time = time.time()
    times = [(t - start_time) for t in np.linspace(start_time, current_time, len(positions))]
    line_pos.set_data(times, positions)
    if len(positions) > max_data_points:
        positions.pop(0)
    if (current_time - start_time > max_time):
        start_time = current_time
        times.pop(0)

    ax.relim()
    ax.autoscale_view()
    canvas.draw()


# Función para procesar la imagen y detectar la pelota
def process_frame(frame):
    global firstFrame, current_frame, positions, Dist, contornoMinimo, contornoMaximo
    frame = vs.read()
    frame = zoom_at(frame, 1.80, None)
    frame = frame
    # resize the frame, convert it to grayscale, and blur it
    frame = imutils.resize(frame, width=frame_width)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (ksize, ksize), 0)  # Posible Cambio Nivel chido = 7,7
    if frame is None:
        print("Lo esta leyendo mal")

    # if the first frame is None, initialize it
    if firstFrame is None:
        firstFrame = gray
        return
    frameDelta = cv2.absdiff(firstFrame, gray)
    thresh = cv2.threshold(frameDelta, 43, 255, cv2.THRESH_BINARY)[1]  # Posible a cambiar nivel chido 30
    # dilate the thresholded image to fill in holes, then find contours
    # on thresholded image
    thresh = cv2.dilate(thresh, None, iterations=iteraciones)  # Es el que mas sirve para cambiarlo, nivel chido 30
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    #manda a llamar al PID si no hay contornos
    if len(cnts) ==0 :
        calcular_PID_funcion(305)
        positions.append(305)
    # loop over the contours
    for c in cnts:
        # if the contour is too small, ignore it
        if cv2.contourArea(c) < contornoMinimo: #750
            continue
        if cv2.contourArea(c) > contornoMaximo: #2050
            continue
        # compute the bounding box for the contour, draw it on the frame,
        # and update the text
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        box = perspective.order_points(box)
        cX = np.average(box[:, 0])
        cY = np.average(box[:, 1])
        (tl, tr, br, bl) = box
        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)
        Dist = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))
        refObj = (box, (cX, cY), Dist / width)
        refCoords = np.vstack([refObj[0], refObj[1]])  ##Aqui
        [xA, yA] = refCoords[4]  # Cordenadas del centro de la pelota
        cv2.line(frame, (int(xA), int(yA)), (int(xB), int(yB)), (240, 0, 159), 2)
        Dist = dist.euclidean((xA, yA), (xB, yB)) / refObj[2]
        calcular_PID_funcion(Dist) #manda a llamar al PID
        positions.append(round(Dist))

        (mX, mY) = midpoint((xA, yA), (xB, yB))
        cv2.putText(frame, "{:.1f}mm".format(Dist), (int(mX), int(mY - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 0, 159), 2)

    video = vs.stream
    fps = str(video.get(cv2.CAP_PROP_FPS))
    cv2.circle(frame, (int(xB), int(yB)), 5, (240, 0, 159), -1)
    cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p") + " " + fps + " fps",
                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
    current_frame = frame

    # Función para capturar video, se esta haciendo en otro hilo


def video_capture():
    global current_frame
    while True:
        frame = vs.read()
        # Procesar el frame en un hilo separado
        process_frame(frame)
        # Mostrar el frame en la interfaz
        if current_frame is not None:
            img = Image.fromarray(cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB))
            # Hacerle resize para que entre

            root.after(0, actualizar_label, img)


def cargar_datos():
    while True:
        root.after(0, update_graph())
        time.sleep(0.1)
    # cleanup the camera and close any open windows


# Los hilos secundarios no pueden usar los widgets de ttk, porque ttk explota, no esta hecho para eso
def actualizar_label(img):
    img_tk = ImageTk.PhotoImage(image=img)
    label.configure(image=img_tk)
    label.img_tk = img_tk


import tkinter as tk
from tkinter import ttk

root = tk.Tk()
root.title("Control PID con Detección de Pelota en Milímetros")
root.geometry("1280x720")

# Frame para controles PID
header_frame = ttk.Frame(root, padding=10)
header_frame.pack(side=tk.TOP, fill=tk.X)

title_label = ttk.Label(header_frame, text="Control PID", font=("Helvetica", 16))
title_label.pack(pady=5)

# Entradas para setpoint y valores PID
setpoint_label = ttk.Label(header_frame, text="Setpoint (mm):")
setpoint_label.pack(side=tk.LEFT, padx=5)
setpoint_entry = ttk.Entry(header_frame, width=5)
setpoint_entry.insert(0, "100")
setpoint_entry.pack(side=tk.LEFT, padx=5)

kp_label = ttk.Label(header_frame, text="Kp:")
kp_label.pack(side=tk.LEFT, padx=5)
kp_entry = ttk.Entry(header_frame, width=5)
kp_entry.insert(0, "0")
kp_entry.pack(side=tk.LEFT, padx=5)

ki_label = ttk.Label(header_frame, text="Ki:")
ki_label.pack(side=tk.LEFT, padx=5)
ki_entry = ttk.Entry(header_frame, width=5)
ki_entry.insert(0, "0")
ki_entry.pack(side=tk.LEFT, padx=5)

kd_label = ttk.Label(header_frame, text="Kd:")
kd_label.pack(side=tk.LEFT, padx=5)
kd_entry = ttk.Entry(header_frame, width=5)
kd_entry.insert(0, "0")
kd_entry.pack(side=tk.LEFT, padx=5)

pid_button = ttk.Button(header_frame, text="Actualizar PID", command=set_pid_values)
pid_button.pack(side=tk.LEFT, padx=5)

setpoint_button = ttk.Button(header_frame, text="Actualizar Setpoint", command=set_setpoint)
setpoint_button.pack(side=tk.LEFT, padx=5)

com_button = ttk.Button(header_frame, text="Comunicar con Micro", command=com_micro)
com_button.pack(side=tk.LEFT, padx=5)

status_label = ttk.Label(header_frame, text="Estado: DES")
status_label.pack(side=tk.LEFT, padx=5)

com_label = ttk.Label(header_frame, text="COMS:")
com_label.pack(side=tk.LEFT, padx=5)

com_list = ttk.Combobox(state="readonly")
com_list.place(x=860, y=50)

port_button = ttk.Button(header_frame, text="Actualizar lista COM", command=find_coms)
port_button.place(x=1005, y=38)

port_button = ttk.Button(header_frame, text="Cambiar fondo", command=change_bg)
port_button.place(x=1130, y=38)
# Frame para gráfica, se grafica el pwm y la posicion de la pelota
graph_frame = ttk.Frame(root, padding=5)
graph_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=False)

# Configurar la gráfica
fig, ax = plt.subplots(figsize=(6, 4))
ax.set_title('Posición de la Pelota', fontsize=14)
ax.set_xlabel('Tiempo')
ax.set_ylabel('Posición (mm) / PWM')
line_pos, = ax.plot([], [], 'r-', label='Posición')
line_ref = ax.axhline(y=100)
ax.legend()
ax.set_xlim(0, max_time)
ax.set_ylim(0, 400)  #

canvas = FigureCanvasTkAgg(fig, master=graph_frame)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=False)

image_frame = ttk.Frame(root, padding=10)
image_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, ancho=tk.CENTER)

# Mostrar video
label = tk.Label(image_frame)
label.pack()

# Iniciar el hilo de captura de video
video_thread = threading.Thread(name="Primer hilo", target=video_capture)
video_thread.daemon = True
grafica_thread = threading.Thread(name="Segundo hilo", target=cargar_datos)
grafica_thread.daemon = True
#PID_thread = threading.Thread(name="Tercer Hilo", target=calcular_PID)
#PID_thread.daemon = True

# Inicializar la cámara
vs = VideoStream(src=2).start()  # Tamano imagen 1280.0 x 720.0
time.sleep(2.0)
# Variables para que funcione el entorno
firstFrame = None
frame_width = 700
ksize = 9
contornoMinimo = 700
contornoMaximo = 2550
iteraciones = 5
video_thread.start()
grafica_thread.start()
#PID_thread.start()
root.protocol("WM_DELETE_WINDOW", on_close) #Para que cuando se cierre la ventana del root se maten todos los hilos
root.mainloop()
vs.stop()
if ser.is_open:
    ser.close()