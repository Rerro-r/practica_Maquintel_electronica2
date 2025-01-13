import csv
import datetime
import os
import serial
import time
import tkinter as tk
from tkinter import ttk
import threading
import queue

Estado = 0
Estado_reset = 0
envio_encoder = False  # Debe ser True cada vez que se desconecta de forma manual o accidental el receptor
begin_reset = 0
envio_encoder_listo = False
Ticks = 0
ti = 0
ti_ant = 0
puertoSerial_c = None
puertoSerial = None
puertoSerial_b = None
constante = None
data_queue = None
data_thread = None

CONSTANTES_ODOMETRO = {
    "Guia de cable": 0.0372 * 3.1416 / 1024,
    "Carrete": 0.0225 * 3.1416 * 1.0216 / 1024,
    "Personalizado": None  # Se calculará dinámicamente
}

current_file_path = os.path.abspath(__file__)
current_folder = os.path.dirname(current_file_path)
logo_path = os.path.join(current_folder, "Logos_Maquintel", "Logos_Maquintel.png")

def puertos_seriales():
    ports = [f"COM{i + 1}" for i in range(24)]
    encontrados = []
    for port in ports:
        try:
            with serial.Serial(port) as s:
                encontrados.append(port)
        except (OSError, serial.SerialException):
            pass
    return encontrados

def actualizar_puertos(*args):
    """Actualiza la lista de puertos COM si no hay conexión establecida."""
    if Estado == 0:
        
        puertos_disponibles = puertos_seriales()
        # Actualiza solo si hay cambios
        if port_lista["values"] != puertos_disponibles:
            port_lista["values"] = puertos_disponibles
            port_imu["values"] = puertos_disponibles


def leer_imu(puerto_imu): #funcion para leer el IMU
    salto_de_linea = 0
    imu = ""
    if puerto_imu: # verificar que el puerto exista
        while salto_de_linea < 3: 
            if puerto_imu.in_waiting > 0:
                lectura = puerto_imu.readline()
                if len(lectura) > 5:
                    imu = imu + lectura.decode('cp1252')
                    salto_de_linea = salto_de_linea + 1
                    time.sleep(0.002) 
        puerto_imu.write("OK".encode('utf-8'))
    return imu

def procesar_imu(imu_data): #funcion para procesar los datos del imu
    acelerometro = [0, 0, 0, 0, 0, 0]
    magnetometro = [0, 0, 0, 0, 0, 0]
    giroscopio = [0, 0, 0, 0, 0, 0]
    lista_IMU = imu_data.split('\n')
    if len(lista_IMU) >= 3:
        for i in lista_IMU:
            if len(i) > 6:
                A = i.split(",")
                if len(A) >= 6:
                    if A[1] == "0":
                        giroscopio = i.split(",")
                    elif A[1] == "1":
                        acelerometro = i.split(",")
                    elif A[1] == "2":
                        magnetometro = i.split(",")
    ace = str(acelerometro[1]) + ";" + str(acelerometro[3]) + ";" + str(acelerometro[4]) + ";" + str(acelerometro[5])
    giro = str(giroscopio[1]) + ";" + str(giroscopio[3]) + ";" + str(giroscopio[4]) + ";" + str(giroscopio[5])
    mag = str(magnetometro[1]) + ";" + str(magnetometro[3]) + ";" + str(magnetometro[4]) + ";" + str(magnetometro[5])
    return ace, giro, mag


def desconectar():
    global Estado, envio_encoder, puertoSerial_c, data_thread
    Estado = 0
    envio_encoder = True
    habilitar_botones(True, False, False)
    mensaje = "STOP"
    print(mensaje)
    if puertoSerial_c != None:
        puertoSerial_c.write(mensaje.encode('utf-8'))
        cerrar_puerto(puertoSerial_c, "C") # Cerrar el puerto serial
        puertoSerial_c = None #importante setear a none para que se pueda volver a abrir
    if data_thread and data_thread.is_alive(): #esperar a que el hilo termine
        Estado = 0
        data_thread.join(timeout=1) # esperar que el hilo termine
        print("Hilo de procesamiento detenido.")

def reset():
    global Estado_reset, begin_reset, puertoSerial_c, Ticks, constante
    mensaje_error.config(text="")
    if Estado == 1:
        try:
            # Intentar convertir el valor ingresado a float
            begin_reset = float(input_reset.get())  # Si es válido, se guarda en begin_reset
            Estado_reset = 1  # Si la conversión es exitosa, se activa el reset
            Ticks = round(begin_reset / constante)
        except ValueError:
            # Si ocurre un error al intentar convertir a float, muestra un mensaje de error
            mensaje_error.config(text="Error: El valor ingresado no es válido. Ingrese un número válido. Ejemplo: 0.05. Use punto para los decimales")

def conexion():
    global Estado, envio_encoder, Estado_reset, port_lista, selec_imu, puertoSerial_c, envio_encoder_listo, Ticks, ti_ant, constante, data_thread
    mensaje_error.config(text="")
    Estado = 1

    if not port_lista.get() or not odometro_lista.get():
        mensaje_error.config(text="Falta seleccionar el puerto o el tipo de odómetro.")
        Estado = 0
        return
    if odometro_lista.get() == "Personalizado" and not input_ratio.get():
        mensaje_error.config(text="Falta seleccionar el radio especial para odómetro personalizado.")
        Estado = 0
        return

    mensaje_error.config(text="")

    hora_actual = datetime.datetime.now().strftime("%H.%M.%S.%f")
    nombre_archivo = os.path.join(current_folder, "CSVs de prueba", f"{hora_actual}.csv")

    with open(nombre_archivo, "w", newline="") as file:
        writer = csv.writer(file, delimiter=";", quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
        writer.writerow(["Hora Unix","Hora Local",  "Distancia", "Ticks", "GIROSCOPIO", "ACELEROMETRO", "MAGNETROMETRO"])

    if port_lista.get() and odometro_lista.get():
        if selec_imu.get():
            puertoSerial = serial.Serial(port_imu.get(), 115200, timeout=2)
            puertoSerial.write("iniciar".encode())
            print("Conexión con IMU establecida")

            # Inicializa la conexión serial para el software sonar
            puertoSerial_b = serial.Serial("COM29", 115200, timeout=2)
            puertoSerial_b.write("iniciar".encode())
            print("Conexión con software sonar establecida")

        puertoSerial_c = serial.Serial(port_lista.get(), 115200, timeout=0.04)
        print(f"Conexión establecida con puerto {port_lista.get()}")

        habilitar_botones(False, True, True)

        if odometro_lista.get() == "Guia de cable":
            encoder_type = 1
            encoder_ratio = 0
            envio_encoder = True
            constante = CONSTANTES_ODOMETRO[odometro_lista.get()]
        elif odometro_lista.get() == "Carrete":
            encoder_type = 2
            encoder_ratio = 0
            envio_encoder = True
            constante = CONSTANTES_ODOMETRO[odometro_lista.get()]
        elif odometro_lista.get() == "Personalizado":
            encoder_type = 3
            try:
                # Intentar convertir el valor ingresado a float
                encoder_ratio = float(input_ratio.get())  # Si es válido, se guarda en begin_reset
                envio_encoder = True
                constante = encoder_ratio * 3.1416 / 1024
            except ValueError:
                # Si ocurre un error al intentar convertir a float, muestra un mensaje de error
                mensaje_error.config(text="Error: El valor ingresado no es válido. Ingrese un número válido. Ejemplo: 0.05. Use punto para los decimales")


        if envio_encoder:
            if envio_encoder_listo: 
                mensaje = f"RUN"
                #print(mensaje)
                puertoSerial_c.write(mensaje.encode('utf-8'))
                envio_encoder = False
            else:# solo envía encoder una vez. Si se quiere cambiar de encoder se debe cerrar la aplicación
                mensaje = f"RUN,{encoder_type},{encoder_ratio}"
                #print(mensaje)
                puertoSerial_c.write(mensaje.encode('utf-8'))
                envio_encoder = False
                envio_encoder_listo = True
    
    
    else:

        Estado = 0


            


    # Iniciar el bucle de procesamiento en un hilo separado
    data_thread = threading.Thread(target=data_processing_loop)
    data_thread.daemon = True #Para que el hilo se cierre cuando se cierra la ventana
    data_thread.start()
    update_gui()

   # gui_thread = threading.Thread(target=update_gui)
   # gui_thread.daemon = True
   # gui_thread.start()

  #  if selec_imu == 1:
   #     puertoSerial.flushInput()
    #    puertoSerial.close()
     #   puertoSerial_b.close()
    #if port_lista.get() and odometro_lista.get():
     #   puertoSerial_c.flushInput()
      #  puertoSerial_c.close()

def data_processing_loop():
    global Estado, Ticks, ti_ant, Estado_reset, constante, data_queue
    Ti = ""
    tiempo_anterior_escritura = time.perf_counter()
    tiempos_de_ciclo = []
    data_queue = queue.Queue()
    distancia_anterior = None
    ticks_anteriores = None
    pitch = "+00.0"
    roll = "+000.0"
    Distancia_str = "+000000.0"
    nombre_archivo = os.path.join(current_folder, "CSVs de prueba", f"{datetime.datetime.now().strftime('%H.%M.%S.%f')}.csv")

    with open(nombre_archivo, "w", newline="") as file:
        writer = csv.writer(file, delimiter=";", quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
        writer.writerow(["Hora Unix","Hora Local",  "Distancia", "Ticks", "GIROSCOPIO", "ACELEROMETRO", "MAGNETROMETRO"])
    
    while Estado == 1:
        start_time = time.perf_counter() #Para control de tiempo del ciclo
        tipo_odometro = odometro_lista.get()
        Ti = ""
        salto_de_linea = 0
        imu = ""  
        ace = ""
        giro = ""
        mag = ""
        if selec_imu.get() == 1:
            imu_data = leer_imu(puertoSerial)
            if imu_data:
                ace, giro, mag = procesar_imu(imu_data)
            else:
                ace = giro = mag = ""
            
        if puertoSerial_c.in_waiting > 0:  # Verifica si hay datos disponibles
            try:
                Tics_bytes = puertoSerial_c.read_until(b'\r\n')
                if not Estado:
                    break

                # Decodificación y conversión a entero directamente:
                try:
                    Ti_int = int(Tics_bytes.decode().strip()) #Decodifica, quita espacios y convierte a entero
                except ValueError: #Manejo de error si no se puede convertir a entero
                    print(f"Error al convertir a entero: {Tics_bytes}")
                    continue #Continua a la siguiente iteracion
                if Ti_int != ticks_anteriores:
                    diferencial = ti_ant - Ti_int
                    ti_ant = Ti_int
                    Ticks -= diferencial

                    Distancia = round(Ticks * constante, 2)
                    if Distancia != distancia_anterior:
                        Distancia_str = f"{Distancia:+08.2f}"
                        distancia_anterior = Distancia #Actualizar la distancia anterior
                        ticks_anteriores = Ti_int #Actualizar los ticks anteriores
                        data_queue.put(Distancia_str) #Solo guardar en la cola si la distancia cambio                  
                else:
                    Distancia_str = f"{distancia_anterior:+08.2f}"

            except UnicodeDecodeError:
                print("Error de decodificación, ignorando los datos corruptos.")
                continue

        
        tiempo_actual = time.perf_counter()
        if tiempo_actual - tiempo_anterior_escritura >= 1/31:
            with open(nombre_archivo, 'a', newline='') as archivo:
                escritor = csv.writer(archivo, delimiter=';', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
                hora_actual_unix = datetime.datetime.now().timestamp()
                hora_actual_local = datetime.datetime.now()
                escritor.writerow([hora_actual_unix, hora_actual_local, Distancia_str, Ticks, giro, ace, mag])
            tiempo_anterior_escritura = tiempo_actual
            
        if selec_imu.get() == 1:
            puertoSerial_b.write(f"$PITCH{pitch},ROLL{roll},DIST{Distancia}\r\n".encode('utf-8'))
        

        elapsed_time = time.perf_counter() - start_time
        tiempos_de_ciclo.append(elapsed_time)
        target_time = 1/31

        if elapsed_time > target_time:
            print(f"No se cumple el tiempo objetivo. Tiempo transcurrido: {elapsed_time:.6f}") #Imprime tiempo transcurrido

        if tiempos_de_ciclo:
            promedio = sum(tiempos_de_ciclo) / len(tiempos_de_ciclo)
            maximo = max(tiempos_de_ciclo)
            minimo = min(tiempos_de_ciclo)
            print(f"Tiempos de ciclo: Promedio: {promedio:.6f}, Maximo: {maximo:.6f}, Minimo: {minimo:.6f}")


def update_gui():
    ventana.after(0, update_gui_inner) #usar after para evitar bucle infinito
def update_gui_inner():
    global data_queue
    try:
        Distancia = data_queue.get_nowait()
        dis.set(Distancia)
    except queue.Empty:
        pass
    except tk.TclError:
        return
    ventana.after(20, update_gui_inner) #actualizar cada 33ms

def on_closing():
    global Estado, puertoSerial_c, puertoSerial, puertoSerial_b
    Estado = 0  # Asegura que los hilos terminen
    time.sleep(0.5)  # Da más tiempo a los hilos para que terminen
    if puertoSerial_c:
        cerrar_puerto(puertoSerial_c, "C")
    if puertoSerial:
        cerrar_puerto(puertoSerial, "IMU")
    if puertoSerial_b:
        cerrar_puerto(puertoSerial_b, "Sonar")
    ventana.destroy()

def cerrar_puerto(puerto, nombre_puerto):
    if puerto:
        try:
            if puerto.is_open: #Verifica si el puerto esta abierto antes de cerrarlo
                puerto.flushInput()
                puerto.flushOutput() #Agregado flushOutput
                puerto.close()
                print(f"Puerto Serial {nombre_puerto} cerrado.")
            else:
                print(f"Puerto Serial {nombre_puerto} ya estaba cerrado.")
        except serial.SerialException as e:
            print(f"Error al cerrar el puerto serial {nombre_puerto}: {e}")


def limpiar_error(event):
    mensaje_error.config(text="")  # Borra el mensaje de error cuando el usuario interactúa con un campo de entrada

def habilitar_botones(conectar_habilitado, reset_habilitado, desconectar_habilitado):
    """Habilita o deshabilita botones según el estado."""
    estado_conectar = "normal" if conectar_habilitado else "disabled"
    estado_reset = "normal" if reset_habilitado else "disabled"
    estado_desconectar = "normal" if desconectar_habilitado else "disabled"
    b_conectar.config(state=estado_conectar)
    b_desconectar.config(state="normal" if not conectar_habilitado else "disabled")
    b_reset.config(state=estado_reset)
    input_reset.config(state=estado_reset)
    port_lista.config(state=estado_conectar)
    odometro_lista.config(state=estado_conectar)
    port_imu.config(state=estado_conectar)
    input_ratio.config(state="normal" if odometro_lista.get() == "Personalizado" and reset_habilitado else "disabled")

# Habilitar o deshabilitar el campo "Radio especial"
def habilitar_radio(*args):
    input_ratio.config(state="normal" if odometro_lista.get() == "Personalizado" else "disabled")

# Configuración de la interfaz gráfica
ventana = tk.Tk()
ventana.title("Odometria - Maquintel")
ventana.geometry('800x250')
ventana.configure(background='dark orange')

logo = tk.PhotoImage(file=logo_path)
label = tk.Label(ventana, image=logo, background='dark orange', width=300)
label.place(x=20, y=0)

etiqueta1 = tk.Label(ventana, text='Puerto: ', bg='white', fg='black')
etiqueta1.place(x=5, y=80)

dis = tk.StringVar(value="000.00")
etiqueta2 = tk.Label(ventana, text='Distancia: ', bg='white', fg='black')
etiqueta2.place(x=5, y=159)
etiqueta3 = tk.Label(ventana, textvariable=dis, bg='white', fg='black', width=9)
etiqueta3.place(x=80, y=159)

port_lista = ttk.Combobox(ventana, width=10, values=puertos_seriales())
port_lista.place(x=60, y=80)

port_imu = ttk.Combobox(ventana, width=10, values=puertos_seriales())
port_imu.place(x=60, y=115)

b_desconectar = tk.Button(ventana, text='Desconectar', command=desconectar, width=9, state="disabled")
b_desconectar.place(x=155, y=115)

b_conectar = tk.Button(ventana, text='Conectar', command=conexion, width=9)
b_conectar.place(x=155, y=78)

b_reset = tk.Button(ventana, text='Reset a (en metros): ', command=reset, state="disabled")
b_reset.place(x=155, y=155)

input_reset = tk.Entry(ventana, width=15, state="disabled")
input_reset.place(x=285, y=159)

odometro_lista = ttk.Combobox(ventana, width=10, state="readonly", values=['Guia de cable', 'Carrete', 'Personalizado'])
odometro_lista.place(x=250, y=105)

etiqueta4 = tk.Label(ventana, text='Tipo odometro', bg='white', fg='black', width=13)
etiqueta4.place(x=245, y=80)

etiqueta5 = tk.Label(ventana, text='Radio especial (en metros)', bg='white', fg='black', width=20)
etiqueta5.place(x=350, y=80)

input_ratio = tk.Entry(ventana, width=15, state="disabled")
input_ratio.place(x=350, y=105)

input_reset.bind("<FocusIn>", limpiar_error)
input_ratio.bind("<FocusIn>", limpiar_error)

selec_imu = tk.IntVar()
check = tk.Checkbutton(ventana, variable=selec_imu, onvalue=1, offvalue=0, bg="dark orange", activebackground="dark orange", text='IMU')
check.place(x=5, y=115)

mensaje_error = tk.Label(ventana, text="", bg="white", fg="red", font=("Helvetica", 10))
mensaje_error.place(x=5, y=190)

odometro_lista.bind("<<ComboboxSelected>>", habilitar_radio)

port_lista.bind("<Button-1>", actualizar_puertos)
port_imu.bind("<Button-1>", actualizar_puertos)

# Iniciar actualización de puertos
ventana.protocol("WM_DELETE_WINDOW", on_closing)
ventana.mainloop()
