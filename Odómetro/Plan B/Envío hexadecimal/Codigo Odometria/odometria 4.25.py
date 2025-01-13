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

def desconectar():
    global Estado, envio_encoder, puertoSerial_c
    Estado = 0
    envio_encoder = True
    habilitar_botones(True, False, False)
    mensaje = "STOP"
    print(mensaje)
    if puertoSerial_c != None:
        puertoSerial_c.write(mensaje.encode('utf-8'))

def reset():
    global Estado_reset, begin_reset, puertoSerial_c, Ticks
    mensaje_error.config(text="")
    if Estado == 1:
        try:
            # Intentar convertir el valor ingresado a float
            begin_reset = float(input_reset.get())  # Si es válido, se guarda en begin_reset
            Estado_reset = 1  # Si la conversión es exitosa, se activa el reset
            if odometro_lista.get() == "Guia de cable":
                Ticks = round((begin_reset * 1024) / (0.0372 * 3.1416))
            elif odometro_lista.get() == "Carrete":
                Ticks = round((begin_reset * 1024) / (0.0225 * 3.1416 * 1.0216))
            elif odometro_lista.get() == "Personalizado":
                Ticks = round((begin_reset * 1024) / (float(input_ratio.get()) * 3.1416))
        except ValueError:
            # Si ocurre un error al intentar convertir a float, muestra un mensaje de error
            mensaje_error.config(text="Error: El valor ingresado no es válido. Ingrese un número válido. Ejemplo: 0.05. Use punto para los decimales")

def conexion():
    global Estado, envio_encoder, Estado_reset, port_lista, selec_imu, puertoSerial_c, envio_encoder_listo, Ticks, ti_ant
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

        puertoSerial_c = serial.Serial(port_lista.get(), 115200, timeout=0.2)
        print(f"Conexión establecida con puerto {port_lista.get()}")

        habilitar_botones(False, True, True)

        if odometro_lista.get() == "Guia de cable":
            encoder_type = 1
            encoder_ratio = 0
            envio_encoder = True
        elif odometro_lista.get() == "Carrete":
            encoder_type = 2
            encoder_ratio = 0
            envio_encoder = True
        elif odometro_lista.get() == "Personalizado":
            encoder_type = 3
            try:
                # Intentar convertir el valor ingresado a float
                encoder_ratio = float(input_ratio.get())  # Si es válido, se guarda en begin_reset
                envio_encoder = True
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

    data_queue = queue.Queue()

    def data_processing_loop():
        global Estado, Ticks, ti_ant, Estado_reset
        Ti = ""
        pitch = "+00.0"
        roll = "+000.0"
        Distancia = "000.01"
        acelerometro = [0, 0, 0, 0, 0, 0]
        magnetometro = [0, 0, 0, 0, 0, 0]
        giroscopio = [0, 0, 0, 0, 0, 0]
        nombre_archivo = os.path.join(current_folder, "CSVs de prueba", f"{datetime.datetime.now().strftime('%H.%M.%S.%f')}.csv")

        with open(nombre_archivo, "w", newline="") as file:
            writer = csv.writer(file, delimiter=";", quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
            writer.writerow(["Hora Unix","Hora Local",  "Distancia", "Ticks", "GIROSCOPIO", "ACELEROMETRO", "MAGNETROMETRO"])
        
        while Estado == 1:
            start_time = time.perf_counter() #Para control de tiempo del ciclo

            ace = " "
            giro = " "
            mag = " "
            Ti = ""
            salto_de_linea = 0
            imu = ""  
            if selec_imu.get() == 1:
                while salto_de_linea < 3: 
                    if puertoSerial.in_waiting > 0:
                        lectura = puertoSerial.readline()
                        if len(lectura) > 5:
                            imu = imu + lectura.decode('cp1252')
                            salto_de_linea = salto_de_linea + 1
                            time.sleep(0.002) 
                puertoSerial.write("OK".encode('utf-8'))
                
                if imu != "":
                    lista_IMU = imu.split('\n') 
                    
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

            if puertoSerial_c.in_waiting > 0:  # Verifica si hay datos disponibles
                try:
                    Tics = puertoSerial_c.readline()
                    if len(Tics) > 0:
                        Ti = "".join(filter(lambda x: x.isdigit() or x in ['-', '+'], str(Tics)))
                        if Ti == "":
                            Ti_int = 0
                        else:
                            Ti_int = int(Ti)
                        diferencial = ti_ant - Ti_int
                        ti_ant = Ti_int
                        Ticks = Ticks - diferencial

                        if odometro_lista.get() == "Guia de cable":
                            Distancia = round((((Ticks * 0.0372 * 3.1416) / 1024) * 1), 2)
                        elif odometro_lista.get() == "Carrete":
                            Distancia = round((((Ticks * 0.0225 * 3.1416) / 1024) * 1.0216), 2)
                        elif odometro_lista.get() == "Personalizado":
                            Distancia = round((((Ticks * float(input_ratio.get()) * 3.1416) / 1024) * 1), 2)

                        string_DIstancia=str(Distancia).split(".")

                        if len(string_DIstancia[1])<2:
                            string_DIstancia[1]= string_DIstancia[1]+"0"
                        Distancia=string_DIstancia[0]+"."+string_DIstancia[1]

                        if float(Distancia) >= 0:
                            Distancia = "+" + "0"*(7-len(str(Distancia))) + str(Distancia)
                        else:
                            Distancia = "-" + "0"*(6-len(str(Distancia[1:]))) + str(Distancia[1:])
                    data_queue.put(Distancia)
                except UnicodeDecodeError:
                    print("Error de decodificación, ignorando los datos corruptos.")
                    continue
            # Actualiza la variable de Tkinter
            formato = f"$PITCH{pitch},ROLL{roll},DIST{Distancia}\r\n"
            
            with open(nombre_archivo, 'a', newline='') as archivo:
                escritor = csv.writer(archivo, delimiter=';', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
                hora_actual_unix = datetime.datetime.now().timestamp()
                hora_actual_local = datetime.datetime.now()
                escritor.writerow([hora_actual_unix, hora_actual_local, Distancia, Ticks, giro, ace, mag])
            
            if selec_imu.get() == 1:
                puertoSerial_b.write(formato.encode('utf-8'))
            
            if Estado_reset == 1:
                try:
                    nueva_dis = "reset" + input_reset.get() + "@"
                    puertoSerial_c.write(nueva_dis.encode('utf-8'))
                    Estado_reset = 0
                except ValueError:
                    pass

            elapsed_time = time.perf_counter() - start_time
            sleep_time = max(0, 1/31 - elapsed_time) #Calcula el tiempo que debe dormir para mantener los 31hz
            time.sleep(sleep_time) #Duerme el tiempo calculado
            

    def update_gui(): #Función para actualizar la GUI
        while True:
            try:
                Distancia = data_queue.get_nowait() #Intenta obtener datos de la cola sin bloquear
                dis.set(Distancia) #Actualiza la interfaz
                ventana.update()
            except queue.Empty: #Si la cola está vacía, espera un poco
                time.sleep(0.01) #Espera 10ms
            except tk.TclError: #Excepción para cuando se cierra la ventana
                break

    # Iniciar el bucle de procesamiento en un hilo separado
    data_thread = threading.Thread(target=data_processing_loop)
    data_thread.daemon = True #Para que el hilo se cierre cuando se cierra la ventana
    data_thread.start()

    gui_thread = threading.Thread(target=update_gui)
    gui_thread.daemon = True
    gui_thread.start()

  #  if selec_imu == 1:
   #     puertoSerial.flushInput()
    #    puertoSerial.close()
     #   puertoSerial_b.close()
    #if port_lista.get() and odometro_lista.get():
     #   puertoSerial_c.flushInput()
      #  puertoSerial_c.close()

def on_closing():
    global Estado, puertoSerial_c, puertoSerial, puertoSerial_b
    Estado = 0
    time.sleep(0.1)  # Dar tiempo a los hilos a terminar
    if puertoSerial_c:
        puertoSerial_c.flushInput() #Vaciar el buffer antes de cerrar
        puertoSerial_c.close()
    if puertoSerial:
        puertoSerial.flushInput()
        puertoSerial.close()
    if puertoSerial_b:
        puertoSerial_b.flushInput()
        puertoSerial_b.close()
    ventana.destroy()

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
