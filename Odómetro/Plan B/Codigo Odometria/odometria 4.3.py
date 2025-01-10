import tkinter as tk
from tkinter import ttk
import serial
import time
import csv
import os
import datetime

# Asegúrate de definir esta variable para el logo y la carpeta actual
current_folder = os.path.dirname(os.path.abspath(__file__))
logo_path = os.path.join(current_folder, "Logos_Maquintel", "Logos_Maquintel.png")

# Función para obtener los puertos seriales (debes definir esta función)
def puertos_seriales():
    # Deberías implementar esta función que devuelva una lista de puertos disponibles
    ports = [f"COM{i + 1}" for i in range(24)]
    encontrados = []
    for port in ports:
        try:
            with serial.Serial(port) as s:
                encontrados.append(port)
        except (OSError, serial.SerialException):
            pass
    return encontrados

class App:
    def __init__(self, root):
        self.root = root
        self.default_width = 350
        self.default_height = 200
        self.is_fullscreen = False
        self.previous_geometry = f"{self.default_width}x{self.default_height}"
        self.init_ui()

    def init_ui(self):
        self.root.title("Odómetro - Maquintel")
        self.root.geometry(f'{self.default_width}x{self.default_height}')
        self.root.configure(background='dark orange')
        self.root.resizable(True, True)

        # Logo
        if os.path.exists(logo_path):
            self.logo = tk.PhotoImage(file=logo_path)
            self.logo_label = tk.Label(self.root, image=self.logo, bg='dark orange')
            self.logo_label.place(x=20, y=0, width=300)

        # Puerto Serial
        self.port_label = tk.Label(self.root, text="Puerto:", bg="white", fg="black")
        self.port_label.place(x=5, y=80)

        self.port_lista = ttk.Combobox(self.root, width=10, values=puertos_seriales())
        self.port_lista.place(x=60, y=80)

        self.port_lista2 = ttk.Combobox(self.root, width=10, values=puertos_seriales())
        self.port_lista2.place(x=60, y=115)

        # Botones
        self.connect_button = tk.Button(self.root, text="Conectar", command=self.conexion, width=9)
        self.connect_button.place(x=155, y=78)

        self.disconnect_button = tk.Button(self.root, text="Desconectar", command=self.desconectar, width=9)
        self.disconnect_button.place(x=155, y=115)

        self.reset_button = tk.Button(self.root, text="Reset a: ", command=self.reset)
        self.reset_button.place(x=155, y=155)

        self.input_entry = tk.Entry(self.root, width=15)
        self.input_entry.place(x=225, y=159)

        # Distancia
        self.distance_label = tk.Label(self.root, text="Distancia:", bg="white", fg="black")
        self.distance_label.place(x=5, y=159)

        self.distance_var = tk.StringVar(value="000.00")
        self.distance_display = tk.Label(self.root, textvariable=self.distance_var, bg="white", fg="black", width=9)
        self.distance_display.place(x=80, y=159)

        # Tipo de Odómetro
        self.odometer_label = tk.Label(self.root, text="Tipo odómetro", bg="white", fg="black", width=13)
        self.odometer_label.place(x=245, y=80)

        self.odometro_lista = ttk.Combobox(self.root, width=10, state="readonly", values=["Guia de cable", "Carrete"])
        self.odometro_lista.place(x=250, y=105)

        # Opción IMU
        self.imu_var = tk.IntVar()
        self.imu_check = tk.Checkbutton(self.root, variable=self.imu_var, onvalue=1, offvalue=0, bg="dark orange",
                                        activebackground="dark orange", text="IMU")
        self.imu_check.place(x=5, y=115)

        # Configuración de pantalla completa
        self.root.bind("<F11>", self.toggle_fullscreen)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def conexion(self):
        # Obtiene la hora actual y la utiliza para nombrar el archivo CSV
        hora_actual = datetime.datetime.now().strftime("%H.%M.%S.%f")
        # Ruta relativa para el archivo CSV
        nombre_archivo = os.path.join(current_folder, f"{hora_actual}.csv")

        # Inicializa las variables globales
        global Estado, Estado_reset, selec_imu
        Estado = 1
        Estado_reset = 0

        # Crea el archivo CSV
        with open(nombre_archivo, "w", newline="") as file:
            writer = csv.writer(file, delimiter=";", quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
            writer.writerow(["Hora", "Distancia", "GIROSCOPIO", "ACELEROMETRO", "MAGNETROMETRO"])

        # Inicializa la conexión serial
        if self.port_lista.get() and self.odometro_lista.get():
            if self.imu_var.get():
                # Inicializa la conexión serial para la IMU
                puertoSerial = serial.Serial(self.port_lista2.get(), 115200, timeout=2)
                puertoSerial.write("iniciar".encode())
                print("Conexión con IMU establecida")
                
                # Inicializa la conexión serial para el software sonar
                puertoSerial_b = serial.Serial("COM29", 115200, timeout=2)
                puertoSerial_b.write("iniciar".encode())
                print("Conexión con software sonar establecida")
            
            # Inicializa la conexión serial para la odometría
            puertoSerial_c = serial.Serial(self.port_lista.get(), 115200, timeout=2)

        else:
            # Si falta algún parámetro para la conexión serial, cambia el estado a 0
            Estado = 0

        Ti = ""
        pitch = "+00.0"
        roll = "+000.0"
        Distancia = "000.01"
        acelerometro = [0, 0, 0, 0, 0, 0]
        magnetometro = [0, 0, 0, 0, 0, 0]
        giroscopio = [0, 0, 0, 0, 0, 0]

        while Estado == 1:
            time.sleep(0.028)  # Tiempo de muestro
            ace = " "
            giro = " "
            mag = " "
            Ti = ""
            salto_de_linea = 0
            imu = ""   

            if self.imu_var.get() == 1:
                while salto_de_linea < 3: 
                    if puertoSerial.in_waiting > 0:
                        lectura = puertoSerial.readline()
                        if len(lectura) > 5:
                            imu = imu + lectura.decode('cp1252')
                            salto_de_linea = salto_de_linea + 1
                            time.sleep(0.002)  # Tiempo de muestro
                puertoSerial.write("OK".encode('utf-8'))

                if imu != "":
                    lista_IMU = imu.split('\n') 
                    if len(lista_IMU) >= 3:
                        # Separan los datos por cada sensor
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

                    ace = f"{acelerometro[1]};{acelerometro[3]};{acelerometro[4]};{acelerometro[5]}"
                    giro = f"{giroscopio[1]};{giroscopio[3]};{giroscopio[4]};{giroscopio[5]}"
                    mag = f"{magnetometro[1]};{magnetometro[3]};{magnetometro[4]};{magnetometro[5]}" 

            if puertoSerial_c.in_waiting > 0:
                Tics = puertoSerial_c.readline()
                if len(Tics) > 0:
                    Ti = "".join(filter(lambda x: x.isdigit(), str(Tics)))
                    if self.odometro_lista.get() == "Guia de cable":
                        Distancia = round((((int(Ti) * 0.0372 * 3.1416) / 1024) * 1), 2)
                    elif self.odometro_lista.get() == "Carrete":
                        Distancia = round((((int(Ti) * 0.0225 * 3.1416) / 1024) * 1.0216), 2)
                    string_DIstancia = str(Distancia).split(".")
                    if len(string_DIstancia[1]) < 2:
                        string_DIstancia[1] = string_DIstancia[1] + "0"
                    Distancia = f"{string_DIstancia[0]}.{string_DIstancia[1]}"

                if float(Distancia) >= 0:
                    if len(str(Distancia)) == 4:
                        Distancia = f"+000{str(Distancia)}"
                    elif len(str(Distancia)) == 5:
                        Distancia = f"+00{str(Distancia)}"
                    elif len(str(Distancia)) == 6:
                        Distancia = f"+0{str(Distancia)}"
                    elif len(str(Distancia)) == 7:
                        Distancia = f"+{str(Distancia)}"
                else:
                    Distancia = str(Distancia)
                    if len(str(Distancia)) == 5:
                        Distancia = Distancia[:1] + "000" + Distancia[1:]
                    elif len(str(Distancia)) == 6:
                        Distancia = Distancia[:1] + "00" + Distancia[1:]
                    elif len(str(Distancia)) == 7:
                        Distancia = Distancia[:1] + "0" + Distancia[1:]

            formato = f"$PITCH{pitch},ROLL{roll},DIST{Distancia}\r\n"
            
            with open(nombre_archivo, 'a', newline='') as archivo:
                escritor = csv.writer(archivo, delimiter=';', quotechar='"', quoting=csv.QUOTE_NONNUMERIC)
                hora_actual = datetime.datetime.now()
                escritor.writerow([hora_actual.time(), Distancia, giro, ace, mag])

            if self.imu_var.get() == 1:
                puertoSerial_b.write(formato.encode('utf-8'))

            self.distance_var.set(Distancia)

            if Estado_reset == 1:
                nueva_dis = f"reset{self.input_entry.get()}@"
                puertoSerial_c.write(nueva_dis.encode('utf-8'))
                Estado_reset = 0

            self.root.update()

        if self.imu_var.get() == 1:
            puertoSerial.flushInput()
            puertoSerial.close()
            puertoSerial_b.close()
        if self.port_lista.get() and self.odometro_lista.get():
            puertoSerial_c.flushInput()
            puertoSerial_c.close()

    def desconectar(self):
        global Estado
        Estado = 0

    def reset(self):
        global Estado_reset
        if Estado == 1:
            Estado_reset = 1

    def toggle_fullscreen(self, event=None):
        self.is_fullscreen = not self.is_fullscreen
        self.root.attributes("-fullscreen", self.is_fullscreen)
        return "break"

    def on_close(self):
        self.root.quit()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
