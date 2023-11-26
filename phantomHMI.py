import math
import os
import sys
from threading import Thread
import time
from tkinter import messagebox
import cv2
import numpy as np
import pandas as pd

from time import time

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.srv import DynamixelCommand

import tkinter as tk

# Nombres de los integrantes del grupo
integrantes = ["Giovanny", "Nicolas"]

'''Definicion de variables Variables'''
#torque de cada junta en bits
Torques=[800,1023,1023,1023,800]

#Definicion de posicion de home y pose
anghomcerrd = [0, 0, 0, -90, 54]
angd2 = [0, -90, 0, 0, 0]
angd3 = [10, -90, 0, 0, 0]
angd4 = [20, -90, 0, 0, 0]
angd5 = [30, -90, 0, 0, 0]
# Convierte los ángulos de grados a radianes
anghomecerrado = [math.radians(a) for a in anghomcerrd]
ang2 = [math.radians(a) for a in angd2]
ang3 = [math.radians(a) for a in angd3]
ang4 = [math.radians(a) for a in angd4]
ang5 = [math.radians(a) for a in angd5]

pointhome=anghomecerrado

estadoHerramienta=False
elapsed_time=0
'''Servicios'''
 
#Publisher para los motores
def joint_publisher(q,td,ts):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)

    #while not rospy.is_shutdown():
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    point = JointTrajectoryPoint()
    point.positions = [q[0], q[1],q[2],q[3],q[4]]    
    point.time_from_start = rospy.Duration(td)
    state.points.append(point)
    pub.publish(state)
    print('published command')
    rospy.sleep(ts)

#Subscriber para la posicion real
def callback(data):
    rospy.loginfo(data.position)
    print(data)
    global angpres
    angpres=data.position

    if data.position[4]>-2:
        print('gripper abierto')
    else:
        print('gripper cerrado')
    
def listener():
    #rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)
    #rospy.spin()


#Dynamixel service
def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

#Ajustar torque
def ajus_torque(T):
    #Definir los límites de torque de los motores.
    for i in range(5):    
        jointCommand('', (i+1), 'Torque_Limit', T[i], 0)    


#Movimiento de junta por junta
def movTotalPartes(Goal,td,ts):
    joint_publisher(Goal,td,ts)
    print('Finalizada la rutina.')
    print(Goal)


def leer_trayectorias():
    global pointhome
    global pointf1
    global pointf2
    global pointg1
    global pointg2
    global pointh1
    global pointh21
    global pointh22
    global pointh23
    global pointn1
    global pointn2
    global pointwi1
    global pointwi2
    global pointwo1
    global pointwo2

    package_dir = os.path.dirname(os.path.abspath(__file__))
    rutdathome= os.path.join(package_dir, "anexos/trayHome.csv")
    rutdath1= os.path.join(package_dir, "anexos/trayH1.csv")
    rutdath21= os.path.join(package_dir, "anexos/trayH21.csv")
    rutdath22= os.path.join(package_dir, "anexos/trayH22.csv")
    rutdath23= os.path.join(package_dir, "anexos/trayH23.csv")
    rutdatn1= os.path.join(package_dir, "anexos/trayN1.csv")
    rutdatn2= os.path.join(package_dir, "anexos/trayN2.csv")
    rutdatg1= os.path.join(package_dir, "anexos/trayG1.csv")
    rutdatg2= os.path.join(package_dir, "anexos/trayG2.csv")
    rutdatf1= os.path.join(package_dir, "anexos/trayF1.csv")
    rutdatf2= os.path.join(package_dir, "anexos/trayF2.csv")
    rutdatwo1= os.path.join(package_dir, "anexos/trayWO1.csv")
    rutdatwo2= os.path.join(package_dir, "anexos/trayWO2.csv")
    rutdatwi1= os.path.join(package_dir, "anexos/trayWI1.csv")
    rutdatwi2= os.path.join(package_dir, "anexos/trayWI2.csv")

    # Utiliza la función read_csv para cargar los datos en un DataFrame
    datahome = pd.read_csv(rutdathome, header=None,sep=',')
    dataf1 = pd.read_csv(rutdatf1, header=None,sep=',')
    dataf2 = pd.read_csv(rutdatf2, header=None,sep=',')
    datag1 = pd.read_csv(rutdatg1, header=None,sep=',')
    datag2 = pd.read_csv(rutdatg2, header=None,sep=',')
    datah1 = pd.read_csv(rutdath1, header=None,sep=',')
    datah21 = pd.read_csv(rutdath21, header=None,sep=',')
    datah22 = pd.read_csv(rutdath22, header=None,sep=',')
    datah23 = pd.read_csv(rutdath23, header=None,sep=',')
    datan1 = pd.read_csv(rutdatn1, header=None,sep=',')
    datan2 = pd.read_csv(rutdatn2, header=None,sep=',')
    datawi1 = pd.read_csv(rutdatwi1, header=None,sep=',')
    datawi2 = pd.read_csv(rutdatwi2, header=None,sep=',')
    datawo1 = pd.read_csv(rutdatwo1, header=None,sep=',')
    datawo2 = pd.read_csv(rutdatwo2, header=None,sep=',')
    
    datahome = (datahome.applymap(replace_commas))
    dataf1 = (dataf1.applymap(replace_commas))
    dataf2 = (dataf2.applymap(replace_commas))
    datag1 = (datag1.applymap(replace_commas))
    datag2 = (datag2.applymap(replace_commas))
    datah1 = (datah1.applymap(replace_commas))
    datah21 = (datah21.applymap(replace_commas))
    datah22 = (datah22.applymap(replace_commas))
    datah23 = (datah23.applymap(replace_commas))
    datan1 = (datan1.applymap(replace_commas))
    datan2 = (datan2.applymap(replace_commas))
    datawi1 = (datawi1.applymap(replace_commas))
    datawi2 = (datawi2.applymap(replace_commas))
    datawo1 = (datawo1.applymap(replace_commas))
    datawo2 = (datawo2.applymap(replace_commas))
    
    pointhome =(datahome.values[0])
    pointf1 =(dataf1.values[0])
    pointf2 =(dataf2.values)
    pointg1 =(datag1.values[0])
    pointg2 =(datag2.values)
    pointh1 =(datah1.values[0])
    pointh21 =(datah21.values)
    pointh22 =(datah22.values[0])
    pointh23 =(datah23.values)
    pointn1 =(datan1.values[0])
    pointn2 =(datan2.values)
    pointwi1 =(datawi1.values[0])
    pointwi2 =(datawi2.values)
    pointwo1 =(datawo1.values[0])
    pointwo2 =(datawo2.values)


def replace_commas(value):
    # Verificar si el valor es una cadena y contiene una coma
    if isinstance(value, str):
        # Reemplazar la coma por un punto y convertir a float
        return float(value.replace(',', '.'))
    else:
        # Devolver el valor sin cambios si no es necesario reemplazar
        return value




'''Definicion de las funciones para mover el robot '''

def gohome():
    global estadoHerramienta
    global pointh1
    global pointh21
    global pointh23
    global anghomecerrado
    global elapsed_time
    if(estadoHerramienta):
        start_time = time()
        movTotalPartes(pointh23[-1],1.5,3)
        for i in range(len(pointh23)):
            movTotalPartes(pointh23[len(pointh23)-i-1],0.4,0.6)
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointh23[len(pointh23)-i-1])+" Cargada")
            ventana.update
        rospy.sleep(5)
        movTotalPartes(pointh1,2,5)
        '''for i in range(len(pointh21),0):
            movTotalPartes(pointh21[i],0.8,1.2)
        '''
        rospy.sleep(5)    
        movTotalPartes(pointhome,1.5,3) 
        rospy.sleep(6)    
        movTotalPartes(pointhome,1.5,3)
        elapsed_time = time() - start_time

        if nodo_iniciado():
            print('Descargar Herramienta')
            mostrar_mensaje_estado("Herramienta descargada con éxito.")   
            mostrar_imagen(imagen_poshome, opcion_elegida.get())
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointhome[-1])+" Descargada")
            estadoHerramienta=False

        else:
            print('Error')
            mostrar_mensaje_estado("Paro de emergencia.")
            mostrar_imagen(imagen_stop, opcion_elegida.get())
            opcion_elegida.set(elapsed_time)

    else:
        mostrar_mensaje_estado("Herramienta ya esta Descargada.")


def accion1():
    global estadoHerramienta
    global pointh1
    global pointh21
    global pointh22
    global pointh23
    global anghomecerrado
    global elapsed_time


    if(estadoHerramienta):
        mostrar_mensaje_estado("Herramienta ya esta Cargada.")
    else:
        start_time = time()
        rospy.sleep(3)
        movTotalPartes(pointh1,1.5,3)
        rospy.sleep(6)                
        movTotalPartes([pointh21[-1][0],pointh21[-1][1],pointh21[-1][2],pointh21[-1][3],math.radians(53)],3,0.1)  
        rospy.sleep(20)
        respuesta = messagebox.askyesno("Confirmación", "¿Desea continuar con la segunda mitad de la acción?")
    
        if respuesta:
            print("Continuando con la segunda mitad de la acción...")
            movTotalPartes(pointh22,0.2,1) 
            movTotalPartes(pointh23[10],1,0.8)
            rospy.sleep(5)
            for i in range(10,len(pointh23)):
                movTotalPartes(pointh23[i],0.2,0.4)
                opcion_elegida.set(str(elapsed_time)+" ,"+str(pointh23[i])+" Cargada")
                ventana.update
            rospy.sleep(3)        
            movTotalPartes(anghomecerrado,1.5,3)    

            elapsed_time = time() - start_time
            if nodo_iniciado():
                print('Cargar Herramienta')
                mostrar_mensaje_estado("Herramienta cargada con éxito.")   
                mostrar_imagen(imagen_pose1, opcion_elegida.get())
                opcion_elegida.set(str(elapsed_time)+" ,"+str(anghomecerrado[-1])+" Cargada")
                estadoHerramienta=True

            else:
                print('Error')
                mostrar_mensaje_estado("Paro de emergencia.")
                mostrar_imagen(imagen_stop, opcion_elegida.get())
                opcion_elegida.set(elapsed_time)
        else:
            print("Acción interrumpida.")
            



def accion2():

    global estadoHerramienta
    global pointwi1
    global pointwi2
    global pointwo1
    global pointwo2
    global anghomecerrado
    global elapsed_time


    if(estadoHerramienta):
        start_time = time()
        movTotalPartes(pointwi1,1.5,3)
        for i in range(len(pointwi2)):
            movTotalPartes(pointwi2[i],0.2,0.8)
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointwi2[i])+" Cargada")   
            ventana.update     
        rospy.sleep(3)
        movTotalPartes(pointwo1,1.5,3)
        for i in range(len(pointwo2)):
            movTotalPartes(pointwo2[i],0.2,0.8)
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointwi2[i])+" Cargada")    
            ventana.update    

        rospy.sleep(3)
        movTotalPartes(anghomecerrado,1.5,3)   
        elapsed_time = time() - start_time         

        if nodo_iniciado():
            print('Espacio de trabajo')
            mostrar_mensaje_estado("Trayectoria Exitosa.")
            mostrar_imagen(imagen_pose2, opcion_elegida.get())
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointwi2[-1])+" Cargada")        
        else:
            print('Error')
            mostrar_mensaje_estado("Paro de emergencia.")
            mostrar_imagen(imagen_stop, opcion_elegida.get())
            opcion_elegida.set(elapsed_time)

    else:
        mostrar_mensaje_estado("Porfavor Cargue la herramienta.")


def accion3():
    global estadoHerramienta
    global pointn1
    global pointn2
    global pointg1
    global pointg2
    global anghomecerrado  
    global elapsed_time

    if(estadoHerramienta):
        start_time = time()
        movTotalPartes(pointn1,1.5,3)
        for i in range(8):
            movTotalPartes(pointn2[i],0.5,0.2)
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointn2[i])+" Cargada")        
            ventana.update
        rospy.sleep(4)
        for i in range(8,40):
            movTotalPartes(pointn2[i],0.2,0.8)  
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointn2[i])+" Cargada")        
            ventana.update
        rospy.sleep(4)
        for i in range(40,len(pointn2)):
            movTotalPartes(pointn2[i],0.4,0.3)
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointn2[i])+" Cargada")        
            ventana.update
        movTotalPartes(pointg1,1.5,3) 
        for i in range(10):
            movTotalPartes(pointg2[i],0.4,0.2) 
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointg2[i])+" Cargada")        
            ventana.update
        rospy.sleep(4)
        for i in range(10,len(pointg2)):
            movTotalPartes(pointg2[i],0.2,0.8)
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointg2[i])+" Cargada")        
            ventana.update
        rospy.sleep(4)
        movTotalPartes(anghomecerrado,2,4)
        elapsed_time = time() - start_time
        if nodo_iniciado():
            print('Has elegido dibujar Iniciales')
            mostrar_mensaje_estado("Trayectoria Exitosa.")
            mostrar_imagen(imagen_pose3, opcion_elegida.get())
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointg2[i])+" Cargada")        
        else:
            print('Error')
            mostrar_mensaje_estado("Paro de emergencia.")
            mostrar_imagen(imagen_stop, opcion_elegida.get())
            opcion_elegida.set(elapsed_time)

    else:
        mostrar_mensaje_estado("Porfavor Cargue la herramienta.")


def accion4():
    global estadoHerramienta
    global pointf1
    global pointf2
    global anghomecerrado
    global elapsed_time

    if(estadoHerramienta):
        start_time = time()
        movTotalPartes(pointf1,1.5,3)        
        for i in range(len(pointf2)):
            movTotalPartes(pointf2[i],0.1,0.7)
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointf2[i])+" Cargada")        
            ventana.update
        rospy.sleep(4)
        movTotalPartes(anghomecerrado,2,4)        

        elapsed_time = time() - start_time
        if nodo_iniciado():
            print('Has elegido figura libre')
            mostrar_mensaje_estado("Trayectoria Exitosa.")
            mostrar_imagen(imagen_pose4, opcion_elegida.get())
            opcion_elegida.set(str(elapsed_time)+" ,"+str(pointf2[i])+" Cargada")        
        else:
            print('Error')
            mostrar_mensaje_estado("Paro de emergencia.")
            mostrar_imagen(imagen_stop, opcion_elegida.get())
            opcion_elegida.set(elapsed_time)
    else:
        mostrar_mensaje_estado("Porfavor Cargue la herramienta.")


def leer_sensor():
    print('Has elegido leer')
    opcion_elegida.set(angpres)
    listener()

# Función para mostrar imágenes
def mostrar_imagen(imagen, opcion_anterior):
    imagen_label_pasada.config(image=imagen_label_actual.cget("image"))
    imagen_label_actual.config(image=imagen)
    opcion_anterior_label.config(text="Anterior: " + opcion_anterior)

# Función para mostrar mensajes de estado
def mostrar_mensaje_estado(mensaje):
    mensaje_estado.set(mensaje) 



# Función para el paro de emergencia
def paro_emergencia():
    respuesta = messagebox.askyesno("Confirmación", "¿Está seguro de activar el paro de emergencia?")
    if respuesta:

        #ventana.destroy()
        rospy.signal_shutdown("Cerrando el nodo de ROS")
        # Actualiza los botones al iniciar la ventana
        actualizar_botones()
        ventana.update()  
        ventana.destroy  
        sys.exit()  
        print("Paro de emergencia activado.")

def iniciar_ros():
    rospy.init_node('joint_listener', anonymous=True)

# Función para verificar si el nodo está iniciado
def nodo_iniciado():
    uri = rospy.get_node_uri()
    return bool(uri)


# Función para mostrar u ocultar el botón según el estado del nodo
def actualizar_botones():
    if nodo_iniciado():
        boton_iniciar_ros.pack_forget()  # Oculta el botón si el nodo está iniciado
    else:
        boton_iniciar_ros.pack()  # Muestra el botón si el nodo no está iniciado

'''Cracion de la interfaz grafica'''
# Crear una ventana principal
ventana = tk.Tk()
ventana.title("Selección de Opciones")

# Etiqueta para mostrar los nombres de los integrantes
nombres_label = tk.Label(ventana, text="Integrantes del grupo: " + ", ".join(integrantes))
nombres_label.pack()

# Cargar imágenes para cada opción

package_dir = os.path.dirname(os.path.abspath(__file__))
imagen_pathhom = os.path.join(package_dir, "anexos/poshome.png")
imagen_pathpose1 = os.path.join(package_dir, "anexos/pose1.png")
imagen_pathpose2 = os.path.join(package_dir, "anexos/pose2.png")
imagen_pathpose3 = os.path.join(package_dir, "anexos/pose3.png")
imagen_pathpose4 = os.path.join(package_dir, "anexos/pose4.png")
imagen_pathstop = os.path.join(package_dir, "anexos/stop.png")
imagen_pathdef = os.path.join(package_dir, "anexos/def.png")


# Crear una etiqueta para mostrar la imagen
# Crear una etiqueta grande para mostrar la imagen
imagen_def = tk.PhotoImage(file=imagen_pathdef)
imagen_poshome = tk.PhotoImage(file=imagen_pathhom)
imagen_pose1 = tk.PhotoImage(file=imagen_pathpose1)
imagen_pose2 = tk.PhotoImage(file=imagen_pathpose2)
imagen_pose3 = tk.PhotoImage(file=imagen_pathpose3)
imagen_pose4 = tk.PhotoImage(file=imagen_pathpose4)
imagen_stop = tk.PhotoImage(file=imagen_pathstop)


imagen_label = tk.Label(ventana, image=imagen_def)
#imagen_label.pack()


# Crear un Frame para contener las imágenes
imagen_frame = tk.Frame(ventana)
imagen_frame.pack()

# Crear etiquetas para mostrar las imágenes actual y pasada
imagen_label_actual = tk.Label(imagen_frame, image=imagen_def)
imagen_label_pasada = tk.Label(imagen_frame, image=imagen_def)
imagen_label_actual.grid(row=0, column=1)
imagen_label_pasada.grid(row=0, column=0)


# Crear una etiqueta para mostrar mensajes de estado
mensaje_estado = tk.StringVar()
mensaje_estado_label = tk.Label(ventana, textvariable=mensaje_estado, font=("Arial", 18), fg="red")
mensaje_estado_label.pack()
# Etiqueta para mostrar la opción anterior
opcion_anterior_label = tk.Label(ventana, text="Anterior:")
opcion_anterior_label.pack() 

# Crear una etiqueta para mostrar la opción elegida abajo a la derecha
opcion_elegida = tk.StringVar()
opcion_elegida_label = tk.Label(ventana, textvariable=opcion_elegida)
opcion_elegida_label.pack()

def accion1hilo():
    Thread(target=accion1).start()

def accion2hilo():
    Thread(target=accion2).start()

def accion3hilo():
    Thread(target=accion3).start()

def accion4hilo():
    Thread(target=accion4).start()

def accion5hilo():
    Thread(target=gohome).start()

boton1 = tk.Button(ventana, text="Cargar Herramienta", command=accion1hilo)
boton2 = tk.Button(ventana, text="Espacio de trabajo", command=accion2hilo)
boton3 = tk.Button(ventana, text="Dibujar Iniciales", command=accion3hilo)
boton4 = tk.Button(ventana, text="Dibujar Figura", command=accion4hilo)
boton5 = tk.Button(ventana, text="Descargar Herramienta", command=accion5hilo)

# Crear un botón de paro de emergencia
boton_emergencia = tk.Button(ventana, text="Paro de Emergencia", command=paro_emergencia, bg="red", fg="white")
boton_emergencia.pack(pady=20)
# Botón para iniciar rospy
boton_iniciar_ros = tk.Button(ventana, text="Iniciar rospy", command=iniciar_ros)
boton_iniciar_ros.pack()

# Ubicar los botones en la parte inferior izquierda
boton4.pack(side="right")
boton3.pack(side="right")
boton2.pack(side="right")
boton1.pack(side="right")
boton5.pack(side="left")

boton_emergencia.pack(side="left")





'''Funcion principal'''


def run():
    rospy.init_node('joint_listener', anonymous=True)
    #listener()
    ajus_torque(Torques)
    leer_trayectorias()
    movTotalPartes([0,0,0,0,0],1.5,3)
    rospy.sleep(0.5)
    movTotalPartes(pointhome,1.5,3)
    # Actualiza los botones al iniciar la ventana
    actualizar_botones()
    ventana.mainloop()



if __name__ == '__main__':
    while not rospy.is_shutdown():
        run()