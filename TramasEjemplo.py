
#import threading
import logging
import time
import sys, os#, threading
import os.path
import threading
from threading import Thread

logging.basicConfig(level=logging.DEBUG, format= '[%(levelname)s] - %(theadName)-10s: %(message)s');
class Estructura:
    
    contador = 0
    
Datos = Estructura

def daemon():
    print("a=")
    print("b=")
    logging.debug('lanzado')
    time.sleep(2)
    logging.debug('Detenido')
    stepperSequenceToDrawALine.append(Datos.contador)
    Datos.contador+=1
    daemon2()

def daemon3():
    print("ZZZZ=")
    logging.debug('lanzado')
    time.sleep(0.01)
    logging.debug('Detenido')
    
def daemon2():
    stepperSequenceToDrawALine.append(Datos.contador)
    Datos.contador+=1
    print(stepperSequenceToDrawALine[0])
    stepperSequenceToDrawALine.pop(0) #.remove[0]]
    print(stepperSequenceToDrawALine[0])
    stepperSequenceToDrawALine.pop(0) 
    
    
stepperSequenceToDrawALine = []
d= Thread(target=daemon, name='Daemon')
d.setDaemon(True)
d.start()
c=Thread(target=daemon)
c.setDaemon(True)
c.start()
print("ya")
c.join()
d.join()
print("ya2")
f = Thread()
f = Thread(target=daemon)
f.start()
f.join()
class MiHilo(threading.Thread):
    
    def __init__(self, TramaNombre, codigo, nombre, edad):
        print("ya3__")
        threading.Thread.__init__(self) #(self, name=TramaNombrenombre, target=MiHilo.run )
    
    def run(self):
        #Aqui el codigo del hilo
        print("__ya3")
        time.sleep(1)
        print("ya3")

# Arranque del hilo
hilo = MiHilo("HiloMio","","","")
hilo.start()
print("ya4")



#def __init__(self, parametros):
#      Thread.__init__(self)


#class Linea_Proceso:
    


    
    