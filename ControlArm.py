import smbus
import time, struct, math
import sys, os, threading
from threading import Thread, Semaphore
#from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QMessageBox, QTableWidget, QTableWidgetItem
#from PyQt5 import uic, QtCore, QtGui
import EEZYbotArm_Pi_I2C_kinematics
import serial.tools.list_ports
import os.path
import RPi.GPIO as gpio #https://pypi.python.org/pypi/RPi.GPIO more info
import logging
#from BufferLineas import bufferLine
#from 3DGcode_execute import 
#from 3DGcode_executer import ComandosLista

class ArraySecuencias:
    velocidad = 0
    ASstepperPositionsOnALine = []
    ASstepperSequenceToDrawALine = []
    PosicionX = 0
    PosicionY = 0
    PosicionZ = 0
    
    

class Control_V1:
    ComandosLista = [];
    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] (%(theadName]-s) %(message)s')
    #semaforo = Semaphore(1)
    #hilo= Control_V1.move_stepper_motors_using_line_step_sequence(semaforo) # move_stepper_motors_using_line_step_sequence(semaforo)
    hiloFin = True
    hiloOn = False
    
    debugInfo = False
    
    currentXPosition =0
    currentYPosition =0
    currentZPosition =0
    
    invalidPositionDetected = False
    invalidAngles = [0,0,0]#stores angles of invalid position detected [base, upper, lower]
        
    #incremental position variables (may change the code architecture and get rid fo these later)
    incrementalStepToX = 0
    incrementalStepToY = 0
    incrementalStepToZ = 0
    
    #stepperPositions holds the stepper motor step numbers at each point on a line. For example, a 200 step/rev
    #stepper motor without microstepping might have a position of 10, which is 10 steps away from its starting position
    #in some direction. -10 would be 10 steps in the opposite direction
    stepperPositionsOnALine = []

    #these variables keep track of each stepper motor's stepper position (see comments immediately above this one for
    # a description of stepper position)
    basePos = 0
    upperPos = 0
    lowerPos = 0

    #stepperSequenceToDrawALine holds the actual stepper sequence data for a line (specifies whether or not a stepper
    #should move). see the readme on github
    stepperSequenceToDrawALine = []



    #START STEPPER MOTOR SETTINGS
    #The NEMA 17 stepper motors that Dobot uses are 200 steps per revolution.
    stepperMotorStepsPerRevolution = 1019
    #I'm using a ramps 1.4 board with all 3 jumpers connected, which gives me a microstepping mode of 1/16.
    #In other words, the motor is set up so it takes 16 steps to move 1 of the default steps.
    #microstepping jumper guide for the a4988 stepper driver: https://www.pololu.com/product/1182
    baseMicrosteppingMultiplier = 1.0#1.0#1.9
    upperArmMicrosteppingMultiplier = 1.0 #0.9
    lowerArmMicrosteppingMultiplier = 1.0 #0.9
    """
    //The NEMA 17 stepper motors Dobot uses are connected to a planetary gearbox, the black cylinders.
    //It basically just means that the stepper motor is rotating a smaller gear. That smaller gear is in turn rotating a larger one.
    //The gears are set up such that rotating the smaller gear by some number of degrees rotates the larger one by a tenth of that number of degrees (10:1 ratio)
    //The bigger gears are actually moving the arm, so the number of steps is increased by a factor of 10 (the gear ratio).
    """
    stepperPlanetaryGearBoxMultiplier = 10
    
    baseActualStepsPerRevolution = 9171
    upperArmActualStepsPerRevolution = 9171
    lowerArmActualStepsPerRevolution = 9171
       
    
    #determines how many slices to slice up a linear movement path into. More slices means straighter and smoother
    #lines. may increase processing time and/or number of steps to take.
    linearLineResolution = 2000#1000

    #determines the maximum number of step sequence data (3-tuples) that one can send at a time. This determines
    #maximum packet size for sending data to the arduino. see the readme in the github repository for more on the
    #serial communication protocol. 1000 means can send 1000 3-tuples, or 3000 characters at a time. There is only
    #about 8000 characters worth of memory on the arduino!! It complains at storing around 5000 characters.
    #NOTE: If you change this, you MUST change the corresponding MAX_INPUT variable in the arduino firmware
    maximumStepSequenceTuplesToSendAtATime = 1#1
 
     
    Offset_x = 60
    Offset_y = -100
    Offset_z = 0
 
    Secuencias = []
    # class initialization function (initialize the GUI)
    #def __init__():
    def inicio():
        Control_V1.hiloFin = True
        Control_V1.Secuencias[:] = []
        # current position variables
        startXYZ = EEZYbotArm_Pi_I2C_kinematics.get_cartesian_coordinate_from_angles_using_forward_kinematics(0,90,0);
        print(startXYZ)
        Control_V1.update_position_based_on_cartesian_coordinate(startXYZ[0],startXYZ[1],startXYZ[2]);
        
    
        print(startXYZ)
        
        #This variable will hold the aqctual number of steps per revolution and is calculate by multiplying the three previous variables together.
        #calculate the actual number of steps it takes for each stepper motor to rotate 360 degrees
        Control_V1.baseActualStepsPerRevolution = Control_V1.stepperMotorStepsPerRevolution * Control_V1.baseMicrosteppingMultiplier * Control_V1.stepperPlanetaryGearBoxMultiplier;
        Control_V1.upperArmActualStepsPerRevolution = Control_V1.stepperMotorStepsPerRevolution * Control_V1.upperArmMicrosteppingMultiplier * Control_V1.stepperPlanetaryGearBoxMultiplier;
        Control_V1.lowerArmActualStepsPerRevolution = Control_V1.stepperMotorStepsPerRevolution * Control_V1.lowerArmMicrosteppingMultiplier * Control_V1.stepperPlanetaryGearBoxMultiplier;
        #END STEPPER MOTOR SETTINGS


        ###
        # General initialization code
        ###




        logging.info("Variables brazo cargadas")
        print("Posicion: X=" + str(Control_V1.currentXPosition) + " Y=" + str(Control_V1.currentYPosition) +" Z=" +  str(Control_V1.currentZPosition) )
        #Control_V1.d.start()


    #called when the move button in the move to coordinate group on the manual control tab is clicked
    #also called by the incremental step buttons. I will probably break this function up at some point.
    def MoveToCoordinate_XYZ(x_pos,stepx,y_pos,stepy,z_pos,stepz,speed):
        
        moveToX = x_pos + Control_V1.Offset_x #self.ui.lineEditMoveToX.text()
        moveToY = y_pos + Control_V1.Offset_y # x_posself.ui.lineEditMoveToY.text()
        moveToZ = z_pos + Control_V1.Offset_z #self.ui.lineEditMoveToZ.text()
        return Control_V1.Mueve(moveToX,moveToY,moveToZ,speed)
    



    #called when the move button in the move to coordinate group on the manual control tab is clicked
    #also called by the incremental step buttons. I will probably break this function up at some point.
    def MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,speed):
        print("Posicion X = ", x_pos," PosicionY = ", y_pos)
        moveToX = x_pos + Control_V1.Offset_x #self.ui.lineEditMoveToX.text()
        moveToY = y_pos + Control_V1.Offset_y # x_posself.ui.lineEditMoveToY.text()
        moveToZ = Control_V1.currentZPosition #self.ui.lineEditMoveToZ.text()
            #print("Comando")

        return Control_V1.Mueve(moveToX,moveToY,moveToZ,speed)
        #print("ok")


    #called when the move button in the move to coordinate group on the manual control tab is clicked
    #also called by the incremental step buttons. I will probably break this function up at some point.
    def MoveToCoordinate_Z(z_pos,speed):

        moveToX = Control_V1.currentXPosition #self.ui.lineEditMoveToX.text()
        moveToY = Control_V1.currentYPosition # x_posself.ui.lineEditMoveToY.text()
        moveToZ = z_pos + Control_V1.Offset_z #self.ui.lineEditMoveToZ.text()
        
        return Control_V1.Mueve(moveToX,moveToY,moveToZ,speed)
        
        
            
    def Mueve(moveToX,moveToY,moveToZ,speed):
        #initalize move to float coordinate values with dummy numbers easily detected if something goes wrong
        moveToXFloat = -992
        moveToYFloat = -992
        moveToZFloat = -992
            # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToXFloat = float(moveToX)
            moveToYFloat = float(moveToY)
            moveToZFloat = float(moveToZ)
        except Exception as e:
            passprint("Error coorcenadas")
##                self.show_a_warning_message_box('Check that your coordinate values are numbers and not letters. The code '
##                                                + 'error is shown below:',
##                                                    repr(e),
##                                                    'Coordinate value conversion to float error')
            return
        #called from an incremental step button





        # divide a line between starting and end points into many equally spaced points and move to each of those points,
        # so the arm moves in a straight line
        startingPointX = Control_V1.currentXPosition
        startingPointY = Control_V1.currentYPosition
        startingPointZ = Control_V1.currentZPosition
        #vector directions for vector equation of line
        directionX = moveToXFloat - startingPointX
        directionY = moveToYFloat - startingPointY
        directionZ = moveToZFloat - startingPointZ
        #determines how many slices to slice up the line in
        linearMovementResolution = Control_V1.linearLineResolution

        #clear the stepper sequence for the line and the stepper positions on the line
        Control_V1.stepperPositionsOnALine[:] = []
        Control_V1.stepperSequenceToDrawALine[:] = []

        #need to uncomment the debugging prints. commented to save time in the for loop
        #debugLineSlicing = self.masterDebug

        #iterate through the line slices (points on the line path for the arm to move along)
        #simulate moving to each sub point, using inverse kinematics in the move_to_cartesian_coordinate function
        #to return the corresponding angles to move to. within the move_to_cartesian function, the angles are sent
        #to another function to convert them to stepper positions. these are stored in the corresponding stepper position
        #variable referenced in the constructor of this class
        nextPointX = 0
        nextPointY = 0
        nextPointZ = 0
        for i in range(1, linearMovementResolution+1):
            nextPointX = startingPointX + (directionX * (i/linearMovementResolution))
            nextPointY = startingPointY + (directionY * (i/linearMovementResolution))
            nextPointZ = startingPointZ + (directionZ * (i/linearMovementResolution))
            #get the stepper position of each sub point on the line
            Control_V1.move_to_cartesian_coordinate(nextPointX, nextPointY, nextPointZ)
            if(Control_V1.invalidPositionDetected):
                #show the appropriate warning message(s)
                Control_V1.check_for_angle_limits_is_valid_with_warnings(Control_V1.invalidAngles[0],Control_V1.invalidAngles[1],Control_V1.invalidAngles[2])
                #reset the invalid variables
                Control_V1.invalidPositionDetected = False
                Control_V1.invalidAngles = [0,0,0]
                print ('invalid detected')
                return

            """
            if(debugLineSlicing):
                print('i:')
                print(i)
                print(nextPointX)
                print(nextPointY)
                print(nextPointZ)
            """
        #since the move has been determined to be valid and will be executed, update the current position and angles 
        Control_V1.update_position_based_on_cartesian_coordinate(nextPointX, nextPointY, nextPointZ)

        #from the list of stepper positions to move to, generate an actual step sequence. an algorithm is used
        #to take as evenly spaced steps as possible so that the movement is more linear
        Control_V1.generate_line_step_sequence()

        #move the motors
        
        Secuencia = ArraySecuencias()
        Secuencia.velocidad = speed
        
        Secuencia.ASstepperPositionsOnALine[:] = []
        Secuencia.ASstepperSequenceToDrawALine[:] = []
        
        Secuencia.ASstepperPositionsOnALine = list(Control_V1.stepperPositionsOnALine)
        Secuencia.ASstepperSequenceToDrawALine = list(Control_V1.stepperSequenceToDrawALine)
        Secuencia.PosicionX = moveToX
        Secuencia.PosicionY = moveToY
        Secuencia.PosicionZ = moveToZ
        Control_V1.Secuencias.append(Secuencia)
        print("Bufer secuencias ",len(Control_V1.ComandosLista))
        return Secuencia
##        Comando_new = SecuenciaComandos;
##        Comando_new.contador = 99999999;
##        Comando_new.
##        Comando_new.trama= Trama('G28',Comandos.G28);
##        ComandosLista.append(Comando_new);
        
        #Control_V1.move_stepper_motors_using_line_step_sequence(speed,Control_V1.semaforo)
        #Control_V1.move_stepper_motors_using_line_step_sequence()
        
        #hilo.start()
        #hilo=MiHilo(1,semaforo)
        #hilo.start()
        #while (not Control_V1.d):
           #pass
        
       # Control_V1.d.start()
        
##        if ( Control_V1.hiloFin == True ):
##            print("Control_V1.d.start() ")
##            #while (not Control_V1.d):
##            #pass
##            #Control_V1.d.join()
##            if(Control_V1.hiloOn==True):
##                Control_V1.tramaSecuencia.join()
##                
##            Control_V1.hiloFin = False
##            Control_V1.hiloOn = True
##            Control_V1.tramaSecuencia.start()
##             
##        else:
##            Control_V1.tramaSecuencia.join()
##            #Control_V1.tramaSecuencia.start()
            

    def stepper_motors_aceleracion(tiempo, lineas, posicion, pos_Acel, pos_Decel):
        if (debugInfo):
            print("Pulso")
 
        time.sleep(WaitTimeIni)
        
        if(posicion<pos_Acel): 
            tiempo = (WaitTimeIni - 0.001)
    
        if(posicion>pos_Decel):
            tiempo = (WaitTimeIni + 0.001)
        
    #where the stepper motors are actually moved
    def move_stepper_motors_using_line_step_sequence():
        #debugs the stepper motor movement code below
        #debug = True
        #semaforo.acquire()
        #Control_V1.semaforo.acquire()
        
        if (Control_V1.debugInfo):
            print(Control_V1.stepperSequenceToDrawALine[0:4])
        print("Bufer x",Control_V1.Secuencias.count)
        #print("x %s y %s z %s",Control_V1.Secuencias[0].PosicionX,Control_V1.Secuencias[0].PosicionY,Control_V1.Secuencias[0].PosicionZ)
        #dictates how fast the stepper motor moves (by changing the "pulse width", look it up if you want to know what that means
        WaitTime = 0.0012
        WaitTimeAcel = 0.0015
        PuslsosCuenta = len(Control_V1.Secuencias[0].ASstepperSequenceToDrawALine)# Se.stepperSequenceToDrawALine)
        PulsosAceleracion = 1
        PulsosDeceleracion = 1
        print('S %s' % PuslsosCuenta)
        print('V %s' % Control_V1.Secuencias[0].velocidad)
        if (PuslsosCuenta<1):
            PuslsosCuenta=1
        
        
        T = math.sqrt(PuslsosCuenta ** 2 + PuslsosCuenta ** 2) / Control_V1.Secuencias[0].velocidad #speed      #total time
        print('T  %s' % T)
        dt = (T / (PuslsosCuenta + PuslsosCuenta) )/4               #time delay every micro_step
        print('dt %s' % dt)
        print(dt)
        if ((PuslsosCuenta > 10) and (PuslsosCuenta < 25)):
            WaitTimeAcel = 0.00017
            PulsosAceleracion = 10
            PulsosDeceleracion = 5
        if (PuslsosCuenta > 25):
            WaitTimeAcel = 0.00026
            PulsosAceleracion = 15
            PulsosDeceleracion = 10
        PuntoDeceleracion = PuslsosCuenta - PulsosDeceleracion
        """
        #set the stepper motor directions
        gpio.output(20, self.stepperSequenceToDrawALine[1])#base direction
        gpio.output(23, self.stepperSequenceToDrawALine[2])#upper arm direction
        gpio.output(18, self.stepperSequenceToDrawALine[3])#lower arm direction
        """
        if (Control_V1.debugInfo):
            print(PuslsosCuenta)
            print("calculado")

        
        #loop through the step sequence of the current line to move along and take steps when necessary
        for i in range(1, (len(Control_V1.stepperSequenceToDrawALine)-1), 6):#note the -1 in the range limit, which accounts for the 'e' end character. Also note that it increments by 6 and starts at index 1 (2nd list item), after 's'
            #stepper_motors_aceleracion(WaitTimeIni, PuslsosCuenta,i,PulsosAceleracion,PulsosDeceleracion)
            time.sleep(dt)
           
            #if(i<9):
            #   time.sleep(WaitTimeIni)
            #   WaitTimeIni = (WaitTimeIni - 0.001)
            
            #base 
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i] == 1) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+3] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x03, 0x01)
                time.sleep(WaitTimeAcel +0.0001)
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i] == 0 )and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+3] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x03, 0x02)
                time.sleep(WaitTimeAcel +0.0001)
            #upper arm
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+1] == 1) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+4] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x01, 0x01)
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+1] == 0) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+4] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x01, 0x02)
            #lower arm 
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+2] == 1) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+5] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x02, 0x01)
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+2] == 0) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+5] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x02, 0x02)
            
            if(i<PulsosAceleracion):
                WaitTimeAcel = (WaitTimeAcel - 0.00005)
    
            if(i>PuntoDeceleracion):
                WaitTimeAcel = (WaitTimeAcel + 0.00005)
            
            time.sleep(WaitTimeAcel)
            #WaitTimeAcel = tiempo
            
        
        if (Control_V1.debugInfo):
            print("ok")
        print("z")
        print(Control_V1.currentZPosition)
        Control_V1.Secuencias.pop(0)
        #Control_V1.semaforo.release()
        Control_V1.hiloFin = True
        #Control_V1.d.join()
    def move_sequence():
        
        #print("Bufer x",3DG.Secuencias.count)
        #print("x %s y %s z %s",Control_V1.Secuencias[0].PosicionX,Control_V1.Secuencias[0].PosicionY,Control_V1.Secuencias[0].PosicionZ)
        #dictates how fast the stepper motor moves (by changing the "pulse width", look it up if you want to know what that means
        WaitTime = 0.0012
        WaitTimeAcel = 0.0015
        PuslsosCuenta = len(Control_V1.ComandosLista[0].SecuenciaLinea.ASstepperSequenceToDrawALine)# Se.stepperSequenceToDrawALine)
        PulsosAceleracion = 1
        PulsosDeceleracion = 1
        print('PuslsosCuenta %s' % PuslsosCuenta)
        print('Velocidad %s' % Control_V1.ComandosLista[0].SecuenciaLinea.velocidad)
        if (PuslsosCuenta<1):
            PuslsosCuenta=1
        
        
        T = math.sqrt(PuslsosCuenta ** 2 + PuslsosCuenta ** 2) / Control_V1.ComandosLista[0].SecuenciaLinea.velocidad #speed      #total time
        print('Tiempo  %s' % T)
        dt = (T / (PuslsosCuenta + PuslsosCuenta) )/4               #time delay every micro_step
        print('dt %s' % dt)
        print(dt)
        if ((PuslsosCuenta > 10) and (PuslsosCuenta < 25)):
            WaitTimeAcel = 0.00017
            PulsosAceleracion = 10
            PulsosDeceleracion = 5
        if (PuslsosCuenta > 25):
            WaitTimeAcel = 0.00026
            PulsosAceleracion = 15
            PulsosDeceleracion = 10
        PuntoDeceleracion = PuslsosCuenta - PulsosDeceleracion
        """
        #set the stepper motor directions
        gpio.output(20, self.stepperSequenceToDrawALine[1])#base direction
        gpio.output(23, self.stepperSequenceToDrawALine[2])#upper arm direction
        gpio.output(18, self.stepperSequenceToDrawALine[3])#lower arm direction
        """
        if (Control_V1.debugInfo):
            print(PuslsosCuenta)
            print("calculado")

        
        #loop through the step sequence of the current line to move along and take steps when necessary
        for i in range(1, (len(Control_V1.ComandosLista[0].SecuenciaLinea.ASstepperSequenceToDrawALine)-1), 6):#note the -1 in the range limit, which accounts for the 'e' end character. Also note that it increments by 6 and starts at index 1 (2nd list item), after 's'
            #stepper_motors_aceleracion(WaitTimeIni, PuslsosCuenta,i,PulsosAceleracion,PulsosDeceleracion)
            time.sleep(dt)
           
            #if(i<9):
            #   time.sleep(WaitTimeIni)
            #   WaitTimeIni = (WaitTimeIni - 0.001)
            
            #base 
            if ((Control_V1.ComandosLista[0].SecuenciaLinea.ASstepperSequenceToDrawALine[i] == 1) and (Control_V1.ComandosLista[0].SecuenciaLinea.ASstepperSequenceToDrawALine[i+3] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x03, 0x01)
                time.sleep(WaitTimeAcel +0.0001)
            if ((Control_V1.ComandosLista[0].SecuenciaLinea.ASstepperSequenceToDrawALine[i] == 0 )and (Control_V1.ComandosLista[0].SecuenciaLinea.ASstepperSequenceToDrawALine[i+3] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x03, 0x02)
                time.sleep(WaitTimeAcel +0.0001)
            #upper arm
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+1] == 1) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+4] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x01, 0x01)
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+1] == 0) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+4] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x01, 0x02)
            #lower arm 
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+2] == 1) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+5] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x02, 0x01)
            if ((Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+2] == 0) and (Control_V1.Secuencias[0].ASstepperSequenceToDrawALine[i+5] == 1)):
                smbus.SMBus(1).write_byte_data(0x07, 0x02, 0x02)
            
            if(i<PulsosAceleracion):
                WaitTimeAcel = (WaitTimeAcel - 0.00005)
    
            if(i>PuntoDeceleracion):
                WaitTimeAcel = (WaitTimeAcel + 0.00005)
            
            time.sleep(WaitTimeAcel)
            #WaitTimeAcel = tiempo
            
        
        if (Control_V1.debugInfo):
            print("ok")
        print("z")
        print(Control_V1.currentZPosition)
        Control_V1.Secuencias.pop(0)
        #Control_V1.semaforo.release()
        Control_V1.hiloFin = True
        #Control_V1.d.join()
        
    #note, this function doesn't really move to a cartesian anymore. just generate a stepper position sequence for moving along a line to a cartestian coordinate
    def move_to_cartesian_coordinate(moveToXFloat, moveToYFloat, moveToZFloat):
        # call inverse kinematics function to convert from cartesian coordinates to angles for Dobot arm
        # moveToAngles is a list of angles (type float) with the following order: [base angle, upper arm angle, lower arm angle]
        # catch any errors
        try:
            moveToAngles = EEZYbotArm_Pi_I2C_kinematics.convert_cartesian_coordinate_to_arm_angles(moveToXFloat,moveToYFloat,moveToZFloat,
            EEZYbotArm_Pi_I2C_kinematics.lengthUpperArm, EEZYbotArm_Pi_I2C_kinematics.lengthLowerArm, EEZYbotArm_Pi_I2C_kinematics.heightFromBase)
        except Exception as e:
            pass
##            self.show_a_warning_message_box('Unknown inverse kinematics error. Check that your coordinate values are within the robot\'s range. '
##                                            + 'The error is shown below:',
##                                                repr(e),
##                                                'Inverse Kinematics Error')
##            return


        # check that inverse kinematics did not run into a range error. If it does, it should return -999 for all angles, so check that.
##        if(moveToAngles[0] == -999):
            
##            self.show_a_warning_message_box('Desired coordinate is outside of the robot\'s range.',
##                                                'It is impossible for the robot arm to reach the coordinate you specified. Build longer arms if this range is desired.'
##                                                + 'You will probably need higher torque stepper motors as well.',
##                                                'Inverse Kinematics Range Error')
##            return



        #these next 4 lines of code transform angles returned from the inverse kinematics code to the correct axes that
        #I defined for the dobot.
        moveToUpperArmAngleFloat = moveToAngles[1]
        moveToLowerArmAngleFloat = moveToAngles[2]

        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        #-90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
        # note that this line is different from the similar line in the move angles function. Has to do with the inverse kinematics function
        # and the fact that the lower arm angle is calculated relative to the upper arm angle.
        transformedLowerArmAngle = 360 + (transformedUpperArmAngle - moveToLowerArmAngleFloat) - 90

        """
        debug = False
        if(debugInfodebug):
            print('ik base angle')
            print(moveToAngles[0])
            print('ik upper angle')
            print(moveToAngles[1])
            print('ik lower angle')
            print(moveToAngles[2])
            print('transformed upper angle:')
            print(transformedUpperArmAngle)
            print('transformed lower angle:')
            print(transformedLowerArmAngle)
        """


        #check that the final angles are mechanically valid. note that this check only considers final angles, and not angles while the arm is moving
        # need to pass in real world angles
        # real world base and upper arm angles are those returned by the ik function.
        # real world lower arm angle is -1 * transformedLowerArmAngle   SUSPICIOUS, DOES THIS CAUSE ANY ERRORS?
        if(Control_V1.check_for_angle_limits_is_valid(moveToAngles[0], moveToAngles[1], -1 * transformedLowerArmAngle)):
            # continue on to execute the move
            pass
        else:
            # exit, don't move. set a flag that indicates an invalid move was detected
            Control_V1.invalidPositionDetected = True
            Control_V1.invalidAngles = [moveToAngles[0], moveToAngles[1], -1 * transformedLowerArmAngle]
            return

        #converts the angles to stepper positions and updates the corresponding self variable
        Control_V1.convert_angles_to_stepper_positions(moveToAngles[0],transformedUpperArmAngle,transformedLowerArmAngle)


    def update_position_based_on_cartesian_coordinate(moveToXFloat, moveToYFloat, moveToZFloat):
        #don't need error checking here since I already know these are valid coordinates to use with inverse kinematics
        
        # call inverse kinematics function to convert from cartesian coordinates to angles for Dobot arm
        # moveToAngles is a list of angles (type float) with the following order: [base angle, upper arm angle, lower arm angle]
        moveToAngles = EEZYbotArm_Pi_I2C_kinematics.convert_cartesian_coordinate_to_arm_angles(moveToXFloat,moveToYFloat,moveToZFloat,
        EEZYbotArm_Pi_I2C_kinematics.lengthUpperArm, EEZYbotArm_Pi_I2C_kinematics.lengthLowerArm, EEZYbotArm_Pi_I2C_kinematics.heightFromBase)
       
        #these next 4 lines of code transform angles returned from the inverse kinematics code to the correct axes that
        #I defined for the dobot.
        moveToUpperArmAngleFloat = moveToAngles[1]
        moveToLowerArmAngleFloat = moveToAngles[2]

        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        #-90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
        # note that this line is different from the similar line in the move angles function. Has to do with the inverse kinematics function
        # and the fact that the lower arm angle is calculated relative to the upper arm angle.
        transformedLowerArmAngle = 360 + (transformedUpperArmAngle - moveToLowerArmAngleFloat) - 90

        """
        debug = False
        if(debug):
            print('ik base angle')
            print(moveToAngles[0])
            print('ik upper angle')
            print(moveToAngles[1])
            print('ik lower angle')
            print(moveToAngles[2])
            print('transformed upper angle:')
            print(transformedUpperArmAngle)
            print('transformed lower angle:')
            print(transformedLowerArmAngle)
        """

        # since movement was successful, update the current position
        # note that float values are rounded to 3 decimal places for display and converted to strings
##        self.ui.labelBaseAngleValue.setText(str(round(moveToAngles[0],3)))
##        self.ui.labelUpperArmAngleValue.setText(str(round(moveToAngles[1],3)))
##        self.ui.labelLowerArmAngleValue.setText(str(round(moveToAngles[2],3)))
##        self.currentBaseAngle = moveToAngles[0]
##        self.currentUpperArmAngle = moveToAngles[1]
##        self.currentLowerArmAngle = moveToAngles[2]
##
##        self.ui.labelCurrentXValue.setText(str(round(moveToXFloat,3)))
##        self.ui.labelCurrentYValue.setText(str(round(moveToYFloat,3)))
##        self.ui.labelCurrentZValue.setText(str(round(moveToZFloat,3)))
        Control_V1.currentXPosition = moveToXFloat
        Control_V1.currentYPosition = moveToYFloat
        Control_V1.currentZPosition = moveToZFloat


    
    def convert_angles_to_stepper_positions(baseAngle,upperArmAngle,lowerArmAngle):

        baseStepNumber = int(( (abs(baseAngle)/360) * Control_V1.baseActualStepsPerRevolution ) + 0.5)
        #need this because of the abs value function, which is needed for proper rounding
        if (baseAngle < 0):
            baseStepNumber *= -1



        upperArmStepNumber = int(( (abs(upperArmAngle)/360) * Control_V1.upperArmActualStepsPerRevolution ) + 0.5)
        #need this because of the abs value function, which is needed for proper rounding
        if (upperArmAngle < 0):
            upperArmStepNumber *= -1


        lowerArmStepNumber = int(( (abs(lowerArmAngle)/360) * Control_V1.lowerArmActualStepsPerRevolution ) + 0.5)
        #need this because of the abs value function, which is needed for proper rounding
        if (lowerArmAngle < 0):
            lowerArmStepNumber *= -1

        #necessary to reverse the direction in which the steppers move, so angles match my defined angles
        baseStepNumber *= -1
        upperArmStepNumber *= -1
        lowerArmStepNumber *= 1

        Control_V1.stepperPositionsOnALine.append(baseStepNumber)
        Control_V1.stepperPositionsOnALine.append(upperArmStepNumber)
        Control_V1.stepperPositionsOnALine.append(lowerArmStepNumber)

        #debug = self.masterDebug
        if(Control_V1.debugInfo):
            print("Base Angle")
            print(baseAngle)
            print("Base Step Number")
            print(baseStepNumber)
            print("Upper Arm Angle")
            print(upperArmAngle)
            print("Upper Arm Step Number")
            print(upperArmStepNumber)
            print("Lower Arm Angle")
            print(lowerArmAngle)
            print("Lower Arm Step Number")
            print(lowerArmStepNumber)


    #creates the line data as specified in the readme in the github repository
    def generate_line_step_sequence():

        Control_V1.stepperSequenceToDrawALine.append('s')

        #initialize variables to hold stepper directions, NOTE that if for some reason they are not set and are left at -1, there should be a gpio error (don't think you can send -1 volts)
        baseDir = -1
        upperArmDir = -1
        lowerArmDir = -1


        #generating the step sequences of a line from the step positions, NOTE the increment by 3
        for j in range(0,len(Control_V1.stepperPositionsOnALine),3):
    
            #it is NOT true that moving along a line means that the steppers will always be moving in the same direction!!!
            #NEED TO SET DIRECTION OF STEPPERS FOR EACH STEP! NEED 6-TUPLES (DIRECTION AND STEP) 
            #set the direction of the steppers.
            #base direction
            if((Control_V1.stepperPositionsOnALine[j] - Control_V1.basePos) > 0):
                baseDir = 1
            else:
                baseDir = 0
            #upper arm direction
            if((Control_V1.stepperPositionsOnALine[j+1] - Control_V1.upperPos) > 0):
                upperArmDir = 1
            else:
                upperArmDir = 0
            #lower arm direction
            if((Control_V1.stepperPositionsOnALine[j+2] - Control_V1.lowerPos) > 0):
                lowerArmDir = 1
            else:
                lowerArmDir = 0

            
            Control_V1.generate_step_sequence_from_position_to_position_simple(abs(Control_V1.stepperPositionsOnALine[j] - Control_V1.basePos),
                                                                  abs(Control_V1.stepperPositionsOnALine[j+1] - Control_V1.upperPos),
                                                                  abs(Control_V1.stepperPositionsOnALine[j+2] - Control_V1.lowerPos),
                                                                  baseDir, upperArmDir, lowerArmDir)
            
            #updating each stepper's step position. necessary for the proper step sequence (with directions) to be generated correctly
            Control_V1.basePos = Control_V1.stepperPositionsOnALine[j]
            Control_V1.upperPos = Control_V1.stepperPositionsOnALine[j+1]
            Control_V1.lowerPos = Control_V1.stepperPositionsOnALine[j+2]

        Control_V1.stepperSequenceToDrawALine.append('e')
        
        
        if(Control_V1.debugInfo):
            print('Stepper Sequence')
            print(Control_V1.stepperSequenceToDrawALine)

    
    def generate_step_sequence_from_position_to_position_simple( numBaseSteps, numUpperArmSteps, numLowerArmSteps, baseDir, upperArmDir, lowerArmDir):
        #of the 3 stepper motors determine which one requires the most steps
        max_steps = float(max(max(numBaseSteps, numUpperArmSteps), numLowerArmSteps))#needs to be float so the rounding in the divisions work

        if(max_steps == 0):
            return

        
        debugNumBaseStepsExpected = numBaseSteps
        debugNumUpperArmStepsExpected = numUpperArmSteps
        debugNumLowerArmStepsExpected = numLowerArmSteps
        debugcounterBase = 0
        debugcounterUpper = 0
        debugcounterLower = 0
        
        
        for i in range(0, int(max_steps)):
            #append the step directions. NOTE that this is INEFFICENT (though might not matter) and CAN BE IMPROVED, but is simple and easy to implement for now. 
            Control_V1.stepperSequenceToDrawALine.append(baseDir)
            Control_V1.stepperSequenceToDrawALine.append(upperArmDir)
            Control_V1.stepperSequenceToDrawALine.append(lowerArmDir)
            
            
            #append the steps
            if (numBaseSteps != 0):
                Control_V1.stepperSequenceToDrawALine.append(1)
                numBaseSteps -= 1
                debugcounterBase += 1
            else:
                Control_V1.stepperSequenceToDrawALine.append(0)

            if (numUpperArmSteps != 0):
                Control_V1.stepperSequenceToDrawALine.append(1)
                numUpperArmSteps -= 1
                debugcounterUpper += 1
            else:
                Control_V1.stepperSequenceToDrawALine.append(0)

            if (numLowerArmSteps != 0):
                Control_V1.stepperSequenceToDrawALine.append(1)
                numLowerArmSteps -= 1
                debugcounterLower += 1
            else:
                Control_V1.stepperSequenceToDrawALine.append(0)

        #sanity check
        if(numBaseSteps < 0 or numUpperArmSteps < 0 or numLowerArmSteps < 0):
            Control_V1.show_a_warning_message_box('something went wrong in the simple generate step sequence from position to position function',
                                            'one of the step counters went negative. that shouldn\'t happen',
                                            'Step sequence generation error')
        """
        print('step sequence generation debug report')
        print('num base steps taken: ' + str(debugcounterBase))
        print('num base steps expected: ' + str(debugNumBaseStepsExpected))
        print('num upper steps taken: ' + str(debugcounterUpper))
        print('num upper steps expected: ' + str(debugNumUpperArmStepsExpected))
        print('num lower steps taken: ' + str(debugcounterLower))
        print('num lower steps expected: ' + str(debugNumLowerArmStepsExpected))
        """

    

    # the algorithm here is fairly complicated. Just know that it tries to generate as linear a path between two points
    # as possible by dividing the steps as evenly as possible among each stepper motor.
    def generate_step_sequence_from_position_to_position(numBaseSteps,numUpperArmSteps,numLowerArmSteps):


        #of the 3 stepper motors determine which one requires the most steps
        max_steps = float(max(max(numBaseSteps, numUpperArmSteps), numLowerArmSteps))#needs to be float so the rounding in the divisions work

        if(max_steps == 0):
            return

        baseStepSpace = max_steps+1
        upperStepSpace = max_steps+1
        lowerStepSpace = max_steps+1

        baseRemainder = 0
        upperRemainder = 0
        lowerRemainder = 0


        if(numBaseSteps != 0):
          baseStepSpace = int((max_steps / numBaseSteps) +.5)#round to nearest int
          baseRemainder = int(max_steps/baseStepSpace)#round to lowest int
          baseRemainder = numBaseSteps - baseRemainder#finish the calculation

        if(numUpperArmSteps != 0):
          upperStepSpace = int((max_steps / numUpperArmSteps) + 0.5)#round to nearest int
          upperRemainder = int(max_steps/upperStepSpace)#round to lowest int
          upperRemainder = numUpperArmSteps - upperRemainder#finish the calculation

        if(numLowerArmSteps != 0):
          lowerStepSpace = int((max_steps / numLowerArmSteps) + .5)#round to nearest int
          lowerRemainder = int(max_steps/lowerStepSpace)#round to lowest int
          lowerRemainder = numLowerArmSteps - lowerRemainder#finish the calculation

        """
        print(baseStepSpace)
        print(upperStepSpace)
        print(lowerStepSpace)
        print(baseRemainder)
        print(upperRemainder)
        print(lowerRemainder)
        """

        baseStepSpacei = 0
        upperStepSpacei = 0
        lowerStepSpacei = 0

        baseTaken = 0
        upperTaken = 0
        lowerTaken = 0

        baseflag = False
        upperflag = False
        lowerflag = False

        #step the motors at the same time by moving them 1 step at essentially the same time
        #Must alternate between HIGH and LOW signals to step the motors. I don't know the physics of why though. Just look up one of the million tutorials on stepper motors if you're curious why.

        for i in range(0,int(max_steps)):
            if ((baseStepSpacei == 0) and (baseTaken < numBaseSteps)):
                #digitalWrite(baseStepPin, HIGH);//only step the motor if it has more steps remaining to take
                baseflag = True
            if ((upperStepSpacei == 0) and (upperTaken < numUpperArmSteps)):
                #digitalWrite(upperArmStepPin, HIGH);
                upperflag = True
            if ((lowerStepSpacei == 0) and (lowerTaken < numLowerArmSteps)):
                #digitalWrite(lowerArmStepPin, HIGH);
                lowerflag = True

                #delay(1);

            if (baseflag):
                #digitalWrite(baseStepPin, LOW);
                baseStepSpacei = baseStepSpace
                baseTaken += 1
                baseflag = False
                self.stepperSequenceToDrawALine.append(1)
            else:
                self.stepperSequenceToDrawALine.append(0)
            if (upperflag):
                #digitalWrite(upperArmStepPin, LOW);
                upperStepSpacei = upperStepSpace
                upperTaken += 1
                upperflag = False
                Control_V1.stepperSequenceToDrawALine.append(1)
            else:
                Control_V1.stepperSequenceToDrawALine.append(0)
            if (lowerflag):
                #digitalWrite(lowerArmStepPin, LOW);
                lowerStepSpacei = lowerStepSpace
                lowerTaken += 1
                lowerflag = False
                Control_V1.stepperSequenceToDrawALine.append(1)
            else:
                Control_V1.stepperSequenceToDrawALine.append(0)
            baseStepSpacei -= 1
            upperStepSpacei -= 1
            lowerStepSpacei -= 1

        ###
        ###
        ###
        ###
        #I think that not adressing this might result in some small systematic error that might build up over a long time.
        #I really need to address this, but I'm out of time and the current algorithm gives a close enough approximation for testing.
        #NEED TO ADDRESS THIS!!!!!!
        ###
        ###
        ###
        """
        #take any remaining steps
        for i in range(0,baseRemainder):
            #digitalWrite(baseStepPin, HIGH);
            #delay(1);
            #digitalWrite(baseStepPin, LOW);
            baseTaken += 1


        for i in range(0,upperRemainder):
            #digitalWrite(upperArmStepPin, HIGH);
            #delay(1);
            #digitalWrite(upperArmStepPin, LOW);
            upperTaken += 1

        for i in range(0,lowerRemainder):
            #digitalWrite(lowerArmStepPin, HIGH);
            #delay(1);
            #digitalWrite(lowerArmStepPin, LOW);
            lowerTaken += 1


        print(baseTaken)
        print(upperTaken)
        print(lowerTaken)
        """

        """
        a,b,c = -1

        for i in range(0,len(self.stepperSequenceToDrawALine),3):
            a = self.stepperSequenceToDrawALine[i]
            b = self.stepperSequenceToDrawALine[i+1]
            c = self.stepperSequenceToDrawALine[i+2]
            print(a)
            print(b)
            print(c)
        """








    #DO NOT CALL THE MOVE ANGLES FUNCTION (DO NOT CLICK THE MOVE ANGLES BUTTON) AS OF NOW,
    #THIS IS OLD CODE THAT DID WORK. I DRASTICALLY ALTERED THE STRUCTURE OF THE SOFTWARE, SO THIS ALMOST CERTAINLY DOESN'T WORK NOW.
    #UNPREDICTABLE BEHAVIOR WILL RESULT. I NEED TO ADDRESS THIS.

    def pushButtonMoveToAngles_clicked(self):
        
        #base
        smbus.SMBus(1).write_byte_data(0x07, 0x03, 0x03)
        #upper arm
        smbus.SMBus(1).write_byte_data(0x07, 0x01, 0x03)
        #lower arm 
        smbus.SMBus(1).write_byte_data(0x07, 0x02, 0x03)
        
        # get moveTo angle text values from lineedits
        moveToBaseAngle = self.ui.lineEditMoveToBaseAngle.text()
        moveToUpperArmAngle = self.ui.lineEditMoveToUpperArmAngle.text()
        moveToLowerArmAngle = self.ui.lineEditMoveToLowerArmAngle.text()

        # check that the values were not empty
        if (moveToBaseAngle == '' or moveToUpperArmAngle == '' or moveToLowerArmAngle == ''):
            self.show_a_warning_message_box('Missing a angle value.',
                                            'Check that you entered a value for each angle.',
                                            'Invalid angles for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToBaseAngleFloat = float(moveToBaseAngle)
            moveToUpperArmAngleFloat = float(moveToUpperArmAngle)
            moveToLowerArmAngleFloat = float(moveToLowerArmAngle)
        except Exception as e:
            self.show_a_warning_message_box('Check that your angle values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Angle value conversion to float error')
            return


        #check that the final angles are mechanically valid. note that this check only considers final angles, and not angles while the arm is moving
        # need to pass in real world angles
        # real world base, upper, and lower arm angles are those entered in the text box.
        if(self.check_for_angle_limits_is_valid(moveToBaseAngleFloat, moveToUpperArmAngleFloat, moveToLowerArmAngleFloat)):
            # continue on to execute the arduino code
            pass
        else:
            # exit, don't move. the check function takes care of the warning message
            return


        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        transformedLowerArmAngle = moveToLowerArmAngleFloat*-1
        print('transformed upper angle:')
        print(transformedUpperArmAngle)
        print('transformed lower angle:')
        print(transformedLowerArmAngle)

        # INSERT CODE HERE TO SEND MOVEMENT COMMANDS TO ARDUINO
        # I'm simply writing three floats to the arduino. See the following two stack exchange posts for more details on this:
        # http://arduino.stackexchange.com/questions/5090/sending-a-floating-point-number-from-python-to-arduino
        # ttps://arduino.stackexchange.com/questions/3753/how-to-send-numbers-to-arduino-uno-via-python-3-and-the-module-serial
        """
        self.arduinoSerial.write( struct.pack('f',moveToBaseAngleFloat) )
        self.arduinoSerial.write( struct.pack('f',transformedUpperArmAngle) )
        self.arduinoSerial.write( struct.pack('f',transformedLowerArmAngle) )
        """

        # if movement was successful, update the current position
        # note that float values are rounded to 3 decimal places for display and converted to strings
        self.ui.labelBaseAngleValue.setText(str(round(moveToBaseAngleFloat,3)))
        self.ui.labelUpperArmAngleValue.setText(str(round(moveToUpperArmAngleFloat,3)))
        self.ui.labelLowerArmAngleValue.setText(str(round(moveToLowerArmAngleFloat,3)))

        """
        # need to implement forward kinematics
        self.ui.labelCurrentXValue.setText(str(round(moveToXFloat,3)))
        self.ui.labelCurrentYValue.setText(str(round(moveToYFloat,3)))
        self.ui.labelCurrentZValue.setText(str(round(moveToZFloat,3)))
        """

        """
        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        for i in range(0,15 ):
            print ( self.arduinoSerial.readline() )
        """
    # this version of the check for valid angles function displays warning messages if it detects invalid angles. non-warning version used to avoid excessive warnings while generating a line step sequence
    # angles passed as arguments here should be real world angles (horizontal = 0, below is negative, above is positive)
    # i.e. they should be set up the same way as the unit circle is
    def check_for_angle_limits_is_valid_with_warnings(baseAngle, upperArmAngle, lowerArmAngle):

        returnBool = True
        # implementing limit switches and IMUs will make this function more accurate and allow the user to calibrate the limits
        # necessary for this function.
        # Not currently checking the base angle

        # check the upperArmAngle
        # max empirically determined to be around 107 - 108 degrees. Using 105.
        # min empirically determined to be around -23/24 degrees. Using -20.
        if (-20 <= upperArmAngle <= 105):
            # do nothing, return value already initialized true
            pass
        else:
            #self.show_a_warning_message_box('Upper arm angle out of range.',
            #                                'Upper arm must have an angle between -20 and 105 degrees. It is mechanically constrained.',
            #                                'Upper Arm Range Error')
            returnBool = False

        # check the lowerArmAngle
        # the valid Lower Arm angle is dependent on the upper arm angle. The real world angle of the lower arm (0 degrees = horizontal) needs to be evaluated.
        # min empirically determined to be around -105 degrees. Using -102.
        # max empirically determined to be around 21 degrees. Using 18.


        if (-102 <= lowerArmAngle <= 18):
            # do nothing, already initialized true
            pass
        else:
            #self.show_a_warning_message_box('Lower arm angle out of range.',
            #                                'Lower arm must have a real world angle between -102 and 18 degrees. It is mechanically constrained.',
            #                                'Lower Arm Range Error')
            returnBool = False



        minAngleBetweenArms = ((180 - 81) + -79)
        maxAngleBetweenArms = ((180 - 51) + 21)
        angleBetweenArms = ((180 - upperArmAngle) + lowerArmAngle)

        if (minAngleBetweenArms <= angleBetweenArms <= maxAngleBetweenArms):
            # do nothing, already initialized true
            pass
        else:
            #self.show_a_warning_message_box('Angle between arms out of range out of range.',
            #                                'Angle between arms (the inner "elbow" angle) must be between ' +
            #                                str(minAngleBetweenArms) + ' and ' + str(maxAngleBetweenArms) + '.' +
            #                                ' It is mechanically constrained.',
            #                                'Inner Elbow Angle Range Error')
            returnBool = False


        return returnBool
    
    #no warning messages shown, just returns false if invalid angles. used to avoid excessive warnings while generating a line step sequence
    # angles passed as arguments here should be real world angles (horizontal = 0, below is negative, above is positive)
    # i.e. they should be set up the same way as the unit circle is
    def check_for_angle_limits_is_valid(baseAngle, upperArmAngle, lowerArmAngle):

        returnBool = True
        # implementing limit switches and IMUs will make this function more accurate and allow the user to calibrate the limits
        # necessary for this function.
        # Not currently checking the base angle

        # check the upperArmAngle
        # max empirically determined to be around 107 - 108 degrees. Using 105.
        # min empirically determined to be around -23/24 degrees. Using -20.
        if (-20 <= upperArmAngle <= 105):
            # do nothing, return value already initialized true
            pass
        else:
            returnBool = False

        # check the lowerArmAngle
        # the valid Lower Arm angle is dependent on the upper arm angle. The real world angle of the lower arm (0 degrees = horizontal) needs to be evaluated.
        # min empirically determined to be around -105 degrees. Using -102.
        # max empirically determined to be around 21 degrees. Using 18.


        if (-102 <= lowerArmAngle <= 18):
            # do nothing, already initialized true
            pass
        else:
            returnBool = False



        minAngleBetweenArms = ((180 - 81) + -79)
        maxAngleBetweenArms = ((180 - 51) + 21)
        angleBetweenArms = ((180 - upperArmAngle) + lowerArmAngle)

        if (minAngleBetweenArms <= angleBetweenArms <= maxAngleBetweenArms):
            # do nothing, already initialized true
            pass
        else:
            returnBool = False


        return returnBool








    #functions for the incremental step buttons
    #these functions are messed up for small step sizes, like 1. possibly due to the unaddressed linear move algorithm. NEED TO ADDRESS THIS
    def pushButtonStepForward_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepForwardSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition + stepSizeFloat
            self.incrementalStepToY = self.currentYPosition
            self.incrementalStepToZ = self.currentZPosition
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepBackward_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepBackwardSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition - stepSizeFloat
            self.incrementalStepToY = self.currentYPosition
            self.incrementalStepToZ = self.currentZPosition
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepLeft_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepLeftSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition
            self.incrementalStepToY = self.currentYPosition - stepSizeFloat
            self.incrementalStepToZ = self.currentZPosition
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepRight_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepRightSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition
            self.incrementalStepToY = self.currentYPosition + stepSizeFloat
            self.incrementalStepToZ = self.currentZPosition
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepUp_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepUpSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition
            self.incrementalStepToY = self.currentYPosition
            self.incrementalStepToZ = self.currentZPosition + stepSizeFloat
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepDown_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepDownSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition
            self.incrementalStepToY = self.currentYPosition
            self.incrementalStepToZ = self.currentZPosition - stepSizeFloat
            self.incrementalStepToZ = self.currentZPosition - stepSizeFloat
            self.pushButtonMoveToCoordinate_clicked(True)

    #error checking for grabbing the step size from the incremental step boxes
    def convert_step_size_numerical_text_to_number_plus_check_is_valid(self, stepSizeText):
        # check that the values were not empty
        if (stepSizeText == ''):
            self.show_a_warning_message_box('No step size value was entered.',
                                            'Check that you entered a value for the size of the step you tried to take.',
                                            'No step size value entered.')
            return None

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            stepSizeFloat = float(stepSizeText)
        except Exception as e:
            self.show_a_warning_message_box('Check that your step size values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Step size value conversion to float error')
            return None

        if (stepSizeFloat < 0):
            self.show_a_warning_message_box('Step sizes can only be positive.',
                                            'You entered a negative step size. Please enter a positive one. The button determines the direction.',
                                            'Invalid Step Size')
            return None

        return stepSizeFloat


    def motorOn():
        smbus.SMBus(1).write_byte_data(0x07, 0x01, 0x00)
        smbus.SMBus(1).write_byte_data(0x07, 0x02, 0x00)
        smbus.SMBus(1).write_byte_data(0x07, 0x03, 0x00)


    def motorOff():
        smbus.SMBus(1).write_byte_data(0x07, 0x01, 0x03)
        smbus.SMBus(1).write_byte_data(0x07, 0x02, 0x03)
        smbus.SMBus(1).write_byte_data(0x07, 0x03, 0x03)
    
    #def End():
        #d.join()
        
    #d= Thread(target=move_stepper_motors_using_line_step_sequence, name='Daemon_MoveSteps')
    #d.setDaemon(True)
    
    tramaSecuencia = Thread(target=move_stepper_motors_using_line_step_sequence, name='Daemon')
    #Thread(name="T_secuencia", target=Control_V1.move_stepper_motors_using_line_step_sequence ,args=(1,))
    
    
##class MiHilo(Thread):
##    # Se le pasa un numero identificador del hilo y un semaforo
##def __init__(self, numero_hilo, semaforo):
##        Thread.__init__(self)
##        self.semaforo=semaforo
##        self.numero_hilo = numero_hilo
##        
##    def run(self):
##        # Espera al semaforo
##        semaforo.acquire()
##        #print "Entra hilo "+str(self.numero_hilo)
##        print ("Entra hilo %s",str(self.numero_hilo))
##        # Pierde un tiempo aleatorio
##        time.sleep(random.randrange(1,10,1))
##        print ("Fin hilo %s" , str(self.numero_hilo))
##        # Pone verde el semaforo para el siguiente y
##        # termina
##        semaforo.release()    
  
  