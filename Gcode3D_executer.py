import RPi.GPIO as GPIO
import Motor_control_new
from Bipolar_Stepper_Motor_Class_new import Bipolar_Stepper_Motor
import time
from numpy import pi, sin, cos, sqrt, arccos, arcsin, log
from collections import deque
#import Adafruit_GPIO.SPI as SPI
#import Adafruit_MCP3008

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
#from ProcesadorLinneas import Tramas_V1


from ControlArm import Control_V1
#dobotGUI = Control_V1;
Control_V1.inicio();

 
################################################################################################
################################################################################################
#################                            ###################################################
#################    Parameters set up       ###################################################
#################                            ###################################################
################################################################################################
################################################################################################

filename='Gcode/Small.gcode'; #file name of the G code commands

GPIO.setmode(GPIO.BCM)
print(GPIO.VERSION)
MX=Bipolar_Stepper_Motor(4,17,2);     #pin number for a1,a2,b1,b2.  a1 and a2 form coil A; b1 and b2 form coil B
MY=Bipolar_Stepper_Motor(23,18,3);
MZ=Bipolar_Stepper_Motor(24,25,1);
MExt=Bipolar_Stepper_Motor(27,22,4);




#TODO EndStop/Home Axis code needed still testing they should be tied to the enable pins of each motor
EndStopX = 14
EndStopY = 15
EndStopZ = 7
ExtHeater = 10
HeatBed = 9
#ExtThermistor = 11
#HeatBedThermistor = 8
#note only can set lists with 0.5.8 and above version of RPi.GPIO orginal PI caps out at 0.5.5?
outputs = [ExtHeater,HeatBed];
inputs = [EndStopX,EndStopY,EndStopZ];

# Software SPI configuration for MCP3008 ADC Chip:
#Extruder is Channel 0
#Heat Bed is Channel 1
CLK  = 8
MISO = 11
MOSI = 2
CS   = 3
#mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
extChannel = 0
heatBedChannel = 1

#   Therm Specs: Honeywell 135-103LAF-J01
#   Resistance @ 25C = 10K Ohms
#   Beta = 3974 @ 0C/50C
#
#   25 C = 298.15 K
ThermBeta = 3974
ThermDefaultTempK = 298.15
ThermDefaultRes = 10000

dx=0.1#0.020; #0.198; #resolution in x direction. Unit: mm  http://prusaprinters.org/calculator/
dy=0.1#0.010; #0.198; #resolution in y direction. Unit: mm  http://prusaprinters.org/calculator/
dz=0.1#0.020; #0.004; #resolution in Z direction. Unit: mm  http://prusaprinters.org/calculator/
dext=0.039; # resolution for Extruder Unit: mm http://forums.reprap.org/read.php?1,144245


#Engraving_speed=40; #unit=mm/sec=0.04in/sec
Engraving_speed=25;
speedWork=40;

extTemp = 0; #global variable for current tempurature settings
heatBedTemp = 0;
extTempQue = deque()
heatBedTempQue = deque()

#######B#########################################################################################
################################################################################################
#################                            ###################################################
#################    Other initialization    ###################################################
#################                            ###################################################
################################################################################################
################################################################################################
    
GPIO.setup(outputs,GPIO.OUT);
GPIO.output(outputs, False);

GPIO.setup(inputs,GPIO.IN,pull_up_down=GPIO.PUD_DOWN);  # pull_up_down=GPIO.PUD_UP  or pull_up_down=GPIO.PUD_DOWN

speed=Engraving_speed/min(dx,dy);      #step/sec

################################################################################################
################################################################################################
#################                                ###############################################
#################    G code reading Functions    ###############################################
#################                                ###############################################
################################################################################################
################################################################################################

def writeToLog(outputText):
    with open('tempout.txt','a') as f:
        f.write(outputText)
    print (outputText)

#these functions are for debugging purposes only
def sampleHeaters(extThermChannel,heatbeadThermChannel):
    sampleHeaterTemp(extThermChannel, "Extruder")
    sampleHeaterTemp(heatbeadThermChannel, "Heated Bed")

def sampleHeaterTemp(channel, name):
    writeToLog("Testing "+ name +" Temperature\n");
    #highTime = get555PulseHighTime(pin);
    #writeToLog(name+ " Thermistor 555 Timer High Pulse Time "+ str(highTime)+"\n")
    estTemp = getTempAtADCChannel(channel)
    writeToLog(name+ " Estimated Tempurature "+ str(estTemp)+"\n")

#function to read ADC channel value and calcuate tempurature of thermistor
#Values:
#   Vin = 3.3V
#   Vout = 3.3V(ADC Value/1024)
#   R1 = 1K Ohms
#   R2 = thermRes = Estimated Thermistor Resistance
#
#   Beta = 3974 @ 0C/50C
#
#   25 C = 298.15 K
def getTempAtADCChannel(channel):
    #adcVal = mcp.read_adc(channel);
    adcVal = 50;
    Vout = 3.3 * (float(adcVal)/1024);
    if(Vout == 0 or Vout == 3.3):
        return 0;
    thermRes = (Vout * 1000)/(3.3 - Vout);
    return ((ThermDefaultTempK * ThermBeta) / log(ThermDefaultRes/thermRes) / (ThermBeta /  log(ThermDefaultRes/thermRes) - ThermDefaultTempK) - 273.15);

#This function takes in the current temp and name of heater and returns the current average  
 #of the last three tempurature readings.  This avoids issues with reading spikes
def getAverageTempFromQue(temp, name):
    retTemp = 0;
    if(name == "Extruder"):
        if(len(extTempQue) > 2):
            extTempQue.pop();
        extTempQue.appendleft(temp)
        retTemp = sum(extTempQue)/len(extTempQue);
    else:
        if(len(heatBedTempQue) > 2):
            heatBedTempQue.pop();
        heatBedTempQue.appendleft(temp)
        retTemp = sum(heatBedTempQue)/len(heatBedTempQue);
    return float(retTemp);

#polling tempurature and setting to +/- 20degC of supplied tempfrom GCode
def checkTemps():
    curExtTemp = getAverageTempFromQue(getTempAtADCChannel(extChannel), "Extruder");#getTempFromTable(ExtThermistor);
    curHeatBedTemp = getAverageTempFromQue(getTempAtADCChannel(heatBedChannel), "HeatBed");#getTempFromTable(HeatBedThermistor);
    print ("Current Extruder temp: " + str(curExtTemp));
    print ("Current HeatBed temp: " + str(curHeatBedTemp));
    if curExtTemp >= extTemp:
        GPIO.output(ExtHeater, False);
    elif curExtTemp <= extTemp:
        GPIO.output(ExtHeater, True);
    if (curHeatBedTemp - 2) >= heatBedTemp:
        GPIO.output(HeatBed, False);
    elif(curHeatBedTemp + 2) <= heatBedTemp:
        GPIO.output(HeatBed, True);

def PenOff(ZMotor):
    # move ZAxis ~5 steps up
    ZMotor.move(1,5)

def PenOn(ZMotor):
    # move ZAxis ~5 steps down
    ZMotor.move(-1,5)
    print("Baja")

def XYposition(lines):
    #given a movement command line, return the X Y position
    xchar_loc=lines.index('X');
    i=xchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    x_pos=float(lines[xchar_loc+1:i]);    
    
    ychar_loc=lines.index('Y');
    i=ychar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    y_pos=float(lines[ychar_loc+1:i]);    

    return x_pos,y_pos;

def homeAxis(motor,endStopPin):
    #need to home the axis
    #step axis until endstop is triggered
    while GPIO.input(endStopPin) == GPIO.HIGH:
    	motor.move(-1,1);
    #then change endstop GPIOs to output to re-enable the motors and fire of 1-5 steps
    #in the opposite direction.  Note* this is first time I've changed the function of GPIO(input to output)
    #in the middle of a program.  I'm not sure if this is even possible but I'm assuming it is.
    GPIO.setup(endStopPin,GPIO.OUT);
    GPIO.output(endStopPin, True);
    motor.move(1,5);
    #Then step endstop GPIO back to input.
    GPIO.output(endStopPin, False);
    GPIO.setup(endStopPin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN); # pull_up_down=GPIO.PUD_UP  or pull_up_down=GPIO.PUD_DOWN
    

def XYExt_position(lines):
    #given a movement command line, return the X Y position
    xchar_loc=lines.index('X');
    i=xchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    x_pos=float(lines[xchar_loc+1:i]);    
    
    ychar_loc=lines.index('Y');
    i=ychar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    y_pos=float(lines[ychar_loc+1:i]);

    extchar_loc=lines.index('E');
    i=extchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    ext_pos=float(lines[extchar_loc+1:i]);

    return x_pos,y_pos,ext_pos;

def SinglePosition(lines,axis):
    extchar_loc=lines.index(axis);
    i=extchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    ext_pos=float(lines[extchar_loc+1:i]);

    return ext_pos;

def IJposition(lines):
    #given a G02 or G03 movement command line, return the I J position
    ichar_loc=lines.index('I');
    i=ichar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    i_pos=float(lines[ichar_loc+1:i]);    
    
    jchar_loc=lines.index('J');
    i=jchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    j_pos=float(lines[jchar_loc+1:i]);    

    return i_pos,j_pos;

def IJEposition(lines):
    #given a G02 or G03 movement command line, return the I J position
    ichar_loc=lines.index('I');
    i=ichar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    i_pos=float(lines[ichar_loc+1:i]);    
    
    jchar_loc=lines.index('J');
    i=jchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    j_pos=float(lines[jchar_loc+1:i]);

    extchar_loc=lines.index('E');
    i=extchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    ext_pos=float(lines[extchar_loc+1:i]);

    return i_pos,j_pos,ext_pos;

def moveto(MX,x_pos,dx,MY,y_pos,dy,speed,engraving):
#Move to (x_pos,y_pos) (in real unit)
    stepx=int(round(x_pos/dx))-MX.position;
    stepy=int(round(y_pos/dy))-MY.position;

    Total_step=sqrt((stepx**2+stepy**2));
            
    if Total_step>0:
        if lines[0:3]=='G0 ': #fast movement
            print ('No Laser, fast movement: Dx=', stepx, '  Dy=', stepy);
            #Motor_control_new.Motor_Step(MX,stepx,MY,stepy,50);
            Control_V1DobotGUI.MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,30);
        else:
            print ('Laser on, movement: Dx=', stepx, '  Dy=', stepy);
            #Motor_control_new.Motor_Step(MX,stepx,MY,stepy,speed);# hard 50 for now
            #try:
            Control_V1.MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,speed);
            #except:
                #pass
    return 0;

def movetothree(MX,x_pos,dx,MY,y_pos,dy,MExt,ext_pos,dext,speed,engraving):
#Move to (x_pos,y_pos) (in real unit)
    #print ('Lassss')
    stepx=int(round(x_pos/dx))-MX.position;
    stepy=int(round(y_pos/dy))-MY.position;
    stepExt=int(round(ext_pos/dext))-MExt.position;

    Total_step=sqrt((stepx**2+stepy**2));
            
    if Total_step>0:
        if lines[0:3]=='G0 ': #fast movement
            print ('fast movement: Dx=', stepx, '  Dy=', stepy);
            #Motor_control_new.Motor_StepThree(MX,stepx,MY,stepy,MExt,stepExt,50);
            Control_V1.MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,speed);
        else:
            print ('movement: Dx=', stepx, '  Dy=', stepy, '  Dex=', stepExt);
            #Motor_control_new.Motor_StepThree(MX,stepx,MY,stepy,MExt,stepExt,speed);
            Control_V1.MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,speed);
    return 0;

###########################################################################################

###shut off heaters
##GPIO.output(outputs, False);
###PenOff(MZ);   # turn off laser
##moveto(MX,0,dx,MY,0,dy,130,False);  # move back to Origin
##
##MX.unhold();
##MY.unhold();
##MZ.unhold();
##
##GPIO.cleanup();

class Tramas_V1:
    lineCtr = 1;
    heaterCheck = 1;
    def LineaProcesa(lines):
        if lines==[]:
            1; #blank lines
        elif lines[0:3]=='G90':
            Control_V1.motorOn();
            print ('start');
        elif lines[0:3]=='G92':
            print ('Reset Extruder to 0');
            MExt.position = 0;
            
        elif lines[0:3]=='G20':# working in inch;
            dx/=25.4;
            dy/=25.4;
            print ('Working in inch');
              
        elif lines[0:3]=='G21':# working in mm;
            print ('Working in mm');
            
        elif lines[0:3]=='G28': # homing all axis
            print ('Homing all axis...');
            #move till endstops trigger
            print ('Homing X axis...');
            homeAxis(MX,EndStopX)
            print ('Homing Y axis...');
            homeAxis(MY,EndStopY)
            print ('Homing Z axis...');
            homeAxis(MZ,EndStopZ)
            
        elif lines[0:3]=='M05': # these will not be used (M05) for the 3D Printer,  I used this code for a pen plotter orginally but I could be used to attach a milling tool
            PenOff(MZ)
            #GPIO.output(Laser_switch,False);
            print ('Pen turned off');
            
        elif lines[0:3]=='M03':
            PenON(MZ)
            #GPIO.output(Laser_switch,True);
            print ('Pen turned on');

        elif lines[0:3]=='M02':
            GPIO.output(Laser_switch,False);
            print ('finished. shuting down');
##            break;
        elif lines[0:3]=='M84':
            Control_V1.motorOff();
            print ('motor off');
##            break;
        elif lines[0:4]=='M104': #Set Extruder Temperature 
            #note that we should just be setting the tempurature here, but because this always fires before M109 call
            #I'm just turning the extruder on as well because then it can start heating up
            extTemp = float(SinglePosition(lines,'S'));
            print ('Extruder Heater On and setting temperature to '+ str(extTemp) +'C');
            GPIO.output(ExtHeater,True);
            sampleHeaters(0,1);
        elif lines[0:4]=='M106': #Fan on 
            #for now we will just print the following text
            print ('Fan On');
        elif lines[0:4]=='M107': #Fan off 
            #for now we will just print the following text
            print ('Fan Off');
        elif lines[0:4]=='M109':  #Set Extruder Temperature and Wait
            #need to set temperature here and wait for correct temp as well
            #for now we will just turn on extruderheater
            #I would like to this all with the raspberry pi but...
            #I may use a simple Arduino(Uno) sketch to handle tempurature regulation 
            #Doing with the RaspPi only would require polling the tempurature(maybe at each Z axis move?)
            print ('Extruder Heater On');
            GPIO.output(ExtHeater,True);
            extTemp = float(SinglePosition(lines,'S'));
            print ('Extruder Heater On and setting temperature to '+ str(extTemp) +'C');
            print ('Waiting to reach target temp...');
            sampleHeaters(extChannel,heatBedChannel);
            temp = getTempAtADCChannel(extChannel)
            while temp < extTemp:
            	time.sleep(0.02);
            	temp = getAverageTempFromQue(getTempAtADCChannel(extChannel), "Extruder");
            	print (str(temp));
            	
        elif lines[0:4]=='M140': #Set Heat Bed Temperature 
            #need to set temperature here as well
            #for now we will just turn on extruderheater
            heatBedTemp = float(SinglePosition(lines,'S'));
            print ('Setting Heat Bed temperature to '+ str(heatBedTemp) +'C');

        elif lines[0:4]=='M190':  #Set HeatBed Temperature and Wait
            #need to set temperature here and wait for correct temp as well
            #for now we will just turn on HeatBedheater
            #I would like to this all with the raspberry pi but...
            #I may use a simple Arduino(Uno) sketch to handle tempurature regulation 
            #Doing with the RaspPi only would require polling the tempurature(maybe at each Z axis move?)
            heatBedTemp = float(SinglePosition(lines,'S'));
            print ('HeatBed Heater On');
            print ('Setting HeatBed temperature to '+ str(heatBedTemp) +'C and waiting');
            GPIO.output(HeatBed,True);
            sampleHeaters(extChannel,heatBedChannel);
            temp = getTempAtADCChannel(heatBedChannel)
            while temp < heatBedTemp:
            	time.sleep(0.02);
            	temp = getAverageTempFromQue(getTempAtADCChannel(heatBedChannel), "HeatBed");
            	print (str(temp));
            
        elif (lines[0:3]=='G1F')|(lines[0:4]=='G1 F'):
            1;#do nothing
        elif (lines[0:3]=='G0 ')|(lines[0:3]=='G1 ')|(lines[0:3]=='G01'):#|(lines[0:3]=='G02')|(lines[0:3]=='G03'):
            #linear engraving movement
            if (lines[0:3]=='G0 '):
                engraving=False;
            else:
                engraving=True;
                #Update F Value(speed) if available 			

            if(lines.find('F') >= 0):
                speed = (SinglePosition(lines,'F')/60)/min(dx,dy);  #getting F value as mm/min so we need to convert to mm/sec then calc and update speed

            if(lines.find('E') < 0 and lines.find('Z') < 0):
                [x_pos,y_pos]=XYposition(lines);
                #moveto(MX,x_pos,dx,MY,y_pos,dy,speed,engraving);
                Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
            elif(lines.find('X') < 0 and lines.find('Z') < 0): #Extruder only
                ext_pos = SinglePosition(lines,'E');
                stepsExt = int(round(ext_pos/dext)) - MExt.position;
                #TODO fix this extMotor Delay
                Motor_control_new.Single_Motor_Step(MExt,stepsExt,speed); #changed from static 40
                #still need to move Extruder using stepExt(signed int)
            elif(lines.find('X') < 0 and lines.find('E') < 0): #Z Axis only
                print ('Moving Z axis only');
                z_pos = SinglePosition(lines,'Z');
                #stepsZ = int(round(z_pos/dz)) - MZ.position;
                #Motor_control_new.Single_Motor_Step(MZ,stepsZ); #changed from static 60
                Control_V1.MoveToCoordinate_Z(z_pos,speedWork);
                #check Extruder and Heat Bed temp after Z axiz move
                checkTemps();
            else:                
               # [x_pos,y_pos,ext_pos]=XYExt_position(lines);
                #movetothree(MX,x_pos,dx,MY,y_pos,dy,MExt,ext_pos,dext,speed,engraving);
                Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
                heaterCheck += 1;
                Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
                print("?");
                #create new moveto function to include Extruder postition
            
        elif (lines[0:3]=='G02')|(lines[0:3]=='G03'): #circular interpolation
            old_x_pos=x_pos;
            old_y_pos=y_pos;
            
            ext_pos = 0;
            
            #still need to add code here to handle extrusion info from the line if it is available
            if(lines.find('E') >= 0):
            	#get E value as well as the rest
            	[x_pos,y_pos]=XYposition(lines);
            	[i_pos,j_pos,ext_pos]=IJEposition(lines);
            else:
            	[x_pos,y_pos]=XYposition(lines);
            	[i_pos,j_pos]=IJposition(lines);

            xcenter=old_x_pos+i_pos;   #center of the circle for interpolation
            ycenter=old_y_pos+j_pos;
            
            
            Dx=x_pos-xcenter;
            Dy=y_pos-ycenter;      #vector [Dx,Dy] points from the circle center to the new position
            
            r=sqrt(i_pos**2+j_pos**2);   # radius of the circle
            
            e1=[-i_pos,-j_pos]; #pointing from center to current position
            if (lines[0:3]=='G02'): #clockwise
                e2=[e1[1],-e1[0]];      #perpendicular to e1. e2 and e1 forms x-y system (clockwise)
            else:                   #counterclockwise
                e2=[-e1[1],e1[0]];      #perpendicular to e1. e1 and e2 forms x-y system (counterclockwise)

            #[Dx,Dy]=e1*cos(theta)+e2*sin(theta), theta is the open angle

            costheta=(Dx*e1[0]+Dy*e1[1])/r**2;
            sintheta=(Dx*e2[0]+Dy*e2[1])/r**2;        #theta is the angule spanned by the circular interpolation curve
                
            if costheta>1:  # there will always be some numerical errors! Make sure abs(costheta)<=1
                costheta=1;
            elif costheta<-1:
                costheta=-1;

            theta=arccos(costheta);
            if sintheta<0:
                theta=2.0*pi-theta;

            no_step=int(round(r*theta/dx/5.0));   # number of point for the circular interpolation
            extruderMovePerStep = 0;
            if ext_pos != 0:
            	extruderMovePerStep = (ext_pos - MExt.position)/no_step;
            
            for i in range(1,no_step+1):
               	tmp_theta=i*theta/no_step;
               	tmp_x_pos=xcenter+e1[0]*cos(tmp_theta)+e2[0]*sin(tmp_theta);
               	tmp_y_pos=ycenter+e1[1]*cos(tmp_theta)+e2[1]*sin(tmp_theta);
               	if extruderMovePerStep == 0:
               		#moveto(MX,tmp_x_pos,dx,MY, tmp_y_pos,dy,speed,True);
               		Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
               	else:
               		#movetothree(MX,tmp_x_pos,dx,MY, tmp_y_pos,dy,MExt,MExt.position+extruderMovePerStep,dext,speed,True);
               		Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
        if heaterCheck >= 2: #checking every fifth extruder motor move 
            print ('Checking Temps');
            checkTemps();
            heaterCheck = 0;   
    
###########################################################################################
#################                           ###############################################
#################    Main program           ###############################################
#################                           ###############################################
###########################################################################################
###########################################################################################
#TODO  G28, M107, M106
#GCode defintion reference: http://reprap.org/wiki/G-code
#Bug - motion is slow on XY moves when steps are ~50 or more on each, speed issue?
#TODO threading for temperature management. For a quick fix now we call checktemps every 25th Ext move
import RPi.GPIO as GPIO
import Motor_control_new
from Bipolar_Stepper_Motor_Class_new import Bipolar_Stepper_Motor
import time
from numpy import pi, sin, cos, sqrt, arccos, arcsin, log
from collections import deque
#import Adafruit_GPIO.SPI as SPI
#import Adafruit_MCP3008

import smbus
import time, struct, math
import sys, os, threading
from threading import Thread
#from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QMessageBox, QTableWidget, QTableWidgetItem
#from PyQt5 import uic, QtCore, QtGui
import EEZYbotArm_Pi_I2C_kinematics
import serial.tools.list_ports
import os.path
import RPi.GPIO as gpio #https://pypi.python.org/pypi/RPi.GPIO more info


from ControlArm import Control_V1
#dobotGUI = Control_V1;
Control_V1.inicio();
 
################################################################################################
################################################################################################
#################                            ###################################################
#################    Parameters set up       ###################################################
#################                            ###################################################
################################################################################################
################################################################################################

filename='Gcode/Small.gcode'; #file name of the G code commands

GPIO.setmode(GPIO.BCM)
print(GPIO.VERSION)
MX=Bipolar_Stepper_Motor(4,17,2);     #pin number for a1,a2,b1,b2.  a1 and a2 form coil A; b1 and b2 form coil B
MY=Bipolar_Stepper_Motor(23,18,3);
MZ=Bipolar_Stepper_Motor(24,25,1);
MExt=Bipolar_Stepper_Motor(27,22,4);




#TODO EndStop/Home Axis code needed still testing they should be tied to the enable pins of each motor
EndStopX = 14
EndStopY = 15
EndStopZ = 7
ExtHeater = 10
HeatBed = 9
#ExtThermistor = 11
#HeatBedThermistor = 8
#note only can set lists with 0.5.8 and above version of RPi.GPIO orginal PI caps out at 0.5.5?
outputs = [ExtHeater,HeatBed];
inputs = [EndStopX,EndStopY,EndStopZ];

# Software SPI configuration for MCP3008 ADC Chip:
#Extruder is Channel 0
#Heat Bed is Channel 1
CLK  = 8
MISO = 11
MOSI = 2
CS   = 3
#mcp = Adafruit_MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
extChannel = 0
heatBedChannel = 1

#   Therm Specs: Honeywell 135-103LAF-J01
#   Resistance @ 25C = 10K Ohms
#   Beta = 3974 @ 0C/50C
#
#   25 C = 298.15 K
ThermBeta = 3974
ThermDefaultTempK = 298.15
ThermDefaultRes = 10000

dx=0.1#0.020; #0.198; #resolution in x direction. Unit: mm  http://prusaprinters.org/calculator/
dy=0.1#0.010; #0.198; #resolution in y direction. Unit: mm  http://prusaprinters.org/calculator/
dz=0.1#0.020; #0.004; #resolution in Z direction. Unit: mm  http://prusaprinters.org/calculator/
dext=0.039; # resolution for Extruder Unit: mm http://forums.reprap.org/read.php?1,144245


#Engraving_speed=40; #unit=mm/sec=0.04in/sec
Engraving_speed=25;
speedWork=40;

extTemp = 0; #global variable for current tempurature settings
heatBedTemp = 0;
extTempQue = deque()
heatBedTempQue = deque()

#######B#########################################################################################
################################################################################################
#################                            ###################################################
#################    Other initialization    ###################################################
#################                            ###################################################
################################################################################################
################################################################################################
    
GPIO.setup(outputs,GPIO.OUT);
GPIO.output(outputs, False);

GPIO.setup(inputs,GPIO.IN,pull_up_down=GPIO.PUD_DOWN);  # pull_up_down=GPIO.PUD_UP  or pull_up_down=GPIO.PUD_DOWN

speed=Engraving_speed/min(dx,dy);      #step/sec

################################################################################################
################################################################################################
#################                                ###############################################
#################    G code reading Functions    ###############################################
#################                                ###############################################
################################################################################################
################################################################################################

def writeToLog(outputText):
    with open('tempout.txt','a') as f:
        f.write(outputText)
    print (outputText)

#these functions are for debugging purposes only
def sampleHeaters(extThermChannel,heatbeadThermChannel):
    sampleHeaterTemp(extThermChannel, "Extruder")
    sampleHeaterTemp(heatbeadThermChannel, "Heated Bed")

def sampleHeaterTemp(channel, name):
    writeToLog("Testing "+ name +" Temperature\n");
    #highTime = get555PulseHighTime(pin);
    #writeToLog(name+ " Thermistor 555 Timer High Pulse Time "+ str(highTime)+"\n")
    estTemp = getTempAtADCChannel(channel)
    writeToLog(name+ " Estimated Tempurature "+ str(estTemp)+"\n")

#function to read ADC channel value and calcuate tempurature of thermistor
#Values:
#   Vin = 3.3V
#   Vout = 3.3V(ADC Value/1024)
#   R1 = 1K Ohms
#   R2 = thermRes = Estimated Thermistor Resistance
#
#   Beta = 3974 @ 0C/50C
#
#   25 C = 298.15 K
def getTempAtADCChannel(channel):
    #adcVal = mcp.read_adc(channel);
    adcVal = 50;
    Vout = 3.3 * (float(adcVal)/1024);
    if(Vout == 0 or Vout == 3.3):
        return 0;
    thermRes = (Vout * 1000)/(3.3 - Vout);
    return ((ThermDefaultTempK * ThermBeta) / log(ThermDefaultRes/thermRes) / (ThermBeta /  log(ThermDefaultRes/thermRes) - ThermDefaultTempK) - 273.15);

#This function takes in the current temp and name of heater and returns the current average  
 #of the last three tempurature readings.  This avoids issues with reading spikes
def getAverageTempFromQue(temp, name):
    retTemp = 0;
    if(name == "Extruder"):
        if(len(extTempQue) > 2):
            extTempQue.pop();
        extTempQue.appendleft(temp)
        retTemp = sum(extTempQue)/len(extTempQue);
    else:
        if(len(heatBedTempQue) > 2):
            heatBedTempQue.pop();
        heatBedTempQue.appendleft(temp)
        retTemp = sum(heatBedTempQue)/len(heatBedTempQue);
    return float(retTemp);

#polling tempurature and setting to +/- 20degC of supplied tempfrom GCode
def checkTemps():
    curExtTemp = getAverageTempFromQue(getTempAtADCChannel(extChannel), "Extruder");#getTempFromTable(ExtThermistor);
    curHeatBedTemp = getAverageTempFromQue(getTempAtADCChannel(heatBedChannel), "HeatBed");#getTempFromTable(HeatBedThermistor);
    print ("Current Extruder temp: " + str(curExtTemp));
    print ("Current HeatBed temp: " + str(curHeatBedTemp));
    if curExtTemp >= extTemp:
        GPIO.output(ExtHeater, False);
    elif curExtTemp <= extTemp:
        GPIO.output(ExtHeater, True);
    if (curHeatBedTemp - 2) >= heatBedTemp:
        GPIO.output(HeatBed, False);
    elif(curHeatBedTemp + 2) <= heatBedTemp:
        GPIO.output(HeatBed, True);

def PenOff(ZMotor):
    # move ZAxis ~5 steps up
    ZMotor.move(1,5)

def PenOn(ZMotor):
    # move ZAxis ~5 steps down
    ZMotor.move(-1,5)
    print("Baja")

def XYposition(lines):
    #given a movement command line, return the X Y position
    xchar_loc=lines.index('X');
    i=xchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    x_pos=float(lines[xchar_loc+1:i]);    
    
    ychar_loc=lines.index('Y');
    i=ychar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    y_pos=float(lines[ychar_loc+1:i]);    

    return x_pos,y_pos;

def homeAxis(motor,endStopPin):
    #need to home the axis
    #step axis until endstop is triggered
    while GPIO.input(endStopPin) == GPIO.HIGH:
    	motor.move(-1,1);
    #then change endstop GPIOs to output to re-enable the motors and fire of 1-5 steps
    #in the opposite direction.  Note* this is first time I've changed the function of GPIO(input to output)
    #in the middle of a program.  I'm not sure if this is even possible but I'm assuming it is.
    GPIO.setup(endStopPin,GPIO.OUT);
    GPIO.output(endStopPin, True);
    motor.move(1,5);
    #Then step endstop GPIO back to input.
    GPIO.output(endStopPin, False);
    GPIO.setup(endStopPin,GPIO.IN,pull_up_down=GPIO.PUD_DOWN); # pull_up_down=GPIO.PUD_UP  or pull_up_down=GPIO.PUD_DOWN
    

def XYExtposition(lines):
    #given a movement command line, return the X Y position
    xchar_loc=lines.index('X');
    i=xchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    x_pos=float(lines[xchar_loc+1:i]);    
    
    ychar_loc=lines.index('Y');
    i=ychar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    y_pos=float(lines[ychar_loc+1:i]);

    extchar_loc=lines.index('E');
    i=extchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    ext_pos=float(lines[extchar_loc+1:i]);

    return x_pos,y_pos,ext_pos;

def SinglePosition(lines,axis):
    extchar_loc=lines.index(axis);
    i=extchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    ext_pos=float(lines[extchar_loc+1:i]);

    return ext_pos;

def IJposition(lines):
    #given a G02 or G03 movement command line, return the I J position
    ichar_loc=lines.index('I');
    i=ichar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    i_pos=float(lines[ichar_loc+1:i]);    
    
    jchar_loc=lines.index('J');
    i=jchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    j_pos=float(lines[jchar_loc+1:i]);    

    return i_pos,j_pos;

def IJEposition(lines):
    #given a G02 or G03 movement command line, return the I J position
    ichar_loc=lines.index('I');
    i=ichar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    i_pos=float(lines[ichar_loc+1:i]);    
    
    jchar_loc=lines.index('J');
    i=jchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    j_pos=float(lines[jchar_loc+1:i]);

    extchar_loc=lines.index('E');
    i=extchar_loc+1;
    while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
        i+=1;
    ext_pos=float(lines[extchar_loc+1:i]);

    return i_pos,j_pos,ext_pos;

def moveto(MX,x_pos,dx,MY,y_pos,dy,speed,engraving):
#Move to (x_pos,y_pos) (in real unit)
    stepx=int(round(x_pos/dx))-MX.position;
    stepy=int(round(y_pos/dy))-MY.position;

    Total_step=sqrt((stepx**2+stepy**2));
            
    if Total_step>0:
        if lines[0:3]=='G0 ': #fast movement
            print ('No Laser, fast movement: Dx=', stepx, '  Dy=', stepy);
            #Motor_control_new.Motor_Step(MX,stepx,MY,stepy,50);
            Control_V1DobotGUI.MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,30);
        else:
            print ('Laser on, movement: Dx=', stepx, '  Dy=', stepy);
            #Motor_control_new.Motor_Step(MX,stepx,MY,stepy,speed);# hard 50 for now
            #try:
            Control_V1.MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,speed);
            #except:
                #pass
    return 0;

def movetothree(MX,x_pos,dx,MY,y_pos,dy,MExt,ext_pos,dext,speed,engraving):
#Move to (x_pos,y_pos) (in real unit)
    #print ('Lassss')
    stepx=int(round(x_pos/dx))-MX.position;
    stepy=int(round(y_pos/dy))-MY.position;
    stepExt=int(round(ext_pos/dext))-MExt.position;

    Total_step=sqrt((stepx**2+stepy**2));
            
    if Total_step>0:
        if lines[0:3]=='G0 ': #fast movement
            print ('fast movement: Dx=', stepx, '  Dy=', stepy);
            #Motor_control_new.Motor_StepThree(MX,stepx,MY,stepy,MExt,stepExt,50);
            Control_V1.MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,speed);
        else:
            print ('movement: Dx=', stepx, '  Dy=', stepy, '  Dex=', stepExt);
            #Motor_control_new.Motor_StepThree(MX,stepx,MY,stepy,MExt,stepExt,speed);
            Control_V1.MoveToCoordinate_XY(x_pos,stepx,y_pos,stepy,speed);
    return 0;

###########################################################################################
###########################################################################################
#################                           ###############################################
#################    Main program           ###############################################
#################                           ###############################################
###########################################################################################
###########################################################################################
#TODO  G28, M107, M106
#GCode defintion reference: http://reprap.org/wiki/G-code
#Bug - motion is slow on XY moves when steps are ~50 or more on each, speed issue?
#TODO threading for temperature management. For a quick fix now we call checktemps every 25th Ext move
try:#read and execute G code
    lineCtr = 1;
    heaterCheck = 1;
    Fichero = open(filename,'r');
    for lines in Fichero: #open(filename,'r'):
        print ('processing line# '+str(lineCtr)+ ': '+lines);
        lineCtr += 1;
        if lines==[]:
            1; #blank lines
        elif lines[0:3]=='G90':
            Control_V1.motorOn();
            print ('start');
        elif lines[0:3]=='G92':
            print ('Reset Extruder to 0');
            MExt.position = 0;
            
        elif lines[0:3]=='G20':# working in inch;
            dx/=25.4;
            dy/=25.4;
            print ('Working in inch');
              
        elif lines[0:3]=='G21':# working in mm;
            print ('Working in mm');
            
        elif lines[0:3]=='G28': # homing all axis
            print ('Homing all axis...');
            #move till endstops trigger
            print ('Homing X axis...');
            homeAxis(MX,EndStopX)
            print ('Homing Y axis...');
            homeAxis(MY,EndStopY)
            print ('Homing Z axis...');
            homeAxis(MZ,EndStopZ)
            
        elif lines[0:3]=='M05': # these will not be used (M05) for the 3D Printer,  I used this code for a pen plotter orginally but I could be used to attach a milling tool
            PenOff(MZ)
            #GPIO.output(Laser_switch,False);
            print ('Pen turned off');
            
        elif lines[0:3]=='M03':
            PenON(MZ)
            #GPIO.output(Laser_switch,True);
            print ('Pen turned on');

        elif lines[0:3]=='M02':
            GPIO.output(Laser_switch,False);
            print ('finished. shuting down');
            break;
        elif lines[0:3]=='M84':
            Control_V1.motorOff();
            print ('motor off');
            break;
        elif lines[0:4]=='M104': #Set Extruder Temperature 
            #note that we should just be setting the tempurature here, but because this always fires before M109 call
            #I'm just turning the extruder on as well because then it can start heating up
            extTemp = float(SinglePosition(lines,'S'));
            print ('Extruder Heater On and setting temperature to '+ str(extTemp) +'C');
            GPIO.output(ExtHeater,True);
            sampleHeaters(0,1);
        elif lines[0:4]=='M106': #Fan on 
            #for now we will just print the following text
            print ('Fan On');
        elif lines[0:4]=='M107': #Fan off 
            #for now we will just print the following text
            print ('Fan Off');
        elif lines[0:4]=='M109':  #Set Extruder Temperature and Wait
            #need to set temperature here and wait for correct temp as well
            #for now we will just turn on extruderheater
            #I would like to this all with the raspberry pi but...
            #I may use a simple Arduino(Uno) sketch to handle tempurature regulation 
            #Doing with the RaspPi only would require polling the tempurature(maybe at each Z axis move?)
            print ('Extruder Heater On');
            GPIO.output(ExtHeater,True);
            extTemp = float(SinglePosition(lines,'S'));
            print ('Extruder Heater On and setting temperature to '+ str(extTemp) +'C');
            print ('Waiting to reach target temp...');
            sampleHeaters(extChannel,heatBedChannel);
            temp = getTempAtADCChannel(extChannel)
            while temp < extTemp:
            	time.sleep(0.02);
            	temp = getAverageTempFromQue(getTempAtADCChannel(extChannel), "Extruder");
            	print (str(temp));
            	
        elif lines[0:4]=='M140': #Set Heat Bed Temperature 
            #need to set temperature here as well
            #for now we will just turn on extruderheater
            heatBedTemp = float(SinglePosition(lines,'S'));
            print ('Setting Heat Bed temperature to '+ str(heatBedTemp) +'C');

        elif lines[0:4]=='M190':  #Set HeatBed Temperature and Wait
            #need to set temperature here and wait for correct temp as well
            #for now we will just turn on HeatBedheater
            #I would like to this all with the raspberry pi but...
            #I may use a simple Arduino(Uno) sketch to handle tempurature regulation 
            #Doing with the RaspPi only would require polling the tempurature(maybe at each Z axis move?)
            heatBedTemp = float(SinglePosition(lines,'S'));
            print ('HeatBed Heater On');
            print ('Setting HeatBed temperature to '+ str(heatBedTemp) +'C and waiting');
            GPIO.output(HeatBed,True);
            sampleHeaters(extChannel,heatBedChannel);
            temp = getTempAtADCChannel(heatBedChannel)
            while temp < heatBedTemp:
            	time.sleep(0.02);
            	temp = getAverageTempFromQue(getTempAtADCChannel(heatBedChannel), "HeatBed");
            	print (str(temp));
            
        elif (lines[0:3]=='G1F')|(lines[0:4]=='G1 F'):
            1;#do nothing
        elif (lines[0:3]=='G0 ')|(lines[0:3]=='G1 ')|(lines[0:3]=='G01'):#|(lines[0:3]=='G02')|(lines[0:3]=='G03'):
            #linear engraving movement
            if (lines[0:3]=='G0 '):
                engraving=False;
            else:
                engraving=True;
                #Update F Value(speed) if available 			

            if(lines.find('F') >= 0):
                speed = (SinglePosition(lines,'F')/60)/min(dx,dy);  #getting F value as mm/min so we need to convert to mm/sec then calc and update speed

            if(lines.find('E') < 0 and lines.find('Z') < 0):
                [x_pos,y_pos]=XYposition(lines);
                #moveto(MX,x_pos,dx,MY,y_pos,dy,speed,engraving);
                Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
            elif(lines.find('X') < 0 and lines.find('Z') < 0): #Extruder only
                ext_pos = SinglePosition(lines,'E');
                stepsExt = int(round(ext_pos/dext)) - MExt.position;
                #TODO fix this extMotor Delay
                Motor_control_new.Single_Motor_Step(MExt,stepsExt,speed); #changed from static 40
                #still need to move Extruder using stepExt(signed int)
            elif(lines.find('X') < 0 and lines.find('E') < 0): #Z Axis only
                print ('Moving Z axis only');
                z_pos = SinglePosition(lines,'Z');
                stepsZ = int(round(z_pos/dz)) - MZ.position;
                #Motor_control_new.Single_Motor_Step(MZ,stepsZ); #changed from static 60
                Control_V1.MoveToCoordinate_Z(z_pos,speedWork);
                #check Extruder and Heat Bed temp after Z axiz move
                checkTemps();
            else:                
                [x_pos,y_pos,ext_pos]=XYExtposition(lines);
                #movetothree(MX,x_pos,dx,MY,y_pos,dy,MExt,ext_pos,dext,speed,engraving);
                Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
                heaterCheck += 1;
                #create new moveto function to include Extruder postition
            
        elif (lines[0:3]=='G02')|(lines[0:3]=='G03'): #circular interpolation
            old_x_pos=x_pos;
            old_y_pos=y_pos;
            
            ext_pos = 0;
            
            #still need to add code here to handle extrusion info from the line if it is available
            if(lines.find('E') >= 0):
            	#get E value as well as the rest
            	[x_pos,y_pos]=XYposition(lines);
            	[i_pos,j_pos,ext_pos]=IJEposition(lines);
            else:
            	[x_pos,y_pos]=XYposition(lines);
            	[i_pos,j_pos]=IJposition(lines);

            xcenter=old_x_pos+i_pos;   #center of the circle for interpolation
            ycenter=old_y_pos+j_pos;
            
            
            Dx=x_pos-xcenter;
            Dy=y_pos-ycenter;      #vector [Dx,Dy] points from the circle center to the new position
            
            r=sqrt(i_pos**2+j_pos**2);   # radius of the circle
            
            e1=[-i_pos,-j_pos]; #pointing from center to current position
            if (lines[0:3]=='G02'): #clockwise
                e2=[e1[1],-e1[0]];      #perpendicular to e1. e2 and e1 forms x-y system (clockwise)
            else:                   #counterclockwise
                e2=[-e1[1],e1[0]];      #perpendicular to e1. e1 and e2 forms x-y system (counterclockwise)

            #[Dx,Dy]=e1*cos(theta)+e2*sin(theta), theta is the open angle

            costheta=(Dx*e1[0]+Dy*e1[1])/r**2;
            sintheta=(Dx*e2[0]+Dy*e2[1])/r**2;        #theta is the angule spanned by the circular interpolation curve
                
            if costheta>1:  # there will always be some numerical errors! Make sure abs(costheta)<=1
                costheta=1;
            elif costheta<-1:
                costheta=-1;

            theta=arccos(costheta);
            if sintheta<0:
                theta=2.0*pi-theta;

            no_step=int(round(r*theta/dx/5.0));   # number of point for the circular interpolation
            extruderMovePerStep = 0;
            if ext_pos != 0:
            	extruderMovePerStep = (ext_pos - MExt.position)/no_step;
            
            for i in range(1,no_step+1):
               	tmp_theta=i*theta/no_step;
               	tmp_x_pos=xcenter+e1[0]*cos(tmp_theta)+e2[0]*sin(tmp_theta);
               	tmp_y_pos=ycenter+e1[1]*cos(tmp_theta)+e2[1]*sin(tmp_theta);
               	if extruderMovePerStep == 0:
               	    #moveto(MX,tmp_x_pos,dx,MY, tmp_y_pos,dy,speed,True);
               	    Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
               	else:
               	    #movetothree(MX,tmp_x_pos,dx,MY, tmp_y_pos,dy,MExt,MExt.position+extruderMovePerStep,dext,speed,True);
               	    Control_V1.MoveToCoordinate_XY(x_pos,dx,y_pos,dy,speed)
        if heaterCheck >= 2: #checking every fifth extruder motor move 
            print ('Checking Temps');
            checkTemps();
            heaterCheck = 0;            
        
except KeyboardInterrupt:
    pass



try:#read and execute G code

    Tramas_V1.lineCtr = 1;
    Tramas_V1.lheaterCheck = 1;
    Fichero = open(filename,'r');
    for lines in Fichero: #open(filename,'r'):
        Tramas_V1.lineCtr += 1;
        hilo= Tramas_V1.LineaProcesa(lines);
        print ('processing line# '+str(lineCtr)+ ': '+lines);
        hilo.start();
        hilo.join();
except KeyboardInterrupt:
    pass




#shut off heaters
GPIO.output(outputs, False);
#PenOff(MZ);   # turn off laser
moveto(MX,0,dx,MY,0,dy,130,False);  # move back to Origin

MX.unhold();
MY.unhold();
MZ.unhold();

GPIO.cleanup();