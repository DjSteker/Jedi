
from threading import Thread, Semaphore

class Tramas_V1:
    
    def LineaProcesa(linea):
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
        