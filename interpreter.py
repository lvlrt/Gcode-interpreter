print("#########################")
print("### 3D PRINT SOFTWARE ###")
print("#########################")
print("         by Lars Veelaert")
print("")
print("starting up...")
input("Press Enter to continue...")
print("")
print("loading modules...")
debug = True
filename='part1.txt'#Naam gcode file
window_scale = 12
window_x = 50
window_y = 50
"""###########################
### MAIN PROGRAM FUNCTIONS ###
###########################"""
#ADJUSTEMENT AND BOOT PARAMETERS
Cur_X = 0
Cur_Y = 0
Cur_Z = 0
Cur_E = 0
Cur_F = 0
print("MAIN PROGRAM FUNCTIONS...")
#load settings and save settings in een file inporteren (mm of inch, rodlenth
#settings weergeven na help in een enkele lijn en dan ook een oproepbooaar command voor alle gegevenes, diyt command laten uitscrhrijven naar een file voor het bekomen van een savefile.
#calibratie samenzetten na de calibratie (adj van unit mag met mm inch of vergelijking tov mm worden geantwoord.
#patterns in de gpio functie gooien om de heroproepbaar heid van de verandering van de steps of loade helemaal in het begin van een aantal parameters door een file
#calibration vanvoor. loaden van file is mogelijk,

#er mogen geen 2 phase differences in 1 keer worden gestuurd, dus er moet een vertraging in met timelaps
#met globals werken (gedaan?)
#scale er nog rechtsreeks in de maat. om exacte maat te kunnen uitkomen
#resolution min gem en lowest akkoord?
#first object pijp omghoog checken
#save after calibration na vraag
#grid ipv minres
#algoritme met uiterste hoeken van surface voor testen of het erop kan (berekening surface sq. (hier ook ineens het heating bed insteken)
#alle keuzes met nummers
#settings in een anders  bestand"
#gcode functions ergens vanboven
#een functie dat hypotetisch met de theoretische waarden de resolutie en alle andere waarden uitrekent
#mooier met tussenlijnen werken en duidelijkere tekstjes, uitleg, dan vraag en daarvoor het standaardantwoord, ev opties onder elkaar
# moveRmin moet vervangen worden door een van de zijden van de driehoek, anders niet heel het oppervlak = wss gewoon moveRmin = movermin/cos30
#possible_print herhaling moest er een fout zijn met while
def help():
    commands = [
        "commandhelp('COMMAND')"
        ,
        "help()"
        ,
        "start_position_monitor()"
        ,
        "start_delta_monitor()"
        ,
        "movetoDelta(x,y,z,e,C0,C1,C2,f)"
        ,
        "cubesize('FILENAME')"
        ,
        "filename = """
        ,
        "print_start('FILENAME')"
        ,
        "home_4axes(stepper1,stepper2,stepper3,stepper4)"
	,
	"info()"
        ]
    commands_explained = {
        "commandhelp": 
        "Explains the command given between brackets and explains how to use it."
        ,
        "help":
        "Displays a list of commands, each command can be explained with 'commandhelp('command')."
        ,
        "start_position_monitor":
        "Starts a tkinter window that displays the current position of the extruder"
        ,
        "start_delta_monitor":
        "starts a tkinterwindows that displays the current position of the carriages of the delta printer"
        ,
        "movetoDelta":
        "Moving the extruder with an algorithm of a delta printer, with possible extrusion and at a certain speed (f), All coordinates are relative and the carriages (C#) start in the lowerleft corner and go clock-wise"
        ,
        "cubesize":
        "Calculates the describing cube from the Gcode-file, Exports the 3 measurements"
        ,
        "filename = """:
        "Filename of the Gcode-file"
        ,
        "print_start":
        "prints 'filename' on 3d printer through gcode interpreter"
        ,
        "home_4axes":
        "Bring 4 axis to home"
	,
	"info":
	"Information and credits"
        }
    commands.sort()
    len_commands = len(commands)
    cnt = 0
    print("LIST OF COMMANDS - help()")
    while cnt < len_commands:
        print(commands[cnt])
        cnt += 1

def commandhelp(command):
    print(commands[command])

def settings():
    open_Position_Monitor = False
    open_Delta_Monitor = False
    SM_pattern_choice = "full"

def info():
	print("universal 3d printer (now only delta-printer) g-code interpreter program written in python by Lars Veelaert. Published under open-source regulations. Lveelaert@gmail.com for more info")
def save_settings(filename):
    return 0
    print("Settings Saved!")
    
def adj_unit(unit):
    return 0

def settings():
    open_Position_Monitor = False
    open_Delta_Monitor = False
    SM_pattern_choice = "full"
    rodlength = 290 #Armlengte van pushrod
    (Carr_dist,total_HeightZrod) = (400,175) #lente zijden van 3hoek gevormt door Carriages, Hoogte van begin sliderod van Carriages
    moveZ_eff = 90.04047938952579
    moveZmax = 130.05
    Carriage_Rod_Usage = 35
    Z_start_eff = 100
    Z_start = 136.48440849717431
    (Step_angle_degrees,Step_wheel_dia,Adj_wheel_perimeter) = (1.8,25,1) #stephoek van steppermotor, grote wiel voor draadomtrek, een procentuele aanpasser
    Scale = 1
    Extruderside_length = 100
    Min_step = 0.39269908169872414
    moveRmin = 74.80462193879666
    MinRes = 0.20985415300387444
    #hier uit filteren
    #opdelen in caliibratie en machine

def info():
	print("universal 3d printer (now only delta-printer) g-code interpreter program written in python by Lars Veelaert. Published under open-source regulations. Lveelaert@gmail.com for more info")
print("done")
"""#######################
### MACHINE PARAMETERS ###
#######################"""
print("MACHINE PARAMETERS...")
#PROFILE
profile = "Modular KNEX delta"
calibrated_obj = "HEX.TXT"
#DIMENSIONS
#hier moeten er settings geload worden van machine
#DEFAULT
rodlength = 290 #Armlengte van pushrod
(Carr_dist,total_HeightZrod) = (400,175) #lente zijden van 3hoek gevormt door Carriages, Hoogte van begin sliderod van Carriages
moveZ_eff = 90.04047938952579
moveZmax = 130.05
Carriage_Rod_Usage = 35
HeightZrod = total_HeightZrod - Carriage_Rod_Usage
Z_start_eff = 100
Z_start = 136.48440849717431
(Step_angle_degrees,Step_wheel_dia,Adj_wheel_perimeter) = (1.8,25,1) #stephoek van steppermotor, grote wiel voor draadomtrek, een procentuele aanpasser
Scale = 1
Extruderside_length = 100
Min_step = 0.39269908169872414
moveRmin = 74.80462193879666
MinRes = 0.20985415300387444
MinResE = 0.05
tkinter_installed = "y"
open_Position_Monitor = False
open_Delta_Monitor = False
symdist = 346.41016151277545
midline = 200
PC0x = window_x/2 - midline
PC0y = window_y/2+symdist/3
PC1x = window_x/2
PC1y = window_y/2-symdist/3
PC2x = window_x/2 + midline
PC2y = window_y/2+symdist/3
dC0 = ((Cur_X + midline)**2+(Cur_Y+symdist/3)**2)**(1/2)
dC1 = (Cur_X**2+(Cur_Y-symdist/3)**2)**(1/2)
dC2 = ((Cur_X - midline)**2+(Cur_Y+symdist/3)**2)**(1/2)
auto_start_DM = False
auto_start_position_monitor_var = True
markersize = 100
refreshrate = 2 #in hz
print("done")
"""##################################
### MACHINE FUNCTION DEPENDENCIES ###
##################################"""
print("MACHINE FUNCTION DEPENDENCIES...")
#IMPORTS
if not(debug):
	import RPi.GPIO as GPIO
import time
import math
#PATTERNS
SM_pattern_choice = "full" #keuze van pattern
SM_pattern = []
SM_FULL = [[1,1,0,0],[0,1,1,0],[0,0,1,1],[1,0,0,1]]
if SM_pattern_choice == "full": #max torque, 4 stands
	SM_pattern = SM_FULL
SM_SEMI = [[1,0,0,0],[1,1,0,0],[0,1,0,0],[0,1,1,0],[0,0,1,0],[0,0,1,1],[0,0,0,1],[1,0,0,1]]
if SM_pattern_choice == "semi": #Changing torque doubles stands to 8
	SM_pattern = SM_SEMI
SM_phases_Num=len(SM_pattern); #Pattern length om samen met speed te werken
print("done")
"""######################
### MACHINE FUNCTIONS ###
######################"""
print("MACHINE FUNCTIONS...")
class Bipolar_Stepper_Motor:    
    phase=0;
    pos=0;
    a1=0;#pin numbers
    a2=0;
    b1=0;
    b2=0;
    Min_step = 0
    def __init__(self,a1,a2,b1,b2,Min_step):
        if not(debug):
            GPIO.setmode(GPIO.BOARD);
        self.a1=a1;
        self.a2=a2;
        self.b1=b1;
        self.b2=b2;
        self.Min_step=Min_step
        if not(debug):
            GPIO.setup(self.a1,GPIO.OUT);
            GPIO.setup(self.a2,GPIO.OUT);
            GPIO.setup(self.b1,GPIO.OUT);
            GPIO.setup(self.b2,GPIO.OUT);     
        self.phase=0;      
        self.pos=0;    
    def move(self, dirction, steps, delay):
        for _ in range(int(steps)):
            next_phase=(self.phase+dirction); #% SM_phases_Num hier stond dit maar ik denk dat dat niet goed is als
            if not(debug):
                GPIO.output(self.a1,phase_seq[next_phase][0]);
                GPIO.output(self.b2,phase_seq[next_phase][1]);
                GPIO.output(self.a2,phase_seq[next_phase][2]);
                GPIO.output(self.b1,phase_seq[next_phase][3]);
            self.phase=next_phase;
            self.pos+=dirction;
            time.sleep(delay);
    def unhold(self):
        if not(debug):
            GPIO.output(self.a1,0);
            GPIO.output(self.a2,0);
            GPIO.output(self.b1,0);
            GPIO.output(self.b2,0);
def GCD(a,b):#greatest common diviser
    while b:
       a, b = b, a%b;
    return a;
def LCM(a,b):#least common multipler
    return a*b/GCD(a,b);
def sign(a): #return the sign of number a
    if a>0:
        return 1;
    elif a<0:
        return -1;
    else:
        return 0;
def Motor_StepCarthesian(stepper1, step1, stepper2, step2, speed):
#   control stepper motor 1 and 2 simultaneously
#   stepper1 and stepper2 are objects of Bipolar_Stepper_Motor class
#   direction is reflected in the polarity of [step1] or [step2]
    dir1=sign(step1);  #get dirction from the polarity of argument [step]
    dir2=sign(step2);
    step1=abs(step1);
    step2=abs(step2);
# [total_micro_step] total number of micro steps
# stepper motor 1 will move one step every [micro_step1] steps
# stepper motor 2 will move one step every [micro_step2] steps
# So [total_mirco_step]=[micro_step1]*[step1] if step1<>0;  [total_micro_step]=[micro_step2]*[step2] if step2<>0 
    if step1==0:
        total_micro_step=step2
        micro_step2=1
        micro_step1=step2+100 #set [micro_step1]>[total_micro_step], so stepper motor will not turn
    elif step2==0:
        total_micro_step=step1;
        micro_step1=1;
        micro_step2=step1+100;
    else:
        total_micro_step=LCM(step1,step2);
        micro_step1=total_micro_step/step1;
        micro_step2=total_micro_step/step2;
    T=sqrt(step1**2+step2**2)/speed;      #total time
    dt=T/total_micro_step;                #time delay every micro_step
    for i in range(1,total_micro_step+1):    #i is the iterator for the micro_step. i cannot start from 0
        time_laps=0;
        if ((i % micro_step1)==0):#motor 1 need to turn one step
            stepper1.move(dir1,1,dt/4.0);
            time_laps+=dt/4.0; 
        if ((i % micro_step2)==0):#motor 2 need to turn one step
            stepper2.move(dir2,1,dt/4.0);
            time_laps+=dt/4.0;
        time.sleep(dt-time_laps);
    return 0;
#CALIBRATION FUNCTIONS
def cubesize(filename):
    min_x = 0
    max_x = 0
    min_y = 0
    max_y = 0
    min_z = 0
    max_z = 0
    try:#read and execute G code
        for lines in open(filename,'r'):
            if (lines[0:3]=='G0 ')|(lines[0:3]=='G1 ')|(lines[0:3]=='G01'):
                (x,y,z,e,f)=XYZEFposition(lines)
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
                if z < min_z:
                    min_z = z
                if z > max_z:
                    max_z = z			
    except KeyboardInterrupt:
        pass
    deltax = max_x-min_x
    deltay = max_y-min_y
    deltaz = max_z-min_z
    print("cubesize:deltax,deltay,deltaz= " + str(deltax) +"," + str(deltay) +"," + str(deltaz))
    return deltax,deltay,deltaz;
def trisurfsq(deltax,deltay):
	moveRmin = deltax+2*deltay*math.tan(math.pi/6)
	print("trisurfsq:moveRmin= " + str(moveRmin))
	return moveRmin;
def def_moveZmax (HeightZrod,deltaz):
	moveZmax = HeightZrod-deltaz
	print("def_moveZmax:moveZmax= "+str(moveZmax))
	return moveZmax
def def_carr_dist(moveRmin,moveZmax,rodlength):
	a = math.atan(moveZmax/moveRmin)
	b = math.asin((moveZmax**2+moveRmin**2)**(1/2)/2/rodlength)
	Carr_dist = (3/2)*(math.sin(a-b)*rodlength+(moveRmin*math.cos(math.pi/6))*(2/3))/math.cos(math.pi/6)
	return Carr_dist
def def_Z_start(Carr_dist,moveRmin,rodlength):
	Z_start = (rodlength**2-(Carr_dist*(3)**(1/2)/3+(1/3)*(moveRmin*math.cos(math.pi/6)))**2)**(1/2)
	return Z_start
def def_moveZ_eff(Carr_dist,moveRmin,Z_start,rodlength):
        moveZ_eff = ((rodlength**2-(Carr_dist*(3)**(1/2)/3-(moveRmin*math.cos(math.pi/6)*(2/3)))**2)**(1/2))-Z_start
        return moveZ_eff
def check_possible_print(Z_start_eff,deltaz,moveZ_eff,Z_start,HeightZrod):
	if Z_start_eff <= Z_start:
		rodlengthmin = deltaz + moveZ_eff + Z_start -Z_start_eff
		if HeightZrod >= rodlengthmin:
			return True
		else:
			return False
	else:
		return False
def def_Min_step_wheel(Step_angle_degrees,Step_wheel_dia,Adj_wheel_perimeter,SM_phases_Num):
    Min_step = Step_angle_degrees/360*Step_wheel_dia*math.pi*Adj_wheel_perimeter/(SM_phases_Num/4)
    return Min_step
def ask_par_float(parameter,question):
    print("-----")
    print(question + "?")
    ask = input("(" + str(parameter) + ") #?= ")
    if ask != "":
        return float(ask)
    else:
        return parameter
def ask_par_str(parameter,question):
    print("-----")
    ask = input(question + "?(" + str(parameter) + ") ?= ")
    if ask != "":
        return ask
    else:
        return parameter
print("done")
"""######################
### SETUP ELECTRONICS ###
######################"""
print("SETUP ELECTRONICS...")
if not(debug):
	GPIO.setmode(GPIO.BOARD)
print("done")
"""#####################
### GCODE PARAMETERS ###
#####################"""
print("GCODE PARAMETERS...")
Cur_F = 1800 #snelheid in de gcode opgegeven
Abs_Rel_CoordinateSystem = "abs"
bed_temp = 60
hotend_temp = 120
Unit = "mm"
if Unit == "mm":
    adj_unit = 1
if Unit =="inch":
    adj_unit = 25.4
print("done")
"""####################
### GCODE FUNCTIONS ###
####################"""
print("GCODE FUNCTIONS...")
def update_refresh_clock():
    global refresh_clock
    refresh_clock = time.clock()
def check_time_for_refresh():
    if time.clock()-refresh_clock > 1/refreshrate:
        update_refresh_clock()
        return True
    else:
        return False
def output_of_circle():
    global CC0
    global CC1
    global CC2
    CC0 = p.create_oval((PC0x-dC0)*window_scale,(PC0y-dC0)*window_scale,(PC0x+dC0)*window_scale,(PC0y+dC0)*window_scale, outline = "green")
    CC1 = p.create_oval((PC1x-dC1)*window_scale,(PC1y-dC1)*window_scale,(PC1x+dC1)*window_scale,(PC1y+dC1)*window_scale, outline = "green")
    CC2 = p.create_oval((PC2x-dC2)*window_scale,(PC2y-dC2)*window_scale,(PC2x+dC2)*window_scale,(PC2y+dC2)*window_scale, outline = "green")
    return CC0,CC1,CC2
def update_output_screen(dC0,dC1,dC2,CX,CY,CZ,CE,CF,C0pos,C1pos,C2pos):
    if check_time_for_refresh():
        if open_Position_Monitor:
            if mach_type == "delta":
                p.delete(CC0)
                p.delete(CC1)
                p.delete(CC2)                
                output_of_circle()
                #scale? en update output met parameters in het starten van het sccherm stken
            #moven van mark
        if open_Delta_Monitor:
            print("open delta")
def start_position_monitor():
    global open_Position_Monitor 
    open_Position_Monitor = True
    global p
    Position_Monitor = Tk()
    p = Canvas(Position_Monitor, width = 600, height = 600)
    p.pack()
    global pos_markh
    global pos_markv
    area = p.create_rectangle(2,2,window_x*window_scale,window_y*window_scale)
    if mach_type == "delta":
        output_of_circle()
    pos_markh = p.create_line(window_x/2*window_scale - markersize/2,window_y/2*window_scale,window_x/2*window_scale + markersize/2,window_y/2*window_scale, fill="red")
    pos_markv = p.create_line(window_x/2*window_scale,window_y/2*window_scale-markersize/2,window_x/2*window_scale,window_y/2*window_scale + markersize/2, fill="red")

def auto_start_position_monitor(bool):
    if bool:
        start_position_monitor()
    
def window():
    global window_x
    global window_y
    PC0x = window_x/2 - midline
    PC0y = window_y/2+symdist/3
    PC1x = window_x/2
    PC1y = window_y/2-symdist/3
    PC2x = window_x/2 + midline
    PC2y = window_y/2+symdist/3

def auto_start_delta_monitor(bool):
    if bool:
        start_delta_monitor()
    
def start_delta_monitor():
    global open_Delta_Monitor
    open_Delta_Monitor = True
    global d
    Delta_Monitor = Tk()
    # Setting of size Carriage windows
    d = Canvas(Delta_Monitor)
    d.pack()
def XYZEFposition(lines):
    lines += " "
    #given a movement command line, return the X Y position
    if lines.count('X') == 1:
        xchar_loc=lines.index('X');
        i=xchar_loc+1;
        while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
                i+=1;
        x=float(lines[xchar_loc+1:i]);
    else:
        x = Cur_X
    if lines.count('Y') == 1:
        ychar_loc=lines.index('Y');
        i=ychar_loc+1;
        while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
                i+=1;
        y=float(lines[ychar_loc+1:i]);    
    else:
        y = Cur_Y
    if lines.count('Z') == 1:
        zchar_loc=lines.index('Z');
        i=zchar_loc+1;
        while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
                i+=1;
        z=float(lines[zchar_loc+1:i]);
    else:
        z = Cur_Z		
    if lines.count('E') == 1:
        echar_loc=lines.index('E');
        i=echar_loc+1;
        while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
                i+=1;
        e=float(lines[echar_loc+1:i]); 
    else:
        e = Cur_E
    
    if lines.count('F') == 1:
        fchar_loc=lines.index('F');
        i=fchar_loc+1;
        while (47<ord(lines[i])<58)|(lines[i]=='.')|(lines[i]=='-'):
                i+=1;
        f=float(lines[fchar_loc+1:i]);  	
    else:
        f = Cur_F
    return x,y,z,e,f;

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

def movetoCarthesian(MX,x_pos,dx,MY,y_pos,dy,speed,engraving):
#Move to (x_pos,y_pos) (in real unit)
    stepx=int(round(x_pos/dx))-MX.position;
    stepy=int(round(y_pos/dy))-MY.position;

    Total_step=sqrt((stepx**2+stepy**2));
	
    if Total_step>0:
        if lines[0:3]=='G0 ': #fast movement
            Motor_control.Motor_StepCarthesian(MX,stepx,MY,stepy,50);
        else:
            Motor_control.Motor_StepCarthesian(MX,stepx,MY,stepy,speed);
    return 0;
	
def movetoDelta(x,Cur_X,y,Cur_Y,z,Cur_Z,e,Cur_E,C0,C1,C2,f,Cur_F,rodlength_SQR,midline,symdist): #Linear movement relative
    Dist_movement = (x**2+y**2)**(1/2)
    steps = round(Dist_movement/MinRes+0.5,0)
    #wat is dit?
    if steps != 0:
        if z/MinRes > steps:
            steps = round(z/MinRes+0.5,0)
        if e/MinResE > steps: #moet dit niet eres zijn
            steps = round(e/MinResE+0.5,0)
        x_step = x/steps
        y_step = y/steps
        z_step = z/steps
        e_step = e/steps
        f_step = f/steps
        Dist_steps = Dist_movement/steps
        stp_cnt = 0
        while stp_cnt < steps:
            strt_time = time.clock()
            stp_cnt += 1
            Cur_X += x_step
            Cur_Y += y_step
            Cur_Z += z_step
            Cur_E += e_step
            Cur_F += f_step
            #hier if om naar interfaces te inputn
            if e_step != 0:
                extrusion = round((Cur_E)/E.Min_step,0)-E.pos
                E.move(sign(extrusion),extrusion,0)
            dC0 = ((Cur_X + midline)**2+(Cur_Y+symdist/3)**2)**(1/2)
            dC1 = (Cur_X**2+(Cur_Y-symdist/3)**2)**(1/2)
            dC2 = ((Cur_X - midline)**2+(Cur_Y+symdist/3)**2)**(1/2)
            C0_MOVE= round(((rodlength_SQR-dC0**2)**(1/2)+Cur_Z)/C0.Min_step)-C0.pos
            C1_MOVE= round(((rodlength_SQR-dC1)**(1/2)+Cur_Z)/C1.Min_step)-C1.pos
            C2_MOVE= round(((rodlength_SQR-dC2)**(1/2)+Cur_Z)/C2.Min_step)-C2.pos
	    #extruder moet hier nog bij
            C0.move(sign(C0_MOVE),C0_MOVE,0)
            C1.move(sign(C1_MOVE),C1_MOVE,0)
            C2.move(sign(C2_MOVE),C2_MOVE,0)
            time.sleep(Dist_steps/Cur_F*60-(time.clock()-strt_time))
            update_output_screen(dC0,dC1,dC2,Cur_X,Cur_Y,Cur_Z,Cur_E,Cur_F,C0.pos,C1.pos,C2.pos)

def home_2axes(stepper1,stepper2):
	while (stepper1.pos,stepper2.pos) != (HOME_STEPS,HOME_STEPS):
		stepper1.move(sign(HOME_STEPS - stepper1.pos),1,step_T)
		stepper2.move(sign(HOME_STEPS - stepper2.pos),1,step_T)
	return 0
	
def home_3axes(stepper1,stepper2,stepper3):
	while (stepper1.pos,stepper2.pos,stepper3.pos) != (HOME_STEPS,HOME_STEPS,HOME_STEPS):
		stepper1.move(sign(HOME_STEPS - stepper1.pos),1,step_T)
		stepper2.move(sign(HOME_STEPS - stepper2.pos),1,step_T)
		stepper3.move(sign(HOME_STEPS - stepper3.pos),1,step_T)
	return 0

def home_4axes(stepper1,stepper2,stepper3,stepper4):
	while (stepper1.pos,stepper2.pos,stepper3.pos,stepper4.pos) != (HOME_STEPS,HOME_STEPS,HOME_STEPS,HOME_STEPS):
		stepper1.move(sign(HOME_STEPS - stepper1.pos),1,step_T)
		stepper2.move(sign(HOME_STEPS - stepper2.pos),1,step_T)
		stepper3.move(sign(HOME_STEPS - stepper3.pos),1,step_T)
		stepper4.move(sign(HOME_STEPS - stepper4.pos),1,step_T)
	return 0
print("done")
"""######################
### GCODE INTERPRETER ###
######################"""
print("GCODE INTERPRETER...") 
def gcode_executer(lines):
    print("line")
    if lines==[]:
        1; #blank lines
    elif lines[0:3]=='G90':
        print( 'start');
    elif lines[0:3]=='G20':# working in inch;
        adj_unit = 25.4
        print('Working in inch');
              
    elif lines[0:3]=='G21':# working in mm;
        adj_unit = 1
        print( 'Working in mm'); 

    elif (lines[0:3]=='G0 ')|(lines[0:3]=='G1 ')|(lines[0:3]=='G01'):
        [x,y,z,e,f]=XYZEFposition(lines);
        if Abs_Rel_CoordinateSystem == "abs":
            x -= Cur_X
            y -= Cur_Y
            z -= Cur_Z
            e -= Cur_E
            f -= Cur_F
        if lines[0:3]=='G0 ': #fast movement
            if mach_type == "delta":
                movetoDelta(x,Cur_X,y,Cur_Y,z,Cur_Z,e,Cur_E,C0,C1,C2,f,Cur_F,rodlength_SQR,midline,symdist) # Linear movement relative
            elif mach_type == "cart":
                movetoCarthesian(C0,C1,C2x_pos,dx,MY,y_pos,dy,speed,);
        else:
            if mach_type == "delta":
                movetoDelta(x,Cur_X,y,Cur_Y,z,Cur_Z,e,Cur_E,C0,C1,C2,f,Cur_F,rodlength_SQR,midline,symdist) # Linear movement relative
            elif mach_type == "cart":
                movetoCarthesian(C0,C1,C2x_pos,dx,MY,y_pos,dy,speed,);
            
    elif (lines[0:3]=='G02')|(lines[0:3]=='G03'): #circular interpolation
        return 0
        """old_x_pos=x_pos;
        old_y_pos=y_pos;

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
        
        for i in range(1,no_step+1):
            tmp_theta=i*theta/no_step;
            tmp_x_pos=xcenter+e1[0]*cos(tmp_theta)+e2[0]*sin(tmp_theta);
            tmp_y_pos=ycenter+e1[1]*cos(tmp_theta)+e2[1]*sin(tmp_theta);
            moveto(MX,tmp_x_pos,dx,MY, tmp_y_pos,dy,speed,True);
        """

    #p.move(pos_markh,x/w*200,-(y/h*200))
    #p.move(pos_markv,x/w*200,-(y/h*200))

def print_start(filename):
    try:#read and execute G code
        for lines in open(filename,'r'):
            gcode_executer(lines)
                  
    except KeyboardInterrupt:
        pass

    if mach_type =="delta":
        C0.unhold()
        C1.unhold()
        C2.unhold()
    elif mach_type == "cart":
        MX.unhold();
        MY.unhold();
            
    if not(debug):
        GPIO.cleanup();

    #Home axes #G23():
            #home_3axes(C0,C1,C2)

def gcode_manual_input():
    line = ""
    while line != "q":
        line = input("Give gcode-line (q to exit) ?= ")
        try:#read and execute G code
            gcode_executer(line)
        except KeyboardInterrupt:
            pass
print("done")

"""######################
### MAIN PROGRAM LINE ###
######################"""
#CALIBRATION
print("")
print("Profile= '" + profile +"'")
print("Saved Calibration = '" + calibrated_obj+ "'")
print("Settings= '" + str(settings())+"'")
print("-----")
calibration_done = False
while not(calibration_done):
    calibration_done = True
    print("CALIBRATION(y/n)")
    do_calibrate=input("(n)?=")
    print("-----")
    if do_calibrate == "n":
        calibration_done = True
    elif do_calibrate == "":
        calibration_done = True
    elif do_calibrate == "y":
        done = False
        while not(done):
            print("OPTIONS:")
            print("1.manual")
            print("2.semi-auto")
            print("3.auto")
            print("4.full-auto")
            calibrate = input("#=")
            if calibrate== "1":
                done = True
            if calibrate== "2":
                done = True
            if calibrate== "3":
                done = True
            if calibrate== "4":
                done = True
        if calibrate == "1":
            #hier echt enkel een aantal possible check's en overwrites zetten
            print("under-construction")
        else:
            if calibrate == "2":
                done = False
                while  not(done):
                    print("-----")
                    print("METHOD surscribing print:")
                    print("1.Measurements")
                    print("2.Move Range min")
                    choice = input("#=")
                    print("-----")
                    if choice == "1":
                        deltax = float(input("deltax? ?="))
                        deltay = float(input("deltay? ?="))
                        deltaz = float(input("deltaz? ?="))
                        moveRmin = trisurfsq(deltax,deltay)
                        done = True
                    elif choice == "2":
                        moveRmin = input("Minimum movement R? ?=")
                        deltaz = input("Minimum Z-axis movement")
                        done = True
                    moveZmax = def_moveZmax(HeightZrod,deltaz)
            else:
                filename =ask_par_str(filename,"filename(gcode)")
            if calibrate == "3":
                (deltax,deltay,deltaz) = cubesize(filename)
                moveRmin = trisurfsq(deltax,deltay)
                moveZmax = def_moveZmax(HeightZrod,deltaz)
            elif calibrate == "4":
                print("fully machinized, under-construction, use auto")
            possible_print=False
            while not(possible_print):
                rodlength=ask_par_float(rodlength,"rodlength")
                total_HeightZrod=ask_par_float(total_HeightZrod,"Height Z-rod")
                Carriage_Rod_Usage=ask_par_float(Carriage_Rod_Usage,"How much is used by carriage(mm)?")
                HeightZrod = total_HeightZrod - Carriage_Rod_Usage
                print("Height Z-rod (minus carriage-usage)= " + str(HeightZrod) + "!")
                print("-----")
                choice = input("Calibrate stepper-motors? (y/n)? (n)?=")
                if choice == "y":
                    done = False
                else:
                    done = True
                while not(done):
                    print("-----")
                    print("OPTIONS: threaded, belt")
                    choice = input("?=")
                    if choice == "threaded":
                        Min_step =input("Min_step?=")
                        done = True
                    elif choice == "belt":
                        Step_angle_degrees=ask_par_float(Step_angle_degrees,"Angle(°)/step")
                        Min_step = def_Min_step_wheel(Step_angle_degrees,Step_wheel_dia,Adj_wheel_perimeter,SM_phases_Num)
                        #meer asks
                        done = True  
                if moveRmin >= rodlength:
                    print("ERROR: rodlength too small")
                else:
                    if moveRmin > (rodlength**2-(rodlength-moveZmax)**2)**(1/2):
                        print("ERROR: Height Z-rod not sufficient")
                        possible_print = False
                    else:
                        possible_print = True
            if possible_print:
                Carr_dist = def_carr_dist(moveRmin,moveZmax,rodlength)
                possible_print = False
                while not(possible_print):
                    done = False
                    while not(done):
                        print("-----")
                        print("Set Carriage-Distance to " + str(Carr_dist) + " plus 'extruder-sides'(=" + str(Extruderside_length) + ") or lower, and give the chosen setting minus 'extruder-sides'(=" + str(Extruderside_length) + ")")
                        Carr_dist_eff = float(input("CHOSEN SETTING: CARR-DIST=?"))
                        if Carr_dist_eff <= Carr_dist:
                            Carr_dist = Carr_dist_eff
                            done = True
                    Z_start = def_Z_start(Carr_dist,moveRmin,rodlength)
                    done = False
                    while not(done):
                        print("-----")
                        print("Set Z_start to " + str(Z_start) +" or lower, and give the chosen setting")
                        Z_start_eff = float(input("CHOSEN SETTING: Z_START=?"))
                        print("")
                        if Z_start_eff <= Z_start:
                            done = True
                    moveZ_eff = def_moveZ_eff(Carr_dist,moveRmin,Z_start,rodlength)
                    possible_print = check_possible_print(Z_start_eff,deltaz,moveZ_eff,Z_start,HeightZrod)
                    if not(possible_print):
                        print("not enough move,lower Carr-Dist for a narrower difference with Z_start")
                        overwrite = input("OVERWRITE?(y/n)")
                        if overwrite == "y":
                            possible_print=True
            else:
                calibration_done = False
        #REPRODUCABILITY
        rodlength_SQR = rodlength**2
        Z_start_SQR = Z_start**2
        print("")
        MinRes = (rodlength_SQR-Z_start_SQR)**(1/2)-(rodlength_SQR-(Z_start+Min_step)**2)**(1/2)
        MaxRes = (rodlength_SQR-(Z_start+moveZ_eff-Min_step)**2)**(1/2)-(rodlength_SQR-(Z_start+moveZ_eff)**2)**(1/2)
        GemRes = moveRmin/(moveZ_eff/Min_step)# moveRmin opnieuw berekenen of move Z_step
        print("Calibration completed!")
        print("----------------------")
        print("")
        print("------------------------------------")
        print("MAX RESOLUTION = " + str(MinRes))
        print("MIN RESOLUTION = " + str(MaxRes)) 
        print("GEM RESOLUTION = " + str(GemRes))
        print("------------------------------------")
        print("")
        print("--------------------")
        print("Save Settings? (y/n)")
        save = input("(n) ?= ")
        print("--------------------")
        if save =="y":
            save_settings(input("filename? ?="))
        print("")
    else:
        calibration_done = False
done = False
while not(done):
    print("Tkinter installed? (y/n)")
    ASK_tkinter_installed = input("("+tkinter_installed+") ?=")
    if ASK_tkinter_installed != "":
        tkinter_installed = ASK_tkinter_installed        
    if tkinter_installed == "y":
        from tkinter import *
        done = True
    if tkinter_installed == "n":
        done = True       
print("--------------------")
#MACHINE CALCULATED VARIABLES
midline = Carr_dist/2
symdist = (Carr_dist**2-midline**2)**(1/2)
rodlength_SQR = rodlength**2
#MACHINE SETTINGS
#pin numbers a1 and a2 form coil A; b1 and b2 form coil B
#						 a1,a2,b1,b2
C0=Bipolar_Stepper_Motor(23,22,24,26,Min_step);     
C1=Bipolar_Stepper_Motor(11,7,5,3,Min_step);
C2=Bipolar_Stepper_Motor(12,23,21,23,Min_step);
E=Bipolar_Stepper_Motor(1,1,1,1,1);
Active_E = E
HOME_STEPS = 0
mach_type = "delta";
#ADJUSTEMENT AND BOOT PARAMETERS
(C0.pos,C1.pos,C2.pos) = (50,50,50) #Effectieve stand bij opstarden machine van 2 axis van carriages op sliderod
(C0.phase,C1.phase,C2.phase,E.phase) = (0,0,0,0) #Phase waar de motor inzit op boot
#END
auto_start_position_monitor(auto_start_position_monitor_var)
update_refresh_clock()
print("")
help()
print("-----")
print("'commandhelp(COMMAND)' for an explenation.")
print("To see this list again type 'help()'.")
print("")
print("READY, Waiting for commands...")

