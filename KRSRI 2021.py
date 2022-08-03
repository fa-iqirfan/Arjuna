from dynamixel_sdk import *
import Adafruit_BBIO.GPIO as GPIO 
import Adafruit_BBIO.UART as UART 
import Adafruit_BBIO.ADC as ADC
import smbus
import amg8833_i2c
from configbuatjalanceritanya2 import *
from configKRSRI import *
import time
import serial 
import math
from threading import Thread 

###########################################################
####                      SERVO                        ####
###########################################################

UART.setup("UART1")

SERIALPORT =  "/dev/ttyUSB0"         # "/dev/ttyUSB0"
BAUDRATE = 57600

dxl 		        = PortHandler(SERIALPORT)
packetHandler	    = PacketHandler(1.0)
goal_posSyncWrite 	= GroupSyncWrite(dxl, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
speedSyncWrite		= GroupSyncWrite(dxl, packetHandler, ADDR_MX_MOV_SPEED, LEN_MX_MOV_SPEED)

# Open port
if dxl.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if dxl.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Initialize Dynamixel Servo
for id in range(1,22):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(dxl, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        model_num, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(dxl, id, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Dynamixel#%d type:%s has been successfully connected" % (id , dxl_model[model_num]))

############# FUNGSI PENGIRIMAN DATA SERVO #############

def packet_kirim():
    # Syncwrite speed
    dxl_comm_result = speedSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Syncwrite goal position
    dxl_comm_result = goal_posSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    goal_posSyncWrite.clearParam()
    speedSyncWrite.clearParam()

def packet_read(id, address):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(dxl, id, address)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d: Presvs:%d" % (id ,dxl_present_position))

def paramAdd(id, goalH, goalL, speedH , speedL):
    dxl_add_param_result = goal_posSyncWrite.addParam(id,[goalL,goalH])
    if dxl_add_param_result != True:
        print("[ID:{}] addparam goal position failed".format(id))
        quit()
    dxl_add_param_result = speedSyncWrite.addParam(id,[speedL,speedH]) 
    if dxl_add_param_result != True:
        print("[ID:{}] addparam speed failed".format(id))
        quit()

###########################################################
#=========================================================#
###########################################################

# for sudut in range(0,360,22):
def sinusPatternKotak(A,B,C,D,alpha,phasa,direksi,kaki):
    global x,y,z
    if direksi == 0 :
        if phasa == 0:
            # Y
            var_y4 = ((-0.5)*(A)*math.sin((90-alpha)*0.0174532925)*math.sin(sudutKotakY[sudut]*0.0174532925)) + B

            # X
            var_x4 = (-0.5)*(A)*math.cos((90-alpha)*0.0174532925)*math.sin((sudutKotakX[sudut])*0.0174532925)

            # Z
            var_z4= ((C*math.sin((sudutKotakZ[sudut])*0.0174532925))-D)

        elif phasa == 1:
            # Y
            var_y4 = ((-0.5)*(A)*math.sin((90-alpha)*0.0174532925)*math.sin(sudutKotakYinv[sudut]*0.0174532925)) + B

            # X
            var_x4 = (-0.5)*(A)*math.cos((90-alpha)*0.0174532925)*math.sin((sudutKotakXinv[sudut])*0.0174532925)

            # Z
            var_z4=((C*math.sin((sudutKotakZinv[sudut])*0.0174532925))-D)
    if direksi == 1 :
        if phasa == 0:
            # Y
            var_y4 = ((-0.5)*(-A)*math.sin((90-alpha)*0.0174532925)*math.sin(sudutKotakY[sudut]*0.0174532925)) + B

            # X
            var_x4 = (-0.5)*(-A)*math.cos((90-alpha)*0.0174532925)*math.sin((sudutKotakX[sudut])*0.0174532925)

            # Z
            var_z4= ((C*math.sin((sudutKotakZ[sudut])*0.0174532925))-D)

        elif phasa == 1:
            # Y
            var_y4 = ((-0.5)*(-A)*math.sin((90-alpha)*0.0174532925)*math.sin(sudutKotakYinv[sudut]*0.0174532925)) + B

            # X
            var_x4 = (-0.5)*(-A)*math.cos((90-alpha)*0.0174532925)*math.sin((sudutKotakXinv[sudut])*0.0174532925)

            # Z
            var_z4=((C*math.sin((sudutKotakZinv[sudut])*0.0174532925))-D)
    

    x[kaki] = (var_x4)
    y[kaki] = (var_y4)
    z[kaki] = (var_z4)


def sinusPattern(A,B,C,D,alpha,phasa,direksi,kaki):
    #print(B)
    # ===== Penjelasan =====
    # mencari titik (x,y,z) dengan sinus pattern 
    # A,B,C,D   merupakan perhitungan kaki berdasarkan inverse kinematik
    # alpha      merupakan sudut dasar teta1 (60 depan, 90 tengah, 120 belakang)
    # phasa     menentukan group 3-3 (setengah siklus sinus) untuk menentukan kaki yang menyentuh lantai dan yang mengangkat
    # direksi   arah pergerakan robot (maju / mundur)    0 = maju, 1 = mundur
    # kaki      menentukan kaki mana yang dihitung (1 kanan depan, 2 kanan tengah, 3 kanan belakang, 4 kiri depan, 5 kiri tengah, 6 kiri belakang)
    # ======================
    if direksi == 0 :
        if phasa == 0:
            var_y4 = (float(-0.5)*float(A)*math.sin((90-alpha)*0.0174532925)*math.sin(float(sudut)*0.0174532925)) + float(B)
            # di python math.pi atau 180*0.0174532925 dan 360*0.0174532925  tidak menghasilkan 0 jadi dibuat ifelse untuk sin 180 dan 360
            if sudut == 180 or sudut == 360 :
                var_x4 = (-0.5)*(A)*math.cos((90-alpha)*0.0174532925)*math.sin(0*0.0174532925)    
            else:
                var_x4 = (-0.5)*(A)*math.cos((90-alpha)*0.0174532925)*math.sin(sudut*0.0174532925)

            if (sudut>=0) and (sudut<90):
                var_z4=-D
            elif (sudut>=90) and (sudut<270):
                var_z4=((C*math.sin((sudut-90)*0.0174532925))-D)
            elif (sudut>=270) and (sudut<=360):
                var_z4=-D
        elif phasa == 1:
            var_y4 = ((0.5)*(A)*math.sin((90-alpha)*0.0174532925)*math.sin(sudut*0.0174532925)) + B
            # di python math.pi atau 180*0.0174532925 dan 360*0.0174532925  tidak menghasilkan 0 jadi dibuat ifelse untuk sin 180 dan 360
            if sudut == 180 or sudut == 360 :
                var_x4 = (0.5)*(A)*math.cos((90-alpha)*0.0174532925)*math.sin(0*0.0174532925)    
            else:
                var_x4 = (0.5)*(A)*math.cos((90-alpha)*0.0174532925)*math.sin(sudut*0.0174532925)

            if (sudut>=0) and (sudut<90):
                var_z4=((-C*math.sin((sudut-90)*0.0174532925))-D)
            elif (sudut>=90) and (sudut<270):
                var_z4=-D
            elif (sudut>=270) and (sudut<=360):
                var_z4=((-C*math.sin((sudut-90)*0.0174532925))-D)

    elif direksi == 1:
        if phasa == 0:
            var_y4 = ((-0.5)*(-A)*math.sin((90-alpha)*0.0174532925)*math.sin(sudut*0.0174532925)) + B
            # di python math.pi atau 180*0.0174532925 dan 360*0.0174532925  tidak menghasilkan 0 jadi dibuat ifelse untuk sin 180 dan 360
            if sudut == 180 or sudut == 360 :
                var_x4 = (-0.5)*(-A)*math.cos((90-alpha)*0.0174532925)*math.sin(0*0.0174532925)    
            else:
                var_x4 = (-0.5)*(-A)*math.cos((90-alpha)*0.0174532925)*math.sin(sudut*0.0174532925)

            if (sudut>=0) and (sudut<90):
                var_z4=-D
            elif (sudut>=90) and (sudut<270):
                var_z4=((C*math.sin((sudut-90)*0.0174532925))-D)
            elif (sudut>=270) and (sudut<=360):
                var_z4=-D
        elif phasa == 1:
            var_y4 = ((0.5)*(-A)*math.sin((90-alpha)*0.0174532925)*math.sin(sudut*0.0174532925)) + B
            # di python math.pi atau 180*0.0174532925 dan 360*0.0174532925  tidak menghasilkan 0 jadi dibuat ifelse untuk sin 180 dan 360
            if sudut == 180 or sudut == 360 :
                var_x4 = (0.5)*(-A)*math.cos((90-alpha)*0.0174532925)*math.sin(0*0.0174532925)    
            else:
                var_x4 = (0.5)*(-A)*math.cos((90-alpha)*0.0174532925)*math.sin(sudut*0.0174532925)

            if (sudut>=0) and (sudut<90):
                var_z4=((-C*math.sin((sudut-90)*0.0174532925))-D)
            elif (sudut>=90) and (sudut<270):
                var_z4=-D
            elif (sudut>=270) and (sudut<=360):
                var_z4=((-C*math.sin((sudut-90)*0.0174532925))-D)

    x[kaki] = var_x4
    y[kaki] = var_y4
    z[kaki] = var_z4
        

def inverseBaru(x,y,z):
    teta1 = math.atan2(x,y)
    A = math.sqrt(x**2 + y**2) - l3 
    B = math.sqrt(A**2 + z**2)
    val1 = ((l2**2 - B**2 - l1**2) / (-2 * B * l1)) 
    if val1 < -1 :
        val1 = -1
    elif val1 > 1 :
        val1 = 1
    alfa = math.acos(val1)
    beta = math.atan2(z,A)
    teta2 = alfa + beta
    val2 = (B**2 - l2**2 - l1**2) / (-2 * l1 * l2)
    if val2 < -1 :
        val2 = -1
    elif val2 > 1 :
        val2 = 1
    gama = math.acos(val2)
    teta3 = math.pi + gama
    return teta1, teta2, teta3

def getSudut(kaki):
    ha, hb, hc = inverseBaru(x[kaki],y[kaki],z[kaki])
    sudutA[kaki] = ((ha*57.2957795))
    sudutB[kaki] = ((hb*57.2957795))
    sudutC[kaki] = ((hc*57.2957795))

    # =======      pengaturan setpoint =========
    # basic
    # Sudut A + 150 
    # Sudut B + 180     #dari pengukuran ~162 or 163 or 164     // Kanan 180 - B
    # Sudut (360 - C) + 180 
    # ==========================================
    if kaki == 0 or kaki == 1 or kaki == 2:
        sudutA[kaki] = sudutA[kaki] + setpointA[kaki]
        sudutB[kaki] = setpointB[kaki] - sudutB[kaki] 
        sudutC[kaki] = sudutC[kaki] - setpointC[kaki]
    else:
        sudutA[kaki] = sudutA[kaki] + setpointA[kaki]   # Servo AX titik tengahnya 150 degree
        sudutB[kaki] = sudutB[kaki] + setpointB[kaki]   # Servo MX titik tengahnya 180 degree
        sudutC[kaki] = (360 - sudutC[kaki]) + setpointC[kaki]

    # setpointA = [ 150, 150, 150, 150, 150, 150 ]
    # setpointB = [ 180, 180, 180, 180, 180, 180 ]
    # setpointC = [ 90, 90, 90, 90, 90, 90 ]

modeKecepatan = "normal"

def translate(kaki):
    for i in range(3):
        if i == 0 :
            # untuk AX base (teta1)
            desimal = int( (sudutA[kaki] / 360) * 4096)         
            H = int(desimal/256)
            L = int(desimal-(H*256))
        elif i == 1:
            # untuk MX (teta2)
            desimal = int( (sudutB[kaki] / 360) * 4096)         
            H = int(desimal/256)
            L = int(desimal-(H*256))
        elif i == 2:
            # untuk MX (teta3)
            desimal = int( (sudutC[kaki] / 360) * 4096)
            H = int(desimal/256)
            L = int(desimal-(H*256))
        # endif

        if tangga == True :
            Hspd=0x3
            Lspd=0xE8
        elif modeKecepatan == "lambat":
            Hspd=0x0
            Lspd=0x7D
        else:
            Hspd=0x3 
            Lspd=0xFF

        idServo = (kaki * 3) + i + 1   
        paramAdd(idServo , H, L, speedH = Hspd, speedL = Lspd) 

def gerakanKaki(A, alpha, phasa, direksi, kaki):
    if tangga == True:
        #print("masuk tangga")
        sinusPatternKotak(A, B[kaki], C[kaki], D[kaki], alpha, phasa, direksi, kaki )
    else:
        sinusPattern(A, B[kaki], C[kaki], D[kaki], alpha, phasa, direksi, kaki )
    getSudut(kaki)
    translate(kaki)

def allGerakanKaki(mode,jarak):
    global pwmKanan
    global pwmKiri
    if modeGripper == True:
        pass
    if jarak == "pwm" :
        for i in range(6):
            if i < 3:
                pwm = pwmKanan
            else:
                pwm = pwmKiri
            gerakanKaki(
                    float(pwm),
                    configKaki[mode][i][1],
                    configKaki[mode][i][2],
                    configKaki[mode][i][3],
                    configKaki[mode][i][4],
                )
    elif jarak == None:
        for i in range(6):
            gerakanKaki(
                    configKaki[mode][i][0],
                    configKaki[mode][i][1],
                    configKaki[mode][i][2],
                    configKaki[mode][i][3],
                    configKaki[mode][i][4],
                )
    else:
        for i in range(6):
            gerakanKaki(
                    jarak,
                    configKaki[mode][i][1],
                    configKaki[mode][i][2],
                    configKaki[mode][i][3],
                    configKaki[mode][i][4],
                )
direksiSusur = "maju"
def perintahGerak(mode, aturNilaiA=False, nilaiA=1):
    global addGrip
    if mode == modePWM:
        if direksiSusur == "maju":
            allGerakanKaki(modeMaju,"pwm")
        elif direksiSusur == "mundur":
            allGerakanKaki(modeMundur,"pwm")
    elif aturNilaiA == True:
        allGerakanKaki(mode,nilaiA)
    else:
        allGerakanKaki(mode,None)

    if addGrip == True:
        gripper(gripReq)
        addGrip = False
    
    packet_kirim()

sudahBerdiri = False
targetGerak = True
geraksiklus = False
onGerak = False

def gerak():
    global sudut, onGerak
    global sudahBerdiri
    global geraksiklus
    global langsungBerdiri
    global awalGerak

    if langsungBerdiri == True:
        onGerak = True
        sudut = 0   #360
        perintahGerak(modeBerdiri)
        modeGerak[2] = oldValue
        time.sleep(0.07)
        sudahBerdiri = True
        langsungBerdiri = False
        awalGerak = True
    elif modeGripper == True:
        pass
    elif modeGerak[2] == newValue or modeGerak[0] == modePWM:
        onGerak = True
        sudahBerdiri = False
        geraksiklus = False
        if modeGerak[0] == modePWM:
            sudut = 0
            while modeGerak[0] == modePWM:
                if sudut < 360 :
                    sudut += 45
                else:
                    sudut = 45
                goal_posSyncWrite.clearParam()
                speedSyncWrite.clearParam()
                perintahGerak(modeGerak[0])
                if tangga==False and modeKecepatan == "normal":
                    time.sleep(0.03)
                elif tangga == True :
                    time.sleep(0.045)
                elif modeKecepatan == "lambat":
                    time.sleep(0.025)
                
        elif modeGerak[0] == modeTengokKiri or modeGerak[0] == modeTengokKanan:
            sudut = 90
            print("masuk mode tengok", modeGerak[0])
            if modeGerak[3] != 0:
                perintahGerak(modeGerak[0],True,modeGerak[3])
            else:
                perintahGerak(modeGerak[0])
            if tangga==False and modeKecepatan == "normal":
                time.sleep(0.025)
            elif tangga == True :
                time.sleep(0.045)
            elif modeKecepatan == "lambat":
                time.sleep(0.325)
        else:
            for i in range(modeGerak[1]):  
                if awalGerak == True:
                    berdiriJalan()
                sudut = 0
                while sudut < 360:       
                    sudut += 45
                    if modeGerak[3] != 0:
                        perintahGerak(modeGerak[0],True,modeGerak[3])
                    else:
                        perintahGerak(modeGerak[0])
                    if tangga == True:
                        time.sleep(0.045)
                    elif modeGerak[0] == 5 or modeGerak[0] == 6:
                        time.sleep(0.01)
                    elif modeGerak[0] == 3 or modeGerak[0]==4 or modeGerak[0]==12 or modeGerak[0]==13 or modeGerak[0]==5 or modeGerak[0]==6:
                        time.sleep(0.02)
                    elif modeGerak[3] != 0 and modeGerak[3] < 3:
                        time.sleep(0.01)
                    else:
                        time.sleep(0.04)
                    
                awalGerak = False
        modeGerak[2] = oldValue
        geraksiklus = True
    elif sudahBerdiri == True:
        pass
    elif targetGerak == True:
        sudut = 0
        print("masuk else")
        berdiri()
    onGerak = False

langsungBerdiri = False
def berdiri():
    global langsungBerdiri
    langsungBerdiri = True

def berdiriLangsung():
    global sudut, awalGerak
    sudut = 360
    perintahGerak(modeBerdiri)
    modeGerak[2] = oldValue
    time.sleep(0.07)
    awalGerak = True

awalGerak = True
def berdiriJalan():
    global sudut
    sudut = 360
    perintahGerak(modeBerdiriJalan)
    modeGerak[2] = oldValue
    time.sleep(0.02)

gripReq = "naik"
addGrip = False

def grip(state):
    global gripReq, addGrip
    if onGerak == False:
        gripper(state)
        packet_kirim()
        addGrip = False
    else:
        gripReq = state
        addGrip = True

def modePergerakan(mode, siklus=1, nilaiA=0):
    modeGerak[0] = mode
    modeGerak[1] = siklus
    modeGerak[2] = newValue
    modeGerak[3] = nilaiA


###########################################################
#=========================================================#
###########################################################

###########################################################
####               Fungsi pembacaan sensor             ####
###########################################################

######################### Ultrasonic ########################
ser = serial.Serial(port = "/dev/ttyO1", baudrate=57600)
ser.close()
ser.open()
if ser.isOpen():
    print("Serial is open!")
else :
    print("serial not open!")

charUltrasonic = {
  "kisa": "A",
  #"kide": "B",
  "tangga": "C",        # deki
  "deka": "D",         # deka
  #"kade": "E",
  "kasa": "F",
  "belakang": "G",      # beka
  "gripper": "H",       # cadangan1
}

nilaiUltrasonic = {
  "kisa": 0,
  #"kide": 0,
  "tangga": 0,
  "deka": 0,           # deka
  #"kade": 0,
  "kasa": 0,
  "belakang": 0,        # beka
  "gripper": 0,         # cadangan1
}

def ultrasonic(arah):
    if(arah == "semua"):
        for arahindict in nilaiUltrasonic:
            ser.flushInput()
            ser.write((charUltrasonic[arahindict]).encode())
            nilaiUltrasonic[arahindict] = int(ser.readline())
            time.sleep(0.01)
    elif(arah == "X"):
        ser.flushInput()
        ser.write(("X").encode())
    elif(arah == "Z"):
        ser.flushInput()
        ser.write(("Z").encode())
    elif(arah == "V"):
        ser.flushInput()
        ser.write(("V").encode())
    elif(arah=="L"):
        ser.flushInput()
        ser.write(("L").encode())
    elif(arah=="N"):
        ser.flushInput()
        ser.write(("N").encode())
    elif(arah == "K"):
        ser.flushInput()
        ser.write(("K").encode())
        var = ser.readline()    # .strip().strip('\x00')
        #print(var)
        #print(arah,"raw =",var)
        try:
            nilaiUltrasonic[arah] = int(var)
        except:
            print("rdrtr")
            pass
        #nilaiUltrasonic[arah] = int(var)
    else:
        ser.flushInput()
        ser.write((charUltrasonic[arah]).encode())
        var = ser.readline()    # .strip().strip('\x00')
        #print(var)
        #print(arah,"raw =",var)
        try:
            nilaiUltrasonic[arah] = int(var)
        except:
            print("rdrtr")
            pass
        #nilaiUltrasonic[arah] = int(var)


########################### Flame ###########################
flameKanan = "P9_18"
flameKiri = "P9_23" 

GPIO.setup(flameKanan,GPIO.IN)
#GPIO.setup(flameKiri,GPIO.IN)

def flame(arah):
    if(GPIO.input(arah) == 0):
        print("ada",arah)
        return True
    else:
        return False


########################## Thermal ##########################
thermalSensor = []
def thermalCreate():
    global thermalSensor
    t0 = time.time()
    while (time.time()-t0)<0.3: # wait 1sec for sensor to start
        try:
            # AD0 = GND, addr = 0x68 | AD0 = 5V, addr = 0x69
            thermalSensor = amg8833_i2c.AMG8833(addr=0x69) # start AMG8833
        except:
            thermalSensor = amg8833_i2c.AMG8833(addr=0x68)
        finally:
            pass
    time.sleep(0.1) # wait for sensor to settle

# If no device is found, exit the script
thermalCreate()
if thermalSensor==[]:
    print("No AMG8833 Found - Check Your Wiring")
else:
    print("AMG8833 Ready")
pixels = []
pix_thermal_to_read = 64 # read all 64 pixels
def readThermal():
    status,pixels = thermalSensor.read_temp(pix_thermal_to_read) # read pixels with status
    print(pixels)
    return pixels


def errorThermal():
    pixel = readThermal()
    error_x = []
    error_y = []    
    idx_api = [i for i, x in enumerate(pixel) if x >= 26]
    for i in range(len(idx_api)):
        if idx_api[i]%8 < 4:
            error_x.append(idx_api[i]%8 - 4)
            error_y.append(int(idx_api[i]/8) - 4)
        else:
            error_x.append(idx_api[i]%8 - 3)
            error_y.append(int(idx_api[i]/8) - 3)
    return error_x, error_y, idx_api

def printApi():
    err_x, err_y , idx= errorThermal()
    val = [0 for _ in range(64)]
    for i in range(len(idx)):
        val[idx[i]] = "X"
    for i in range(8):
        print((val[int(i*8)], val[int(i*8)+1], val[int(i*8)+2], val[int(i*8)+3], val[int(i*8)+4], val[int(i*8)+5], val[int(i*8)+6], val[int(i*8)+7]))
    print("============================================================")



def printThermalRapi():
    hasil = readThermal()
    print("len thermal = {}".format(len(hasil)))
    #time.sleep(1)
    #print(hasil)
    #for i in range(len(hasil)):
    #    print(hasil[i])
    for i in range(8):
        print((hasil[int(i*8)], hasil[int(i*8)+1], hasil[int(i*8)+2], hasil[int(i*8)+3], hasil[int(i*8)+4], hasil[int(i*8)+5], hasil[int(i*8)+6], hasil[int(i*8)+7]))
    print("============================================================")
########################### Color ##########################
s1 = "P8_8"
s2 = "P8_15"
s3 = "P8_16"
outColor = "P9_12"
red = 0
green = 0
blue = 0

# parameter warna oren
red_low = 0
red_hi = 18
blue_low = 40
blue_hi = 75
green_low = 50
green_hi = 90

# parameter warna kuning
red_low_k = 19
red_hi_k = 70
blue_low_k = 60
blue_hi_k = 120
green_low_k = 25
green_hi_k = 90

GPIO.setup(outColor,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(s1,GPIO.OUT)
GPIO.setup(s2,GPIO.OUT)
GPIO.setup(s3,GPIO.OUT)
GPIO.output(s1,GPIO.LOW)

def readColor(s2_state,s3_state):
    GPIO.output(s2,s2_state)
    GPIO.output(s3,s3_state)
    start = time.time()*1000
    GPIO.wait_for_edge(outColor, GPIO.FALLING)
    waktu = time.time()*1000 - start
    return waktu

def cekColor():
    global red, green, blue
    red   = readColor(GPIO.LOW, GPIO.LOW)
    blue  = readColor(GPIO.LOW, GPIO.HIGH)
    green = readColor(GPIO.HIGH, GPIO.HIGH)
    if (red > red_low and red <= red_hi ) and (blue > blue_low and blue <= blue_hi) and (green > green_low and green <= green_hi):  # IF warna == oren *Bukan nilai rgb oren, tapi hasil deteksi warna oren di program baca raw
        return "Oren"
    elif (red > red_low_k and red <= red_hi_k ) and (blue > blue_low_k and blue <= blue_hi_k) and (green > green_low_k and green <= green_hi_k):  # IF warna == oren *Bukan nilai rgb oren, tapi hasil deteksi warna oren di program baca raw
        return "kuning"
    else:
        return "warna lain"


ADC.setup("P9_40", ADC.IN)
irBelakang = ADC.read("P9_40")
ADC.setup("P9_37", ADC.IN)()
irDepan = ADC.read("P9_37")

irDepan = 0 
irBelakang = 0
def infraredB():
    global irBelakang,targetGerak
    targetGerak = False
    irBelakang = ADC.read("P9_40")
    if(int(irBelakang) < 1):
        targetGerak = False
        print("maju ruang2")
        print(irBelakang)
        irBelakang = ADC.read("P9_40")
        modePergerakan(modeMaju,nilaiA=4)
    elif(int(irBelakang) == 1):
        print("berhenti ruang2")
        print(irBelakang)
        menghadap("kanan")
        targetGerak = True
def infraredD():
    global irDepan, targetGerak
    targetGerak = False
    irDepan = ADC.read("P9_37")
    if(int(irDepan) > 0):
        print("maju ruang1")
        print(irDepan)
        modePergerakan(modeMaju, nilaiA = 4)
        irDepan = ADC.read("P9_37")
    elif(int(irDepan) == 0):
        print("mundur ruang1")
        print(irBelakang)
    targetGerak = True




########################### proxi ###########################
# GPIO.setup("P9_37", GPIO.OUT)
# GPIO.input("P9_37")

# GPIO.setup("P9_40", GPIO.OUT)
# GPIO.input("P9_40")

########################## Compass ##########################
buscompas =smbus.SMBus(2)
addresscmps=0x60

rawCompas = 0       # nilai compas dari cmps12 (0-255)

# settingan nilai bearing compas (0-255) 
nilaiAtasLapangan = 0
nilaiKiriLapangan = 0
nilaiKananLapangan = 0
nilaiBawahLapangan = 0

nilaiArahCompas = {
    "atas" : 17.0,
    "kiri" : 272.21, #271.4
    "kanan" : 91.4,
    "bawah" : 177.2,
}
def enableCompas(input="X"):
    if input == "mag":
        ser.flushInput()
        ser.write(("N").encode())
    elif input == "state":
        ser.flushInput()
        ser.write(("M").encode())
    else:
        ser.flushInput()
        ser.write(("X").encode())

def compas():
    global rawCompas
    ser.flushInput()
    ser.write(("K").encode())
    var = ser.readline()   
    try:
        rawCompas = int(var)
        derajatCompas =  (rawCompas+1) * 1.40625
        time.sleep(0.01)
        return rawCompas, derajatCompas
    except:
        print("read compas error - communication problem")
        derajatCompas =  (rawCompas+1) * 1.40625
        time.sleep(0.01)
        return rawCompas, derajatCompas

def cekStateCompas():
    data = buscompas.read_byte_data(addresscmps,0x1E)
    print(data)

sortedNilaiCompas=sorted(nilaiArahCompas.items(), key = lambda kv:(kv[1], kv[0]))
q1 = 0
q2 = 0
q3 = 0
q4 = 0
edge = ""
arahMuka = ""

def getQuarter():
    global q1,q2,q3,q4,edge
    q1 = sortedNilaiCompas[0][1] - 45
    if q1 <0 :
        q1+=360
        edge = "hi"
    else:
        edge = "low"
    q2 = sortedNilaiCompas[1][1] - 45
    q3 = sortedNilaiCompas[2][1] - 45
    q4 = sortedNilaiCompas[3][1] - 45

def cekArahMuka():
    global arahMuka
    ultrasonic("N")
    raw, derajat = compas()
    getQuarter()
    if edge == "low":
        if derajat > q1 and derajat <= q2 :
            arahMuka = sortedNilaiCompas[0][0]
        elif derajat > q2 and derajat <= q3 :
            arahMuka = sortedNilaiCompas[1][0]
        elif derajat > q3 and derajat <= q4 :
            arahMuka = sortedNilaiCompas[2][0]
        elif derajat > q4 or derajat <= q1 :
            arahMuka = sortedNilaiCompas[3][0]
    elif edge == "hi":
        if derajat > q1 or derajat <= q2 :
            arahMuka = sortedNilaiCompas[0][0]
        elif derajat > q2 and derajat <= q3 :
            arahMuka = sortedNilaiCompas[1][0]
        elif derajat > q3 and derajat <= q4 :
            arahMuka = sortedNilaiCompas[2][0]
        elif derajat > q4 and derajat <= q1 :
            arahMuka = sortedNilaiCompas[3][0]
    ultrasonic("Z")





####################### Tombol mulai ########################
Mulai = False
Selesai = False

tombolMulai = "P8_17"
GPIO.setup(tombolMulai, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def tungguTombolMulai():
    global Mulai
    while(not Mulai):
        print("test")
        GPIO.wait_for_edge(tombolMulai, GPIO.FALLING)
        Mulai = True



########################### Pompa ###########################
GPIO.setup("P8_11", GPIO.OUT)
GPIO.output("P8_11", GPIO.HIGH)

def semprot():
    ser.flushInput()
    code = 'P'
    ser.write(code.encode())
    GPIO.output("P8_11", GPIO.HIGH)

########################### Mosfet ###########################
GPIO.setup("P9_14", GPIO.OUT)
GPIO.output("P9_14", GPIO.HIGH)


########################### gripper #########################
servoGripperLinear = 20
servoGripperCengkram = 21
modeGripper = False

g_naik  = 24 
g_turun = 173 
g_buka  =  130 
g_setengah = 193
g_tutup  = 247

def gripper(perintah):
    if(perintah == "turun"):
        sudutg = g_turun
        idServog = servoGripperLinear
    elif(perintah == "naik"):
        sudutg = g_naik
        idServog = servoGripperLinear
    elif(perintah == "buka"):
        sudutg = g_buka
        idServog = servoGripperCengkram
    elif(perintah == "setengah"):
        sudutg = g_setengah
        idServog = servoGripperCengkram
    elif(perintah == "tutup"):
        sudutg = g_tutup
        idServog = servoGripperCengkram

    desimal = int( (sudutg / 300) * 1024)
    print(sudutg)
    H = int(desimal/256)
    L = int(desimal-(H*256))
    paramAdd(idServog , H, L, speedH = 0x2, speedL = 0x68) 
    print("telah addParam gripper")


#############################################################
#################### Fungsi Skema Logika ####################

############### Fungsi menghadap sisi lapangan ############## 
def menghadap(arah):
    global targetGerak
    enableCompas("mag")
    print("masuk menghadap",arah)
    raw, compasSekarang = compas()
    targetGerak = False
    countBatas = 0
    if int(compasSekarang) > nilaiArahCompas[arah] and compasSekarang-nilaiArahCompas[arah] < 180:
        print("hdp-1")
        while compasSekarang > nilaiArahCompas[arah]+ 3 and countBatas < 60:
            if abs(compasSekarang-nilaiArahCompas[arah]) > 15:
                modePergerakan(modePutarKiri)
                _, compasSekarang = compas()
                #print(compasSekarang)
            elif abs(compasSekarang-nilaiArahCompas[arah]) >0 and abs(compasSekarang-nilaiArahCompas[arah])<=15:
                modePergerakan(modePutarKiri, nilaiA = 0.5)
                _, compasSekarang = compas()
                #print(compasSekarang)
            countBatas += 1
            print(countBatas)
            time.sleep(0.01)
    
    elif int(compasSekarang) < nilaiArahCompas[arah] and nilaiArahCompas[arah]-compasSekarang < 180:
        print("hdp-2")
        while compasSekarang < nilaiArahCompas[arah] - 3 and countBatas < 60:
            if abs(compasSekarang-nilaiArahCompas[arah]) > 15:
                modePergerakan(modePutarKanan)
                _, compasSekarang = compas()
                #print(compasSekarang)
            elif abs(compasSekarang-nilaiArahCompas[arah])>0 and abs(compasSekarang-nilaiArahCompas[arah]) <=15:  #25
                modePergerakan(modePutarKanan, nilaiA = 0.5)
                _, compasSekarang = compas()
                #print(compasSekarang)
            countBatas +=1
            #print(countBatas)
            time.sleep(0.01)

    elif compasSekarang > nilaiArahCompas[arah] and compasSekarang-nilaiArahCompas[arah] > 180:
        print("hdp-3")
        awal = compasSekarang
        while compasSekarang < nilaiArahCompas[arah]+360 - 3:
            if compasSekarang > 360:
                if abs(compasSekarang-nilaiArahCompas[arah]-360) > 15:  #20
                    modePergerakan(modePutarKanan)
                    _, compasSekarang = compas()
                    #print(compasSekarang)
                    if compasSekarang < awal-180 :
                        compasSekarang += 360
                else:
                    modePergerakan(modePutarKanan,nilaiA=0.5)
                    _, compasSekarang = compas()
                    #print(compasSekarang)
                    if compasSekarang < awal-180 :
                        compasSekarang += 360
            else:
                if abs(compasSekarang-nilaiArahCompas[arah]) > 15:  #20
                    modePergerakan(modePutarKanan)
                    _, compasSekarang = compas()
                    #print(compasSekarang)
                    if compasSekarang < awal-180 :
                        compasSekarang += 360
                else:
                    modePergerakan(modePutarKanan,nilaiA=0.5)
                    _, compasSekarang = compas()
                    #print(compasSekarang)
                    if compasSekarang < awal-180 :
                        compasSekarang += 360

            #time.sleep(0.01)

    elif compasSekarang < nilaiArahCompas[arah] and nilaiArahCompas[arah]-compasSekarang > 180:
        awal = compasSekarang
        while compasSekarang > nilaiArahCompas[arah]-360 +3:
            if compasSekarang < 0:
                if abs(compasSekarang-nilaiArahCompas[arah]+360) > 15: #20
                    modePergerakan(modePutarKiri)
                    _, compasSekarang = compas()
                    #print(compasSekarang)
                    if compasSekarang > awal + 180 :
                        compasSekarang -= 360
                else:
                    modePergerakan(modePutarKiri,nilaiA=0.5)
                    _, compasSekarang = compas()
                    #print(compasSekarang)
                    if compasSekarang > awal + 180 :
                        compasSekarang -= 360
            else:
                if abs(compasSekarang-nilaiArahCompas[arah]) > 15: #20
                    modePergerakan(modePutarKiri)
                    _, compasSekarang = compas()
                    #print(compasSekarang)
                    if compasSekarang > awal + 180 :
                        compasSekarang -= 360
                else:
                    modePergerakan(modePutarKiri,nilaiA=0.5)
                    _, compasSekarang = compas()
                    #print(compasSekarang)
                    if compasSekarang > awal + 180 :
                        compasSekarang -= 360

            #time.sleep(0.01)

    berdiri()
    cekArahMuka()
    ultrasonic("Z")
    time.sleep(0.05)
    print("selesai menghadap")
    targetGerak = True

def putar(arah):
    #print("putar",arah)
    cekArahMuka()
    print("putar",arah,arahMuka)
    if arah == "kiri" :
        if arahMuka == "atas":
            menghadap("kiri")
        elif arahMuka == "kiri":
            menghadap("bawah")
        elif arahMuka == "bawah":
            menghadap("kanan")
        elif arahMuka == "kanan":
            menghadap("atas")
    elif arah == "kanan":
        if arahMuka == "atas":
            menghadap("kanan")
        elif arahMuka == "kiri":
            menghadap("atas")
        elif arahMuka == "bawah":
            menghadap("kiri")
        elif arahMuka == "kanan":
            menghadap("bawah")

def berbelok(arah):
    global targetGerak
    print("masuk menghadap",arah)
    raw, compasSekarang = compas()
    targetGerak = False
    if compasSekarang > nilaiArahCompas[arah] and compasSekarang-nilaiArahCompas[arah] < 180:
        while compasSekarang > nilaiArahCompas[arah] + 5:
            modePergerakan(modeBelokKiri)
            _, compasSekarang = compas()
            print(compasSekarang)
    
    elif compasSekarang < nilaiArahCompas[arah] and nilaiArahCompas[arah]-compasSekarang < 180:
        while compasSekarang < nilaiArahCompas[arah] - 5:
            modePergerakan(modeBelokKanan)
            _, compasSekarang = compas()
            print(compasSekarang)

    elif compasSekarang > nilaiArahCompas[arah] and compasSekarang-nilaiArahCompas[arah] > 180:
        awal = compasSekarang
        while compasSekarang < nilaiArahCompas[arah]+360 - 5:
            modePergerakan(modeBelokKanan)
            _, compasSekarang = compas()
            print(compasSekarang)
            if compasSekarang < awal-180 :
                compasSekarang += 360
    
    elif compasSekarang < nilaiArahCompas[arah] and nilaiArahCompas[arah]-compasSekarang > 180:
        awal = compasSekarang
        while compasSekarang > nilaiArahCompas[arah]-360 +5:
            modePergerakan(modeBelokKiri)
            _, compasSekarang = compas()
            print(compasSekarang)
            if compasSekarang > awal + 180 :
                compasSekarang -= 360
    berdiri()
    cekArahMuka()
    time.sleep(0.2)
    targetGerak = True

def belok(arah):
    global lagibelok
    cekArahMuka()
    print("belok",arah,arahMuka)
    if arah == "kiri" :
        if arahMuka == "atas":
            berbelok("kiri")
        elif arahMuka == "kiri":
            berbelok("bawah")
        elif arahMuka == "bawah":
            berbelok("kanan")
        elif arahMuka == "kanan":
            berbelok("atas")
    elif arah == "kanan":
        if arahMuka == "atas":
            berbelok("kanan")
        elif arahMuka == "kiri":
            berbelok("atas")
        elif arahMuka == "bawah":
            berbelok("kiri")
        elif arahMuka == "kanan":
            berbelok("bawah")

def belokKiri():
    global belok
    global targetGerak
    modeTangga(False)
    menghadap("kanan")
    mulai = time.time()
    targetGerak = False
    time.sleep(0.005)
    while(time.time() - mulai < 1):
        modePergerakan(modeMaju)
    targetGerak = True
    targetGerak = False
    time.sleep(0.005)
    while(time.time() - mulai < 2):
        modePergerakan(modeGeserKiri)
    print("selesai belok")
    targetGerak  = True
    belok        = False

def belokKanan():
    global belok
    global targetGerak
    mulaiKanan = time.time()
    targetGerak = False
    time.sleep(0.01)
    while(time.time() - mulaiKanan < 1.6):
        modePergerakan(modeMaju)
    ultrasonic("kasa")
    time.sleep(1)
    putar("kanan")
    targetGerak  = True
    belok        = False

def belokKiriTangga():
    global belok
    global targetGerak
    modeTangga(True)
    mulai = time.time()
    targetGerak = False
    while(time.time() - mulai < 1): 
        modePergerakan(modeMaju)
    ultrasonic("kisa")
    time.sleep(0.1)
    modeTangga(False)
    print("sampai di tengah")
    targetGerak  = True
    belok        = False

def ambilKorban():
    global targetGerak
    grip("setengah")
    targetGerak = False
    ultrasonic("X")
    time.sleep(0.05)
    ultrasonic("gripper")
    while(nilaiUltrasonic["gripper"] >= 3 ): 
        modePergerakan(modeMaju, nilaiA = 1)
        ultrasonic("gripper")
        print(nilaiUltrasonic["gripper"])
    berdiri()
    grip("tutup")
    time.sleep(0.4)
    grip("naik")
    ultrasonic("Z")
    targetGerak = True

def ambilKorbanSamping():
    global targetGerak
    grip("setengah")
    targetGerak = False
    ultrasonic("X")
    time.sleep(0.05)
    ultrasonic("gripper")
    while(nilaiUltrasonic["gripper"] >= 4 ):  #>=
        modePergerakan(modeMaju, nilaiA = 1)
        ultrasonic("gripper")
        print(nilaiUltrasonic["gripper"])
    berdiri()
    grip("tutup")
    time.sleep(0.4)
    grip("naik")
    ultrasonic("Z")
    targetGerak = True

def ambilKorban2belakang():
    global targetGerak
    # berdiri()

    grip("setengah")
    targetGerak = False
    ultrasonic("L")
    time.sleep(0.2)
    ultrasonic("tangga")
    while(nilaiUltrasonic["tangga"] >= 43 ):  #>=
        modePergerakan(modeMaju, nilaiA = 1)
        ultrasonic("tangga")
        print(nilaiUltrasonic["tangga"])
    berdiri()
    grip("tutup")
    time.sleep(0.4)
    grip("naik")
    ultrasonic("Z")
    targetGerak = True

def pergerakanHadapApiR1():
    global selesai
    global targetGerak
    print("masuk hadap api 1")
    menghadap("kanan")
    targetGerak = False
    ultrasonic("V")
    time.sleep(0.2)
    ultrasonic("kisa")  # cek ultrasonic kiri
    print("kisa hadap api",nilaiUltrasonic["kisa"])
    while nilaiUltrasonic["kisa"] <= 15:
        modePergerakan(modeKepitingKanan,nilaiA=1)
        ultrasonic("kisa")
    ultrasonic("Z")
    targetGerak = True
    selesai = True

def cariApi():
    global selesai
    global targetGerak
    semprot()
    targetGerak = False
    semprotTime = time.time()
    while (time.time() - semprotTime < 1):
        berdiri()
        time.sleep(0.2)
        modePergerakan(modeTengokKiri)
        time.sleep(0.2)
        modePergerakan(modeTengokKanan)
        time.sleep(0.2)
    targetGerak = True

def cariKorbanR1():
    global selesai
    global targetGerak
    ultrasonic("kisa")
    targetGerak = False
    while(nilaiUltrasonic["kisa"] <= 26 ):
        modePergerakan(modeKepitingKanan,nilaiA=1)
        print("kisa cari korban kanan" , nilaiUltrasonic["kisa"])
        ultrasonic("kisa")
    targetGerak = True
    ultrasonic("Z")
    time.sleep(0.2)
    ultrasonic("gripper")
    if(nilaiUltrasonic["gripper"] < 18 ):
        posisiKorbanR1 = "di kanan"
    else:
        targetGerak = False
        ultrasonic("V")
        time.sleep(0.05)
        ultrasonic("kisa")
        while(nilaiUltrasonic["kisa"] >= 10 ): 
            modePergerakan(modeGeserKiri)
            print("kisa cari korban kiri", nilaiUltrasonic["kisa"])
            ultrasonic("kisa")
        ultrasonic("kisa")
        while(nilaiUltrasonic["kisa"] >= 6 ): 
            modePergerakan(modeKepitingKiri,nilaiA=1)
            print("kisa cari korban kiri", nilaiUltrasonic["kisa"])
            ultrasonic("kisa")
        menghadap("kanan")
        targetGerak = True
        ultrasonic("X")
        time.sleep(0.2)
        ultrasonic("gripper")
        if nilaiUltrasonic["gripper"] < 24:
            posisiKorbanR1 = "di kiri"
        else:
            posisiKorbanR1 = "tidak ditemukan"
    print(posisiKorbanR1)
    targetGerak = True
    ultrasonic("Z")
    return posisiKorbanR1

def selamatkanKorbanR1(posisiKorbanR1):
    global selesai
    global targetGerak
    if posisiKorbanR1 == "tidak ditemukan" :
       ultrasonic("kisa")
       while nilaiUltrasonic["kisa"] <= 13:
          modePergerakan(modeKepitingKanan, nilaiA= 3)
          print(nilaiUltrasonic["kisa"])
          ultrasonic("kisa")
          return selamatkanKorbanR1(cariKorbanR1())
    ambilKorban()
    if(posisiKorbanR1 == "di kanan"):
        wakPut = time.time()
        targetGerak = False
        while (time.time() - wakPut <3):
            modePergerakan(modeMundurKiri)
        targetGerak  = True
        wakGes = time.time()
        targetGerak = False
        while (time.time() - wakGes <1):
            modePergerakan(modePutarKanan)
    elif(posisiKorbanR1 == "di kiri"):
        wakPut = time.time()
        targetGerak = False
        while (time.time() - wakPut <2.6):
            modePergerakan(modeMundurKanan)
        targetGerak  = True
        wakGes = time.time()
        targetGerak = False
        while (time.time() - wakGes <1):
            modePergerakan(modePutarKiri)
    time.sleep(0.1)
    menghadap("kiri")
    ultrasonic("kasa")
    targetGerak = False
    while (nilaiUltrasonic["kasa"] >= 16 ):
        modePergerakan(modeKepitingKanan, nilaiA = 3)
        ultrasonic("kasa")
    targetGerak = True
    ultrasonic("L")
    time.sleep(0.2)
    ultrasonic("tangga")
    targetGerak = False
    while nilaiUltrasonic["tangga"] >= 23:
        print(" selamatkan",nilaiUltrasonic["tangga"])
        modePergerakan(modeMaju)
        ultrasonic("tangga")
    targetGerak = True
    ultrasonic("Z")
    grip("buka")
    ultrasonic("gripper")
    print("selamatkan",nilaiUltrasonic["gripper"])
    targetGerak=False
    while nilaiUltrasonic["gripper"] <= 6:
        modePergerakan(modeMundur)
        ultrasonic("gripper")
    targetGerak=True
    selesai = True

def keluarR1():
    global selesai
    global targetGerak
    cekArahMuka()
    menghadap("kiri")
    targetGerak = False
    ultrasonic("L")
    time.sleep(0.01)
    ultrasonic("tangga")
    while(nilaiUltrasonic["tangga"] < 58): # <=65
        modePergerakan(modeMundur)
        ultrasonic("tangga")
    targetGerak = True
    menghadap("kiri")
    ultrasonic("V")
    targetGerak = False
    time.sleep(0.05)
    ultrasonic("kasa")
    while (nilaiUltrasonic["kasa"] <58):
        modePergerakan(modeKepitingKiri, nilaiA=5)
        ultrasonic("kasa")
        print(nilaiUltrasonic["kasa"])
    targetGerak = True
    selesai = True

def pergerakanHadapApiR2():
    global selesai
    global targetGerak
    menghadap("kanan")
    time.sleep(0.1)
    ultrasonic("V")
    time.sleep(0.1)
    ultrasonic("kasa")
    targetGerak = False
    print("kasaa hadap api",nilaiUltrasonic["kasa"])
    while(nilaiUltrasonic["kasa"] > 14):
        modePergerakan(modeKepitingKanan, nilaiA=1)
        ultrasonic("kasa")
    while(nilaiUltrasonic["kasa"] < 14):
        modePergerakan(modeKepitingKiri, nilaiA=1)
        ultrasonic("kasa")
    targetGerak = True
    menghadap("kanan")
    ultrasonic("Z")
    targetGerak = False
    print("selesai hadap api")
    selesai =True

def cariKorbanR2() :
    global selesai
    global targetGerak
    grip("turun")
    _, cmpsnowss=compas()
    cnthdp1 = 0
    while abs(91.4-cmpsnowss) > 3 and cnthdp1 < 3:
        menghadap("kanan")
        _,cmpsnowss=compas()
        cnthdp1 += 1
        print(cnthdp1)
    ultrasonic("X")
    time.sleep(0.02)
    ultrasonic("gripper")
    if(nilaiUltrasonic["gripper"] < 15 ):
        posisiKorbanR2 = "di depan"
        ultrasonic("Z")
    else:
        ultrasonic("gripper")
        targetGerak = False
        while(nilaiUltrasonic["gripper"] > 15 ):
            modePergerakan(modeMaju, nilaiA= 4) 
            ultrasonic("gripper")
        ultrasonic("gripper")
        targetGerak = True
        targetGerak = False
        while(nilaiUltrasonic["gripper"] > 10 ):
            modePergerakan(modeMaju, nilaiA= 1)
            ultrasonic("gripper")
        targetGerak = True
        cariApi()
        time.sleep(0.5)
        _,_,api = errorThermal()
        if api != []:
            print("api ga mati R2")
            pergerakanHadapApiR2()
            cariApi()
        ultrasonic("V")
        time.sleep(0.1)
        targetGerak = False
        ultrasonic("kasa")
        while(nilaiUltrasonic["kasa"] > 4 ):  
            modePergerakan(modeKepitingKanan, nilaiA= 3)
            print("kasa cari korban kanan", nilaiUltrasonic["kasa"])
            ultrasonic("kasa")
        menghadap("kanan")
        ultrasonic("X")
        time.sleep(0.3)
        ultrasonic("gripper")
        #menghadap("kanan")
        if (nilaiUltrasonic["gripper"] < 19 ):
            posisiKorbanR2 = "di kanan"
        else:
            menghadap("kanan")
            ultrasonic("V")
            time.sleep(0.1)
            ultrasonic("kasa")
            targetGerak = False
            while(nilaiUltrasonic["kasa"] <= 18 ):  
                modePergerakan(modeGeserKiri)
                print("kisa cari korban kiri", nilaiUltrasonic["kasa"])
                ultrasonic("kasa")
            ultrasonic("kasa")
            while(nilaiUltrasonic["kasa"] <= 22 ):  
                modePergerakan(modeKepitingKiri)
                print("kisa cari korban kiri", nilaiUltrasonic["kisa"])
                ultrasonic("kasa")
            targetGerak = True
            menghadap("kanan")
            ultrasonic("X")
            time.sleep(0.3)
            ultrasonic("gripper")
            if (nilaiUltrasonic["gripper"] < 15 ):
                posisiKorbanR2 = "di kiri"
            else:
                menghadap("kanan")
                posisiKorbanR2 = "di belakang"
    targetGerak = True
    ultrasonic("Z")
    print(posisiKorbanR2)
    return posisiKorbanR2  

def majuBawaKorbanR2():
    global targetGerak
    targetGerak = False
    ultrasonic("L")
    time.sleep(0.3)
    ultrasonic("tangga")
    time.sleep(0.005)
    while (nilaiUltrasonic["tangga"] > 26) :
        modePergerakan(modeMaju)
        ultrasonic("tangga")
        time.sleep(0.005)
    targetGerak = True
    ultrasonic("Z")
    time.sleep(0.01)

def selamatkanKorbanR2(posisiKorbanR2):
    global selesai
    global targetGerak
    if posisiKorbanR2 == "tidak ditemukan" :
        menghadap("kanan")
        majuTanpaKorban = time.time()
        while (time.time()- majuTanpaKorban < 4):
            modePergerakan(modeMundur)
        menghadap("atas")
        majuBawaKorbanR2()
    elif(posisiKorbanR2 == "di depan"):
        ambilKorbanSamping()
        cariApi()
        time.sleep(0.5)
        _,_,api = errorThermal()
        if api != []:
            print("api ga mati R2 depan")
            cariApi()
        menghadap("atas")
        majuBawaKorbanR2()
    elif(posisiKorbanR2 == "di kanan"):
        targetGerak = False
        putarSedikit = time.time()
        while (time.time()- putarSedikit < 0.3):
            modePergerakan(modePutarKanan,nilaiA=1)
        targetGerak = True
        ambilKorban()
        targetGerak = False
        mundurKorban2 = time.time()
        while (time.time()- mundurKorban2 < 0.6):
            modePergerakan(modeMundur)
        menghadap("atas")
        targetGerak = False
        majuKorbanKanan = time.time()
        while (time.time() - majuKorbanKanan < 1.5) :
            modePergerakan(modeMaju)
        majuBawaKorbanR2()
    elif(posisiKorbanR2 == "di kiri"):
        time.sleep(0.1)
        menghadap("kanan")
        ambilKorban()
        targetGerak = False
        mundurKorban2 = time.time()
        while (time.time()- mundurKorban2 < 0.6):
            modePergerakan(modeMundur)
        targetGerak = True
        menghadap("atas")
        menghadap("atas")
        majuBawaKorbanR2()
    elif(posisiKorbanR2 == "di belakang"):
        grip("setengah")
        targetGerak = False
        balikKorban = time.time()
        while (time.time()- balikKorban < 1):
            modePergerakan(modeMaju, nilaiA= 2)
        targetGerak = True
        ultrasonic("X")
        time.sleep(0.1)
        ultrasonic("gripper")
        targetGerak = False
        while (nilaiUltrasonic["gripper"] >48 ):
            modePergerakan(modeMaju, nilaiA= 2)
            ultrasonic("gripper")
        ultrasonic("gripper")
        while (nilaiUltrasonic["gripper"] > 40 ):
            modePergerakan(modeMaju, nilaiA= 1)
            ultrasonic("gripper")
        targetGerak = True
        ultrasonic("V")
        time.sleep(0.3)
        ultrasonic("kasa")
        targetGerak = False
        while (nilaiUltrasonic["kasa"] > 7 ):
            modePergerakan(modeKepitingKanan)
            ultrasonic("kasa")
        ultrasonic("kasa")
        targetGerak = False
        while (nilaiUltrasonic["kasa"] < 11  ):
            modePergerakan(modeKepitingKiri, nilaiA = 1)
            ultrasonic("kasa")
        ultrasonic("Z")
        time.sleep(0.1)
        targetGerak = False
        menghadap("kanan")
        targetGerak = True
        ultrasonic("X")
        time.sleep(0.1)
        ultrasonic("gripper")
        targetGerak = False
        while (nilaiUltrasonic["gripper"] > 15 ):
            modePergerakan(modeMaju, nilaiA= 1)
            ultrasonic("gripper")
        ultrasonic("gripper")
        if nilaiUltrasonic["gripper"] < 18:
            ambilKorban2belakang()
        else:
            grip("naik")
        targetGerak = False
        menghadap("atas")
        targetGerak = False
        mundurrKorban2 = time.time()
        while (time.time()- mundurrKorban2 < 1.3):
            modePergerakan(modeGeserKiri)
        targetGerak = True
        time.sleep(0.2)
        menghadap("atas")
        majuBawaKorbanR2()
    ultrasonic("Z")
    berdiri()
    grip("buka")
    ultrasonic("gripper")
    print("selamatkan",nilaiUltrasonic["gripper"])
    ultrasonic("L")
    time.sleep(0.2)
    ultrasonic("tangga")
    targetGerak=False
    while (nilaiUltrasonic["tangga"] < 35):
        modePergerakan(modeMundur)
        ultrasonic("tangga")
    targetGerak=True
    ultrasonic("Z")
    menghadap("atas")
    selesai = True

panjangRobot = 30
kp=0.36
ki=0.076101       
kd=0
Ts=0.0549
last_error=0
pwmKanan = 0
pwmKiri = 0

belok = False

kpTangga=  0.315
kiTangga=  0.066417163 
kdTangga=  0

def susurKananTangga():
    global last_error
    global pwmKanan
    global pwmKiri
    waktu_tunggu=time.time()
    while((time.time()-waktu_tunggu)<0.0549):
        ultrasonic("kasa")
        time.sleep(0.005)
        #ultrasonic("gripper")
        #time.sleep(0.005)

    error=(7-nilaiUltrasonic["kasa"])
    P=(kp*error)
    It=((error+last_error)*ki)*Ts
    rate=(error-last_error)
    D=(rate*kdTangga)/Ts
    MV=(P+D+It)
    pwmKiri=8-MV
    pwmKanan=8+MV
    last_error=error
    if(pwmKanan>8):
        pwmKanan=8	
    if(pwmKanan<1):
        pwmKanan=1
    if(pwmKiri>8):
        pwmKiri=8
    if(pwmKiri<1):
        pwmKiri=1
    modePergerakan(modePWM)

def susurKiriTangga():
    global last_error
    global pwmKanan
    global pwmKiri
    waktu_tunggu=time.time()
    while((time.time()-waktu_tunggu)<0.0549):
        ultrasonic("kisa")
        time.sleep(0.005)
        ultrasonic("gripper")
        time.sleep(0.005)

    error=(nilaiUltrasonic["kisa"]-7)
    P=(kp*error)
    It=((error+last_error)*ki)*Ts
    rate=(error-last_error)
    D=(rate*kdTangga)/Ts
    MV=(P+D+It)
    pwmKiri=8-MV
    pwmKanan=8+MV
    last_error=error
    if(pwmKanan>6.5):
        pwmKanan=6.5	
    if(pwmKanan<1):
        pwmKanan=1
    if(pwmKiri>6.5):
        pwmKiri=6.5
    if(pwmKiri<1):
        pwmKiri=1
    modePergerakan(modePWM)

def susurKiriBalik():
    global last_error
    global pwmKanan
    global pwmKiri
    global direksiSusur
    direksiSusur = "maju"
    print("balik balik balik")
    modeTangga(True)
    waktu_tunggu=time.time()
    while((time.time()-waktu_tunggu)<0.0549):
        ultrasonic("kisa")
        time.sleep(0.005)
        ultrasonic("kasa")
        time.sleep(0.005)
        ultrasonic("gripper")
    
    error=(nilaiUltrasonic["kisa"]-7)
    P=(kpTangga*error)
    It=((error+last_error)*kiTangga)*Ts
    rate=(error-last_error)
    D=(rate*kdTangga)/Ts
    MV=(P+D+It)
    pwmKiri=7-MV
    pwmKanan=8+MV
    last_error=error
    if(pwmKanan>8):
       pwmKanan=8	
    if(pwmKanan<6):
       pwmKanan=6
    if(pwmKiri>8):
       pwmKiri=8
    if(pwmKiri<6):
       pwmKiri=6
    modePergerakan(modePWM)

def putarKiri():
    global targetGerak
    targetGerak = False
    putarKiri = time.time()
    while(time.time() - putarKiri < 0.5):
        modePergerakan(modePutarKiri)
    putarKiriKecil = time.time()
    while(time.time() - putarKiriKecil < 0.5):
        modePergerakan(modePutarKiri, nilaiA =2)
    targetGerak = True

def putarKanan():
    global targetGerak
    targetGerak = False
    putarKanan = time.time()
    while(time.time() - putarKanan < 0.5):
        modePergerakan(modePutarKanan)
    putarKananKecil = time.time()
    while(time.time() - putarKananKecil < 0.5):
        modePergerakan(modePutarKanan, nilaiA =2)
    targetGerak = True

def R2Home():
    global selesai,direksiSusur
    global targetGerak
    targetGerak = True
    mundurGeser = time.time()
    targetGerak = False
    direksiSusur="mundur"
    while(time.time() - mundurGeser <1):
        modePergerakan(modeMundur)
    ultrasonic("belakang")
    while(nilaiUltrasonic["belakang"] >= 11):
       modePergerakan(modeMundur)
       ultrasonic("belakang")
       print("gesereeeeeer dulu")
    targetGerak = True
    menghadap("atas")
    menghadap("atas")
    targetGerak = False
    ultrasonic("kisa")
    while(nilaiUltrasonic["kisa"] >= 8):
        modePergerakan(modeGeserKiri)
        ultrasonic("kisa")
        print("gesereeeeeer dulu")
    targetGerak = True
    menghadap("atas")
    ultrasonic("kasa")
    while(nilaiUltrasonic["kasa"] > 45):
        susurKiriBalik()
        ultrasonic("kasa")
        print("susurkiri I", nilaiUltrasonic["kasa"])
    ultrasonic("kasa")
    while(nilaiUltrasonic["kasa"] < 45):
        susurKiriBalik()
        ultrasonic("kasa")
        print("susurkiri II", nilaiUltrasonic["kasa"])
    targetGerak = False

    ultrasonic("gripper")
    while(nilaiUltrasonic["gripper"] > 7):
        modeTangga(True)
        susurKiriBalik()
        ultrasonic("gripper")
        print("susurkiri III", nilaiUltrasonic["kasa"])
    ultrasonic("gripper")
    targetGerak = False
    print("uvuvu")
    modeTangga(False)
    menghadap("kanan")
    menghadap("kanan")
    ultrasonic("kisa")
    targetGerak = False
    ultrasonic("X")
    time.sleep(0.1)
    ultrasonic("belakang")
    while(nilaiUltrasonic["belakang"] <= 50) :
        modePergerakan(modeMaju)
        ultrasonic("belakang")
    targetGerak = True
    menghadap("bawah")
    selesai = True

def ruang2():
    global selesai
    global targetGerak
    global irBelakang
    modeTangga(False)
    menghadap("kanan")
    targetGerak = False
    targetGerak = False
    ultrasonic("X")
    time.sleep(0.1)
    ultrasonic("belakang")
    grip("turun")
    while nilaiUltrasonic["belakang"] < 43:
        modePergerakan(modeMaju)
        ultrasonic("belakang")
    pergerakanHadapApiR2()
    menghadap("kanan")
    posisiKorbanR2 = cariKorbanR2()
    selamatkanKorbanR2(posisiKorbanR2)
    selesai = True

def pertigaanR2():
    global selesai
    global targetGerak,direksiSusur
    menghadap("kiri")
    time.sleep(0.2)
    targetGerak = False
    ultrasonic("gripper")
    while(nilaiUltrasonic["gripper"] > 45):  #5
        modePergerakan(modeMaju)
        ultrasonic("gripper")
        print("masi lebi 35", nilaiUltrasonic["gripper"])
    targetGerak = True
    menghadap("kiri")
    targetGerak = False
    ultrasonic("gripper")
    while(nilaiUltrasonic["gripper"] > 5):  #5
        modePergerakan(modeMaju)
        ultrasonic("gripper")
    targetGerak = True
    menghadap("bawah")
    while arahMuka == "bawah" :
        ultrasonic("kasa")
        targetGerak = False
        while(nilaiUltrasonic["kasa"] > 7):
            modePergerakan(modeKepitingKanan)
            ultrasonic("kasa")
        targetGerak = True
        modeTangga(True)
        targetGerak = False
        time.sleep(0.005)
        ultrasonic("gripper")
        ultrasonic("kisa")
        direksiSusur = "maju"
        while nilaiUltrasonic["gripper"] >10:
            susurKananTangga()
            ultrasonic("gripper")
        targetGerak = True
        modeTangga(False)
        menghadap("kanan")
    selesai = True

def ruang1():
    global selesai
    global targetGerak
    global irDepan
    print("masuk ruang 1")
    pergerakanHadapApiR1()
    targetGerak = False
    grip("turun")
    irDepan = ADC.read("P9_37")
    while (int(irDepan) > 0):
       infraredD()
       irDepan = ADC.read("P9_37")
    menghadap("kanan")
    pergerakanHadapApiR1()
    targetGerak = True
    grip("turun")
    print("gripper turun cari korban")
    targetGerak = False
    ultrasonic("X")
    time.sleep(0.05)
    ultrasonic("gripper")
    while nilaiUltrasonic["gripper"] >= 10:
        if nilaiUltrasonic["gripper"] > 13:
            modePergerakan(modeMaju, nilaiA= 4)
        elif nilaiUltrasonic["gripper"] <=13:
            modePergerakan(modeMaju,nilaiA=1)
        print(nilaiUltrasonic["gripper"])
        ultrasonic("gripper")
    targetGerak = True
    ultrasonic("Z")
    hdpsemprot=0
    menghadap("kanan")
    _, nlcompas = compas()
    while hdpsemprot < 3 and abs(91.4-nlcompas)>3:
        menghadap("kanan")
        hdpsemprot +=1
    cariApi()
    _,_,api = errorThermal()
    time.sleep(0.2)
    if api != []:
        print("api ga mati R1")
        pergerakanHadapApiR1()
        menghadap("kanan")
        cariApi()

    posisiKorbanR1 = cariKorbanR1()
    selamatkanKorbanR1(posisiKorbanR1)
    keluarR1()
    print(posisiKorbanR1)
    selesai = True

def homeR1():
    global selesai, direksiSusur
    global targetGerak
    cekArahMuka()
    if arahMuka == "bawah" :
        ultrasonic("V")
        time.sleep(0.05)
        ultrasonic("kasa")
        targetGerak = False
        while nilaiUltrasonic["kasa"] > 6:
            modePergerakan(modeKepitingKanan, nilaiA=5)
            ultrasonic("kasa")
        
        ultrasonic("Z")
        time.sleep(0.1)
        targetGerak = False
        ultrasonic("belakang")
        modeTangga(True)
        direksiSusur = "maju"
        while nilaiUltrasonic["belakang"] < 47:
            susurKananTangga()
            ultrasonic("belakang")
            print("belakang <47")

        ultrasonic("V")
        time.sleep(0.1)
        ultrasonic("kisa")
        while nilaiUltrasonic["kisa"] < 30 :
            susurKananTangga()
            ultrasonic("kisa")
            print(nilaiUltrasonic["kisa"])
            print("masi di tengah tangga")

        print("mau belok kiri lorong")
        majuTangga = time.time()
        while(time.time() - majuTangga < 1.1):
            modePergerakan(modeMaju,nilaiA=8)
            print("counting maju abis tangga")
        modeTangga(False)
        
        targetGerak = True

    elif arahMuka == "atas":
        ultrasonic("V")
        time.sleep(0.05)
        ultrasonic("kisa")
        targetGerak = False
        while nilaiUltrasonic["kisa"] > 6:
            modePergerakan(modeKepitingKiri,nilaiA=5)
            ultrasonic("kisa")
        
        ultrasonic("Z")
        time.sleep(0.1)
        targetGerak = False
        ultrasonic("gripper")
        modeTangga(True)
        direksiSusur="mundur"
        while nilaiUltrasonic["gripper"] < 47:
            susurKiriTangga()
            ultrasonic("gripper")
            print("gripper <47")

        ultrasonic("V")
        time.sleep(0.1)
        ultrasonic("kasa")
        while nilaiUltrasonic["kasa"] < 30 :
            susurKiriTangga()
            ultrasonic("kasa")
        print("mau belok kanan mundur lorong")
        majuTangga = time.time()
        while(time.time() - majuTangga < 1):
            modePergerakan(modeMundur)
        modeTangga(False)
        
        targetGerak = True
    
    menghadap("kanan")
    menghadap("kanan")
    targetGerak = False
    ultrasonic("X")
    time.sleep(0.2)
    ultrasonic("belakang")
    while nilaiUltrasonic["belakang"] < 65:
        modePergerakan(modeMaju)
        ultrasonic("belakang")
    menghadap("kanan")
    ultrasonic("X")
    time.sleep(0.2)
    targetGerak = False
    while nilaiUltrasonic["belakang"] < 105:
        modePergerakan(modeMaju)
        ultrasonic("belakang")
    menghadap("kanan")
    targetGerak = False
    ultrasonic("V")
    time.sleep(0.2)
    print("mau geser counting")
    ultrasonic("kisa")
    while nilaiUltrasonic["kisa"] >17  :
        modePergerakan(modeGeserKiri,nilaiA = 5)
        ultrasonic("kisa")
        print("masamama")
    targetGerak = True
    selesai = True

def modeTangga(aktif) : 
    global B,C,D,tangga
    if aktif == True :
        B = list(Btangga)
        C = list(Ctangga)
        D = list(Dtangga)
        tangga = True

    else:
        B = list(Bnormal)
        C = list(Cnormal)
        D = list(Dnormal)
        tangga = False


tangga = False
selesai = False
class Thread1(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
    def run(self):
        global B,C,D,modeGripper,selesai,tangga,targetGerak,Mulai
        try:
            while True:
                tungguTombolMulai()
                while selesai ==False:
                    homeR1()
                    ruang1()
                    pertigaanR2()
                    ruang2()
                    R2Home()
        except(KeyboardInterrupt, SystemExit):
            pass

class Thread2(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
    def run(self):
        try:
            while True:
                gerak()
        except(KeyboardInterrupt, SystemExit):
            pass

def multithread():
    thread1 = Thread1()
    thread2 = Thread2()

    thread1.start()
    thread2.start()

    try:
        while thread1.is_alive():
            thread1.join()
            thread2.join()
    except(KeyboardInterrupt, SystemExit):
        print("shutting down ...")

if __name__ == '__main__':
    berdiriLangsung()
    multithread()