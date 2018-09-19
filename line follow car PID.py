#Imporditakse teeigd:
import vrep                  #V-rep's library
import sys
import time
def convertsensor(a): #Function which is going to convert sensors values
    if a >0.95:        #When value is bigger than 0.95 function returns 0
        return 0.0      #When it is seeing line then it return 1.
    else:
        return 1.0       
vrep.simxFinish(-1) #It is closing all open connections with VREP
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
if clientID!=-1:  #It is checking if connection is successful
    print 'Connected to remote API server'   
else:
    print 'Connection not successful'
    sys.exit('Could not connect')

#Getting motor handles
errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,"left_joint",vrep.simx_opmode_oneshot_wait)
errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,"right_joint",vrep.simx_opmode_oneshot_wait)
sensor_h=[] #handles list
sensor_val=[] #Sensor value list
#Getting sensor handles list
for x in range(0,6):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'line_sensor'+str(x),vrep.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle) #It is adding sensor handle values
        errorCode,detectionstate, sensorreadingvalue=vrep.simxReadVisionSensor(clientID,sensor_h[x],vrep.simx_opmode_streaming)
        sensor_val.append(1.0) #It is adding 1.0 to fill the sensor values on the list. In the while loop it is going to overwrite the values
maxkiirus = 15 #Maximal speed
previouserror =0 #previous error
error = 0 #error
integral = 0 # PID values before while loop
derivative = 0
proportional = 0
time.sleep(1)
t = time.time() #It is saving the time which is now
while (1):  #Cycle which doesn't end
    #It is writing down sensor handles and reading values
    summa = 0  #It is zeroing the  sum 
    andur = 0	#and the sensor values 
    for x in range(0,6):
        errorCode,detectionstate, sensorreadingvalue=vrep.simxReadVisionSensor(clientID,sensor_h[x],vrep.simx_opmode_buffer)
        #Reading sensor values
        sensor_val[x]=sensorreadingvalue[1][0] #It is overwriting the sensor values
        andur = andur+convertsensor(sensor_val[x]) #It is adding the sensor values by each other
        summa=summa +convertsensor(sensor_val[x])*x*10.0 #Calculating the sum where the robots position is

  #      print "Positsiooni väärtus kokku45 :",sensor_val[x] ,x
  #  print "Positsiooni väärtus kokku :",summa #Position sum
  #  print "Mitu andurit näeb joont :", andur #How many sensors are seeing the line
    if andur == 0 and previouserror >0: #When no sensors doesn't see and previous error is bigger than zero
        error = maxkiirus # Error is going to be maximal speed
    elif andur == 0 and previouserror<0: #When no sensors doesn't see and previous error is lower than zero
        error =-maxkiirus # Error is going to be -maximal speed
    else:
        if andur == 0: # When sensor doesn't see
            positsioon = proportional # Position is going to be proportional error
        else:
            positsioon =summa/andur # otherwise position is the division between sum and sensor values
        print "Positsioon on :", positsioon
        proportional = positsioon -25 # Proportional error
        viivitus = round((time.time()-t),5) #calculating delay time
        integral = integral + proportional # Integral error
        if integral >2500: #When integral value is going to be too big
            integral =0    #integral is set to zero
        elif integral <-2500:
            integral =0
     #   print "Integraal on :", integral
        derivative = proportional - previouserror # Derivative error - in reality this has no effect.
     #   print "Derivatiivne on :", derivative
        previouserror = proportional #It is remembering previous error
        error =  proportional*0.3 + integral*0.0003 +derivative*0.5 #Calculating overall error using PID.
      #  print "Error on: ", error
      #  print "viivitus on: ", viivitus
    #When position (error) is to big/small, then the speed is set max/-max.
    if error > maxkiirus: #When error is bigger than maximal speed,
       error = maxkiirus  #error is set to max speed
    elif error < -maxkiirus: # It is because the wheels shouldn't rotate backwards.
       error = -maxkiirus
    if error <0: # In here it is calculating the speed of the wheels
        omega_right=maxkiirus+error
        omega_left=maxkiirus
    else:
        omega_right=maxkiirus
        omega_left=maxkiirus-error    
    #print "Vasak ratas pöörleb nurkkiirusega :",omega_left #Left wheel is rotating with angular speed of
    #print "Parem ratas pöörleb nurkkiirusega :",omega_right #Right wheel is rotating with angular speed of
    errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,omega_left, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,omega_right, vrep.simx_opmode_streaming)
    t = time.time() #Taking new time moment





















