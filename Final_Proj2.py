# *****************************************************************************
# ***************************  Python Source Code  ****************************
# *****************************************************************************
#
#   DESIGNER NAME:  Michael Bauer and Erik Howell 
#
#       FILE NAME:  Final_Proj.py
#
# DESCRIPTION
#    This code provides a functioning GUI using Tkinter that controls the
#    Raspberry Pi Car. It allows for turning, speed changes, line tracing,
#    object detection, mode changes (Manual and Automatic), a start button,
#    and a stop button. 
#
# *****************************************************************************
import tkinter as TK
import time
import RPi.GPIO as GPIO
import threading

#---------------------------------------------------
# Constants to be used in program
#---------------------------------------------------
# GPIO number based on BCM GPIO numbering scheme
trigPin = 23
echoPin = 24

leftMotorPin = 17 # --> lab6_part3 enablePin was the PWM
rightMotorPin = 18 # --> ---

leftMotoRP1 = 5
leftMotoRP2 = 6
rightMotoRP1 = 13 
rightMotoRP2 = 19

leftSenPin = 20
rightSenPin = 21



# Constants for Ultra Sonic
TEN_MIRCO_SEC = 0.00001 #10us
SPEED_OF_SOUND = 340.0
ONE_MIL_SEC = 0.1
ONE_SEC = 1
ONE_MIL = 1000000
TEN_THOUSAND = 10000.0
ONE_MILTH = 0.000001

MAX_DISTANCE = 220
timeOut = MAX_DISTANCE*60
TEN_CEN = 10

# CONSTANTS FOR MOTORS
START_NUM = 0
END_NUM = 100
DUTY_CYCLE_DELAY = 0.01
TURNAROUND_TIME = 0.5
PWM_FREQUENCY = 500
INC_RATE = 1
DEC_RATE = -1

SPEED = 65
CHANGE_DIR = 35

# Variables for Motors
forward = True
mode = False

# Variables for threading
drive_thread = None
ultra_sonic_thread = None
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function creates the GUI.
#
# INPUT PARAMETERS:
#   leftMotorPWM --> PWM of the left motor
#   rightMotorPWM --> PWM of the right motor
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   
# -----------------------------------------------------------------------------
def create_gui(leftMotorPWM, rightMotorPWM):
    
# -------- Variable used in create_gui(leftMotorPWM, rightMotorPWM) ----------- 
    global window
# -----------------------------------------------------------------------------

    # Create the main window for GUI control application
    window = TK.Tk()
    window.title("Car Controller")
    
    window.leftMotorPWM = leftMotorPWM
    window.rightMotorPWM = rightMotorPWM
    
    # Create the button to switch modes. 
    modeButton = TK.Button(window, text="Mode", command=toggle_mode)
    modeButton.grid(row=0, column=1, padx=10, pady=5, sticky="ew")
    
    # Create frame for the motor speed.
    speedFrame = TK.LabelFrame(window, text="Motor Speed")
    speedFrame.grid(row=1, column=1, padx=10, pady=5, sticky="ew")

    # Create scale to change the motor speed.
    scaleSpeed = TK.Scale(speedFrame, from_=START_NUM, to=END_NUM, orient=TK.HORIZONTAL, command=update_motor_speed)
    scaleSpeed.grid(row=1, column=1, padx=10, pady=5, sticky="ew")
    
    # Create frame for the angle at which the car moves.
    angleFrame = TK.LabelFrame(window, text="Angle")
    angleFrame.grid(row=2, column=1, padx=10, pady=5, sticky="ew")
    
    # Create scale to change the wheel angles
    scaleAngle = TK.Scale(angleFrame, from_=-100, to=END_NUM, orient=TK.HORIZONTAL, command=change_angle)
    scaleAngle.grid(row=2, column=1, padx=5, pady=5, sticky="ew")

    # Create button to start car.
    startButton = TK.Button(window, text="Start", bg="Green", command=start_car)
    startButton.grid(row=3, column=0, padx=5, pady=5)
    
    # Create button to change direction of car wheels.
    directionButton = TK.Button(window, text="Change Direction", command=change_direction)
    directionButton.grid(row=3, column=1, padx=5, pady=5)
    
    # Create button to stop car.
    stopButton = TK.Button(window, text="Stop", bg="Red", command=stop_car)
    stopButton.grid(row=3, column=2, padx=5, pady=5)
    
    # Create button to destroy the GUI
    quitButton = TK.Button(window, text="Quit", command=window.destroy)  
    quitButton.grid(row=4, column=1, padx=5, pady=5)
    
    # Ensure the car starts with 0 angle (GOING FORWARD).
    scaleAngle.set(0)

    
    # Start the mainloop
    window.mainloop()
    
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function setup the RPi GPIO pins using the GPIO library. 
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   leftMotorPWM --> PWM of the left motor
#   rightMotorPWM --> PWM of the right Motor
# -----------------------------------------------------------------------------
def setup_gpio():
  
  GPIO.setmode(GPIO.BCM)
  
  # ----------------- Ultra Sonics ------------------------
  GPIO.setup(trigPin, GPIO.OUT)
  GPIO.setup(echoPin, GPIO.IN)
  
  # -------------------- Motors ---------------------------
  GPIO.setup(leftMotorPin, GPIO.OUT)
  GPIO.setup(rightMotorPin, GPIO.OUT)
  
  leftMotorPWM = GPIO.PWM(leftMotorPin, PWM_FREQUENCY)
  rightMotorPWM = GPIO.PWM(rightMotorPin, PWM_FREQUENCY)
  
  leftMotorPWM.start(0)
  rightMotorPWM.start(0)
  
  # ---------- Direction Changing of Motors ---------------
  GPIO.setup(leftMotoRP1, GPIO.OUT)
  GPIO.setup(leftMotoRP2, GPIO.OUT)
  GPIO.setup(rightMotoRP1, GPIO.OUT)
  GPIO.setup(rightMotoRP2, GPIO.OUT)
  
  # ---------- Infrared Reflective Sensor -----------------
  GPIO.setup(leftSenPin, GPIO.IN)
  GPIO.setup(rightSenPin, GPIO.IN)
  

  
  return leftMotorPWM, rightMotorPWM


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function updates the pwm of the motors. Overall changing the speed
#   at which it spins.
#
# INPUT PARAMETERS:
#   duty - the speed at which the motors will  turn
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   
# -----------------------------------------------------------------------------
def update_motor_speed(duty):
    window.leftMotorPWM.ChangeDutyCycle(int(duty))
    window.rightMotorPWM.ChangeDutyCycle(int(duty))


# -----------------------------------------------------------------------------
# DESCRIPTION
#   Called from the angleScale in the GUI.
#   This function maps the numbers -100 to 100 onto the scale of 0 to 100. It
#   then sets the motor speed with the newly mapped numbers.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#   
# ----------------------------------------------------------------------------- 
def change_angle(number):

    left_motor_speed = max(0, min(50 + int(number) / 2, 100))
    right_motor_speed = max(0, min(50 - int(number) / 2, 100))
    
    window.leftMotorPWM.ChangeDutyCycle(left_motor_speed)
    window.rightMotorPWM.ChangeDutyCycle(right_motor_speed)


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function toggles automatic mode and manual mode. If it was in manual
#   mode it switches to automatic and starts a thread for automatic_drive(left/
#   SenPin, rightSenPin) and loop(). If it was in automatic mode it switches toc
#   manual mode and stops the threads.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#   
# -----------------------------------------------------------------------------     
def toggle_mode():
    
# ------------------------ Variables used in toggle_mode() --------------------
    global mode, drive_thread, ultra_sonic_thread
# -----------------------------------------------------------------------------    
    
    if(not mode):
        print("\nSwitching to automatic\n")
        mode = True
        drive_thread = threading.Thread(target=automatic_drive, args=(leftSenPin, rightSenPin))
        ultra_sonic_thread = threading.Thread(target=loop)
        
        drive_thread.start()
        ultra_sonic_thread.start()
    else:
        print("\nSwitching to manual\n")
        if((drive_thread is not None) or (ultra_sonic_thread is not None)):
            mode = False
            drive_thread.join()
            ultra_sonic_thread.join()
        stop_car()


# -----------------------------------------------------------------------------c
# DESCRIPTION
#   This function stops the car from moving. It makes all of the motor pins low.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#   
# -----------------------------------------------------------------------------  
def stop_car():
    GPIO.output(leftMotoRP1, GPIO.LOW)
    GPIO.output(leftMotoRP2, GPIO.LOW)
    
    GPIO.output(rightMotoRP1, GPIO.LOW)
    GPIO.output(rightMotoRP2, GPIO.LOW)


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function starts the car going in the direction it previously was
#   moving.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#   
# -----------------------------------------------------------------------------  
def start_car():
    global forward
    
    if(forward):
        GPIO.output(leftMotoRP1, GPIO.HIGH)
        GPIO.output(leftMotoRP2, GPIO.LOW)
        
        GPIO.output(rightMotoRP1, GPIO.HIGH)
        GPIO.output(rightMotoRP2, GPIO.LOW)
        
        forward = True
    
    else:
        GPIO.output(leftMotoRP1, GPIO.LOW)
        GPIO.output(leftMotoRP2, GPIO.HIGH)
        
        GPIO.output(rightMotoRP1, GPIO.LOW)
        GPIO.output(rightMotoRP2, GPIO.HIGH)
        
        forward = False
    
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function reads the left Infared Reflective Sensor and the right
#   Infared Reflective Sensor.
#
# INPUT PARAMETERS:
#   leftSenPin --> BCM GPIO pin number for the left Infared Reflective Sensor
#   rightSenPin --> BCM GPIO pin number for the right Infared Reflective Sensor
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   returns the input of the leftSenPin and rightSen Pin. They are ints that
#   are binary (0 or 1).
#   
# -----------------------------------------------------------------------------  
def read_sensors(leftSenPin, rightSenPin):
    return GPIO.input(leftSenPin), GPIO.input(rightSenPin)


# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function runs in a loop that reads the left and right sensor. If both
#   sensors detect the line it continues straight, if only the left detects the
#   line it turns right, and if only the right detects the line it turns left.
#
# INPUT PARAMETERS:
#   leftSenPin --> BCM GPIO pin number for the left Infared Reflective Sensor
#   rightSenPin --> BCM GPIO pin number for the right Infared Reflective Sensor
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#   
# -----------------------------------------------------------------------------  
def automatic_drive(leftSenPin, rightSenPin):
    
# ----- Variables local to automatic_drive(leftSenPin, rightSenPin) ------
    detected = 0
    not_detected = 1
    
    readLeft, readRight = 0, 0
# -------------------------------------------------------------------------
    
    GPIO.setmode(GPIO.BCM)
    start_car()
    
    while(mode):
        readLeft, readRight = read_sensors(leftSenPin, rightSenPin)
        
        # Follow the black line

        if ((readLeft == detected) and (readRight == detected)):
            # Move Forward
            window.leftMotorPWM.ChangeDutyCycle(SPEED)
            window.rightMotorPWM.ChangeDutyCycle(SPEED)
            time.sleep(ONE_MIL_SEC)
            
        
        elif ((readLeft == detected) and (readRight == not_detected)):
            # Move Right
            
            window.leftMotorPWM.ChangeDutyCycle(CHANGE_DIR)
            window.rightMotorPWM.ChangeDutyCycle(START_NUM)
            time.sleep(ONE_MIL_SEC)
        
        elif ((readLeft == not_detected) and (readRight == detected)):
            # Move Left
            
            window.leftMotorPWM.ChangeDutyCycle(START_NUM)
            window.rightMotorPWM.ChangeDutyCycle(CHANGE_DIR)
            time.sleep(ONE_MIL_SEC)
            

            
        # Follow the white line
        """
        if ((readLeft == not_detected) and (readRight == not_detected)):
            # Move Forward
            window.leftMotorPWM.ChangeDutyCycle(50)
            window.rightMotorPWM.ChangeDutyCycle(50)
            time.sleep(ONE_MIL_SEC)
        
        elif ((readLeft == not_detected) and (readRight == detected)):
            # Move Right
            window.leftMotorPWM.ChangeDutyCycle(35)
            window.rightMotorPWM.ChangeDutyCycle(0)
            time.sleep(ONE_MIL_SEC)
            
        elif ((readLeft == detected) and (readRight == not_detected)):
            # Move Left
            window.leftMotorPWM.ChangeDutyCycle(0)
            window.rightMotorPWM.ChangeDutyCycle(35)
            time.sleep(ONE_MIL_SEC)
           
        """
        
        
        
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function changes the direction of the motors using the L293D H-Bridge.
#
# INPUT PARAMETERS:
#   none
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
#   
# -----------------------------------------------------------------------------   
def change_direction():
    
# -------------- Variable used in change_direction() --------------------------
    global forward
# -----------------------------------------------------------------------------
    if (forward):
        # Now moving back
        forward = False
        GPIO.output(leftMotoRP1, GPIO.LOW)
        GPIO.output(leftMotoRP2, GPIO.HIGH)
        
        GPIO.output(rightMotoRP1, GPIO.LOW)
        GPIO.output(rightMotoRP2, GPIO.HIGH)
    
    else:
        # Now moving forward
        forward = True
        GPIO.output(leftMotoRP1, GPIO.HIGH)
        GPIO.output(leftMotoRP2, GPIO.LOW)
        
        GPIO.output(rightMotoRP1, GPIO.HIGH)
        GPIO.output(rightMotoRP2, GPIO.LOW)
        
        
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function is the loop that calls the input function. It then updates
#   the servo motor.
#
# INPUT PARAMETERS:
#   pin - echoPin
#   level - HIGH or LOW
#   timeOut - MAX_DISTANCE*60
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   pulseTime - time it takes to get low again
#   
# -----------------------------------------------------------------------------
def measure_return_echo(pin, level, timeOut):
    t0 = time.time()

    while(GPIO.input(pin) != level):
        if((time.time() - t0) > timeOut* ONE_MILTH):
            return 0;
    
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time() - t0) > timeOut*ONE_MILTH):
            return 0;
    
    pulseTime = (time.time() - t0)*ONE_MIL
    return pulseTime


# -----------------------------------------------------------------------------
# DESCRIPTION
#  This function sends a trigger pulse to measure the distance.
#
# INPUT PARAMETERS:
#  
#
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   none
# -----------------------------------------------------------------------------
def send_trigger_pulse():

    GPIO.output(trigPin, GPIO.HIGH)
    time.sleep(TEN_MIRCO_SEC)
    GPIO.output(trigPin, GPIO.LOW)
    
    pingTime = measure_return_echo(echoPin, GPIO.HIGH, timeOut)
    distance = pingTime * SPEED_OF_SOUND / 2.0 / TEN_THOUSAND
    
    if((distance < TEN_CEN) and (distance != 0)):
        
        stop_car()
        time.sleep(5)
        start_car()
        
# -----------------------------------------------------------------------------
# DESCRIPTION
#   This function is the loop that calls the trigger pulse. It then sleeps a
#   second.
#
# INPUT PARAMETERS:
#   none
# OUTPUT PARAMETERS:
#   none
#
# RETURN:
#   
# -----------------------------------------------------------------------------
def loop():
    
    while(mode):
        send_trigger_pulse()
        time.sleep(ONE_SEC)


#---------------------------------------------------------------------
#  main() function
#---------------------------------------------------------------------
def main ():
  #-------------------------------------
  # Variables local to this function
  #-------------------------------------

  leftMotorPWM, rightMotorPWM = 0, 0
  

  print()
  print("*********** PROGRAM IS RUNNING ************")
  print()
  print("Press CTRL-C to end the program")

  try:
      leftMotorPWM, rightMotorPWM = setup_gpio()
      create_gui(leftMotorPWM, rightMotorPWM)
  
  except KeyboardInterrupt:
    print()
    print("CTRL - C detected. ")
    print()
  
  finally:    
    
    # Clean up and stop the motors
    GPIO.cleanup()
    leftMotorPWM.stop()
    rightMotorPWM.stop()
    
    print("GPIO Ports have been cleaned  up")
    print()
    print("*********** Program terminated **************")
    print()
    


# if file execute standalone then call the main function.
if __name__ == '__main__':
  main()




