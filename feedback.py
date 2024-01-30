from __future__ import print_function
import roslibpy # pip install roslibpy
#import numpy as np # pip install numpy

client = roslibpy.Ros(host='192.168.0.100', port=9090)
client.run()

talker = roslibpy.Topic(client, '/cmd_vel', 'geometry_msgs/Twist')
listener = roslibpy.Topic(client, '/bunker_status', 'bunker_msgs/BunkerStatus')

goal_vel = 0.05

# Gains
p = 2

actual_vel = 0
error = 0
signal = 0

e_prev = 0



def PID(Kp, Ki, Kd, setpoint, measurement):
    global time, integral, time_prev, e_prev

    # PID calculations
    e = setpoint - measurement
        
    P = Kp*e
    # integral = integral + Ki*e*(time - time_prev)
    integral = 0
    D = Kd*(e - e_prev)/(time - time_prev)

    # calculate manipulated variable - MV 
    output =  P + integral + D

    # output = P 
    # update stored data for next iteration
    e_prev = e
    time_prev = time
    time+=1
    return output


time_prev = 0
time = 1

def run(message):
    actual_vel = message['linear_velocity']
    error = goal_vel - actual_vel
    signal = p * error
    if client.is_connected:
                    talker.publish(roslibpy.Message({
                        'linear': {
                        'x': PID(3,1,0,0.2,actual_vel),
                        'y': 0,
                        'z': 0
                        },
                        'angular': {
                        'x': 0,
                        'y': 0,
                        'z': 0 # 'z': -np.sign(float(data[0]))*float(data[1])*(0.4/90)
                        }
                    }))
    print("Goal Velocity: " + str(goal_vel) + " | Actual Velocity: " + str(round(actual_vel, 2)) + " | Time: " + str(time))
    time_prev = time

listener.subscribe(run)

try:
    while True:
        pass
except KeyboardInterrupt:
    client.terminate()