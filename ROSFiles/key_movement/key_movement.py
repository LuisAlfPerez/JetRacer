#!/usr/bin/env python3
from pynput import keyboard
from std_msgs.msg import Float32
import rospy

steering = 0
motor = 0
publisherMotor = None
publisherSteering = None
print('ready')


def publisher():
    global publisherMotor
    global publisherSteering
    rospy.init_node('motors', anonymous=True)
    publisherMotor = rospy.Publisher('motor', Float32, queue_size=10)
    publisherSteering = rospy.Publisher('steering', Float32, queue_size=10)

def on_press(key):
    global publisherMotor
    global publisherSteering
    global motor 
    global steering

    try:
        if key == keyboard.Key.right:
                #print(car.steering_gain)
                #print(car.steering_offset)
                steering += .1
                publisherSteering.publish(float(steering))
        if key == keyboard.Key.left:
                #print(car.steering_gain)
                #print(car.steering_offset)
                steering -= .1
                publisherSteering.publish(float(steering))
        if key == keyboard.Key.up:
                #print(car.steering_gain)
                #print(car.steering_offset)
                motor -= .1
                publisherMotor.publish(float(motor))
        if key == keyboard.Key.down:
                #print(car.steering_gain)
                #print(car.steering_offset)
                motor += .1
                publisherMotor.publish(float(motor))
        if key == keyboard.Key.space:
                #print(car.steering_gain)
                #print(car.steering_offset)
                motor = 0
                steering = 0
                publisherSteering.publish(float(steering))
                publisherMotor.publish(float(motor))

    except AttributeError as e:
        print(e)
        print('special key pressed: {0}'.format(
            key))

def on_release(key):
    #print('Key released: {0}'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        motor = 0
        steering = 0
        publisherSteering.publish(float(steering))
        publisherSteering.publish(float(motor))
        return False



try:
        publisher()
except:
        pass

with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()
