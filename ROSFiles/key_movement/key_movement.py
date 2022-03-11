from setup.nvidia_racecar import NvidiaRacecar
from pynput import keyboard
from std_msgs.msg import Float32
import rospy

steering = 0
motor = 0
print('ready')


def publisher():
    rospy.init_node('motors', anonymous=True)
    publisherMotor = rospy.Publisher('motor', Float32, queue_size=10)
    publisherSteering = rospy.Publisher('steering', Float32, queue_size=10)

def on_press(key):
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
                publisherSteering.publish(float(motor))
        if key == keyboard.Key.down:
                #print(car.steering_gain)
                #print(car.steering_offset)
                motor += .1
                publisherSteering.publish(float(motor))
        if key == keyboard.Key.space:
                #print(car.steering_gain)
                #print(car.steering_offset)
                motor = 0
                steering = 0
                publisherSteering.publish(float(steering))
                publisherSteering.publish(float(motor))

    except AttributeError:
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

with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()

try:
        publisher()
    except:
        pass

