from setup.nvidia_racecar import NvidiaRacecar
from pynput import keyboard

car = NvidiaRacecar()
car.steering = 0
car.steering_gain = 0.5
car.steering_offset = 0.2
car.throttle = 0
car.throttle_gain = 1
print('ready')

def on_press(key):
    try:
        if key == keyboard.Key.right:
            car.steering += .1
            print(car.steering)
        if key == keyboard.Key.left:
            car.steering -= .1
            print(car.steering)
        if key == keyboard.Key.up:
            car.throttle -= .1
            print(car.throttle)
        if key == keyboard.Key.down:
            car.throttle += .1
            print(car.throttle)
        if key == keyboard.Key.space:
            car.throttle = 0
            car.steering = 0
    except AttributeError:
        print('an error ocurred')

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()



