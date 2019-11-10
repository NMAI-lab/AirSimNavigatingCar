import airsim
import time

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = airsim.CarControls()

# get state of the car
car_state = client.getCarState()
print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))
      
# set the controls for car
car_controls.throttle = 1
car_controls.steering = 0
client.setCarControls(car_controls)
print("Going forward")

time_end = time.time() + 30
while time.time() < time_end:
    car_state = client.getCarState()
    if (car_state.speed < 15):
        car_controls.throttle = 1.0
    else:
        car_controls.throttle = 0.0
    client.setCarControls(car_controls)

car_state = client.getCarState()
print("Speed %0d" % (car_state.speed))

print("Car traveled approximately 500 m at 15 m/s (54km/h)")

# apply brakes
car_controls.brake = 1
client.setCarControls(car_controls)
print("Applying brakes")

car_state = client.getCarState()

while (car_state.speed != 0):
    time.sleep(1)
    car_state = client.getCarState()
    print("Speed %0d" % (car_state.speed))

print("Car stopped")
    
car_controls.brake = 0



