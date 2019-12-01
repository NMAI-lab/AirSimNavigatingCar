import airsim


class Distance:
    LOW = 1
    MEDIUM = 2
    HIGH = 3


class ACC:
    """

    """

    def __init__(self, distance, speed):
        # set distance and speed for ACC algorithm
        self.min_distance = distance
        self.max_speed = speed
        self.set_speed = speed

        # connect to the AirSim simulator
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()
        return

    """
    Controls the speed of the car and keeps it around the control speed.    
    """

    def control_speed(self):
        car_state = self.client.getCarState()
        print("Car speed: \n" + str(car_state.speed))

        if car_state.speed < (self.set_speed - 5):
            self.car_controls.throttle = 1.0
            self.car_controls.brake = 0
        elif car_state.speed < self.set_speed:
            self.car_controls.throttle = 0.5
            self.car_controls.brake = 0
        elif car_state.speed > (self.set_speed + 5):
            self.car_controls.throttle = 0.0
            self.car_controls.brake = 1
        else:
            self.car_controls.throttle = 0.0
            self.car_controls.brake = 0

        self.client.setCarControls(self.car_controls)
        return

    """
    Controls the distance of the car from the leading vehicle. Keeps the distance 
    above the set distance by updating the speed of the vehicle.  
    """

    def control_distance(self):
        distance = 0

        print("Distance: \n" + str(distance))
        return

    """
    Updates the set_speed of the car
    """

    def update_speed(self, speed):
        if speed > self.max_speed:
            self.set_speed = self.max_speed
        elif speed < 0:
            self.set_speed = 0
        else:
            self.set_speed = speed

        return

    """
    Updates the distance of the car
    """

    def update_distance(self, distance):
        if distance in Distance:
            self.min_distance = distance
        else:
            print("Input distance is not a valid option! Setting to moderate distance.")
            self.min_distance = Distance.MEDIUM

        return


# test ACC
acc = ACC(Distance.MEDIUM, 15)

while True:
    acc.control_speed()
