from math import pi
import time
from controller import Robot, Motor, DistanceSensor

INF = float("+inf")


class PID:
    def __init__(self, Kp, Ki, Kd, target):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = target
        self.error = 0
        self.error_sum = 0
        self.error_diff = 0
        self.previous_error = 0

    def update(self, current_value):
        self.error = self.target - current_value
        self.error_sum += self.error
        self.error_diff = self.error - self.previous_error
        self.previous_error = self.error
        proportional = self.Kp * self.error
        integral = self.Ki * self.error_sum
        derivative = self.Kd * self.error_diff
        pid = proportional + integral + derivative

        print(f"PID raw: {self.error} | {self.error_sum} | {self.error_diff}")
        print(f"PID splited: {proportional} | {integral} | {derivative}")
        print(f"PID: {pid}")

        return pid


class MyRobot:
    def initMotors(self):
        self.front_left_motor = self.me.getDevice("front left wheel")
        self.front_right_motor = self.me.getDevice("front right wheel")
        self.back_left_motor = self.me.getDevice("back left wheel")
        self.back_right_motor = self.me.getDevice("back right wheel")

        self.front_left_motor.setPosition(INF)
        self.front_right_motor.setPosition(INF)
        self.back_left_motor.setPosition(INF)
        self.back_right_motor.setPosition(INF)

        self.front_left_motor.setVelocity(0.0)
        self.front_right_motor.setVelocity(0.0)
        self.back_left_motor.setVelocity(0.0)
        self.back_right_motor.setVelocity(0.0)

    def initSensors(self):
        self.front_sensors = [
            self.me.getDevice("so0"),
            self.me.getDevice("so1"),
            self.me.getDevice("so2"),
            self.me.getDevice("so3"),
            self.me.getDevice("so4"),
            self.me.getDevice("so5"),
            self.me.getDevice("so6"),
            self.me.getDevice("so7"),
        ]
        # print(f"Front sensors: {self.front_sensors}")

        self.back_sensors = [
            self.me.getDevice("so8"),
            self.me.getDevice("so9"),
            self.me.getDevice("so10"),
            self.me.getDevice("so11"),
            self.me.getDevice("so12"),
            self.me.getDevice("so13"),
            self.me.getDevice("so14"),
            self.me.getDevice("so15"),
        ]

        for sensor in self.front_sensors:
            sensor.enable(self.timestep)
            # print(f"Sensor Sampling Period - {sensor.getSamplingPeriod()}")
            # print(f"Sensor Value - {sensor.getValue()}")
            # print(f"Sensor Max Value - {sensor.getMaxValue()}")
            # print(f"Sensor Min Value - {sensor.getMinValue()}")
            # print(f"Sensor Aperture - {sensor.getAperture()}")
            # print(f"Sensor Lookup Table - {sensor.getLookupTable()}")
            # print(f"Sensor Type - {sensor.getType()}")

        for sensor in self.back_sensors:
            sensor.enable(self.timestep)

    def __init__(self) -> None:
        self.me = Robot()
        self.timestep = int(self.me.getBasicTimeStep()) * 2
        print(f"Basic time step: {self.timestep}")
        # self.wheels_radius = 0.111  # m
        self.max_speed = 6.4
        self.direction = "F"
        self.right_direction = "F"
        self.left_direction = "F"
        self.default_speed = 0.5 * self.max_speed
        self.right_speed = self.default_speed
        self.left_speed = self.default_speed
        self.safeDistance = 200.0
        # self.pid_r_wall = PID(0.0022, 0.000001, 0.17, self.safeDistance)
        self.pid_r_wall = PID(0.05, 0.000001, 0.01, self.safeDistance)
        self.initMotors()
        self.initSensors()

    def readSensors(self):
        self.front_sensor_values = [sensor.getValue() for sensor in self.front_sensors]
        self.back_sensor_values = [sensor.getValue() for sensor in self.back_sensors]
        print(
            f"Front: {self.front_sensor_values} - Back: {self.back_sensor_values}"
            # f"Front: {self.front_sensor_values}"
        )

    def applySpeed(self):
        self.front_left_motor.setVelocity(self.left_speed)
        self.back_left_motor.setVelocity(self.left_speed)
        self.front_right_motor.setVelocity(self.right_speed)
        self.back_right_motor.setVelocity(self.right_speed)

    def delay(self, ms):
        targetTime = ms / 1000.0
        timeLeft = 0.00

        initTime = self.me.getTime()
        print(f"Init time: {initTime}")

        while timeLeft < targetTime:
            currentTime = self.me.getTime()
            print(f"Current time: {currentTime}")
            timeLeft = currentTime - initTime
            print(f"Time left: {timeLeft}")
            self.me.step(self.timestep)

    def rotate(self, angle):
        # to rotate 90 degrees needs 64 steps
        # 1. calculate the number of steps needed to rotate
        print(f"Angle: {angle}")
        steps = int(abs(angle) * 64 / (pi / 2))
        velocity = 1
        if angle < 0:
            velocity = -1
        self.right_speed = velocity
        self.left_speed = -velocity
        currentStep = 0
        self.applySpeed()
        while currentStep < steps:
            self.me.step(self.timestep)
            currentStep += 1

        self.right_speed = 0
        self.left_speed = 0
        self.applySpeed()

    def run(self):
        # steps = 0
        while self.me.step(self.timestep) != -1:
            self.readSensors()
            test = self.detectWalls()

            # move forward
            self.right_speed = self.default_speed
            self.left_speed = self.default_speed
            self.applySpeed()

            # detect walls
            right_error = self.detectWallRight()

            # # self.detectWallLeft()
            front_obstacle = self.detectFrontWall()
            # print(f"Obstacle: {front_obstacle}")

            if front_obstacle < 650:
                print("Rotating ******************************************************")
                self.rotate(pi / 6)
            else:
                self.left_speed = self.default_speed
                self.right_speed = self.default_speed + right_error
                self.right_speed = 2 if self.right_speed < 2 else self.right_speed
                self.right_speed = 6 if self.right_speed > 6 else self.right_speed
                print(f"Right Speed: {self.right_speed}")

            self.applySpeed()

    def detectWalls(self):
        print(
            f"Sensor values Left: {self.front_sensor_values[0]} | {self.back_sensor_values[7]}"
        )
        print(
            f"Sensor values Right: {self.front_sensor_values[7]} | {self.back_sensor_values[0]}"
        )

        left_median = (self.front_sensor_values[0] + self.back_sensor_values[7]) / 2
        right_median = (self.front_sensor_values[7] + self.back_sensor_values[0]) / 2
        print(f"Left median: {left_median}")
        print(f"Right median: {right_median}")

        # max_sensor = max(
        #     self.front_sensor_values[6],
        #     self.front_sensor_values[7],
        #     # self.back_sensor_values[0],
        # )
        # # max_sensor = max(
        # #     self.front_sensor_values[6],
        # #     self.front_sensor_values[7],
        # #     self.back_sensor_values[0],
        # #     self.back_sensor_values[1],
        # # )
        # # max_sensor = self.front_sensor_values[7]
        # # print(f"Max sensor: {max_sensor}")

        # currentDistanceFromWall = 1024 - max_sensor
        # print(f"Current distance from right wall: {currentDistanceFromWall}")

        # error = self.pid_r_wall.update(currentDistanceFromWall)
        # # print(f"Error: {error}")

        return left_median + right_median

    def detectWallRight(self):
        print(
            f"Sensor values: {self.front_sensor_values[6]} | {self.front_sensor_values[7]}"
        )
        max_sensor = max(
            self.front_sensor_values[6],
            self.front_sensor_values[7],
            # self.back_sensor_values[0],
        )
        # max_sensor = max(
        #     self.front_sensor_values[6],
        #     self.front_sensor_values[7],
        #     self.back_sensor_values[0],
        #     self.back_sensor_values[1],
        # )
        # max_sensor = self.front_sensor_values[7]
        # print(f"Max sensor: {max_sensor}")

        currentDistanceFromWall = 1024 - max_sensor
        print(f"Current distance from right wall: {currentDistanceFromWall}")

        error = self.pid_r_wall.update(currentDistanceFromWall)
        # print(f"Error: {error}")

        return error

    def detectWallLeft(self):
        max_sensor = max(
            self.front_sensor_values[0],
            self.front_sensor_values[1],
            # self.back_sensor_values[6],
        )
        print(f"Max sensor: {max_sensor}")

        currentDistanceFromWall = 1024 - max_sensor
        print(f"Current distance from left wall: {currentDistanceFromWall}")

        error = self.pid_l_wall.update(currentDistanceFromWall)
        print(f"Error: {error}")

        return error

    def detectFrontWall(self):
        max_sensor = (self.front_sensor_values[3] + self.front_sensor_values[4]) / 2
        currentDistanceFromWall = 1024 - max_sensor
        print(f"Current distance from front wall: {currentDistanceFromWall}")
        return currentDistanceFromWall


def main():
    robot = MyRobot()
    robot.rotate(pi / 9)
    robot.delay(100)
    robot.rotate(-pi / 10)
    robot.run()


if __name__ == "__main__":
    main()
