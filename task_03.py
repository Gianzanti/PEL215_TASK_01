import math
from controller import Robot, Motor, DistanceSensor

INF = float("+inf")


def max_ignore_nan(arr):
    content = list(filter(lambda x: not math.isnan(x), arr))
    if (len(content)) == 0:
        return None
    return max(content)


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

        # print(f"PID raw: {self.error} | {self.error_sum} | {self.error_diff}")
        # print(f"PID splited: {proportional} | {integral} | {derivative}")
        # print(f"PID: {pid}")

        return pid


class MyRobot:
    def __init__(self) -> None:
        self.me = Robot()
        self.timestep = int(self.me.getBasicTimeStep()) * 2
        self.max_speed = 6.4  # => 0.7 m/s
        self.default_speed = 0.5 * self.max_speed
        self.right_speed = self.default_speed
        self.left_speed = self.default_speed
        self.r_Distance = 1
        self.f_Distance = 0
        self.l_Distace = 3
        # self.pid_r_wall = PID(0.0022, 0.000001, 0.17, self.safeDistance)
        self.pid_r_wall = PID(0.5, 0, 0.1, self.r_Distance)
        # self.pid_f_wall = PID(0.0022, 0.000001, 0.17, self.f_Distance)
        self.pid_l_wall = PID(0.1, 0, 0.01, self.l_Distace)
        self.initMotors()
        self.initSensors()

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
        self.sensors = [
            {
                "name": "so0",
                "sensor": self.me.getDevice("so0"),
                "type": "front",
                "position": 0,
                "value": 0,
            },
            {
                "name": "so1",
                "sensor": self.me.getDevice("so1"),
                "type": "front",
                "position": 40,
                "value": 0,
            },
            {
                "name": "so2",
                "sensor": self.me.getDevice("so2"),
                "type": "front",
                "position": 60,
                "value": 0,
            },
            {
                "name": "so3",
                "sensor": self.me.getDevice("so3"),
                "type": "front",
                "position": 80,
                "value": 0,
            },
            {
                "name": "so4",
                "sensor": self.me.getDevice("so4"),
                "type": "front",
                "position": 100,
                "value": 0,
            },
            {
                "name": "so5",
                "sensor": self.me.getDevice("so5"),
                "type": "front",
                "position": 120,
                "value": 0,
            },
            {
                "name": "so6",
                "sensor": self.me.getDevice("so6"),
                "type": "front",
                "position": 140,
                "value": 0,
            },
            {
                "name": "so7",
                "sensor": self.me.getDevice("so7"),
                "type": "front",
                "position": 180,
                "value": 0,
            },
            {
                "name": "so8",
                "sensor": self.me.getDevice("so8"),
                "type": "back",
                "position": 180,
                "value": 0,
            },
            {
                "name": "so9",
                "sensor": self.me.getDevice("so9"),
                "type": "back",
                "position": 220,
                "value": 0,
            },
            {
                "name": "so10",
                "sensor": self.me.getDevice("so10"),
                "type": "back",
                "position": 240,
                "value": 0,
            },
            {
                "name": "so11",
                "sensor": self.me.getDevice("so11"),
                "type": "back",
                "position": 260,
                "value": 0,
            },
            {
                "name": "so12",
                "sensor": self.me.getDevice("so12"),
                "type": "back",
                "position": 280,
                "value": 0,
            },
            {
                "name": "so13",
                "sensor": self.me.getDevice("so13"),
                "type": "back",
                "position": 300,
                "value": 0,
            },
            {
                "name": "so14",
                "sensor": self.me.getDevice("so14"),
                "type": "back",
                "position": 320,
                "value": 0,
            },
            {
                "name": "so15",
                "sensor": self.me.getDevice("so15"),
                "type": "back",
                "position": 0,
                "value": 0,
            },
        ]
        for item in self.sensors:
            item["sensor"].enable(self.timestep)

    def readSensors(self):
        for item in self.sensors:
            # item["value"] = item["sensor"].getValue()
            distance = (
                -4.9e-03 * item["sensor"].getValue() + 5.44
                if item["sensor"].getValue() != 0
                else float("NaN")
            )
            item["value"] = distance
            if not math.isnan(distance):
                print(f"{item['name']}: {item['value']:.2f} ", end="")

        print("")

    def applySpeed(self):
        self.front_left_motor.setVelocity(self.left_speed)
        self.back_left_motor.setVelocity(self.left_speed)
        self.front_right_motor.setVelocity(self.right_speed)
        self.back_right_motor.setVelocity(self.right_speed)

    def delay(self, ms):
        targetTime = ms / 1000.0
        initTime = self.me.getTime()
        while self.me.getTime() - initTime < targetTime:
            self.me.step(self.timestep)

    def rotate(self, angle):
        # to rotate 90 degrees needs 64 steps
        # 1. calculate the number of steps needed to rotate
        print(f"Angle: {angle}")
        steps = int(abs(angle) * 64 / (math.pi / 2))
        print(f"Steps: {steps}")
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

    def moveStraight(self, distance):
        speedFactor = 0.5
        self.default_speed = speedFactor * self.max_speed  # 0.7 m/s
        self.default_speed = self.default_speed if distance > 0 else -self.default_speed
        self.right_speed = self.default_speed
        self.left_speed = self.default_speed

        linear_velocity = 0.7 * speedFactor

        timeRequired = abs(distance) / linear_velocity
        # print(f"Time required: {timeRequired}")

        timeSteps = int(timeRequired / (self.timestep / 1000.0))
        # print(f"Time steps: {timeSteps}")
        self.applySpeed()

        while timeSteps > 0:
            timeSteps -= 1
            # print(f"Steps left: {timeSteps}")
            self.me.step(self.timestep)

        self.right_speed = 0
        self.left_speed = 0
        self.applySpeed()

    def forward(self, distance):
        # to move 2 meters needs 285 steps
        # 1. calculate the number of steps needed to move
        print(f"Distance: {distance}m")
        steps = int(abs(distance) * (285 / 2))
        print(f"Steps: {steps}")
        velocity = 1
        if distance < 0:
            velocity = -1
        self.right_speed = velocity
        self.left_speed = velocity
        currentStep = 0
        self.applySpeed()
        while currentStep < steps:
            self.me.step(self.timestep)
            currentStep += 1

        self.right_speed = 0
        self.left_speed = 0
        self.applySpeed()

    # sensor value - distance
    # 7.2 ->  5.4m
    # 27.8 ->  5.3m
    # 48.1 ->  5.2m
    # 68.7 ->  5.1m
    # 89.3 -> 5m
    # 109.9 ->  4.9m
    # 130.5 ->  4.8m
    # 150.5 ->  4.7m
    # 171.1 ->  4.6m
    # 190.5 ->  4.5m
    # 212.1 ->  4.4m
    # 415 ->  3.4m
    # 620 ->  2.4m
    # 825 ->  1.4m
    # 845 ->  1.3m
    # 865 ->  1.2m
    # 885 ->  1.1m
    # 905 -> 1m
    # 925 ->  0.9m
    # 945 ->  0.8m
    # 965 ->  0.7m
    # 985 ->  0.6m

    # linearization equation for this table
    # y = -4,9E-03*x + 5,44

    def followRightWall(self):
        distance = max_ignore_nan(
            [
                self.sensors[6]["value"],
                self.sensors[7]["value"],
            ]
        )

        # if distance == None:
        #     distance = max_ignore_nan(
        #         [
        #             self.sensors[5]["value"],
        #             self.sensors[8]["value"],
        #         ]
        #     )

        # print(f"Right Distance: {distance}")
        if distance != None:
            # distance = -4.9e-03 * distance + 5.44
            print(f"Right Distance: {distance}m")
            return distance
        else:
            print(f"Right Distance > 5.5m")
            return None

    def avoidFrontWall(self):
        distance = max_ignore_nan(
            [
                self.sensors[2]["value"],
                self.sensors[3]["value"],
                self.sensors[4]["value"],
                self.sensors[5]["value"],
            ]
        )
        # print(f"Right Distance: {distance}")
        if distance != None:
            # distance = -4.9e-03 * distance + 5.44
            print(f"Front Distance: {distance}m")
            return distance
        else:
            print(f"Front Distance > 5.5m")
            return None

    def avoidLeftWall(self):
        distance = max_ignore_nan(
            [
                self.sensors[0]["value"],
                self.sensors[1]["value"],
            ]
        )
        # print(f"Right Distance: {distance}")
        if distance != None:
            # distance = -4.9e-03 * distance + 5.44
            print(f"Left Distance: {distance}m")
            return distance
        else:
            print(f"Left Distance > 5.5m")
            return None

    def run(self):
        while self.me.step(self.timestep) != -1:
            self.readSensors()

            r_dist = self.followRightWall()
            l_dist = self.avoidLeftWall()
            if r_dist != None:
                error = self.pid_r_wall.update(r_dist)
                print(f"Right Error: {error}")
                error = 0.5 + error
                error = 0.1 if (error < 0.1) else error
                error = 1 if (error > 1) else error
                self.right_speed = error * self.max_speed
                print(f"Right Speed: {self.right_speed}")
            elif l_dist != None and l_dist < 1.2:
                error = self.pid_l_wall.update(l_dist)
                print(f"Left Error: {error}")
                error = 0.5 + error
                error = 0.1 if (error < 0.1) else error
                error = 1 if (error > 1) else error
                self.right_speed = -error * self.max_speed
                print(f"Right Speed: {self.right_speed}")
            else:
                self.right_speed = self.default_speed

            f_dist = self.avoidFrontWall()
            if f_dist != None and f_dist < 1:
                self.rotate(math.pi / 6)

            if (
                (r_dist == None)
                and (f_dist == None)
                and (l_dist == None or l_dist > 1.5)
            ):
                self.right_speed = 0.4 * self.max_speed

            self.left_speed = self.default_speed
            self.applySpeed()

            # if not done:
            #     self.moveStraight(4)
            #     self.delay(500)
            #     self.moveStraight(-4)
            #     done = True

            # self.right_speed = 2
            # self.left_speed = 2
            # steps += 1
            # self.applySpeed()
            # print(f"Steps: {steps}")

            # self.readSensors()
            # currentDistanceFromWall = (
            #     1024 - (self.sensors[3]["value"] + self.sensors[4]["value"]) / 2
            # )
            # print(f"Current distance from front wall: {currentDistanceFromWall}")

            # 2mts da parede: 585 588 = 586,5 - 1024 = 437,5
            # 1.5mts da parede: 699 699 = 699 - 1024 = 325
            # 1mts da parede: 820 796 = 808 - 1024 = 216
            # 0.5mts da parede: 909 905 = 907 - 1024 = 117

            # self.readSensors()
            # done = self.faceWall()
            # while (not done) and self.me.step(self.timestep) != -1:
            #     self.readSensors()
            #     done = self.faceWall()

            # print("Done")
            # # test = self.detectWalls()

            # # move forward
            # self.readSensors()
            # self.correctAlignment()
            # self.deviate()
            # self.forward(0.1)

            # correct alignment

            # # detect walls
            # right_error = self.detectWallRight()

            # # # self.detectWallLeft()
            # front_obstacle = self.detectFrontWall()
            # # print(f"Obstacle: {front_obstacle}")

            # if front_obstacle < 650:
            #     print("Rotating ******************************************************")
            #     self.rotate(math.pi / 6)
            # else:
            #     self.left_speed = self.default_speed
            #     self.right_speed = self.default_speed + right_error
            #     self.right_speed = 2 if self.right_speed < 2 else self.right_speed
            #     self.right_speed = 6 if self.right_speed > 6 else self.right_speed
            #     print(f"Right Speed: {self.right_speed}")

            # self.applySpeed()

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
        # max_sensor = max(
        #     self.sensors[6]["value"],
        #     self.sensors[7]["value"],
        # )
        max_sensor = (self.sensors[7]["value"] + self.sensors[8]["value"]) / 2
        print(f'Sensor values: {self.sensors[7]["value"]} | {self.sensors[8]["value"]}')
        currentDistanceFromWall = 1024 - max_sensor
        print(f"Current distance from right wall: {currentDistanceFromWall}")

        # error = self.pid_r_wall.update(currentDistanceFromWall)
        # # print(f"Error: {error}")

        return currentDistanceFromWall

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

    def faceWallOld(self):
        # check front alignment
        front_alignment = self.sensors[3]["value"] - self.sensors[4]["value"]
        print(f"Front alignment: {front_alignment}")

        if abs(front_alignment) < 10 and self.sensors[3]["value"] > 0:
            print("Front aligned...")

            currentDistanceFromWall = (
                1024 - (self.sensors[3]["value"] + self.sensors[4]["value"]) / 2
            )
            print(f"Current distance from front wall: {currentDistanceFromWall}")

            # 2mts da parede: 585 588 = 586,5 - 1024 = 437,5
            # 1.5mts da parede: 699 699 = 699 - 1024 = 325
            # 1mts da parede: 820 796 = 808 - 1024 = 216
            # 0.5mts da parede: 909 905 = 907 - 1024 = 117

            if currentDistanceFromWall < 120:
                self.rotate(math.pi / 2)
            else:
                self.forward((currentDistanceFromWall - 120) / (2 * 110))

            return

        # check lateral alignment
        right_alignment = self.sensors[7]["value"] - self.sensors[8]["value"]
        print(f"Right alignment: {right_alignment}")
        left_alignment = self.sensors[0]["value"] - self.sensors[15]["value"]
        print(f"Left alignment: {left_alignment}")

        if (abs(right_alignment) < 30 and self.sensors[7]["value"] > 0) or (
            abs(left_alignment) < 30 and self.sensors[0]["value"] > 0
        ):
            print("Lateral aligned...")
            # check wall distance
            rightDistance = self.detectWallRight()
            if rightDistance > 150:
                self.rotate(-math.pi / 70)
            else:
                self.forward(0.05)
            return

        # check where to turn to align
        left_side = max(
            self.sensors[0]["value"],
            self.sensors[1]["value"],
            self.sensors[2]["value"],
            self.sensors[3]["value"],
        )

        right_side = max(
            self.sensors[4]["value"],
            self.sensors[5]["value"],
            self.sensors[6]["value"],
            self.sensors[7]["value"],
        )

        print(f"Right side: {right_side}")
        print(f"Left side: {left_side}")

        if right_side > left_side and right_side > 600:
            print("Turning right...")
            self.rotate(-math.pi / 70)
            return

        if right_side < left_side and left_side > 600:
            print("Turning left...")
            self.rotate(math.pi / 70)
            return

        self.forward(0.1)

    def faceWall(self):
        # check front alignment
        front_alignment = self.sensors[3]["value"] - self.sensors[4]["value"]
        print(f"Front alignment: {front_alignment}")

        if abs(front_alignment) < 10 and self.sensors[3]["value"] > 0:
            print("Front aligned...")

            currentDistanceFromWall = (
                1024 - (self.sensors[3]["value"] + self.sensors[4]["value"]) / 2
            )
            print(f"Current distance from front wall: {currentDistanceFromWall}")

            # 2mts da parede: 585 588 = 586,5 - 1024 = 437,5
            # 1.5mts da parede: 699 699 = 699 - 1024 = 325
            # 1mts da parede: 820 796 = 808 - 1024 = 216
            # 0.5mts da parede: 909 905 = 907 - 1024 = 117

            if currentDistanceFromWall < 120:
                self.rotate(math.pi / 2)
                return True
            else:
                self.forward((currentDistanceFromWall - 120) / (2 * 110))

            return False

        # # check lateral alignment
        # right_alignment = self.sensors[7]["value"] - self.sensors[8]["value"]
        # print(f"Right alignment: {right_alignment}")
        # left_alignment = self.sensors[0]["value"] - self.sensors[15]["value"]
        # print(f"Left alignment: {left_alignment}")

        # # if (abs(right_alignment) < 30 and self.sensors[7]["value"] > 0) or (
        # #     abs(left_alignment) < 30 and self.sensors[0]["value"] > 0
        # # ):
        # #     print("Lateral aligned...")
        # #     # check wall distance
        # #     # rightDistance = self.detectWallRight()
        # #     # if rightDistance > 150:
        # #     #     self.rotate(-math.pi / 70)
        # #     # else:
        # #     #     self.forward(0.05)
        # #     # return
        # #     return False

        # else:
        # check where to turn to align
        left_side = max(
            self.sensors[0]["value"],
            self.sensors[1]["value"],
            self.sensors[2]["value"],
            self.sensors[3]["value"],
        )

        right_side = max(
            self.sensors[4]["value"],
            self.sensors[5]["value"],
            self.sensors[6]["value"],
            self.sensors[7]["value"],
        )

        print(f"Right side: {right_side}")
        print(f"Left side: {left_side}")

        if right_side > left_side:  # and right_side > 600:
            print("Turning right...")
            self.rotate(-math.pi / 70)
            return False

        if right_side < left_side:  # and left_side > 600:
            print("Turning left...")
            self.rotate(math.pi / 70)
            return False

        # self.forward(0.1)
        return False

    def correctAlignment(self):
        # check lateral alignment
        right_alignment = self.sensors[7]["value"] - self.sensors[8]["value"]
        print(f"Right alignment: {right_alignment}")
        left_alignment = self.sensors[0]["value"] - self.sensors[15]["value"]
        print(f"Left alignment: {left_alignment}")

        if (abs(right_alignment) < 30 and self.sensors[7]["value"] > 0) or (
            abs(left_alignment) < 30 and self.sensors[0]["value"] > 0
        ):
            print("Lateral aligned...")
            # check wall distance
            rightDistance = self.detectWallRight()
            if rightDistance > 150:
                self.rotate(-math.pi / 70)
            elif rightDistance < 130:
                self.rotate(math.pi / 70)
            # else:
            #     self.forward(0.05)
            return

    def deviate(self):
        left_side = max(
            self.sensors[2]["value"],
            self.sensors[3]["value"],
        )

        right_side = max(
            self.sensors[4]["value"],
            self.sensors[5]["value"],
        )

        print(f"Right side: {right_side}")
        print(f"Left side: {left_side}")

        if right_side > left_side:  # and right_side > 600:
            print("Turning left...")
            self.rotate(math.pi / 70)

        if right_side < left_side:  # and left_side > 600:
            print("Turning right...")
            self.rotate(-math.pi / 70)


def main():
    robot = MyRobot()
    robot.run()


if __name__ == "__main__":
    main()
