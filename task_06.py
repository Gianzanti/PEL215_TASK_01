import math
import numpy as np
from controller import Robot
from pid import PID

INF = float("+inf")


def max_ignore_nan(arr):
    content = list(filter(lambda x: not math.isnan(x), arr))
    if (len(content)) == 0:
        return None
    return max(content)


class MyRobot:
    def __init__(self) -> None:
        self.me = Robot()
        self.timestep = int(self.me.getBasicTimeStep()) * 2
        print(f"Time step: {self.timestep}")
        self.max_speed = 6.4  # => 0.7 m/s
        self.regular_speed = 5  # => 0.5 m/s
        self.speed_factor = 0.5
        self.default_speed = self.speed_factor * self.regular_speed

        self.r_Distance = 1.2

        Ku = 0.11
        # 4325 / 8 = 540.625
        Tu = 540.625 * self.timestep / 1000

        # self.Kp = 0.15
        self.Kp = 0.60 * Ku
        self.Ki = 2 * self.Kp / Tu
        self.Kd = self.Kp * Tu / 8

        # self.Kp = 0.12
        # self.Ki = 0.0
        # self.Kd = 0.0

        print(f"Kp: {self.Kp}")
        print(f"Ki: {self.Ki}")
        print(f"Kd: {self.Kd}")

        self.control_rightDistance = PID(
            Kp=self.Kp,
            Ki=self.Ki,
            Kd=self.Kd,
            outMax=self.speed_factor,
            outMin=-self.speed_factor,
            lim_int_min=-self.max_speed,
            lim_int_max=self.max_speed,
            T=self.timestep / 1000,
            τ=1 / (10 * self.timestep / 1000),
        )

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
                "meters": float("NaN"),
            },
            {
                "name": "so1",
                "sensor": self.me.getDevice("so1"),
                "type": "front",
                "position": 40,
                "value": 0,
                "meters": float("NaN"),
            },
            {
                "name": "so2",
                "sensor": self.me.getDevice("so2"),
                "type": "front",
                "position": 60,
                "value": 0,
                "meters": float("NaN"),
            },
            {
                "name": "so3",
                "sensor": self.me.getDevice("so3"),
                "type": "front",
                "position": 80,
                "value": 0,
                "meters": float("NaN"),
            },
            {
                "name": "so4",
                "sensor": self.me.getDevice("so4"),
                "type": "front",
                "position": 100,
                "value": 0,
                "meters": float("NaN"),
            },
            {
                "name": "so5",
                "sensor": self.me.getDevice("so5"),
                "type": "front",
                "position": 120,
                "value": 0,
                "meters": float("NaN"),
            },
            {
                "name": "so6",
                "sensor": self.me.getDevice("so6"),
                "type": "front",
                "position": 140,
                "value": 0,
                "meters": float("NaN"),
            },
            {
                "name": "so7",
                "sensor": self.me.getDevice("so7"),
                "type": "front",
                "position": 180,
                "value": 0,
                "meters": float("NaN"),
            },
            {
                "name": "so8",
                "sensor": self.me.getDevice("so8"),
                "type": "back",
                "position": 180,
                "value": 0,
                "meters": float("NaN"),
            },
            {
                "name": "so9",
                "sensor": self.me.getDevice("so9"),
                "type": "back",
                "position": 180,
                "value": 0,
                "meters": float("NaN"),
            },
            # {
            #     "name": "so10",
            #     "sensor": self.me.getDevice("so10"),
            #     "type": "back",
            #     "position": 180,
            #     "value": 0,
            #     "meters": float("NaN"),
            # },
        ]
        for item in self.sensors:
            item["sensor"].enable(self.timestep)

    def readSensors(self):
        for item in self.sensors:
            item["value"] = item["sensor"].getValue()
            distance = (
                -5e-03 * item["sensor"].getValue() + 5.0
                if item["sensor"].getValue() != 0
                else float("NaN")
            )
            item["meters"] = round(distance, 3)
            # if not math.isnan(distance):
            #     print(
            #         f"{item['name']}: {item['value']:.2f} {item['meters']:.2f} ", end=""
            #     )

        # print("")

    def applySpeed(self):
        # print(f"Left Speed: {self.left_speed}")
        # print(f"Right Speed: {self.right_speed}")
        self.front_left_motor.setVelocity(self.left_speed)
        self.back_left_motor.setVelocity(self.left_speed)
        self.front_right_motor.setVelocity(self.right_speed)
        self.back_right_motor.setVelocity(self.right_speed)

    def rotate(self, angle):
        # to rotate 90 degrees needs 64 steps
        # 1. calculate the number of steps needed to rotate
        # print(f"Angle: {angle}")
        steps = int(abs(angle) * 64 / (math.pi / 2))
        # print(f"Steps: {steps}")
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

    def checkFrontWall(self):
        distanceA = max_ignore_nan(
            [
                self.sensors[3]["meters"],
                self.sensors[4]["meters"],
            ]
        )

        distanceB = max_ignore_nan(
            [
                self.sensors[5]["meters"],
            ]
        )

        if distanceA != None:
            distance = distanceA * 1.1
            print(f"Main Front Distance: {distance} m")
            return distance
        elif distanceB != None:
            distance = distanceB * 2
            print(f"Auxiliar Front Distance: {distance} m")
            return distance
        else:
            return None

    def faceWall(self):
        # check front alignment
        front_alignment = self.sensors[3]["value"] - self.sensors[4]["value"]
        # print(f"Front alignment: {front_alignment}")

        if abs(front_alignment) < 15 and (
            self.sensors[3]["value"] > 0 or self.sensors[4]["value"] > 0
        ):
            # print("Front aligned...")
            distance = self.checkFrontWall()
            # print(
            #     f"Distance FW: {distance} m *****************************************"
            # )

            if distance <= self.r_Distance:
                self.rotate(math.radians(89))
                return True
            else:
                self.left_speed = self.default_speed
                self.right_speed = self.default_speed

        else:  # check where to turn to align
            left_side = max_ignore_nan(
                [
                    self.sensors[1]["value"],
                    self.sensors[2]["value"],
                    # self.sensors[3]["value"],
                ]
            )
            left_side = left_side if left_side != None else 0

            right_side = max_ignore_nan(
                [
                    # self.sensors[4]["value"],
                    self.sensors[5]["value"],
                    self.sensors[6]["value"],
                ]
            )
            right_side = right_side if right_side != None else 0

            diff = right_side - left_side

            # print(f"Right side: {right_side}")
            # print(f"Left side: {left_side}")
            # print(f"Diff: {diff}")

            if abs(diff) > 45:
                if diff > 0:
                    self.left_speed = self.default_speed
                    self.right_speed = -self.default_speed
                    # print("Turning right...")
                else:
                    self.left_speed = -self.default_speed
                    self.right_speed = self.default_speed
                    # print("Turning left...")

            self.applySpeed()
        return False

    def checkRightWall(self):
        distance = max_ignore_nan(
            [
                self.sensors[6]["meters"],
                self.sensors[7]["meters"],
            ]
        )
        if distance != None:
            print(f"Right Distance: {distance}m")
            return distance
        else:
            print(f"Right Distance > 5.5m")
            return None

    def alignToRightWall(self):
        right_alignment = self.sensors[7]["value"] - self.sensors[8]["value"]

        if abs(right_alignment) < 1 and (
            self.sensors[7]["value"] > 0 or self.sensors[8]["value"] > 0
        ):
            distance = self.checkRightWall()
            return True

        else:
            front = self.sensors[7]["value"]
            front = front if front != None else 0
            rear = self.sensors[8]["value"]
            rear = rear if rear != None else 0

            diff = front - rear

            if abs(diff) > 0.5:
                if diff > 0:
                    self.left_speed = self.default_speed
                    self.right_speed = -self.default_speed
                else:
                    self.left_speed = -self.default_speed
                    self.right_speed = self.default_speed

        return False

    def detectWallRight(self, setPoint):
        # print(
        #     f'Sensor values: {self.sensors[4]["value"]} | {self.sensors[5]["value"]} | {self.sensors[6]["value"]} | {self.sensors[7]["value"]} | {self.sensors[8]["value"]}'
        # )
        max_sensor = max_ignore_nan(
            [
                self.sensors[6]["meters"],
                self.sensors[7]["meters"],
            ]
        )
        sensor8 = (
            self.sensors[8]["meters"]
            if not math.isnan(self.sensors[8]["meters"])
            else None
        )
        sensor5 = (
            self.sensors[5]["meters"]
            if not math.isnan(self.sensors[5]["meters"])
            else None
        )
        sensor4 = (
            self.sensors[4]["meters"]
            if not math.isnan(self.sensors[4]["meters"])
            else 5
        )
        max_sensor = max_sensor if max_sensor != None else sensor8
        max_sensor = max_sensor if max_sensor != None else sensor5
        max_sensor = max_sensor if max_sensor != None else sensor4

        print(f"Current distance from right wall: {max_sensor}")

        error = self.control_rightDistance.update(
            measurement=max_sensor, setPoint=setPoint
        )
        # print(f"Error: {error}")
        return error

    def getRightDistance(self):
        sensorA = max_ignore_nan(
            [
                self.sensors[6]["meters"],
                self.sensors[7]["meters"],
            ]
        )

        sensorB = max_ignore_nan(
            [
                # self.sensors[4]["meters"],
                self.sensors[5]["meters"],
                self.sensors[8]["meters"],
                self.sensors[9]["meters"],
                # self.sensors[10]["meters"],
            ]
        )

        if sensorA != None:
            return sensorA
        elif sensorB != None:
            return sensorB
        else:
            return 3.0

    def avoidFrontObstacle(self, distance, angle=30):
        mainLeftFront = (
            self.sensors[3]["meters"] * 1.1
            if not math.isnan(self.sensors[3]["meters"])
            else None
        )
        auxLeftFront = (
            self.sensors[2]["meters"] * 1.4
            if not math.isnan(self.sensors[2]["meters"])
            else None
        )

        mainRightFront = (
            self.sensors[4]["meters"] * 1.1
            if not math.isnan(self.sensors[4]["meters"])
            else None
        )
        auxRightFront = (
            self.sensors[5]["meters"] * 1.4
            if not math.isnan(self.sensors[5]["meters"])
            else None
        )

        if mainLeftFront != None and mainRightFront != None:
            if mainLeftFront < distance or mainRightFront < distance:
                diff = mainLeftFront - mainRightFront
                print(f"Diff: {diff}")
                if diff > 0.5:
                    print(f"Main Front Left Distance: {mainLeftFront} m")
                    return -angle
                else:
                    print(f"Main Front Right Distance: {mainRightFront} m")
                    return angle

        elif mainLeftFront != None and mainLeftFront < distance:
            print(f"Main Front Left Distance: {mainLeftFront} m")
            return -angle

        elif mainRightFront != None and mainRightFront < distance:
            print(f"Main Front Right Distance: {mainRightFront} m")
            return angle

        elif auxLeftFront != None and auxRightFront != None:
            if auxLeftFront < distance or auxRightFront < distance:
                diff = auxLeftFront - auxRightFront
                if diff > 0.5:
                    print(f"Aux Front Left Distance: {auxLeftFront} m")
                    return -angle
                else:
                    print(f"Aux Front Right Distance: {auxRightFront} m")
                    return angle

        elif auxLeftFront != None and auxLeftFront < distance:
            print(f"Aux Front Left Distance: {auxLeftFront} m")
            return -angle

        elif auxRightFront != None and auxRightFront < distance:
            print(f"Aux Front Right Distance: {auxRightFront} m")
            return angle

        else:
            print("No front obstacle detected")
            return None

    def run(self):
        state = "get_front_wall"
        while self.me.step(self.timestep) != -1:
            self.readSensors()

            match state:
                case "get_front_wall":
                    # print("Positioning front wall")
                    if self.faceWall():
                        state = "align_to_right"

                case "align_to_right":
                    # print("Aligning to right wall")
                    if self.alignToRightWall():
                        state = "control"

                case "control":
                    # print("Controlling...")

                    setPoint = self.r_Distance
                    position = self.getRightDistance()
                    print(f"Position: {position} m")
                    error = self.control_rightDistance.update(
                        measurement=position, setPoint=setPoint
                    )

                    self.right_speed = self.default_speed
                    self.left_speed = self.default_speed

                    front_obstacle = self.avoidFrontObstacle(setPoint, 46)

                    if front_obstacle != None:
                        self.rotate(math.radians(front_obstacle))
                    else:
                        self.right_speed = (
                            self.speed_factor + error
                        ) * self.regular_speed

                        print(f"Right Speed: {self.right_speed}")

                    self.applySpeed()


if __name__ == "__main__":
    robot = MyRobot()
    robot.run()
