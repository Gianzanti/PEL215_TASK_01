import math
from controller import Robot

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
        return pid


class MyRobot:
    def __init__(self) -> None:
        self.me = Robot()
        self.timestep = int(self.me.getBasicTimeStep()) * 2
        self.max_speed = 6.4  # => 0.7 m/s
        self.default_speed = 0.3 * self.max_speed
        self.right_speed = self.default_speed
        self.left_speed = self.default_speed
        self.r_Distance = 1
        self.pid_r_wall = PID(2, 0, 0.5, self.r_Distance)
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
        ]
        for item in self.sensors:
            item["sensor"].enable(self.timestep)

    def readSensors(self):
        for item in self.sensors:
            item["value"] = item["sensor"].getValue()
            distance = (
                -4.9e-03 * item["sensor"].getValue() + 5.44
                if item["sensor"].getValue() != 0
                else float("NaN")
            )
            item["meters"] = distance
            if not math.isnan(distance):
                print(
                    f"{item['name']}: {item['value']:.2f} {item['meters']:.2f} ", end=""
                )

        print("")

    def applySpeed(self):
        self.front_left_motor.setVelocity(self.left_speed)
        self.back_left_motor.setVelocity(self.left_speed)
        self.front_right_motor.setVelocity(self.right_speed)
        self.back_right_motor.setVelocity(self.right_speed)

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

    def checkFrontWall(self):
        distance = max_ignore_nan(
            [
                self.sensors[3]["meters"],
                self.sensors[4]["meters"],
            ]
        )
        if distance != None:
            print(f"Front Distance: {distance}m")
            return distance
        else:
            print(f"Front Distance > 5.5m")
            return None

    def faceWall(self):
        # check front alignment
        front_alignment = self.sensors[3]["value"] - self.sensors[4]["value"]
        print(f"Front alignment: {front_alignment}")

        if abs(front_alignment) < 15 and (
            self.sensors[3]["value"] > 0 or self.sensors[4]["value"] > 0
        ):
            print("Front aligned...")
            distance = self.checkFrontWall()
            print(
                f"Distance FW: {distance} m *****************************************"
            )

            if distance < 1:
                self.rotate(math.radians(80))
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

            print(f"Right side: {right_side}")
            print(f"Left side: {left_side}")
            print(f"Diff: {diff}")

            if abs(diff) > 45:
                if diff > 0:
                    self.left_speed = self.default_speed
                    self.right_speed = -self.default_speed
                    print("Turning right...")
                else:
                    self.left_speed = -self.default_speed
                    self.right_speed = self.default_speed
                    print("Turning left...")

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
        # check front alignment
        right_alignment = self.sensors[7]["value"] - self.sensors[8]["value"]
        print(f"Right alignment: {right_alignment}")

        if abs(right_alignment) < 1 and (
            self.sensors[7]["value"] > 0 or self.sensors[8]["value"] > 0
        ):
            print("Right aligned...")
            distance = self.checkRightWall()
            print(f"Distance RW: {distance}m *****************************************")
            return True

        else:
            front = self.sensors[7]["value"]
            front = front if front != None else 0
            rear = self.sensors[8]["value"]
            rear = rear if rear != None else 0

            diff = front - rear

            print(f"Front: {front}")
            print(f"Rear: {rear}")
            print(f"Diff: {diff}")

            if abs(diff) > 0.5:
                if diff > 0:
                    self.left_speed = self.default_speed
                    self.right_speed = -self.default_speed
                    print("Turning right...")
                else:
                    self.left_speed = -self.default_speed
                    self.right_speed = self.default_speed
                    print("Turning left...")

        # self.applySpeed()
        return False

    def detectWallRight(self):
        print(
            f'Sensor values: {self.sensors[4]["value"]} | {self.sensors[5]["value"]} | {self.sensors[6]["value"]} | {self.sensors[7]["value"]} | {self.sensors[8]["value"]}'
        )
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

        error = self.pid_r_wall.update(max_sensor)
        print(f"Error: {error}")
        return error

    def run(self):
        state = "get_front_wall"

        while self.me.step(self.timestep) != -1:
            self.readSensors()

            # match statement starts here .
            match state:
                case "get_front_wall":
                    print("Positioning front wall")
                    if self.faceWall():
                        state = "align_to_right"

                case "align_to_right":
                    print("Aligning to right wall")
                    if self.alignToRightWall():
                        state = "control"

                case "control":
                    print("Controlling...")

                    # move forward
                    self.right_speed = self.default_speed
                    self.left_speed = self.default_speed

                    # detect walls
                    right_error = self.detectWallRight()

                    front_obstacle = self.checkFrontWall()
                    print(f"Obstacle: {front_obstacle}")

                    if front_obstacle != None and front_obstacle < 1.0:
                        print(
                            "Rotating ******************************************************"
                        )
                        self.rotate(math.radians(60))
                    else:
                        # self.left_speed = self.default_speed
                        # self.default_speed = 0.3 * self.max_speed
                        self.right_speed = (0.3 + right_error) * self.max_speed
                        self.right_speed = (
                            0.5 if self.right_speed < 0.5 else self.right_speed
                        )
                        self.right_speed = (
                            6 if self.right_speed > 6 else self.right_speed
                        )
                        print(f"Right Speed: {self.right_speed}")

                    self.applySpeed()

                case _:
                    print("You do not have any access to the code")


if __name__ == "__main__":
    robot = MyRobot()
    robot.run()
