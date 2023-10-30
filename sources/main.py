import math

''' TODO '''
class Vector:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z
    
    def length(self) -> float:
        return math.sqrt(self.dotMultiplyVector(self))
    
    def angle(self, vector: Vector) -> float:
        pass
    
    def normalize(self) -> Vector:
        return self.multiplyScalar(1 / self.length())
    
    def rotate(self, axis: Vector, angle: float) -> Vector:
        pass

    def add(self, vector: Vector) -> Vector:
        return Vector(self.x + vector.x, self.y + vector.y, self.z + vector.z)
    
    def multiplyScalar(self, scalar: float) -> Vector:
        return Vector(self.x * scalar, self.y * scalar, self.z * scalar)

    def dotMultiplyVector(self, vector: Vector) -> float:
        return self.x * vector.x + self.y * vector.y + self.z * vector.z

    def crossMultiplyVector(self, vector: Vector) -> Vector:
        pass

''' TODO '''
class Point:
    def __init__(self, position: Vector, axis: Vector, angle: float) -> None:
        self.position = position
        self.axis = axis
        self.angle = angle

''' TODO '''
class Angle:
    def __init__(self, min: float, max: float, value: float) -> None:
        self.min = min
        self.max = max
        self.value = value

''' TODO '''
class Drive:
    def __init__(self, axis: Vector, angle: Angle, speed: float) -> None:
        self.axis = axis
        self.angle = angle
        self.speed = speed

''' TODO '''
class Joint:
    def __init__(self, drive: Drive, length: float, radius: float) -> None:
        self.drive = drive
        self.length = length
        self.radius = radius

''' TODO '''
class Robot:
    def __init__(self, base: Point, offset: float, joints: list[Joint]) -> None:
        self.base = base
        self.offset = offset
        self.joints = joints
        self.tip = self.forwardKinematics(self.collectAngles())

    def collectAngles(self) -> list[float]:
        angles: list[float] = []
        for joint in self.joints:
            angles.append(joint.drive.angle.value)
        return angles
    
    def forwardKinematics(angles: list[float]) -> Point:
        pass

    def inverseKinematics(tip: Point) -> list[float]:
        pass