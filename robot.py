import wpilib

from networktables import NetworkTables
import math


class SwerveModule():
    max_voltage = 5

    def __init__(self,turn_motor, drive_motor, encoder, zero, p, i, d):
        self.drive_motor = drive_motor
        self.zero = zero
        self.pid_controller = wpilib.PIDController(p,i,d, encoder, turn_motor)
        self.pid_controller.setInputRange(0,5)
        self.pid_controller.setOutputRange(-1,1)
        self.pid_controller.setContinuous(True)
        self.pid_controller.setSetpoint(0 + zero)
        self.pid_controller.setPercentTolerance(10)
        self.pid_controller.enable()
        self.encoder = encoder


    def drive(self, speed, angle):
        self.drive_motor.set(speed)
        wpilib.DriverStation.reportWarning(str(angle), False)
        voltage = (angle/360)*5 + self.zero
        voltage = voltage % 5
        setpoint = voltage



        self.pid_controller.setSetpoint(setpoint)

class SwerveDrive():
    l = 23.25
    w = 23.25

    def __init__(self,back_right, back_left, front_right, front_left):
        self.back_right = back_right
        self.back_left = back_left
        self.front_right = front_right
        self.front_left = front_left

        pass

    def drive(self, x1, y1, x2):
        r = math.hypot(self.l, self.w)

        rearX = x1 - x2 * (self.l / r)
        frontX = x1 + x2 * (self.l / r)
        leftY = y1 - x2 * (self.l / r)
        rightY = y1 + x2 * (self.l / r)



        back_right_speed = math.hypot(rearX, leftY)
        back_left_speed = math.hypot(rearX, rightY)
        front_right_speed = math.hypot(frontX, leftY)
        front_left_speed = math.hypot(frontX, rightY)

        back_right_angle = math.degrees(math.atan2(rearX,leftY))
        back_left_angle = math.degrees(math.atan2(rearX, rightY))
        front_right_angle = math.degrees(math.atan2(frontX, leftY))
        front_left_angle = math.degrees(math.atan2(frontX, rightY))

        self.back_right.drive(back_right_speed, back_right_angle)
        self.back_left.drive(back_left_speed, back_left_angle)
        self.front_right.drive(front_right_speed, front_right_angle)
        self.front_left.drive(front_left_speed, front_left_angle)










class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.front_right_drive = wpilib.Spark(3)
        self.front_right_turn = wpilib.Spark(2)
        self.front_right_turn.setInverted(True)

        self.front_left_drive = wpilib.Spark(6)
        self.front_left_turn = wpilib.Spark(7)
        self.front_left_turn.setInverted(True)

        self.rear_right_drive = wpilib.Spark(1)
        self.rear_right_turn = wpilib.Spark(0)
        self.rear_right_turn.setInverted(True)

        self.rear_left_drive = wpilib.Spark(4)
        self.rear_left_turn = wpilib.Spark(5)
        self.rear_left_turn.setInverted(True)

        self.front_right_encoder = wpilib.AnalogInput(2)
        self.front_left_encoder = wpilib.AnalogInput(3)
        self.rear_right_encoder = wpilib.AnalogInput(0)
        self.rear_left_encoder = wpilib.AnalogInput(1)


        self.joystick = wpilib.XboxController(2)


        NetworkTables.initialize(server='roborio-379-frc.local')
        self.table = NetworkTables.getTable('SmartDashboard')

        self.back_right_swerve = SwerveModule(self.rear_right_turn, self.rear_right_drive, self.rear_right_encoder, 3.25, 1, .1, .55)
        self.back_left_swerve = SwerveModule(self.rear_left_turn, self.rear_left_drive, self.rear_left_encoder,
                                              .067, 1, .1, .55)
        self.front_right_swerve = SwerveModule(self.front_right_turn, self.front_right_drive, self.front_right_encoder,
                                              4.79, 1, .1, .55)
        self.front_left_swerve = SwerveModule(self.front_left_turn, self.front_left_drive, self.front_left_encoder,
                                              2.09, 1, .1, .55)
        self.swerve_drive = SwerveDrive(self.back_right_swerve, self.back_left_swerve, self.front_right_swerve, self.front_left_swerve)

        self.rear_right_drive.setInverted(True)
        self.front_right_drive.setInverted(True)

        self.left_joystick=wpilib.Joystick(0)
        self.right_joystick=wpilib.Joystick(1)


    def teleopPeriodic(self):
        self.deadZoneX = .05
        self.deadZoneY = .05
        if(self.left_joystick.getX() < self.deadZoneX) or (self.left_joystick.getX() > -(self.deadZoneX)):
            self.left_joystick_outputX = self.left_joystick.getX()
        else:
            self.left_joystick_X = self.left_joystick(x)

        self.swerve_drive.drive(self.left_joystick.getX(), self.left_joystick.getY()*-1, self.right_joystick.getX())






if __name__ == '__main__':
    wpilib.run(MyRobot)