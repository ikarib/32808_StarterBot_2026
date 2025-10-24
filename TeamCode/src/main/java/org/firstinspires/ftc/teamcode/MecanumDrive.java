package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotorSimple rightBackDrive = null;
    private IMU imu = null;

    public void init(HardwareMap hardwareMap) {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorSimple.class, "right_back_drive");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        //rightBackDrive.setZeroPowerBehavior(BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param strafe  the horizontal speed of the robot, derived from input
     * @param forward the vertical speed of the robot, derived from input
     * @param turn    the turn speed of the robot, derived from input
     */
    void driveFieldCentric(double strafe, double forward, double turn) {
        // Square the value of the inputs to allow for finer control
        strafe *= Math.abs(strafe);
        forward *= Math.abs(forward);
        turn *= Math.abs(turn);

        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Second, rotate angle by the angle the robot is pointing, derived from the gyro
        theta -= imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//        r /= Math.abs(Math.sin(theta)) + Math.abs(Math.cos(theta));

        // Third, convert back to cartesian
        forward = r * Math.sin(theta);
        strafe = r * Math.cos(theta);

        // Finally, call the drive method with robot relative speeds
        driveRobotCentric(strafe, forward, turn);
    }

    void driveRobotCentric(double strafe, double forward, double turn){
        // maxPower is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double maxSpeed = 0.5; // 1 is full speed
        double maxPower = Math.max(Math.abs(forward)+Math.abs(strafe)+Math.abs(turn), 1)/maxSpeed;
        // Set power levels for each drive wheel
        leftFrontDrive.setPower((forward + strafe + turn)/maxPower);
        rightFrontDrive.setPower((forward - strafe - turn)/maxPower);
        leftBackDrive.setPower((forward - strafe + turn)/maxPower);
        rightBackDrive.setPower((forward + strafe - turn)/maxPower);
    }

    void stop() {
        driveRobotCentric(0,0,0);
    }
}
