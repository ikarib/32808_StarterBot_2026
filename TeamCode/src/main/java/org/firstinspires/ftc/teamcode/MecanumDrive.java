package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    private GoBildaPinpointDriver pinpoint;
    private Pose2D currentPose;
    private PIDFController strafe, forward, turn;
    private double maxSpeed = 1; // 1 is full speed

    /*
     * Here we create three timers which we use in different parts of our code. Each of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they count independently
     * from each other.
     */
    private final ElapsedTime driveTimer = new ElapsedTime();

    // Declare OpMode members.
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive;
    private DcMotorSimple rightBackDrive;

    public void init(HardwareMap hardwareMap) {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotorSimple.class, "right_back_drive");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make the robot go forward. So, adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90° drives may require direction flips
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

        stop();

        // Configure the goBILDA Pinpoint Odometry sensor
        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far forwards from the tracking point the X (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         *
         *  The Y pod offset refers to how far sideways from the tracking point the Y (forward) odometry pod is.
         *  Right of the center is a positive number, left of center is a negative number.
         */
        pinpoint.setOffsets(72, 156, DistanceUnit.MM);

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (strafe) pod should
         * increase when you move the robot to the right. And the Y (forward) pod should increase when
         * you move the robot forward.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();

        // Set up PID controllers used in driving the robot.
        strafe = new PIDFController(4.0,0.05,0.2,0); strafe.setTolerance(0.01); // in meters
        forward = new PIDFController(4.0,0.05,0.2,0); forward.setTolerance(0.01); // in meters
        turn = new PIDFController(0.015,0,0.0003,0); turn.setTolerance(1); // in degrees
    }

    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(pose);
        currentPose = pose;
    }

    public void humanInputs(Gamepad driver) {
        // Square the value of the inputs to allow for finer control
        double strafe = driver.right_stick_x; strafe *= Math.abs(strafe);
        double forward = -driver.right_stick_y; forward *= Math.abs(forward);
//        double turn = driver.left_stick_x; turn *= Math.abs(turn);
        double turn = Math.hypot(driver.left_stick_x, driver.left_stick_y);
        if (turn > 0) {
            double desiredHeading = Math.toDegrees(Math.atan2(-driver.left_stick_x, -driver.left_stick_y));
            turn *= this.turn.calculate(wrapAngle(desiredHeading - currentPose.getHeading(AngleUnit.DEGREES)));
        }
        setMaxSpeed(0.5);
        driveFieldCentric(strafe, forward, turn);
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
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // Second, rotate angle by the angle the robot is pointing, derived from the gyro
        pinpoint.update();
        currentPose = pinpoint.getPosition();
        theta -= currentPose.getHeading(AngleUnit.RADIANS);
//        r /= Math.abs(Math.sin(theta)) + Math.abs(Math.cos(theta));

        // Third, convert back to cartesian
        forward = r * Math.sin(theta);
        strafe = r * Math.cos(theta);

        // Finally, call the drive method with robot relative speeds
        driveRobotCentric(strafe, forward, turn);
    }

    public void setMaxSpeed(double speed) {
        maxSpeed = speed;
    }

    void driveRobotCentric(double strafe, double forward, double turn){
        // maxPower is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double maxPower = Math.max(Math.abs(forward)+Math.abs(strafe)+Math.abs(turn), 1)/maxSpeed;
        // Set power levels for each drive wheel
        leftFrontDrive.setPower((forward + strafe + turn)/maxPower);
        rightFrontDrive.setPower((forward - strafe - turn)/maxPower);
        leftBackDrive.setPower((forward - strafe + turn)/maxPower);
        rightBackDrive.setPower((forward + strafe - turn)/maxPower);
    }

    /**
     * @param targetPosition Pose2D
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean moveTo(Pose2D targetPosition) {
        strafe.setSetPoint(targetPosition.getX(DistanceUnit.METER));
        forward.setSetPoint(targetPosition.getY(DistanceUnit.METER));
        double desiredHeading = targetPosition.getHeading(AngleUnit.DEGREES);
        setMaxSpeed(0.5);

        driveFieldCentric(strafe.calculate(currentPose.getX(DistanceUnit.METER)),
                forward.calculate(currentPose.getY(DistanceUnit.METER)),
                turn.calculate(wrapAngle(desiredHeading-currentPose.getHeading(AngleUnit.DEGREES))));

        /*
         * Here we check if we are within tolerance of our target position or not.
         * If we have not reached our target yet, then we reset the driveTimer.
         * Only after we reach the target can the timer count higher than our holdSeconds variable.
         */
        if (!(strafe.atSetPoint() && forward.atSetPoint() && turn.atSetPoint()))
            driveTimer.reset();

        return (driveTimer.seconds() > 1);
    }

    public static double wrapAngle(double angle) {
        angle %= 360; // normalize angle between -360 and +360
        if (angle > 180)
            angle -= 360;
        else if (angle <= -180)
            angle += 360;
        return angle;
    }

    void stop() {
        driveRobotCentric(0,0,0);
    }

    Pose2D getCurrentPose() {
        return currentPose;
    }
}
