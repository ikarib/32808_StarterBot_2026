package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    public static double FEED_TIME = 0.20; //The feeder servos run this long in seconds when a shot is requested.
    public static double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    public static double LAUNCHER_TARGET_VELOCITY = 1125;
    public static double LAUNCHER_MIN_VELOCITY = 1075;

    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    /*
     * The number of seconds that we wait between each of our 3 shots from the launcher. This
     * can be much shorter, but the longer break is reasonable since it maximizes the likelihood
     * that each shot will score.
     */
    final double TIME_BETWEEN_SHOTS = 2;

    /*
     * Here we create three timers which we use in different parts of our code. Each of these is an
     * "object," so even though they are all an instance of ElapsedTime(), they count independently
     * from each other.
     */
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();


    /*
     * TECH TIP: State Machines
     * We use "state machines" to control our launcher motor and feeder servos in auto and teleop.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through code,
	 * while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This state machine is called the "LaunchState." It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at IDLE. When the user requests a launch, we enter PREPARE where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states, but this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH
    }

    /*
     * Here we create the instance of LaunchState that we use in code. This creates a unique object
     * which can store the current condition of the shooter. In other applications, you may have
     * multiple copies of the same enum which have different names. Here we just have one.
     */
    private LaunchState launchState;


    /*
     * This code runs ONCE when the driver hits INIT.
     */
    public void init(HardwareMap hardwareMap) {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        /*
         * Setting zeroPowerBehavior to FLOAT enables a "coast mode." This causes the motor to
         * slow down without braking so that we can spin it up faster when need it again.
		 */
        launcher.setZeroPowerBehavior(FLOAT);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER run mode.
         * If you notice that you have no control over the velocity of the motor, and it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
         * This control method reads the current speed as reported by the motor's encoder and applies a varying
         * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
         * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
         * applied to the motor directly.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
         * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
         * Here we set the aforementioned PID coefficients. You shouldn't have to do this for any
         * other motors on this robot.
         */
        launcher.setVelocityPIDFCoefficients(300, 0, 0, 10);


        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(CRServo.Direction.REVERSE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        launchState = LaunchState.IDLE;

        /*
         * We also set the servo power to 0 here to make sure that the servo controller is booted
         * up and ready to go.
         */
        stop();
    }

    // the user would like to fire a new shot
    void shoot() {
        if (launchState == LaunchState.IDLE) {
            launchState = LaunchState.PREPARE;
            shotTimer.reset();
        }
    }

    /**
     * Launches one ball, when a shot is requested spins up the motor and once it is above a minimum
     * velocity, runs the feeder servos for the right amount of time to feed the next ball.
     */
    public void updateState() {
        switch (launchState) {
            case IDLE:
                break;
            case PREPARE:
                // shot has already been requested and we need to continue to move through
                // the state machine and launch the ball.
                startFlywheel();
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                    startFeeder();
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    stopFeeder();
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        // one cycle after a ball has been successfully launched
                        launchState = LaunchState.IDLE;
                    }
                }
                break;
        }
    }

    public void stop() {
        stopFlywheel();
        stopFeeder();
    }
    public void startFlywheel() {
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
    }

    public void stopFlywheel() {
        launcher.setVelocity(0);
        launchState = LaunchState.IDLE;
    }

    public boolean isIdle() {
        return launchState == LaunchState.IDLE;
    }
    private void startFeeder() {
        leftFeeder.setPower(FULL_SPEED);
        rightFeeder.setPower(FULL_SPEED);
    }
    private void stopFeeder() {
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    public String getLaunchState() {
        return launchState.toString();
    }

    public double getVelocity() {
        return launcher.getVelocity()/28*60; // RPM
    }
}
