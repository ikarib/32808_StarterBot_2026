package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * This file includes an autonomous file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a mecanum system for robot mobility,
 * one high-speed motor driving two "launcher wheels," and two servos which feed that launcher.
 *
 * This robot starts up against the goal and launches all three projectiles before driving away
 * off the starting line.
 *
 * This program leverages a "state machine" - an Enum which captures the state of the robot
 * at any time. As it moves through the autonomous period and completes different functions,
 * it will move forward in the enum. This allows us to run the autonomous period inside of our
 * main robot "loop," continuously checking for conditions that allow us to move to the next step.
 */

@SuppressWarnings("unused")
@Autonomous(name="StarterBotAuto", group="StarterBot")
public class StarterBotAuto extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();
    Launcher launcher = new Launcher();

    /*
     * Here is our auto state machine enum. This captures each action we'd like to do in auto.
     */
    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_TO_SPIKE_MARK,
        COMPLETE
    }

    /*
     * Here we set the first step of our autonomous state machine by setting autoStep = AutoStep.LAUNCH.
     * Later in our code, we will progress through the state machine by moving to other enum members.
     * We do the same for our launcher state machine, setting it to IDLE before we use it later.
     */
    private AutonomousState autonomousState = AutonomousState.LAUNCH;

    /*
     * Here we create an enum not to create a state machine, but to capture which alliance we are on.
     */
    private enum Alliance {
        RED,
        BLUE
    }

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;
    Pose2D startPose, spikeMarkPose;

    int shotsToFire = 3; //The number of shots to fire in this auto.

    @Override
    public void runOpMode() {
        // This code runs ONCE when the driver hits INIT.
        drive.init();
        launcher.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses START)
        //waitForStart();

        // This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
        while (opModeInInit()) {
            // Here we allow the driver to select which alliance we are on using the gamepad.
            if (gamepad1.bWasPressed()) {
                alliance = Alliance.RED;
            } else if (gamepad1.xWasPressed()) {
                alliance = Alliance.BLUE;
            }

            telemetry.addData("Press X", "for BLUE");
            telemetry.addData("Press B", "for RED");
            telemetry.addData("Selected Alliance", alliance);
            telemetry.update();
        }

        // set starting position
        if (alliance==Alliance.RED) {
            startPose = new Pose2D(DistanceUnit.METER, 1.3, 1.3, AngleUnit.DEGREES, -45);
            spikeMarkPose = new Pose2D(DistanceUnit.METER, 1.2, 0.6, AngleUnit.DEGREES, 0);
        } else {
            startPose = new Pose2D(DistanceUnit.METER, -1.3, 1.3, AngleUnit.DEGREES, 45);
            spikeMarkPose = new Pose2D(DistanceUnit.METER, -1.2, 0.6, AngleUnit.DEGREES, 0);
        }
        drive.setPosition(startPose);

        // This code runs REPEATEDLY after the driver hits START but before they hit STOP.
        while (opModeIsActive()) {
            /*
             * TECH TIP: Switch Statements
             * switch statements are an excellent way to take advantage of an enum. They work very
             * similarly to a series of "if" statements, but allow for cleaner and more readable code.
             * We switch between each enum member and write the code that should run when our enum
             * reflects that state. We end each case with "break" to skip out of checking the rest
             * of the members of the enum for a match, since if we find the "break" line in one case,
             * we know our enum isn't reflecting a different state.
             */
            switch (autonomousState) {
                /*
                 * Since the first state of our auto is LAUNCH, this is the first "case" we encounter.
                 * This case is very simple. We call our .launch() function with "true" in the parameter.
                 * This "true" value informs our launch function that we'd like to start the process of
                 * firing a shot. We will call this function with a "false" in the next case. This
                 * "false" condition means that we are continuing to call the function every loop,
                 * allowing it to cycle through and continue the process of launching the first ball.
                 */
                case LAUNCH:
                    launcher.shoot();
                    autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                    break;

                case WAIT_FOR_LAUNCH:
                    /*
                     * A technique we leverage frequently in this code are functions which return a
                     * boolean. We are using this function in two ways. This function actually moves the
                     * motors and servos in a way that launches the ball, but it also "talks back" to
                     * our main loop by returning either "true" or "false". We've written it so that
                     * after the shot we requested has been fired, the function will return "true" for
                     * one cycle. Once the launch function returns "true", we proceed in the code, removing
                     * one from the shotsToFire variable. If shots remain, we move back to the LAUNCH
                     * state on our state machine. Otherwise, we reset the encoders on our drive motors
                     * and move onto the next state.
                     */
                    if (launcher.isIdle()) {
                        shotsToFire -= 1;
                        if (shotsToFire > 0) {
                            autonomousState = AutonomousState.LAUNCH;
                        } else {
                            drive.stop();
                            launcher.stop();
                            autonomousState = AutonomousState.DRIVING_TO_SPIKE_MARK;
                        }
                    }
                    break;

                case DRIVING_TO_SPIKE_MARK:
                    /*
                     * This is another function that returns a boolean. This time we return "true" if
                     * the robot has been within a tolerance of the target position for "holdSeconds."
                     * Once the function returns "true" we reset the encoders again and move on.
                     */
                    if (drive.moveTo(spikeMarkPose))
                        autonomousState = AutonomousState.COMPLETE;
                    break;
            }

            // update state machine
            launcher.updateState();

            /*
             * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
             * part of the code exists outside of our switch statement, it will run once every loop.
             * No matter what state our robot is in. This is the huge advantage of using state machines.
             * We can have code inside of our state machine that runs only when necessary, and code
             * after the last "case" that runs every loop. This means we can avoid a lot of
             * "copy-and-paste" that non-state machine autonomous routines fall into.
             */
            telemetry.addData("AutoState", autonomousState);
            telemetry.addData("LauncherState", launcher.getLaunchState());
            telemetry.addData("Launcher vel", "%.3f rpm", launcher.getVelocity());
            telemetry.addData("Current Pose", "%.3f %.3f %.1f", drive.getCurrentPose().getX(DistanceUnit.METER), drive.getCurrentPose().getY(DistanceUnit.METER), drive.getCurrentPose().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        drive.stop();
        launcher.stop();
    }
}



