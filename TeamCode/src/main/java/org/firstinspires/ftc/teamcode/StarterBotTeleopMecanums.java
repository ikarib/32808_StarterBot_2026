package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")

public class StarterBotTeleopMecanums extends LinearOpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    Launcher launcher = new Launcher();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() {
        mecanumDrive.init(hardwareMap);
        launcher.init(hardwareMap);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double strafe = gamepad1.right_stick_x;
            double forward = -gamepad1.right_stick_y;
            double turn = gamepad1.left_stick_x;
            //double desired_heading = Math.toDegrees(Math.atan2(-gamepad1.left_stick_x,gamepad1.left_stick_y));
            //double turn = heading_control.calculate(wrapAngle(desired_heading - mecanumDrive.getHeading()));
            mecanumDrive.driveFieldCentric(strafe, forward, turn);

            /*
             * Here we give the user control of the speed of the launcher motor without automatically
             * queuing a shot.
             */
            if (gamepad1.yWasPressed()) // start flywheel
                launcher.startFlywheel();
            if (gamepad1.bWasPressed()) // stop flywheel
                launcher.stopFlywheel();

            if (gamepad1.rightBumperWasPressed())
                launcher.shoot();

            // update state machine
            launcher.updateState();

            // Show the state and motor powers
            telemetry.addData("State", launcher.getLaunchState());
            telemetry.addData("Launcher vel", "%.3f rpm", launcher.getVelocity()/28*60);

            telemetry.update();
        }

        mecanumDrive.stop();
        launcher.stop();
    }
}