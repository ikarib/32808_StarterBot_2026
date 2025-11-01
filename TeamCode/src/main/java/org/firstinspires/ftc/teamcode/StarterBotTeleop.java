package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a mecanum system for robot mobility,
 * one high-speed motor driving two "launcher wheels," and two servos which feed that launcher.
 *
 */

@SuppressWarnings("unused")
@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
public class StarterBotTeleop extends LinearOpMode {
    MecanumDrive drive = new MecanumDrive();
    Launcher launcher = new Launcher();


    @Override
    public void runOpMode() {
        // This code runs ONCE when the driver hits INIT.
        drive.init();
        launcher.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
		
        // Wait for the game to start (driver presses START)
        waitForStart();
        drive.setPosition(new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0));

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive.humanInputs(gamepad1);

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
            telemetry.addData("Launcher vel", "%.3f rpm", launcher.getVelocity());
            telemetry.addData("Current Pose", "%.3f %.3f %.1f", drive.getCurrentPose().getX(DistanceUnit.METER), drive.getCurrentPose().getY(DistanceUnit.METER), drive.getCurrentPose().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }

        drive.stop();
        launcher.stop();
    }
}