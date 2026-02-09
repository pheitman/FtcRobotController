package org.firstinspires.ftc.teamcode.intro2robotics;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode includes all the skeletal structure that all linear OpModes contain.
 */

@TeleOp(name="TeleOpMode", group="Intro2Robotics")
public class TeleOpMode extends LinearOpMode {

    // Declare OpMode members.
    @Override
    public void runOpMode() {

        telemetry.addData("State", "Initializing robot hardware" );
        telemetry.update();
/*
        MecanumDriveTrain driveTrain = new MecanumDriveTrain( hardwareMap, false );
        SmartGamepad myGamepad1 = new SmartGamepad( gamepad1 );
*/
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("State", "Running");
        telemetry.update();

        while( opModeIsActive()  ) {
/*
           double leftStickY = -myGamepad1.getLeftStickY(); // Remember, this is reversed!
           double leftStickX = myGamepad1.getLeftStickX() * 1.1; // Counteract imperfect strafing
           double rightStickX = myGamepad1.getRightStickX();

            telemetry.addData("leftStick X", leftStickX);
            telemetry.addData("leftStick Y", leftStickY);
            telemetry.addData("rightStick X", rightStickX);
            telemetry.update();

          driveTrain.drive(leftStickX, leftStickY, rightStickX);
 */
        }
    }
}
