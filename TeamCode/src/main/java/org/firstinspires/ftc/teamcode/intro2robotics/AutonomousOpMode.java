package org.firstinspires.ftc.teamcode.intro2robotics;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode includes all the skeletal structure that all linear OpModes contain.
 */

@Autonomous(name="AutonomousOpMode", group="Intro2Robotics")
public class AutonomousOpMode extends LinearOpMode implements OpModeIsActive {

    enum State { WaitingForStart, StartMoving, Turning, MovingToFinalPosition, Stopped }

    static final double DRIVE_SPEED = 0.50;
    static final double TURN_SPEED = 0.50;

    static final double INITIAL_DISTANCE = 24.0;
    static final double TURN_DISTANCE = 24.0;
    static final double FINAL_DISTANCE = 12.0;

    // Declare OpMode members.
    @Override
    public void runOpMode() {

        telemetry.addData("State", "Initializing robot hardware" );
        telemetry.update();

        MecanumDriveTrain driveTrain = new MecanumDriveTrain( hardwareMap, true );

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("State", "Running" );
        telemetry.update();

        AutonomousOpMode.State state = State.WaitingForStart;

        // run until the end of the match (driver presses STOP)
        while( opModeIsActive() ) {

            telemetry.addData( "State", state );

            switch( state ) {

                case WaitingForStart:

                    state = State.StartMoving;

                    break;

                case StartMoving:

                    // S1: Forward 24 Inches with 10 Sec timeout
                    driveTrain.drive( DRIVE_SPEED, INITIAL_DISTANCE, INITIAL_DISTANCE, 10.0, this, telemetry );

                    sleep(2000 );

                    state = State.Turning;

                    break;

                case Turning:

                    // S2: Turn Right 12 Inches with 10 Sec timeout                   if( sideColor
                    driveTrain.drive( TURN_SPEED, TURN_DISTANCE, -TURN_DISTANCE, 10.0, this, telemetry );

                    sleep(2000 );

                    state = State.MovingToFinalPosition;

                    break;

                case MovingToFinalPosition:

                    // S3: Reverse 24 Inches with 10 Sec timeout
                    driveTrain.drive( DRIVE_SPEED, -FINAL_DISTANCE, -FINAL_DISTANCE, 10.0, this, telemetry );

                    state = AutonomousOpMode.State.Stopped;

                    break;

                case Stopped:

                    break;
            }

            driveTrain.displayPosition( telemetry, "Moving" );
        }

        driveTrain.stopMoving();

        driveTrain.displayPosition( telemetry, "Stopped" );
    }

    @Override
    public boolean isActive() {
        return opModeIsActive();
    }
}
