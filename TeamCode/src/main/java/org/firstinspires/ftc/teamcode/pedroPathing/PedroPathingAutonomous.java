package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.DemoDriveTrain;
import org.firstinspires.ftc.teamcode.hardware.MinigameGamepad;
import org.firstinspires.ftc.teamcode.hardware.OpModeIsActive;

/**
 * Device  Port   Variable name               Control Filename

 * Motor   Port 0 rightFrontMotor                  cm0
 * Motor   Port 1 rightRearMotor                   cm1
 * Motor   Port 1 leftRearMotor                    cm2
 * Motor   Port 3 leftFrontMotor                   cm3
 *
 * Servo   Port 0 block                            s0
 * Servo   Port 1 basket                           s1
 * Servo   Port 2                                  s2
 * Servo   Port 3                                  s3
 * Servo   Port 4                                  s4
 * Servo   Port 5                                  s5
 *
 * I2C     Port 0 colorSensor                      i0
 * I2C     Port 1 IRSensor                         i1
 * I2C     Port 2                                  i2
 * I2C     Port 3                                  i3
 */

@Autonomous(name = "PedroPathingAutonomous", group="Demo")
public class PedroPathingAutonomous extends LinearOpMode implements OpModeIsActive {

    enum State { WaitingForStart, StartMoving, LookingForColor, LookingForIRSensor, StartDroppingCube, FinishedDroppingCube, MovingToEnd, Turning, MovingToFinalPosition, Stopped }

    static final double DRIVE_FUDGE_FACTOR = 1.1;

    static final double ARC_RADIUS = 41;

    static final double DECISION_DISTANCE = ARC_RADIUS + 2;

    public PedroPathingAutonomous() {
        telemetry.addLine( "Creating PedroPathingAutonomous" );
        telemetry.update();

    }
    @Override
    public void runOpMode() {

        telemetry.addLine( "Initializing hardware" );
        telemetry.update();

        DemoDriveTrain driveTrain = new DemoDriveTrain( hardwareMap, false );
        MinigameGamepad myGamepad = new MinigameGamepad( gamepad1 );

        waitForStart();

        State state = State.WaitingForStart;
        long sleepTime = 0;

        while( opModeIsActive() ) {

            // Temporarily run the robot with the gamepad to verify that the drive train works

            double leftStickY = -myGamepad.getLeftStickY(); // Remember, this is reversed!
            double leftStickX = myGamepad.getLeftStickX() * 1.1; // Counteract imperfect strafing
            double rightStickX = myGamepad.getRightStickX();

            telemetry.addData("leftStick X", leftStickX );
            telemetry.addData("leftStick Y", leftStickY );
            telemetry.addData("rightStick X", rightStickX );
            telemetry.update();

            driveTrain.drive( leftStickX, leftStickY, rightStickX );

            telemetry.addData( "State", state );

            switch( state ) {

                case WaitingForStart:

                   state = State.StartMoving;

                    break;

                case StartMoving:

                    state = State.LookingForColor;

                    break;

                case LookingForColor:

                    state = State.LookingForIRSensor;

                    break;

                case LookingForIRSensor:

                    state = State.StartDroppingCube;

                    break;

                case StartDroppingCube:

                    sleepTime = 2000;

                    state = State.FinishedDroppingCube;

                    break;

                case FinishedDroppingCube:
                    
                    state = State.MovingToEnd;

                    break;

                case MovingToEnd:

                    state = State.MovingToFinalPosition;
                    
                    break;

                case Turning:

                    state = State.MovingToFinalPosition;

                    break;

                case MovingToFinalPosition:

                    state = State.Stopped;

                    break;

                case Stopped:

                    break;
            }

            driveTrain.displayPosition( telemetry,"Moving" );

            if( sleepTime > 0 ) {
                sleep( sleepTime );
                sleepTime = 0;
            }
        }

        driveTrain.stopMoving();

        driveTrain.displayPosition( telemetry, "Stopped" );
    }

    @Override
    public boolean isActive() {
        return opModeIsActive();
    }
}
