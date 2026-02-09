package org.firstinspires.ftc.teamcode.intro2robotics;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class MecanumDriveTrain {

    static final String LEFT_FRONT_WHEEL = "cm1";
    static final String RIGHT_FRONT_WHEEL = "cm0";
    static final String LEFT_REAR_WHEEL = "cm3";
    static final String RIGHT_REAR_WHEEL = "cm2";

    DcMotor LF;
    DcMotor LR;
    DcMotor RF;
    DcMotor RR;
    double leftTargetDistance;
    double rightTargetDistance;
    private final ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // eg: Bilda 5203 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.09;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public MecanumDriveTrain( HardwareMap hardwareMap, boolean useEncoders ) {

        LF = hardwareMap.dcMotor.get( LEFT_FRONT_WHEEL );
        LF.setPower(0);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setDirection(DcMotor.Direction.REVERSE);
        
        LR = hardwareMap.dcMotor.get( LEFT_REAR_WHEEL );
        LR.setPower(0);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setDirection(DcMotor.Direction.REVERSE);

        RF = hardwareMap.dcMotor.get( RIGHT_FRONT_WHEEL );
        RF.setPower(0);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        RR = hardwareMap.dcMotor.get( RIGHT_REAR_WHEEL );
        RR.setPower(0);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if( useEncoders ) {
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else {
            LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void drive( double x, double y, double strafe ) {

         // Denominator is the largest motor power (absolute value) or 1
       // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(strafe), 1);
        double frontLeftPower = (y + x + strafe) / denominator;
        double backLeftPower = (y - x + strafe) / denominator;
        double frontRightPower = (y - x - strafe) / denominator;
        double backRightPower = (y + x - strafe) / denominator;

        LF.setPower(frontLeftPower);
        LR.setPower(backLeftPower);
        RF.setPower(frontRightPower);
        RR.setPower(backRightPower);
    }

    public void drive(double speed, double leftInches, double rightInches, double timeout,
                      OpModeIsActive opMode, Telemetry telemetry ) {

        // Ensure that the opmode is still active
        if (opMode.isActive()) {

            // reset the timeout time
            runtime.reset();

            startMoving( speed, leftInches, rightInches );

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while( opMode.isActive() &&
                    runtime.seconds() < timeout &&
                    !hasReachedTarget() ) {

                displayPosition( telemetry, "Running" );
            }

            stopMoving();

            displayPosition( telemetry, "Stopped" );

            //  sleep(250);   // optional pause after each move
        }
    }

    public void startMoving( double speed, double leftInches, double rightInches ) {

        leftTargetDistance = leftInches;
        rightTargetDistance = rightInches;

        // Determine new target position, and pass to motor controller
        int newLeftTarget = LF.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        LF.setTargetPosition( newLeftTarget );
        LR.setTargetPosition( newLeftTarget );

        int newRightTarget = RF.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
        RF.setTargetPosition( newRightTarget );
        RR.setTargetPosition( newRightTarget );

        // Turn On RUN_TO_POSITION
        LF.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        RF.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        LR.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        RR.setMode( DcMotor.RunMode.RUN_TO_POSITION );

        LF.setPower( Math.abs(speed) );
        RF.setPower( Math.abs(speed) );
        LR.setPower( Math.abs(speed) );
        RR.setPower( Math.abs(speed) );
    }

    public void stopMoving() {
        // Stop all motion;
        LF.setPower( 0 );
        RF.setPower( 0 );
        LR.setPower( 0 );
        RR.setPower( 0 );

        // Turn off RUN_TO_POSITION
        LF.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        RF.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        LR.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        RR.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
    }

    public boolean hasReachedTarget() {

        return !(LF.isBusy() && RF.isBusy());
    }

    public void displayPosition( Telemetry telemetry, String status ) {

        telemetry.addData("Target Distance", "%4.2f\" : %4.2f\"", leftTargetDistance, rightTargetDistance );
        telemetry.addData("Delta Distance", "%4.2f\" : %4.2f\"",
                ( LF.getTargetPosition() - LF.getCurrentPosition() ) / COUNTS_PER_INCH,
                ( RF.getTargetPosition() - RF.getCurrentPosition() ) / COUNTS_PER_INCH );
        telemetry.addData("Target", "Running to %7d : %7d", LF.getTargetPosition(), RF.getTargetPosition() );
        telemetry.addData("Position", "Running at %7d : %7d", LF.getCurrentPosition(), RF.getCurrentPosition() );
        telemetry.addData("Status", status );
        telemetry.update();
    }
}
