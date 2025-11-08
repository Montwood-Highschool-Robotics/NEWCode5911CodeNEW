package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous
public class ChickenStars extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.937;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private ElapsedTime runtime = new ElapsedTime();


    DcMotor FLW;
    DcMotor FRW;
    DcMotor BLW;
    DcMotor BRW;
    DcMotor Intake_Carrier;
    DcMotor lift;
    DcMotor Intake;
    DcMotor Outtake;
    double Time;



    private double sleep(double time) {
      return 0;
    }

    @Override
    public void runOpMode() {
        FLW = hardwareMap.dcMotor.get("FLW");
        FRW = hardwareMap.dcMotor.get("FRW");
        BLW = hardwareMap.dcMotor.get("BLW");
        BRW = hardwareMap.dcMotor.get("BRW");
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor Outtake = hardwareMap.dcMotor.get("Outtake");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake");
        CRServo Intake_Carrier = hardwareMap.crservo.get("Intake_Carrier");


        FLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    waitForStart();
    // driving to the goal for first cycle
    driveForward(1,10,10);
    turn(10,10,-10);
    driveForward(1,10,10);
        // Getting our first set of artifacts
    driveForward(1,60, 10);
    turn(10,-23.6,23.6);
    driveForward(1,21,10);
    turn(10,21.8,-21.8);
    intake(1,500,10,10);
    shoot(1,500);
















































































































    }
    public void shoot(double speed ,double Time){
        Outtake.setPower(speed);
        sleep(Time);
        Outtake.setPower(0);
    }
    public void intake(double speed,double Time,double dist, double timeout){

        Intake_Carrier.setPower(speed);
        Intake.setPower(speed);
        driveForward(speed, dist, timeout);
        sleep(Time);
        Intake_Carrier.setPower(0);
        Intake.setPower(0);
    }

    public void driveForward(double speed, double dist, double timeout) {
        encoderDrive(speed, dist, dist, timeout);

    }

    public void turnLeft(double angle) {
        encoderDrive(.4, -angle, angle, 10);
    }



    public void turnRight(double speed, double angle, double timeout) {
        encoderDrive(speed, angle, -angle, timeout);
    }

    public void turn(double timeout, double angleL, double angleR) {
        encoderDrive(.4, angleL, angleR, timeout);
    }

    public void encoderDrive(double speed, double distL, double distR, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        if (opModeIsActive()) {


            newLeftTarget = FLW.getCurrentPosition() + (int) (distL * COUNTS_PER_INCH);
            newRightTarget = FRW.getCurrentPosition() + (int) (distR * COUNTS_PER_INCH);
            FLW.setTargetPosition(-newLeftTarget);
            BLW.setTargetPosition(-newLeftTarget);
            FRW.setTargetPosition(newRightTarget);
            BRW.setTargetPosition(newRightTarget);


            FLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            FLW.setPower(Math.abs(speed));
            BLW.setPower(Math.abs(speed));
            FRW.setPower(Math.abs(speed));
            BRW.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FLW.isBusy() && FRW.isBusy()) &&
                    (FLW.isBusy() && FRW.isBusy())) {


                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        FLW.getCurrentPosition(),
                        BLW.getCurrentPosition(),
                        FRW.getCurrentPosition(),
                        BRW.getCurrentPosition());
                telemetry.update();
            }


            FLW.setPower(0);
            BLW.setPower(0);
            FRW.setPower(0);
            BRW.setPower(0);


            FLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderStrafe(double speed, double dist, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;


        if (opModeIsActive()) {


            newLeftTarget = FLW.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
            newRightTarget = FRW.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
            FLW.setTargetPosition(-newLeftTarget);
            BLW.setTargetPosition(newLeftTarget);
            FRW.setTargetPosition(-newRightTarget);
            BRW.setTargetPosition(newRightTarget);


            FLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRW.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            FLW.setPower(Math.abs(speed));
            BLW.setPower(Math.abs(speed));
            FRW.setPower(Math.abs(speed));
            BRW.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (FLW.isBusy() && FRW.isBusy()) &&
                    (FLW.isBusy() && FRW.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        FLW.getCurrentPosition(),
                        BLW.getCurrentPosition(),
                        FRW.getCurrentPosition(),
                        BRW.getCurrentPosition());
                telemetry.update();
            }


            FLW.setPower(0);
            BLW.setPower(0);
            FRW.setPower(0);
            BRW.setPower(0);

            FLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

