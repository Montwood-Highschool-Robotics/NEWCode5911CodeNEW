package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.openftc.easyopencv.OpenCvWebcam;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */

@Autonomous
public  class Test_Auto extends LinearOpMode { // Ramtonomous - property of benji

    //Incase Slides act up change the variables below:
    public static int MAX_EXT = 2500;
    public static int MID_EXT = 915;
    public static int NO_EXT = -58;


//    private PIDController controller;
    //All the slide/Arm variables have an A at the beginning
    public static double Ap = 0.015, Ai = 0, Ad = 0;
    public static double Af = 0.05;
    public static int Atarget = 0;

    //All the Pivot variables have a P at the beginning
    public static double Pp = .005, Pi = 0, Pd = 0;
    public static double Pf = .005;
    public static int Ptarget = 0;

    public static int P_Down = -2900;
    public static int P_Score = 150;


    private final double ticks_in_deg = 700 / 180;
    //Motors
    private Servo Wrist;
    private DcMotor FRW;
    private DcMotor FLW;
    private DcMotor BRW;
    private DcMotor BLW;
    private DcMotor Arm1;
    private DcMotor Arm2;
    private DcMotor Pivot;
    private CRServo left;
    private CRServo right;
    private boolean APidOn;
    private boolean PPidOn;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // PIXELS AS IN THE HIT MOVIE WITH ADAM SANDLER
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 383.6;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.937;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // UNITS ARE METERS
    static final double SPEED = .5;
    OpenCvWebcam webcam;
    private double speed;

    @Override
    public void runOpMode() throws InterruptedException {


//        APidOn = true;
//        PPidOn = true;
//        controller = new PIDController(Ap, Ai, Ad);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        BRW = hardwareMap.dcMotor.get("BRW");
        BLW = hardwareMap.dcMotor.get("BLW");
        FRW = hardwareMap.dcMotor.get("FRW");
        FLW = hardwareMap.dcMotor.get("FLW");


        FLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FRW.setDirection(DcMotorSimple.Direction.REVERSE);
        BRW.setDirection(DcMotorSimple.Direction.REVERSE);



        FLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        waitForStart(); // hype up the robot!!

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */


        // don't mind the Logan-proofing
        // Left side
        // Basket Auto
        // Score pre loaded specimen

        // Moving Toward basket (beginning of drop-in replacement)

        /*Gabe's Tooltips
        -Distance is like in Physics where it is negative direction so it goes backwards...    I feel the sudden urge to jump off a bridge*/
        driveForward(1,98.5,10);
        encoderStrafe(-1,51.25,10);








        // FAILURE AREA New Code (end of drop-in replacement)
        /* fitter, happier, more productive. comfortable, not drinking too much. regular exercise at the gym, 3 days a week. getting on better with your associate employee contemporaries. a patient better driver, a safer car. baby smiling in backseat. no bad dreams, no paranoia. careful to all animals, no longer washing spiders down the plughole. keep in contact with old friends. enjoy a drink now and then. will frequently check credit at moral bank. hole in wall. favors for favors. fond but not in love. charity standing orders.*/


    }

    public void driveForward(double speed, double dist, double timeout) {
        encoderDrive(speed, dist, dist, timeout);

    }

    public void turnLeft(double angle) {
        encoderDrive(.4, -angle, angle, 10);
    }

    public void intake(double speed, long duration) {
        left.setPower(-speed);
        right.setPower(speed);
        sleep(duration);
        left.setPower(0);
        right.setPower(0);
    }                                     // Solution - just delete sleep(timeout*1000) and long timeout

//    public void pivotMove(int target) {
//        //controller.setPID(Pp, Pi, Pd);
//        int PivPos = Pivot.getCurrentPosition();
//        //double Ppid = controller.calculate(PivPos, Ptarget);
//        double Pff = Math.cos(Math.toRadians(Atarget / ticks_in_deg)) * Pf;
//       // double Ppower = Ppid + Pff;
//        //Finishes the math by setting the power to the pid + ff
//        Pivot.setPower(Ppower * 1);
//        telemetry.addData("pos", PivPos);
//        telemetry.addData("target", Ptarget);
//        telemetry.update();
//        Ptarget = target;
//    }

//    public void slideMove(int Starget) {
//        controller.setPID(Ap, Ai, Ad);
//        int armPos = Arm1.getCurrentPosition();
//        double pid = controller.calculate(armPos, Atarget);
//        double ff = Math.cos(Math.toRadians(Atarget / ticks_in_deg)) * Af;
//        double power = pid + ff;
//        //Finishes the math by setting the power to the pid + ff
//        Arm1.setPower(power * .6);
//        Arm2.setPower(-power * .6);
//        telemetry.addData("pos", armPos);
//        telemetry.addData("target", Atarget);
//        telemetry.update();
//        Atarget = Starget;
//    }


    public void extendSlide(double speed, long duration) {
        Arm1.setPower(-speed);
        Arm2.setPower(speed);
        sleep(duration);
        Arm1.setPower(0);
        Arm2.setPower(0);
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
            FLW.setTargetPosition(newLeftTarget);
            BLW.setTargetPosition(newLeftTarget);
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
