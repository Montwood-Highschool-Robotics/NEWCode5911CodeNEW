package org.firstinspires.ftc.teamcode._TeleOPs;


//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class
Drive_Code extends LinearOpMode {


//    //In case Slides act up change the variables below:
//    public static int MAX_EXT = 2500;
//    public static int MID_EXT = 915;
//    public static int NO_EXT = -58;
//
//
//    private PIDController controller;
//    //All the slide/Arm variables have an A at the beginning
//    public static double Ap = 0.015, Ai = 0, Ad = 0, Af = 0.05;
//    //public static double Af = 0.05;
//    public static int Atarget = 0;
//
//    //All the Pivot variables have a P at the beginning
//    public static double Pp = .005, Pi = 0, Pd = 0, Pf = .005;
//    //public static double Pf = .005;
//    public static int Ptarget = 0;
//
//    public static int P_Down = -2900;
//    public static int P_Up = 150;
//

    private final double ticks_in_deg = 700 / 180;
    //Motors
//    private CRServo Claw;
    private Servo Belt;
    private DcMotor FRW;
    private DcMotor FLW;
    private DcMotor BRW;
    private DcMotor BLW;
    //private DcMotor lift;
    private DcMotor Intake;
    private DcMotor Outtake;
    private boolean APidOn;
    private boolean PPidOn;


    public void runOpMode() throws InterruptedException {

//
//        APidOn = true;
//        PPidOn = true;
//        controller = new PIDController(Ap,Ai,Ad);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //Motors
        DcMotor FRW = hardwareMap.dcMotor.get("FRW");
        DcMotor FLW = hardwareMap.dcMotor.get("FLW");
        DcMotor BRW = hardwareMap.dcMotor.get("BRW");
        DcMotor BLW = hardwareMap.dcMotor.get("BLW");
        //DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor Outtake = hardwareMap.dcMotor.get("Outtake");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake");


//        CRServo left = hardwareMap.crservo.get("left");
//        CRServo right = hardwareMap.crservo.get("right");
//        Servo Belt = hardwareMap.servo.get("Belt");


        waitForStart();
        while (opModeIsActive()) {//        CRServo Claw =hardwareMap.crservo.get("claw");

            FLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//            Atarget = NO_EXT;


            double y = -gamepad1.left_stick_y; // Remember, this is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = .8 * (gamepad1.right_stick_x);
//            telemetry.addData("Pid Value:", APidOn);

            //           double i = 0;
//
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), .65);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FLW.setPower(frontLeftPower * 1);
            BLW.setPower(backLeftPower * 1);
            FRW.setPower(frontRightPower * -1);
            BRW.setPower(backRightPower * -1);


            //PidOn is a boolean
            if (APidOn) {
//
//                controller.setPID(Ap, Ai, Ad);
//                int armPos = Arm1.getCurrentPosition();
//                double pid = controller.calculate(armPos, Atarget);
//                double ff = Math.cos(Math.toRadians(Atarget / ticks_in_deg)) * Af;
//                double power = pid + ff;
//                //Finishes the math by setting the power to the pid + ff
//                Arm1.setPower(power * .6);
//                Arm2.setPower(-power * .6);
//                telemetry.addData("pos", armPos);
//                telemetry.addData("target", Atarget);
//                telemetry.update();
//                if (gamepad2.dpad_down) {
//                    Atarget = NO_EXT;
//                } else if (gamepad2.dpad_right) {
//                    Atarget = MID_EXT;
//                } else if (gamepad2.dpad_up) {
//                    Atarget = MAX_EXT;
//                }
            }
            //          Slide Control Independent from boolean
//            if (gamepad2.share) {
//                APidOn = true;
//            }
//
//            if (gamepad2.left_stick_y > .1) {
//                APidOn = false;
//                telemetry.update();
//                Arm1.setPower(-1);
//                Arm2.setPower(1);
//            } else if (gamepad2.left_stick_y < -.1) {
//                APidOn = false;
//                telemetry.update();
//                Arm1.setPower(1);
//                Arm2.setPower(-1);
//
//            } else {
//                Arm1.setPower(0);
//                Arm2.setPower(0);
//            }


            //Pivot

            //PidOn is a boolean
//            if (PPidOn){
//
//                controller.setPID(Pp,Pi,Pd);
//                int PivPos = Pivot.getCurrentPosition();
//                double Ppid = controller.calculate(PivPos, Ptarget);
//                double Pff = Math.cos(Math.toRadians(Atarget/ ticks_in_deg)) * Pf;
//                double Ppower = Ppid + Pff;
//                //Finishes the math by setting the power to the pid + ff
//                Pivot.setPower(Ppower * .8);
//                telemetry.addData("pos", PivPos);
//                telemetry.addData("target", Ptarget);
//                telemetry.update();
//                if(gamepad2.left_trigger > .1){
//                    Ptarget = P_Down ;
//                } else if (gamepad2.right_trigger > .1) {
//                    Ptarget = P_Up;
//                }
//
//            }
//            if (gamepad2.dpad_left){
//                PPidOn = true;
//            }

            {

            }
        }
    }
}


