package org.firstinspires.ftc.teamcode._TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class
CANUSAY extends LinearOpMode {




    private final double ticks_in_deg = 700 / 180;
    //Motors
//    private CRServo Claw;
    private Servo Belt;
    private DcMotor FRW;
    private DcMotor FLW;
    private DcMotor BRW;
    private DcMotor BLW;


    public void runOpMode() throws InterruptedException {


        DcMotor FRW = hardwareMap.dcMotor.get("FRW");
        DcMotor FLW = hardwareMap.dcMotor.get("FLW");
        DcMotor BRW = hardwareMap.dcMotor.get("BRW");
        DcMotor BLW = hardwareMap.dcMotor.get("BLW");


        waitForStart();
        while (opModeIsActive()) {//        CRServo Claw =hardwareMap.crservo.get("claw");

            FLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BLW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BRW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




            double y = -gamepad1.left_stick_y; // Remember, this is reversed
            double x = gamepad1.left_stick_x; // Counteract imperfect strafing
            double rx = .8 * (gamepad1.right_stick_x);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), .65);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FLW.setPower(frontLeftPower * 1);
            BLW.setPower(backLeftPower * 1);
            FRW.setPower(frontRightPower * 1);
            BRW.setPower(backRightPower * 1);



        }
waitForStart();

    }

}

