package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Solo", group="Linear Opmode")

public class Solo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor arm = null;

    Servo wrist;
    Servo leftclaw;
    Servo rightclaw;

    CRServo lefthang;

    CRServo righthang;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        telemetry.update();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        arm = hardwareMap.get(DcMotor.class, "arm");

        wrist = hardwareMap.get(Servo.class, "wrist");
        leftclaw = hardwareMap.get(Servo.class, "leftclaw");
        rightclaw = hardwareMap.get(Servo.class, "rightclaw");

        lefthang = hardwareMap.get(CRServo.class, "lefthang");
        righthang = hardwareMap.get(CRServo.class, "righthang");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double modifier = 1;

            fl.setPower(modifier*(y + x + turn));
            fr.setPower(modifier*(y - x - turn));
            bl.setPower(modifier*(y - x + turn));
            br.setPower(modifier*(y + x - turn));

            if(gamepad1.x) { //reset arm
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(0);
            }
            if (gamepad1.a) { //deposit position
                arm.setTargetPosition(1400);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);
            }
            if (gamepad1.b) { //intake position
                arm.setTargetPosition(5);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.4);
            }
            if (gamepad1.y) {
                arm.setTargetPosition(1200);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);
            }
            if (gamepad1.left_trigger > 0){ //left claw close
                leftclaw.setPosition(0.4);
            }
            if (gamepad1.left_bumper){ //left claw open
                leftclaw.setPosition(0.55);
            }
            if (gamepad1.right_trigger > 0){ //right claw close
                rightclaw.setPosition(0.49);
            }
            if (gamepad1.right_bumper){ //right claw open
                rightclaw.setPosition(0.34);
            }
            if (gamepad1.dpad_down){
                wrist.setPosition(0.45);
            }
            if(gamepad1.dpad_up){
                wrist.setPosition(0.9);
            }
            if (gamepad1.dpad_left){
                lefthang.setPower(-1);
                righthang.setPower(1);
            }
            else if (gamepad1.dpad_right){
                lefthang.setPower(1);
                righthang.setPower(-1);
            }
            else {
                lefthang.setPower(0);
                righthang.setPower(0);
            }



        }
    }
}
