package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    DcMotor fl; DcMotor fr; DcMotor bl; DcMotor br;
    DcMotor leftlift; DcMotor rightlift; DcMotor backlift; DcMotor intake;
    Servo leftintakearm; Servo rightintakearm;
    Servo mainrelease; Servo auxrelease;
    Servo leftboxarm; Servo rightboxarm;
    Servo launcher;

    public RobotHardware(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br,
                         DcMotor leftlift, DcMotor rightlift, DcMotor backlift, DcMotor intake,
                         Servo leftintakearm, Servo rightintakearm, Servo mainrelease, Servo auxrelease,
                         Servo leftboxarm, Servo rightboxarm, Servo launcher) {
        this.fl = fl; this.fr = fr; this.bl = bl; this.br = br;
        this.leftlift = leftlift; this.rightlift = rightlift; this.backlift = backlift; this.intake = intake;
        this.leftintakearm = leftintakearm; this.rightintakearm = rightintakearm;
        this.mainrelease = mainrelease; this.auxrelease = auxrelease;
        this.leftboxarm = leftboxarm; this.rightboxarm = rightboxarm;
        this.launcher = launcher;
    }

    public void innitHardwareMap() {
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftlift.setDirection(DcMotor.Direction.FORWARD);
        rightlift.setDirection(DcMotor.Direction.REVERSE);
        backlift.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftlift.setTargetPosition(0);
        rightlift.setTargetPosition(0);
        backlift.setTargetPosition(0);
        leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftlift.setPower(1);
        rightlift.setPower(1);
        backlift.setPower(1);

        leftintakearm.setDirection(Servo.Direction.REVERSE);
        leftboxarm.setDirection(Servo.Direction.REVERSE);
        mainrelease.setDirection(Servo.Direction.REVERSE);
    }

    public void setlift(int targetValue) {
        leftlift.setTargetPosition(targetValue);
        rightlift.setTargetPosition(targetValue);
        backlift.setTargetPosition(targetValue);
    }

    public void resetlift() {
        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftlift.setTargetPosition(0);
        rightlift.setTargetPosition(0);
        backlift.setTargetPosition(0);
        leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftlift.setPower(1);
        rightlift.setPower(1);
        backlift.setPower(1);
    }
    public void intakeup() {leftintakearm.setPosition(0.73); rightintakearm.setPosition(0.73);}
    public void intakedown(){leftintakearm.setPosition(0.41); rightintakearm.setPosition(0.41);}
    public void boxup() {leftboxarm.setPosition(0.77); rightboxarm.setPosition(0.77);}
    public void boxdown() {leftboxarm.setPosition(0.18); rightboxarm.setPosition(0.18);}
    public void releaseone() {mainrelease.setPosition(0.55); auxrelease.setPosition(0.8);}
    public void releasetwo() {mainrelease.setPosition(0.6); auxrelease.setPosition(0.8);}
    public void boxtransferready() {mainrelease.setPosition(0.4); auxrelease.setPosition(0);}
    public void boxintakeready() {auxrelease.setPosition(0); mainrelease.setPosition(0.6);}
    public void intakefirstpixel() {leftintakearm.setPosition(0.56); rightintakearm.setPosition(0.56);}
    public void intakesecondpixel() {leftintakearm.setPosition(0.53); rightintakearm.setPosition(0.53);}
    public void intakethirdpixel() {leftintakearm.setPosition(0.5); rightintakearm.setPosition(0.5);}
    public void intakefourthpixel() {leftintakearm.setPosition(0.47); rightintakearm.setPosition(0.47);}
    public void intakefifthpixel() {leftintakearm.setPosition(0.41); rightintakearm.setPosition(0.41);}










}
