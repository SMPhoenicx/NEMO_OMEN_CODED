package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OnePersonOpMode", group="Linear OpMode")
public class OnePersonOpMode extends LinearOpMode {

    // DRIVE MOTORS
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // MECHANISMS
    private DcMotor intake;
    private DcMotorEx fly1, fly2;
    private Servo vertTrans;  // Vertical actuator
    private CRServo trans;    // Continuous lift servo
    private Servo spin;       // Spin Dexter servo

    // BUTTON DEBOUNCE
    private boolean spinPressed = false;
    private boolean flyOn = false;
    private boolean tranOn = false;

    double flySpeed = 0;


    private double[] spinZero = {0.0+2.0/27.0,17.0/45.0+2.0/27.0,67.0/90.0+2.0/27.0};

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        double lastTime = 0;
        int spinIndex = 0;

        // HARDWARE MAPPING
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        fly1       = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2       = hardwareMap.get(DcMotorEx.class, "fly2");
        intake     = hardwareMap.get(DcMotor.class, "in");

        vertTrans  = hardwareMap.get(Servo.class,"trans1");
        trans      = hardwareMap.get(CRServo.class,"trans2");
        spin       = hardwareMap.get(Servo.class,"spin");

        //wheeee
        //region CONTROL VARS
        //GAMEPAD 1
        boolean lb1Pressed = false;
        boolean rb1Pressed = false;
        boolean b1Pressed = false;
        boolean a1Pressed = false;
        boolean x1Pressed = false;
        boolean y1Pressed = false;
        boolean down1Pressed = false;
        boolean up1Pressed = false;
        boolean right1Pressed = false;
        boolean left1Pressed = false;
        //GAMEPAD 2
        boolean lb2Pressed = false;
        boolean rb2Pressed = false;
        boolean b2Pressed = false;
        boolean a2Pressed = false;
        boolean x2Pressed = false;
        boolean y2Pressed = false;
        boolean down2Pressed = false;
        boolean up2Pressed = false;
        boolean right2Pressed = false;
        boolean left2Pressed = false;
        //endregion

        // DIRECTIONS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        fly1.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        //MODES
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vertTrans.setPosition(0);
        trans.setPower(0);
        spin.setPosition(spinZero[0]);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ---------- DRIVE (optional) ----------
            double axial   = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw     = gamepad1.right_stick_x;

            double flPow = axial + lateral + yaw;
            double frPow = axial - lateral - yaw;
            double blPow = axial - lateral + yaw;
            double brPow = axial + lateral - yaw;

            double max = Math.max(Math.abs(flPow), Math.abs(frPow));
            max = Math.max(max, Math.abs(blPow));
            max = Math.max(max, Math.abs(brPow));
            if (max > 1.0) {
                flPow /= max;
                frPow /= max;
                blPow /= max;
                brPow /= max;
            }

            frontLeft.setPower(flPow);
            frontRight.setPower(frPow);
            backLeft.setPower(blPow);
            backRight.setPower(brPow);

            // ---------- INTAKE ----------
            if(gamepad1.right_bumper && !rb1Pressed) {
                if(intake.getPower() <= 0) intake.setPower(1);
                else intake.setPower(0);
            }
            //OUTTAKE
            if(gamepad1.left_bumper && !lb1Pressed) {
                intake.setPower(-0.6);
            }

            // ---------- FLYWHEELS ----------
            if(gamepad1.a && !a1Pressed) {
                flyOn = !flyOn;
                flySpeed = flySpeed <= 0 ? 1000:flySpeed;
            }

            if(flyOn){
                fly1.setVelocity(flySpeed);
                fly2.setVelocity(flySpeed);
            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }

            if(gamepad1.right_trigger > 0 && (runtime.milliseconds() - lastTime > 250)) {
                flySpeed += 50;
                lastTime = runtime.milliseconds();
            }
            if(gamepad1.left_trigger > 0 && (runtime.milliseconds() - lastTime > 250)) {
                flySpeed -= (flySpeed > 0)? 50:0;
                lastTime = runtime.milliseconds();
            }

            // ---------- TRANS SERVO ----------
            if(gamepad1.dpad_up && !up1Pressed) {
                tranOn = !tranOn;
            }
            if(tranOn) {
                trans.setPower(-1);
                vertTrans.setPosition(0.27);
            } else {
                trans.setPower(0);
                vertTrans.setPosition(0);
            }

            // ---------- SPINDEXER ----------
            if(gamepad1.b && !spinPressed){//0.111111, 0.377777, 0.7444444444
                spinIndex+=1;
                if(spinIndex>2) spinIndex = 0;
                spin.setPosition(spinZero[spinIndex]);
                spinPressed = true;
            } else if(!gamepad1.b){
                spinPressed = false;
            }

            // ---------- TELEMETRY ----------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Fly state", flyOn);
            telemetry.addData("Fly power", flySpeed);
            telemetry.addData("Intake", intake.getPower());
            telemetry.addData("VertTrans", vertTrans.getPosition());
            telemetry.addData("Spin", spin.getPosition());
            telemetry.update();

            //region CONTROL RESETS
            b1Pressed = gamepad1.b;
            a1Pressed = gamepad1.a;
            x1Pressed = gamepad1.x;
            y1Pressed = gamepad1.y;
            down1Pressed = gamepad1.dpad_down;
            up1Pressed = gamepad1.dpad_up;
            left1Pressed = gamepad1.dpad_left;
            right1Pressed = gamepad1.dpad_right;
            lb1Pressed = gamepad1.left_bumper;
            rb1Pressed = gamepad1.right_bumper;

            b2Pressed = gamepad2.b;
            a2Pressed = gamepad2.a;
            x2Pressed = gamepad2.x;
            y2Pressed = gamepad2.y;
            down2Pressed = gamepad2.dpad_down;
            up2Pressed = gamepad2.dpad_up;
            left2Pressed = gamepad2.dpad_left;
            right2Pressed = gamepad2.dpad_right;
            lb2Pressed = gamepad2.left_bumper;
            rb2Pressed = gamepad2.right_bumper;
            //endregion

            sleep(20);
        }
    }
}
