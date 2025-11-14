package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OnePersonOpMode", group="Linear OpMode")
public class OnePersonOpMode extends LinearOpMode {

    //region HARDWARE DECLARATIONS
// Drive Motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // Mechanism Motors
    private DcMotorEx fly1 = null;
    private DcMotorEx fly2 = null;
    private DcMotor intake = null;
    
    // Servos
    private Servo vertTrans;  // Vertical actuator
    private CRServo spin = null;    // spino
    private Servo hood;
    
    //SENSOR
    private AnalogInput spinEncoder;
    
    //endregion
   
    //EXTRA VARS
    private double hoodAngle = 0;
    private double vertTranAngle = 0;

    //region CAROUSEL SYSTEM
    // Carousel PIDF Constants
    private double pidKp = 0.0057;
    private double pidKi = 0.00166;
    private double pidKd = 0.00002;
    private double pidKf = 0.0;

    // Carousel PID State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double integralLimit = 500.0;
    private double pidLastTimeMs = 0.0;

    // Carousel Control Parameters
    private final double positionToleranceDeg = 2.0;
    private final double outputDeadband = 0.03;

    // Carousel Positions (6 presets, every 60 degrees)
    // 57, 177, and 297 face the intake; others face the transfer
    private final double[] CAROUSEL_POSITIONS = {57.0, 117.0, 177.0, 237.0, 297.0, 357.0};
    private int carouselIndex = 0;
    private int prevCarouselIndex = 0;
    //endregion


    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //region OPERATIONAL VARIABLES

        // Mechanism States
        boolean tranOn = false;
        boolean intakeOn = false;
        double intakePower = 0;
        boolean flyOn = false;

        // Drive Variables
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        // Flywheel Control
        double flySpeed = 1160;
        
        double lastTime = 0;
        
        //endregion

        //region HARDWARE INITIALIZATION
        // Initialize Drive Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        fly1       = hardwareMap.get(DcMotorEx.class, "fly1");
        fly2       = hardwareMap.get(DcMotorEx.class, "fly2");
        intake     = hardwareMap.get(DcMotor.class, "in");
       
        spin = hardwareMap.get(CRServo.class, "spin");
        hood = hardwareMap.get(Servo.class, "hood");
        vertTrans = hardwareMap.get(Servo.class, "vtrans");

        spinEncoder = hardwareMap.get(AnalogInput.class, "espin");
        
      
        // DIRECTIONS
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        fly1.setDirection(DcMotor.Direction.REVERSE);
        fly2.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        
        spin.setDirection(CRServo.Direction.FORWARD);
        hood.setDirection(Servo.Direction.FORWARD);

        //MODES
        fly1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //endregion

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //region DRIVE
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            turn = -gamepad1.right_stick_x;
            
            moveRobot(drive, strafe, turn);
            //endregion

            //region INTAKE CONTROL
            if (gamepad1.rightBumperWasPressed()) {
                intakePower = 1;
                intakeOn = !intakeOn;
            }

            // Outtake
            if (gamepad1.leftBumperWasPressed()) {
                intakePower = -0.6;
            }

            if (intakeOn) {
                intake.setPower(intakePower);
            }
            else {
                intake.setPower(0);
            }
            //endregion

            //region TRANSFER CONTROL
            if (gamepad2.triangleWasPressed()) {
                vertTranAngle = vertTranAngle == 50? 0:50;
            }

            vertTrans.setPosition(vertTranAngle);

            //endregion

            //region CAROUSEL CONTROL
            double nowMs = runtime.milliseconds();
            double dtSec = (nowMs - pidLastTimeMs) / 1000.0;
            if (dtSec <= 0.0) dtSec = 1.0 / 50.0;

            // Carousel Navigation
            //Left and Right go to intake positions, aka the odd numbered indices on the pos array
            if (gamepad2.dpadLeftWasPressed()) {
                carouselIndex += carouselIndex % 2 != 0 ? 1 : 0;
                carouselIndex = (carouselIndex + 2) % CAROUSEL_POSITIONS.length;
            }
            if (gamepad2.dpadRightWasPressed()) {
                carouselIndex += carouselIndex % 2 != 0 ? 1 : 0;
                carouselIndex = (carouselIndex - 2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
            }
            //Down and Up go to transfer positions, aka the even numbered indices on the pos array
            if (gamepad2.dpadUpWasPressed()) {
                carouselIndex += carouselIndex % 2 == 0 ? 1 : 0;
                carouselIndex = (carouselIndex - 2 + CAROUSEL_POSITIONS.length) % CAROUSEL_POSITIONS.length;
            }
            if (gamepad2.dpadDownWasPressed()) {
                carouselIndex += carouselIndex % 2 == 0 ? 1 : 0;
                carouselIndex = (carouselIndex + 2) % CAROUSEL_POSITIONS.length;
            }

            // Update Carousel PID
            double targetAngle = CAROUSEL_POSITIONS[carouselIndex];
            updateCarouselPID(targetAngle, dtSec);
            //endregion

            //region FLYWHEEL
            if (gamepad2.right_trigger > 0.3 && !(gamepad2.left_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flySpeed += 20;
            }
            if (gamepad2.left_trigger > 0.3 && !(gamepad2.right_trigger > 0.3) && (runtime.milliseconds() - lastTime > 200)) {
                flySpeed -= 20;
            }

            // Flywheel Toggle
            if (gamepad2.crossWasPressed()) {
                flyOn = !flyOn;
            }

            // Voltage Compensation
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            double baseF = 12.0 / 2450.0;
            double compensatedF = baseF * (13.0 / voltage);
            //Set custom PID values
            fly1.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, compensatedF);
            fly2.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, compensatedF);

            // Set Flywheel Velocity
            if (flyOn) {
                fly1.setVelocity(flySpeed);
                fly2.setVelocity(flySpeed);
            } else {
                fly1.setVelocity(0);
                fly2.setVelocity(0);
            }
            //endregion


            // ---------- TELEMETRY ----------
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Flywheel Speed", "%.0f", flySpeed);
            telemetry.addData("Hood Angle", "%.1f°", hoodAngle);
            telemetry.addData("Carousel Target", "%.1f°", targetAngle);
            telemetry.update();
        }
    }

    //region HELPER METHODS
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void updateCarouselPID(double targetAngle, double dt) {
        double ccwOffset = -6.0;
        // read angles 0..360
        double angle = mapVoltageToAngle360(spinEncoder.getVoltage(), 0.01, 3.29);

        //raw error
        double rawError = -angleError(targetAngle, angle);

        //adds a constant term if it's in a certain direction.
        // we either do this or we change the pid values for each direction.
        // gonna try and see if simpler method works tho
        double compensatedTarget = targetAngle;
        if (rawError < 0) { // moving CCW
            compensatedTarget = (targetAngle + ccwOffset) % 360.0;
        }
        // compute shortest signed error [-180,180]
        double error = -angleError(compensatedTarget, angle);

        // integral with anti-windup
        integral += error * dt;
        integral = clamp(integral, -integralLimit, integralLimit);

        // derivative
        double d = (error - lastError) / Math.max(dt, 1e-6);

        // PIDF output (interpreted as servo power)
        double out = pidKp * error + pidKi * integral + pidKd * d;

        // small directional feedforward to overcome stiction when error significant
        if (Math.abs(error) > 1.0) out += pidKf * Math.signum(error);

        // clamp to [-1,1] and apply deadband
        out = Range.clip(out, -1.0, 1.0);
        if (Math.abs(out) < outputDeadband) out = 0.0;

        // if within tolerance, zero outputs and decay integrator to avoid bumping
        if (Math.abs(error) <= positionToleranceDeg) {
            out = 0.0;
            integral *= 0.2;
        }

        // apply powers (flip one if your servo is mirrored - change sign if needed)
        spin.setPower(out);

        // store errors for next derivative calculation
        lastError = error;

        // telemetry for PID (keeps concise, add more if you want)
        telemetry.addData("Carousel Target", "%.1f°", targetAngle);

    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double mapVoltageToAngle360(double v, double vMin, double vMax) {
        double angle = 360.0 * (v - vMin) / (vMax - vMin);
        angle = (angle + 360) % 360;
        return angle;
    }

    // Compute shortest signed difference between two angles
    private double angleError(double target, double current) {
        double error = target - current;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }
    //endregion
}
