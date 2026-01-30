//package org.firstinspires.ftc.teamcode;
//import androidx.annotation.NonNull;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.TrajectoryBuilder;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import com.acmerobotics.roadrunner.Vector2d;
//@Config
//@Autonomous(name = "Red Close Auto")
//public class RedCloseAuto extends LinearOpMode {
//    // Define your robot's starting position
//    private static final Pose2d zeroPose = new Pose2d(0, 0, 0);
//    private static final Vector2d px = new Vector2d(20, 0);
//    private static final Vector2d py = new Vector2d(0, 20);
//    private static final Pose2d STARTING_POSE = new Pose2d(12, -60, Math.toRadians(90));
//
//    // Sample positions (adjust these based on your field measurements)
//    private static final Pose2d SPECIMEN_DROP = new Pose2d(9, -32, Math.toRadians(90));
//    private static final Vector2d SAMPLE_1 = new Vector2d(24, -12);
//    private static final Vector2d SAMPLE_2 = new Vector2d(0, -12);
//    private static final Vector2d SAMPLE_3 = new Vector2d(-24, -12);
//    private static final Vector2d BUCKET_POS = new Vector2d(0, -48);
//    private static final Vector2d PARK_POS = new Vector2d(55, -55);
//    private DcMotor rext = null;//ext are extension
//    private DcMotor lext = null;
//    private DcMotor rpivot = null; //pivot arm up and down
//    private DcMotor lpivot = null;
//    private CRServo servo1 = null; //intake
//    private CRServo servo2 = null;
//    private Servo sWrist = null; //wrist joint
//    private Servo Sample = null;
//
//    private static final int liftMax = 1000;
//    private static final int liftMin = -2900;
//    private static final int pivotMax = 1100;
//    private static final int pivotMin = 800;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        rext = hardwareMap.get(DcMotor.class, "rext");
//        lext = hardwareMap.get(DcMotor.class, "lext");
//        rpivot = hardwareMap.get(DcMotor.class, "rpivot");
//        lpivot = hardwareMap.get(DcMotor.class, "lpivot");
//        servo1 = hardwareMap.get(CRServo.class, "s1");
//        servo2 = hardwareMap.get(CRServo.class, "s2");
//        sWrist = hardwareMap.get(Servo.class, "sWrist");
//        sWrist.setPosition(0);
//        Sample = hardwareMap.get(Servo.class, "Sample");
//        Sample.setPosition(0);
//
//        Lift lift = new Lift(hardwareMap);
//        Pivot pivot = new Pivot(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
//        Wrist wrist = new Wrist(hardwareMap);
//        SampleArm sampleArm = new SampleArm(hardwareMap);
//        MecanumDrive drive = new MecanumDrive(hardwareMap, STARTING_POSE);
//
//        TrajectoryActionBuilder pushSamples = drive.actionBuilder(SPECIMEN_DROP)
//                .waitSeconds(4.5)
//                .strafeTo(new Vector2d(9,-32))
//                .strafeTo(new Vector2d(35,-35))
//                .splineToConstantHeading(new Vector2d(35,-34),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(35,-15),Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(45,-15),Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(45,-45),Math.toRadians(90))
//                .splineTo(new Vector2d(43,-13),Math.toRadians(90))
//                .splineTo(new Vector2d(56,-13),Math.toRadians(-90))
//                .splineTo(new Vector2d(56, -55), Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(45,-55), Math.toRadians(-90));
////
////                .stopAndAdd(wrist.wristUp())asdlf
////                .stopAndAdd(pivot.pivotDown(600))
////                .stopAndAdd(lift.liftUp(-300))
////                .stopAndAdd(intake.intakeUp())
////                //lift to prepare for outtake
////                .stopAndAdd(pivot.pivotUp(800))
////                .stopAndAdd(lift.liftUp(-2000));
//
//
//        TrajectoryActionBuilder placeSpecimen = drive.actionBuilder(STARTING_POSE)
//                .strafeTo(SPECIMEN_DROP.position)
//                //wait ant put speciamskdfjas;dlfkja onto the cage
//                .waitSeconds(1)
//                .stopAndAdd(pivot.pivotDown(800))
//                .stopAndAdd(lift.liftDown(1000));
////                .stopAndAdd(intake.intakeDown());
//
//
//        TrajectoryActionBuilder placeSpecimen1 = drive.actionBuilder(new Pose2d(45, -55, Math.toRadians(90)))
//                //wait ant put speciamskdfjas;dlfkja onto the cage
//                .waitSeconds(27.5)
//                .strafeTo(new Vector2d(-6,-30));
////                .stopAndAdd(pivot.pivotDown(800))
////                .stopAndAdd(lift.liftDown(1000))
////                .stopAndAdd(intake.intakeDown());
//        //.stopAndAdd(pushSamples.build());
//
//        TrajectoryActionBuilder speciman2 = drive.actionBuilder(new Pose2d(-6, -30, Math.toRadians(90)))
//                .waitSeconds(31.5)
//                .splineToConstantHeading(new Vector2d(45, -55), Math.toRadians(90));
//        //.stopAndAdd(pivot.pivotUp(1200))
//        //.stopAndAdd(lift.liftDown(-500));
//
//        TrajectoryActionBuilder placeSpecimen2 = drive.actionBuilder(new Pose2d(45, -55, Math.toRadians(90)))
//                //wait ant put speciamskdfjas;dlfkja onto the cage
//                .waitSeconds(35.5)
//                .strafeTo(new Vector2d(-4,-30));
////                .stopAndAdd(pivot.pivotDown(800))
////                .stopAndAdd(lift.liftDown(1000))
////                .stopAndAdd(intake.intakeDown());
//        //.stopAndAdd(pushSamples.build());
//
//        TrajectoryActionBuilder speciman3 = drive.actionBuilder(new Pose2d(-4, -30, Math.toRadians(90)))
//                .waitSeconds(39.5)
//                .splineToConstantHeading(new Vector2d(45, -55), Math.toRadians(90));
//        //.stopAndAdd(pivot.pivotUp(1200))
//        //.stopAndAdd(lift.liftDown(-500));
//
//        TrajectoryActionBuilder placeSpecimen3 = drive.actionBuilder(new Pose2d(45, -55, Math.toRadians(90)))
//                //wait ant put speciamskdfjas;dlfkja onto the cage
//                .waitSeconds(43.5)
//                .strafeTo(new Vector2d(0,-30));
////                .stopAndAdd(pivot.pivotDown(800))
////                .stopAndAdd(lift.liftDown(1000))
////                .stopAndAdd(intake.intakeDown());
//        //.stopAndAdd(pushSamples.build());
//
//
//
////        TrajectoryActionBuilder park = drive.actionBuilder(SPECIMEN_DROP)
////
////                .waitSeconds(5)
////                .strafeTo(new Vector2d(9, -55))
////
////                .strafeTo(PARK_POS);
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        // TODO: Set lift to specimen height
//        //Actions.runBlocking(autoSequence);
//        /**Actions.runBlocking(
//         new SequentialAction(
//         autoSequence.build()
//         )
//         );**/
//        Actions.runBlocking(//lift arm and move to specimen at same time
//                new ParallelAction(
//                        //pivot.pivotUp(800),
//                        //lift.liftUp(-2200),
//                        placeSpecimen.build(),
//                        pushSamples.build(),
//                        placeSpecimen1.build(),
//                        speciman2.build(),
//                        placeSpecimen2.build(),
//                        speciman3.build(),
//                        placeSpecimen3.build()
//                )
//        );
//        //Actions.runBlocking(new SequentialAction(//push samples afterward
//
////                pushSamples.build()
//        //));
//    }
//
//    public class Lift {
//
//        public Lift(HardwareMap hardwareMap) {
//
//            rext.setDirection(DcMotor.Direction.FORWARD);
//            lext.setDirection(DcMotor.Direction.REVERSE);
//
//            lext.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
//            lext.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
//        }
//
//        public class LiftUp implements Action {
//            private boolean initialized = false;
//            private int liftMin;
//
//            public LiftUp(int liftmin) {
//                liftMin = liftmin;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    rext.setPower(-1);
//                    lext.setPower(-1);
//                    initialized = true;
//                }
//                double pos = lext.getCurrentPosition();
//                telemetry.addData("Status", "liftPos: " + pos);
//                telemetry.update();
//                packet.put("liftPos", pos);
//                if (pos > liftMin) {
//                    return true;
//                } else {
//                    rext.setPower(0);
//                    lext.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public class LiftDown implements Action {
//            private boolean initialized = false;
//            private int liftMax;
//
//            public LiftDown(int liftmax) {
//                liftMax = liftmax;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    rext.setPower(1);
//                    lext.setPower(1);
//                    initialized = true;
//                }
//                double pos = lext.getCurrentPosition();
//                telemetry.addData("Status", "liftPos: " + pos);
//                telemetry.update();
//                packet.put("liftPos", pos);
//                if (pos < liftMax) {
//                    return true;
//                } else {
//                    rext.setPower(0);
//                    lext.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public Action liftUp(int liftMin) {
//            return new Lift.LiftUp(liftMin);
//        }
//
//        public Action liftDown(int liftMax) {
//            return new Lift.LiftDown(liftMax);
//        }
//    }
//
//    public class Pivot {
//
//        public Pivot(HardwareMap hardwareMap) {
//
//            rpivot.setDirection(DcMotor.Direction.FORWARD);
//            lpivot.setDirection(DcMotor.Direction.FORWARD);
//
//            lpivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
//            lpivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
//        }
//
//        public class PivotUp implements Action {
//            private boolean initialized = false;
//            private int pivotMax;
//
//            public PivotUp(int pivotmax) {
//                pivotMax = pivotmax;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    rpivot.setPower(1);
//                    lpivot.setPower(1);
//                    initialized = true;
//                }
//                double pos = lpivot.getCurrentPosition();
//                telemetry.addData("Status", "pivotPos: " + pos);
//                telemetry.update();
//                packet.put("pivotPos", pos);
//                if (pos < pivotMax) {
//                    return true;
//                } else {
//                    rpivot.setPower(0);
//                    lpivot.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public class PivotDown implements Action {
//            private boolean initialized = false;
//            int pivotMin;
//
//            public PivotDown(int pivotmin) {
//                pivotMin = pivotmin;
//            }
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    rpivot.setPower(-1);
//                    lpivot.setPower(-1);
//                    initialized = true;
//                }
//                double pos = lpivot.getCurrentPosition();
//                telemetry.addData("Status", "pivotPos: " + pos);
//                telemetry.update();
//                packet.put("pivotPos", pos);
//                if (pos > pivotMin) {
//                    return true;
//                } else {
//                    rpivot.setPower(0);
//                    lpivot.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public Action pivotUp(int pivotMax) {
//            return new Pivot.PivotUp(pivotMax);
//        }
//
//        public Action pivotDown(int pivotMin) {
//            return new Pivot.PivotDown(pivotMin);
//        }
//    }
//
//    public class Intake {//dont know how to implement intake on and off, probably like.wait but doesn twokr
//        //unless in the runOpMode soooo idk
//
//        public Intake(HardwareMap hardwareMap) {
//
//            servo1.setDirection(DcMotorSimple.Direction.FORWARD);
//            servo2.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//
//        public class IntakeUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    servo1.setPower(0.9);
//                    servo2.setPower(0.9);
//                    initialized = true;
//                    resetRuntime();
//                }
//                if (getRuntime() < 1) {
//                    return true;
//                } else {
//                    servo1.setPower(0);
//                    servo2.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public class IntakeDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    servo1.setPower(-1);
//                    servo2.setPower(-1);
//                    initialized = true;
//                    resetRuntime();
//                }
//                if (getRuntime() < 1) {
//                    return true;
//                } else {
//                    servo1.setPower(0);
//                    servo2.setPower(0);
//                    return false;
//                }
//            }
//        }
//
//        public Action intakeUp() {
//            return new Intake.IntakeUp();
//        }
//
//        public Action intakeDown() {
//            return new Intake.IntakeDown();
//        }
//    }
//
//    public class Wrist {//dont know how to implement intake on and off, probably like.wait but doesn twokr
//        //unless in the runOpMode soooo idk
//
//        public Wrist(HardwareMap hardwareMap) {
//
//            sWrist.setDirection(Servo.Direction.FORWARD);
//        }
//
//        public class WristUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    sWrist.setPosition(0.65);
//                    initialized = true;
//                    resetRuntime();
//                }
//                if (getRuntime() < 1) {
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public class WristDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    sWrist.setPosition(0.4);
//                    initialized = true;
//                    resetRuntime();
//                }
//                if (getRuntime() < 1) {
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public Action wristUp() {
//            return new RedCloseAuto.Wrist.WristUp();
//        }
//
//        public Action wristDown() {
//            return new RedCloseAuto.Wrist.WristDown();
//        }
//    }
//    public class SampleArm {//dont know how to implement intake on and off, probably like.wait but doesn twokr
//        //unless in the runOpMode soooo idk
//
//        public SampleArm(HardwareMap hardwareMap) {
//
//            Sample.setDirection(Servo.Direction.FORWARD);
//        }
//
//        public class SampleArmUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    Sample.setPosition(0);
//                    initialized = true;
//                    resetRuntime();
//                }
//                if (getRuntime() < 1) {
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public class SampleDown implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    Sample.setPosition(0.85);
//                    initialized = true;
//                    resetRuntime();
//                }
//                if (getRuntime() < 1) {
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public Action SampleArmUp() {
//            return new SampleArm.SampleArmUp();
//        }
//
//        public Action SampleArmDown() {
//
//            return new SampleArm.SampleDown();
//        }
//    }
//}