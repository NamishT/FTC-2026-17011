package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Autonomous(name = "Red Odom Auto", group = "Red")
public class RedEncoderAuto extends OpMode {


    private Drivebase drivebase;
    private Shooter shooter;
    private Intake intake;


    // Pedro Pathing Start & Park Coordinates
    private static final double START_X = 56.0;
    private static final double START_Y = 8.0;
    private static final double START_H = 90.0;


    // 90 deg clockwise from 90 deg = 0 deg
    private static final double GARDEN_H = 0.0;
    private static final double GARDEN_DISTANCE_IN = 49.33; // Distance from (56, 8) to (6.6715, 8.4861)


    // Final Park Pose (point9 from Pedro file)
    private static final double PARK_X = 5.4943;
    private static final double PARK_Y = 109.291;
    private static final double PARK_H = 90.0;


    // Drive Speeds
    public static double DRIVE_POWER = 0.50;
    public static double TURN_POWER = 0.32;


    // Shooter & Intake Constants
    public static double VELOCITY_P_TARGET = 1130;
    public static double VELOCITY_N_TARGET = 1100;
    public static double SHOOTER_READY_PERCENT = 0.98;


    public static double SHOOT_TIME_MS = 2000;
    public static double INTAKE_TIME_MS = 1500;
    public static double MOVE_TIMEOUT_MS = 6500;


    private ElapsedTime actionTimer;
    private ElapsedTime moveTimer;


    private double gardenTargetX = START_X;
    private double gardenTargetY = START_Y;


    private enum AutoState {
        START_SHOOT_SPINUP,
        START_SHOOT_FEED,


        TURN_90_CW,
        DRIVE_TO_GARDEN,
        GARDEN_INTAKE,


        RETURN_TO_START,
        GO_TO_PARK,


        DONE
    }


    private AutoState autoState = AutoState.START_SHOOT_SPINUP;


    private boolean shooterReady() {
        return shooter.getVelocityP() >= VELOCITY_P_TARGET * SHOOTER_READY_PERCENT
                && shooter.getVelocityN() >= VELOCITY_N_TARGET * SHOOTER_READY_PERCENT;
    }


    private void runShooter() {
        if (!shooterReady()) {
            shooter.setPower(1.0);
        } else {
            shooter.setVelocityP(VELOCITY_P_TARGET);
            shooter.setVelocityN(VELOCITY_N_TARGET);
        }
    }


    private void feedWhenReady() {
        if (shooterReady()) {
            intake.setPowerGateOpen();
            intake.setPower(-1.0);
        } else {
            intake.setPowerGateClose();
            intake.setPower(0);
        }
    }


    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap);
        drivebase.setFieldStartPose(START_X, START_Y, START_H);


        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        actionTimer = new ElapsedTime();
        moveTimer = new ElapsedTime();


        shooter.setPower(0);
        intake.setPower(0);
        intake.setPowerGateClose();
        intake.closeFlush();


        autoState = AutoState.START_SHOOT_SPINUP;


        telemetry.addLine("RED ODOM AUTO READY");
        telemetry.addData("Start Pose", START_X + ", " + START_Y + " @ " + START_H);
        telemetry.update();
    }


    @Override
    public void start() {
        drivebase.setFieldStartPose(START_X, START_Y, START_H);
        moveTimer.reset();
        autoState = AutoState.START_SHOOT_SPINUP;
    }


    @Override
    public void loop() {
        drivebase.updatePinpoint();


        // Only spin shooter during the initial preload shot
        if (autoState == AutoState.START_SHOOT_SPINUP || autoState == AutoState.START_SHOOT_FEED) {
            runShooter();
        } else {
            shooter.setPower(0);
        }


        updateAuto();


        telemetry.addData("Auto State", autoState);
        telemetry.addData("X", drivebase.getFieldX());
        telemetry.addData("Y", drivebase.getFieldY());
        telemetry.addData("Heading", drivebase.getFieldHeadingDeg());
        telemetry.addData("Shooter Ready", shooterReady());
        telemetry.update();
    }


    private void updateAuto() {
        switch (autoState) {


            // 1. Shoot preloads at start position (56, 8, 90)
            case START_SHOOT_SPINUP:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (shooterReady()) {
                    actionTimer.reset();
                    autoState = AutoState.START_SHOOT_FEED;
                }
                break;


            case START_SHOOT_FEED:
                intake.closeFlush();
                feedWhenReady();


                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    moveTimer.reset();
                    autoState = AutoState.TURN_90_CW;
                }
                break;


            // 2. Turn 90 deg clockwise (90 -> 0 deg)
            case TURN_90_CW:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (drivebase.turnToFieldHeading(GARDEN_H, TURN_POWER) || moveTimer.milliseconds() >= 3000) {
                    drivebase.stopRobot();
                    // Compute straight-ahead target 49.33 in along current 0 deg heading
                    double rad = Math.toRadians(GARDEN_H);
                    gardenTargetX = drivebase.getFieldX() + GARDEN_DISTANCE_IN * Math.cos(rad);
                    gardenTargetY = drivebase.getFieldY() + GARDEN_DISTANCE_IN * Math.sin(rad);
                    moveTimer.reset();
                    autoState = AutoState.DRIVE_TO_GARDEN;
                }
                break;


            // 3. Drive straight to Garden pickup
            case DRIVE_TO_GARDEN:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (drivebase.driveToFieldPose(gardenTargetX, gardenTargetY, GARDEN_H, DRIVE_POWER)
                        || moveTimer.milliseconds() >= MOVE_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    intake.setPowerGateClose();
                    intake.setPower(-1.0);
                    actionTimer.reset();
                    autoState = AutoState.GARDEN_INTAKE;
                }
                break;


            // 4. Pick up from Garden
            case GARDEN_INTAKE:
                intake.closeFlush();
                intake.setPowerGateClose();
                intake.setPower(-1.0);


                if (actionTimer.milliseconds() >= INTAKE_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    moveTimer.reset();
                    autoState = AutoState.RETURN_TO_START;
                }
                break;


            // 5. Return to start position (56, 8, 90) - no shooting
            case RETURN_TO_START:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (drivebase.driveToFieldPose(START_X, START_Y, START_H, DRIVE_POWER)
                        || moveTimer.milliseconds() >= MOVE_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    moveTimer.reset();
                    autoState = AutoState.GO_TO_PARK;
                }
                break;


            // 6. Go directly from start position to final park pose (5.4943, 109.291, 90)
            case GO_TO_PARK:
                intake.closeFlush();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (drivebase.driveToFieldPose(PARK_X, PARK_Y, PARK_H, DRIVE_POWER)
                        || moveTimer.milliseconds() >= MOVE_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    autoState = AutoState.DONE;
                }
                break;


            case DONE:
                drivebase.stopRobot();
                shooter.setPower(0);
                intake.setPower(0);
                intake.setPowerGateClose();
                intake.closeFlush();
                break;
        }
    }


    @Override
    public void stop() {
        drivebase.stopRobot();
        shooter.setPower(0);
        intake.setPower(0);
        intake.setPowerGateClose();
        intake.closeFlush();
    }
}