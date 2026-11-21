package org.firstinspires.ftc.teamcode.pedro;


import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Autonomous(name = "Red Odom Auto", group = "Red")
public class RedEncoderAuto extends OpMode {


    private Drivebase drivebase;
    private Shooter shooter;
    private Intake intake;


    // Field Coordinates


    // Box 1: Start & Preload Shot (shooters facing Red Hive / +Y = 90 deg)
    private static final double BOX1_X = 56.0, BOX1_Y = 6.0, BOX1_H = 90.0;


    // Box 2: Clearance step off wall before turning 90 deg CW
    private static final double BOX2_X = 44.0, BOX2_Y = 24.0, BOX2_H = 90.0;


    // Box 3: Turned to 0 deg (shooters facing +X / up), hug wall into Garden corner
    private static final double BOX3_PRE_GARDEN_X = 30.0, BOX3_Y = 0.0, BOX3_H = 0.0;
    private static final double BOX3_GARDEN_X = 9.5;
    private static final double BOX3_EXIT_X = 20.0, BOX3_EXIT_Y = 20.0;


    // Box 4: Flower (faces 270 deg -> stay here, shoot first, then intake)
    private static final double BOX4_X = 36.0, BOX4_Y = 74.0, BOX4_H = 270.0;


    // Box 5: Opposite Red Hive shot (270 deg)
    private static final double BOX5_X = 60.0, BOX5_Y = 134.5, BOX5_H = 270.0;


    // Box 6: Final Park (270 deg)
    private static final double BOX6_X = 9.5, BOX6_Y = 115.3, BOX6_H = 270.0;


    // Drive Speeds
    public static double DRIVE_POWER = 0.95;
    public static double GARDEN_CREEP_POWER = 0.38;
    public static double GARDEN_BACKUP_POWER = 0.85;
    public static double TURN_POWER = 0.65;


    // Shooter & Intake Constants
    public static double VELOCITY_P_TARGET = 1030;
    public static double VELOCITY_N_TARGET = 1100;
    public static double SHOOTER_BANG_BANG_PERCENT = 0.98;
    public static double SHOOTER_FEED_PERCENT = 0.93;


    // Timers & Timeouts (ms)
    public static double SHOOT_TIME_MS = 5000;
    public static double GATE_CLOSE_DELAY_MS = 300;
    public static double INTAKE_TIME_MS = 1800;
    public static double MOVE_TIMEOUT_MS = 2000;
    public static double GARDEN_IN_TIMEOUT_MS = 2800;
    public static double GARDEN_OUT_TIMEOUT_MS = 2200;
    public static double TURN_TIMEOUT_MS = 2200;


    private ElapsedTime actionTimer;
    private ElapsedTime moveTimer;


    private enum AutoState {
        // 1. Stay completely still at Box 1: spin up -> shoot -> close gate
        BOX1_SHOOT_SPINUP,
        BOX1_SHOOT_FEED,
        BOX1_CLOSE_GATE,


        // 2. Move to Box 2 for turning clearance (intake running)
        MOVE_TO_BOX2,


        // 3. Turn to 0 deg, align to wall, drive into Garden (Box 3) with intake running
        TURN_90_CW_AT_BOX2,
        STRAFE_TO_WALL_BOX3,
        DRIVE_INTO_GARDEN_BOX3,
        GARDEN_SETTLE_INTAKE,


        // 4. Clear corner, drive to Flower (Box 4): stay -> shoot first -> intake
        CLEAR_GARDEN_CORNER,
        DRIVE_TO_BOX4,
        BOX4_SHOOT_SPINUP,
        BOX4_SHOOT_FEED,
        BOX4_CLOSE_GATE,
        BOX4_FLOWER_INTAKE,


        // 5. Drive to Box 5 (opposite side), stay still: spin up -> shoot -> close gate
        DRIVE_TO_BOX5,
        BOX5_SHOOT_SPINUP,
        BOX5_SHOOT_FEED,
        BOX5_CLOSE_GATE,


        // 6. Drive to Box 6 (final park)
        DRIVE_TO_BOX6_PARK,


        DONE
    }


    private AutoState autoState = AutoState.BOX1_SHOOT_SPINUP;


    private void saveEndPose() {
        OpModeStorage.autonomousEndPose = PoseFactory.degrees().of(
                drivebase.getFieldX(),
                drivebase.getFieldY(),
                drivebase.getFieldHeadingDeg()
        );
    }


    private boolean shooterAtTarget() {
        return shooter.getVelocityP() >= VELOCITY_P_TARGET * SHOOTER_BANG_BANG_PERCENT
                && shooter.getVelocityN() >= VELOCITY_N_TARGET * SHOOTER_BANG_BANG_PERCENT;
    }


    private boolean feedReady() {
        return shooter.getVelocityP() >= VELOCITY_P_TARGET * SHOOTER_FEED_PERCENT
                && shooter.getVelocityN() >= VELOCITY_N_TARGET * SHOOTER_FEED_PERCENT;
    }


    private void runShooter() {
        if (!shooterAtTarget()) {
            shooter.setPower(1.0);
        } else {
            shooter.setVelocityP(VELOCITY_P_TARGET);
            shooter.setVelocityN(VELOCITY_N_TARGET);
        }
    }


    private void feedWhenReady() {
        if (feedReady()) {
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
        drivebase.setFieldStartPose(BOX1_X, BOX1_Y, BOX1_H);


        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        actionTimer = new ElapsedTime();
        moveTimer = new ElapsedTime();


        shooter.setPower(0);
        intake.setPower(0);
        intake.setPowerGateClose();
        intake.closeFlush();


        autoState = AutoState.BOX1_SHOOT_SPINUP;
        saveEndPose();


        telemetry.addLine("RED ODOM AUTO READY (DIAGRAM 1 -> 6)");
        telemetry.update();
    }


    @Override
    public void start() {
        moveTimer.reset();
        actionTimer.reset();
        autoState = AutoState.BOX1_SHOOT_SPINUP;
    }


    @Override
    public void loop() {
        drivebase.updatePinpoint();
        saveEndPose();       // Continuously update pose for TeleOp field-centric transfer
        intake.closeFlush(); // Always keep intake flush closed


        // Keep shooter spun up through Box 5 shot, then turn off while parking at Box 6
        if (autoState != AutoState.DRIVE_TO_BOX6_PARK && autoState != AutoState.DONE) {
            runShooter();
        } else {
            shooter.setPower(0);
        }


        updateAuto();


        telemetry.addData("State", autoState);
        telemetry.addData("X", "%.1f", drivebase.getFieldX());
        telemetry.addData("Y", "%.1f", drivebase.getFieldY());
        telemetry.addData("Heading", "%.1f", drivebase.getFieldHeadingDeg());
        telemetry.addData("MoveTimer (ms)", (int) moveTimer.milliseconds());
        telemetry.addData("Feed Ready", feedReady());
        telemetry.update();
    }


    private void updateAuto() {
        switch (autoState) {


            // --- 1. Stay Still & Preload Shoot at Box 1 ---
            case BOX1_SHOOT_SPINUP:
                drivebase.stopRobot();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (shooterAtTarget()) {
                    actionTimer.reset();
                    autoState = AutoState.BOX1_SHOOT_FEED;
                }
                break;


            case BOX1_SHOOT_FEED:
                drivebase.stopRobot();
                feedWhenReady();


                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    actionTimer.reset();
                    autoState = AutoState.BOX1_CLOSE_GATE;
                }
                break;


            case BOX1_CLOSE_GATE:
                drivebase.stopRobot();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (actionTimer.milliseconds() >= GATE_CLOSE_DELAY_MS) {
                    moveTimer.reset();
                    autoState = AutoState.MOVE_TO_BOX2;
                }
                break;


            // --- 2. Move to Box 2 for Turning Clearance (Intake Running) ---
            case MOVE_TO_BOX2:
                intake.setPowerGateClose();
                intake.setPower(-1.0);
                if (drivebase.driveToFieldPose(BOX2_X, BOX2_Y, BOX2_H, DRIVE_POWER)
                        || moveTimer.milliseconds() >= MOVE_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    moveTimer.reset();
                    autoState = AutoState.TURN_90_CW_AT_BOX2;
                }
                break;


            // --- 3. Turn to 0 deg, Align to Wall, Drive Into Garden (Box 3) ---
            case TURN_90_CW_AT_BOX2:
                intake.setPowerGateClose();
                intake.setPower(-1.0);
                if (drivebase.turnToFieldHeading(BOX3_H, TURN_POWER)
                        || moveTimer.milliseconds() >= TURN_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    moveTimer.reset();
                    autoState = AutoState.STRAFE_TO_WALL_BOX3;
                }
                break;


            case STRAFE_TO_WALL_BOX3:
                intake.setPowerGateClose();
                intake.setPower(-1.0);
                if (drivebase.driveToFieldPose(BOX3_PRE_GARDEN_X, BOX3_Y, BOX3_H, DRIVE_POWER)
                        || moveTimer.milliseconds() >= MOVE_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    moveTimer.reset();
                    autoState = AutoState.DRIVE_INTO_GARDEN_BOX3;
                }
                break;


            case DRIVE_INTO_GARDEN_BOX3:
                intake.setPowerGateClose();
                intake.setPower(-1.0);
                if (drivebase.driveToFieldPose(BOX3_GARDEN_X, BOX3_Y, BOX3_H, GARDEN_CREEP_POWER)
                        || moveTimer.milliseconds() >= GARDEN_IN_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    actionTimer.reset();
                    autoState = AutoState.GARDEN_SETTLE_INTAKE;
                }
                break;


            case GARDEN_SETTLE_INTAKE:
                drivebase.stopRobot();
                intake.setPowerGateClose();
                intake.setPower(-1.0);
                if (actionTimer.milliseconds() >= INTAKE_TIME_MS) {
                    intake.setPower(0);
                    moveTimer.reset();
                    autoState = AutoState.CLEAR_GARDEN_CORNER;
                }
                break;


            // --- 4. Clear Corner & Drive to Flower (Box 4): Stay, Shoot First, Then Intake ---
            case CLEAR_GARDEN_CORNER:
                if (drivebase.driveToFieldPose(BOX3_EXIT_X, BOX3_EXIT_Y, BOX3_H, GARDEN_BACKUP_POWER)
                        || moveTimer.milliseconds() >= GARDEN_OUT_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    moveTimer.reset();
                    autoState = AutoState.DRIVE_TO_BOX4;
                }
                break;


            case DRIVE_TO_BOX4:
                if (drivebase.driveToFieldPose(BOX4_X, BOX4_Y, BOX4_H, DRIVE_POWER)
                        || moveTimer.milliseconds() >= MOVE_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    autoState = AutoState.BOX4_SHOOT_SPINUP;
                }
                break;


            case BOX4_SHOOT_SPINUP:
                drivebase.stopRobot();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (shooterAtTarget()) {
                    actionTimer.reset();
                    autoState = AutoState.BOX4_SHOOT_FEED;
                }
                break;


            case BOX4_SHOOT_FEED:
                drivebase.stopRobot();
                feedWhenReady();


                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    actionTimer.reset();
                    autoState = AutoState.BOX4_CLOSE_GATE;
                }
                break;


            case BOX4_CLOSE_GATE:
                drivebase.stopRobot();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (actionTimer.milliseconds() >= GATE_CLOSE_DELAY_MS) {
                    actionTimer.reset();
                    autoState = AutoState.BOX4_FLOWER_INTAKE;
                }
                break;


            case BOX4_FLOWER_INTAKE:
                drivebase.stopRobot();
                intake.setPowerGateClose();
                intake.setPower(-1.0);


                if (actionTimer.milliseconds() >= INTAKE_TIME_MS) {
                    intake.setPower(0);
                    moveTimer.reset();
                    autoState = AutoState.DRIVE_TO_BOX5;
                }
                break;


            // --- 5. Drive to Opposite Side (Box 5), Stay Still & Shoot ---
            case DRIVE_TO_BOX5:
                if (drivebase.driveToFieldPose(BOX5_X, BOX5_Y, BOX5_H, DRIVE_POWER)
                        || moveTimer.milliseconds() >= MOVE_TIMEOUT_MS) {
                    drivebase.stopRobot();
                    autoState = AutoState.BOX5_SHOOT_SPINUP;
                }
                break;


            case BOX5_SHOOT_SPINUP:
                drivebase.stopRobot();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (shooterAtTarget()) {
                    actionTimer.reset();
                    autoState = AutoState.BOX5_SHOOT_FEED;
                }
                break;


            case BOX5_SHOOT_FEED:
                drivebase.stopRobot();
                feedWhenReady();


                if (actionTimer.milliseconds() >= SHOOT_TIME_MS) {
                    intake.setPower(0);
                    intake.setPowerGateClose();
                    actionTimer.reset();
                    autoState = AutoState.BOX5_CLOSE_GATE;
                }
                break;


            case BOX5_CLOSE_GATE:
                drivebase.stopRobot();
                intake.setPower(0);
                intake.setPowerGateClose();


                if (actionTimer.milliseconds() >= GATE_CLOSE_DELAY_MS) {
                    moveTimer.reset();
                    autoState = AutoState.DRIVE_TO_BOX6_PARK;
                }
                break;


            // --- 6. Drive to Box 6 Park ---
            case DRIVE_TO_BOX6_PARK:
                if (drivebase.driveToFieldPose(BOX6_X, BOX6_Y, BOX6_H, DRIVE_POWER)
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
                break;
        }
    }


    @Override
    public void stop() {
        saveEndPose();
        drivebase.stopRobot();
        shooter.setPower(0);
        intake.setPower(0);
        intake.setPowerGateClose();
        intake.closeFlush();
    }
}