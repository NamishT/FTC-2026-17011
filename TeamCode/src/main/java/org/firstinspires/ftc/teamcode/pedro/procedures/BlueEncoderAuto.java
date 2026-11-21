package org.firstinspires.ftc.teamcode.pedro.procedures;


import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedro.OpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Autonomous(name = "Blue Odom Auto", group = "Blue")
public class BlueEncoderAuto extends OpMode {


    private Drivebase drivebase;
    private Shooter shooter;
    private Intake intake;


    private final PoseFactory poseFactory = PoseFactory.degrees();


    // 180-deg Field Rotation of Red Coordinates (144 - Red_X, 144 - Red_Y, Red_H + 180)
    // Matches Blue Diagram: Top-left start -> Top-left Garden -> Mid-field (4) -> Top-right Park (6)


    // Box 1: Start & Preload Shot (shooters facing Blue Hive / -Y = 270 deg)
    private static final double BOX1_X = 88.0, BOX1_Y = 135.0, BOX1_H = 270.0;


    // Box 2: Clearance step off wall before turning 90 deg CW
    private static final double BOX2_X = 100.0, BOX2_Y = 120.0, BOX2_H = 270.0;


    // Box 3: Turned 90 deg CW to 180 deg (shooters facing -X / down), hug wall into Blue Garden corner
    private static final double BOX3_PRE_GARDEN_X = 114.0, BOX3_Y = 140.5, BOX3_H = 180.0;
    private static final double BOX3_GARDEN_X = 134.5;
    private static final double BOX3_EXIT_X = 124.0, BOX3_EXIT_Y = 124.0; // Clear corner before heading to Box 4


    // Box 4: Mid-field waypoint (faces 90 deg / left)
    private static final double BOX4_X = 108.0, BOX4_Y = 70.0, BOX4_H = 90.0;


    // Box 6: Final Park in top-right zone (goes straight from Box 4 to Box 6, skipping Box 5)
    private static final double BOX6_X = 134.5, BOX6_Y = 34.7, BOX6_H = 90.0;


    // Drive Speeds
    public static double DRIVE_POWER = 0.80;
    public static double GARDEN_CREEP_POWER = 0.32;
    public static double GARDEN_BACKUP_POWER = 0.75;
    public static double TURN_POWER = 0.50;


    // Shooter & Intake Constants
    public static double VELOCITY_P_TARGET = 1130;
    public static double VELOCITY_N_TARGET = 1100;
    public static double SHOOTER_BANG_BANG_PERCENT = 0.98;
    public static double SHOOTER_FEED_PERCENT = 0.93;


    // Timers & Timeouts (ms)
    public static double SHOOT_TIME_MS = 5000;
    public static double GATE_CLOSE_DELAY_MS = 300;
    public static double INTAKE_TIME_MS = 1800;
    public static double MOVE_TIMEOUT_MS = 4000;
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


        // 3. Turn 90 deg CW to 180 deg, align to wall, drive into Garden (Box 3)
        TURN_90_CW_AT_BOX2,
        STRAFE_TO_WALL_BOX3,
        DRIVE_INTO_GARDEN_BOX3,
        GARDEN_SETTLE_INTAKE,


        // 4. Clear corner and go directly to Box 4 (rotating to 90 deg)
        CLEAR_GARDEN_CORNER,
        DRIVE_TO_BOX4,


        // 6. Drive directly from Box 4 to Box 6 (final park)
        DRIVE_TO_BOX6_PARK,


        DONE
    }


    private AutoState autoState = AutoState.BOX1_SHOOT_SPINUP;


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


    private void saveEndPose() {
        OpModeStorage.autonomousEndPose = poseFactory.of(
                drivebase.getFieldX(),
                drivebase.getFieldY(),
                drivebase.getFieldHeadingDeg()
        );
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


        telemetry.addLine("BLUE ODOM AUTO READY (1 -> 2 -> 3 -> 4 -> 6)");
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
        saveEndPose();
        intake.closeFlush();


        // Only spin shooter during the initial preload shot at Box 1
        if (autoState == AutoState.BOX1_SHOOT_SPINUP || autoState == AutoState.BOX1_SHOOT_FEED) {
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


            // --- 3. Turn 90 deg CW to 180 deg, Align to Wall, Drive Into Garden (Box 3) ---
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


            // --- 4. Clear Corner & Drive Directly to Box 4 (Rotating to 90 deg) ---
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
                    moveTimer.reset();
                    autoState = AutoState.DRIVE_TO_BOX6_PARK;
                }
                break;


            // --- 6. Drive Directly from Box 4 to Box 6 Park ---
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
