package org.firstinspires.ftc.teamcode.pedro;


import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;




@Autonomous(name = "Red Side Tilt Auto", group = "Red")
public class RedSideSafe extends OpMode {


    private Follower follower;


    private Shooter shooter;
    private Intake intake;


    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(4.9368, 23.4104, 0);
    private final Pose point1 = poseFactory.of(57.8264, 23.434, 90);
    private final Pose point1Control1 = poseFactory.of(34.9328, 32.771, 0);
    private final Pose point2Start = poseFactory.of(57.8264, 23.434, 90);
    private final Pose point2 = poseFactory.of(28.6549, 3.7948, 0);
    private final Pose point3 = poseFactory.of(5.4191, 4.0229, -0.5623);
    private final Pose point4Start = poseFactory.of(5.4191, 4.0229, 0);
    private final Pose point4 = poseFactory.of(9.6678, 98.7754, 90);
    private final Pose point4Control1 = poseFactory.of(41.0252, 58.7618, 0);

    public static double VELOCITY_P_TARGET = 1000;
    public static double VELOCITY_N_TARGET = 1000;


    public static double SHOOTER_READY_PERCENT = 0.98;


    public static double SHOOT_TIME_MS = 2000;




    private ElapsedTime shootTimer;




    private enum AutoState {


        START_TO_SHOOT,
        WAIT_START_TO_SHOOT,


        PRELOAD_SPINUP,
        PRELOAD_FEED,


        SHOOT_TO_GARDEN,
        WAIT_SHOOT_TO_GARDEN,


        PICKUP_GARDEN,
        WAIT_PICKUP_GARDEN,

        PARK,
        WAIT_PARK,


        DONE
    }




    private AutoState autoState = AutoState.START_TO_SHOOT;


    public Path path1() {
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }





    private void runShooter() {


        shooter.setVelocityP(VELOCITY_P_TARGET);
        shooter.setVelocityN(VELOCITY_N_TARGET);
    }




    private boolean shooterReady() {


        return shooter.getVelocityP() >=
                VELOCITY_P_TARGET * SHOOTER_READY_PERCENT


                &&


                shooter.getVelocityN() >=
                        VELOCITY_N_TARGET * SHOOTER_READY_PERCENT;
    }


    @Override
    public void init() {


        Scheduler.reset();


        // Initialize Pedro
        follower = Constants.create(hardwareMap);


        follower.setPose(start);


        follower.update();




        // Initialize current subsystems
        shooter = new Shooter(hardwareMap);


        intake = new Intake(hardwareMap);




        shootTimer = new ElapsedTime();




        // Stop mechanisms initially
        shooter.setPower(0);


        intake.setPower(0);


        intake.setPowerGateClose();


        intake.closeFlush();




        autoState = AutoState.START_TO_SHOOT;




        telemetry.addLine("RED SIDE TILT AUTO READY");


        telemetry.addData("Start X", start.x());
        telemetry.addData("Start Y", start.y());


        telemetry.update();
    }




    @Override
    public void start() {


        autoState = AutoState.START_TO_SHOOT;
    }






    @Override
    public void loop() {


        follower.update();


        Scheduler.execute();


        updateAuto();




        // Telemetry
        telemetry.addData("Auto State", autoState);


        telemetry.addData("X", follower.pose().x());


        telemetry.addData("Y", follower.pose().y());


        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );


        telemetry.addData(
                "Shooter P",
                shooter.getVelocityP()
        );


        telemetry.addData(
                "Shooter N",
                shooter.getVelocityN()
        );


        telemetry.addData(
                "Shooter Ready",
                shooterReady()
        );


        telemetry.addData(
                "Intake Power",
                intake.getRightIntakePower()
        );


        telemetry.update();
    }






    private void updateAuto() {


        switch (autoState) {


            case START_TO_SHOOT:


                // Start spinning both shooter motors
                runShooter();


                schedule(
                        follow(follower, path1())
                );


                autoState = AutoState.WAIT_START_TO_SHOOT;


                break;




            case WAIT_START_TO_SHOOT:


                runShooter();


                if (!follower.isBusy()) {


                    autoState = AutoState.PRELOAD_SPINUP;
                }


                break;




            case PRELOAD_SPINUP:


                runShooter();


                intake.setPower(0);


                intake.setPowerGateClose();


                if (shooterReady()) {


                    intake.setPowerGateOpen();


                    intake.setPower(-1.0);


                    shootTimer.reset();


                    autoState = AutoState.PRELOAD_FEED;
                }


                break;




            case PRELOAD_FEED:


                runShooter();


                intake.setPowerGateOpen();


                intake.setPower(-1.0);


                if (shootTimer.milliseconds() >= SHOOT_TIME_MS) {


                    intake.setPower(0);


                    intake.setPowerGateClose();


                    autoState = AutoState.SHOOT_TO_GARDEN;
                }


                break;




            case SHOOT_TO_GARDEN:


                intake.setPowerGateOpen();


                intake.setPower(-1.0);


                schedule(
                        follow(follower, path2())
                );


                autoState = AutoState.WAIT_SHOOT_TO_GARDEN;


                break;




            case WAIT_SHOOT_TO_GARDEN:


                intake.setPower(-1.0);


                if (!follower.isBusy()) {


                    autoState = AutoState.PICKUP_GARDEN;
                }


                break;




            case PICKUP_GARDEN:


                intake.setPowerGateOpen();


                intake.setPower(-1.0);


                schedule(
                        follow(follower, path3())
                );


                autoState = AutoState.WAIT_PICKUP_GARDEN;


                break;




            case WAIT_PICKUP_GARDEN:


                intake.setPower(-1.0);


                if (!follower.isBusy()) {


                    intake.setPower(0);


                    intake.setPowerGateClose();


                    autoState = AutoState.PARK;
                }
                runShooter();


                intake.setPowerGateOpen();


                intake.setPower(-1.0);


                if (shootTimer.milliseconds() >= SHOOT_TIME_MS) {


                    intake.setPower(0);


                    intake.setPowerGateClose();


                    autoState = AutoState.PARK;
                }


                break;


            case PARK:


                intake.setPower(0);


                intake.setPowerGateClose();


                schedule(
                        follow(follower, path4())
                );


                autoState = AutoState.WAIT_PARK;


                break;




            case WAIT_PARK:


                if (!follower.isBusy()) {


                    autoState = AutoState.DONE;
                }


                break;


            case DONE:


                shooter.setPower(0);


                intake.setPower(0);


                intake.setPowerGateClose();


                intake.closeFlush();


                break;
        }
    }


    @Override
    public void stop() {


        shooter.setPower(0);


        intake.setPower(0);


        intake.setPowerGateClose();


        intake.closeFlush();
    }
}
