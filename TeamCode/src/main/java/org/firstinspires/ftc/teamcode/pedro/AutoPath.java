//package org.firstinspires.ftc.teamcode.pedro;
//
//import static com.pedropathing.api.Paths.*;
//import com.pedropathing.api.Paths;
//
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.ivy.Scheduler;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.commands.Commands.*;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.limeLightV2;
//
//
//
//
//
//
//@Autonomous(name = "AutoPath", group = "Autonomous")
//public class AutoPath extends LinearOpMode {
//
//    private Follower follower;
//    private Intake intake;
//    private Shooter shooter;
//    private limeLightV2 limelight;
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    private final Pose start = poseFactory.of(4.7722, 136.9329, 0);
//    private final Pose path1 = poseFactory.of(57.9187, 32.5464, 90);
//    private final Pose path1Control1 = poseFactory.of(14.0061, 0, 0);
//    private final Pose point2 = poseFactory.of(35.9548, 8.6901, -132.6348);
//    private final Pose point3 = poseFactory.of(9.1527, 8.8356, 179.6888);
//    private final Pose point4Start = poseFactory.of(9.1527, 8.8356, 180);
//    private final Pose point4 = poseFactory.of(58.1581, 33.0364, 90);
//    private final Pose point5 = poseFactory.of(13.5247, 105.3067, 90);
//    private final Pose point5Control1 = poseFactory.of(17.2198, 21.0091, 0);
//
//
//    // Autonomous routine
//    public Command autoRoutine() {
//        return sequential(
//                follow(follower, path1()),
//                follow(follower, path2()),
//                follow(follower, path3()),
//                follow(follower, path4()),
//                follow(follower, path5())
//        );
//    }
//
//    @Override
//    public void runOpMode() {
//        Scheduler.reset();
//        follower = Constants.create(hardwareMap);
//        follower.setPose(start);
//        follower.update();
//
//        waitForStart();
//        schedule(autoRoutine());
//
//        while (opModeIsActive()) {
//            follower.update();
//            Scheduler.execute();
//
//            telemetry.addData("x", follower.pose().x());
//            telemetry.addData("y", follower.pose().y());
//            telemetry.addData("heading", follower.pose().heading());
//
//            if (follower.currentPath() != null) {
//                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
//                telemetry.addData("Path number", follower.pathIndex());
//            }
//
//            telemetry.update();
//        }
//    }
//
//    public Path path1() {
//        return Paths.curve(start, path1Control1, path1).linear(start, path1);
//    }
//
//    public Path path2() {
//        return Paths.line(path1, point2).tangent();
//    }
//
//    public Path path3() {
//        return Paths.line(point2, point3).tangent();
//    }
//
//    public Path path4() {
//        return Paths.line(point4Start, point4).linear(point4Start, point4);
//    }
//
//    public Path path5() {
//        return Paths.curve(point4, point5Control1, point5).constant(point5);
//    }
//}