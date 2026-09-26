package org.firstinspires.ftc.teamcode.pedro;//package org.firstinspires.ftc.teamcode.pedro;
//
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.ivy.Scheduler;
//import com.pedropathing.localization.Localizer;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.revhub.localizers.PinpointLocalizer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import static com.pedropathing.api.Paths.line;
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//
//@Autonomous
//public class ExampleAuto extends OpMode {
//
//    private Follower follower;
//
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    // Poses
//    private final Pose startPose = poseFactory.of(24, 24, 0);
//    private final Pose scorePose = poseFactory.of(48, 48, 90);
//    private final Pose parkPose = poseFactory.of(72, 48, 90);
//
//    // Path from start to scoring position
//    private Path startToScore() {
//        return line(startPose, scorePose)
//                .linear(startPose, scorePose);
//    }
//
//    // Path from scoring position to parking position
//    private Path park() {
//        return line(scorePose, parkPose)
//                .linear(scorePose, parkPose);
//    }
//
//    // Autonomous routine
//    private Command autoRoutine() {
//        return sequential(
//                follow(follower, startToScore()),
//
//                // Add mechanism commands here if needed
//
//                follow(follower, park())
//        );
//    }
//
//    @Override
//    public void init() {
//
//        Scheduler.reset();
//
//        // Create Pinpoint localizer
//        Localizer localizer = new PinpointLocalizer(hardwareMap);
//
//        // Create Pedro follower
//        follower = Constants.create(localizer);
//
//        // Set starting position
//        follower.setPose(startPose);
//
//        follower.update();
//    }
//
//    @Override
//    public void start() {
//        schedule(autoRoutine());
//    }
//
//    @Override
//    public void loop() {
//
//        follower.update();
//
//        Scheduler.execute();
//    }
//}
