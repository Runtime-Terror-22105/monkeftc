package com.example.meepmeepvisual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MEEPMEEPVISUAL {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(14.6457, 16.063)
                .setConstraints(100, 40, Math.toRadians(360), Math.toRadians(360), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9.47, 65.39, Math.toRadians(-80.54)))
                                .splineTo(new Vector2d(20.19, 44.13), Math.toRadians(-60.79))
                                .splineTo(new Vector2d(51.81, 41.09), Math.toRadians(0.00))
                                .lineTo(new Vector2d(-57.89, 35.91))
                                .splineTo(new Vector2d(51.63, 36.27), Math.toRadians(-0.09))
                                .lineTo(new Vector2d(49.85, 60.92))
                                .splineTo(new Vector2d(62.17, 60.57), Math.toRadians(9.46))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
/* Bottom Right
drive.trajectorySequenceBuilder(new Pose2d(-35.59, -67.97, Math.toRadians(90.00)))
                                .UNSTABLE_addTemporalMarkerOffset(16.56,() -> {})
                                .splineTo(new Vector2d(-36.00, -50.08), Math.toRadians(89.54))
                                .lineTo(new Vector2d(-34.90, -31.19))
                                .lineTo(new Vector2d(-44.79, -11.93))
                                .lineTo(new Vector2d(-57.69, -12.10))
                                .lineTo(new Vector2d(20.23, -11.93))
                                .lineTo(new Vector2d(35.25, -15.11))
                                .splineTo(new Vector2d(51.68, -35.25), Math.toRadians(0.00))
                                .lineTo(new Vector2d(23.94, -10.69))
                                .lineTo(new Vector2d(-57.51, -12.28))
                                .build()
 */

/*Bottom Left

 */



// RED MIDDLE

/*
TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(11.08, -63.60, Math.toRadians(90.00)))
.splineTo(new Vector2d(11.79, -33.59), Math.toRadians(90.00))
.lineTo(new Vector2d(52.70, -36.45))
.lineTo(new Vector2d(15.90, -11.43))
.lineTo(new Vector2d(-57.17, -11.79))
.lineTo(new Vector2d(31.09, -13.58))
.lineTo(new Vector2d(52.88, -36.45))
.lineTo(new Vector2d(36.80, -36.98))
.lineTo(new Vector2d(48.42, -61.64))
.splineTo(new Vector2d(60.03, -62.35), Math.toRadians(-1.88))
.build();

 */

// RED LEFT
/*
    TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(14.11, -63.96, Math.toRadians(90.00)))
            .splineTo(new Vector2d(7.86, -42.16), Math.toRadians(116.57))
            .lineTo(new Vector2d(26.98, -37.70))
            .splineTo(new Vector2d(51.81, -30.73), Math.toRadians(0.00))
            .lineTo(new Vector2d(11.61, -10.18))
            .lineTo(new Vector2d(-57.71, -11.61))
            .splineTo(new Vector2d(15.90, -16.08), Math.toRadians(-3.47))
            .splineTo(new Vector2d(51.81, -35.20), Math.toRadians(-4.09))
            .lineTo(new Vector2d(49.49, -62.17))
            .splineTo(new Vector2d(60.74, -62.00), Math.toRadians(-1.82))
            .build();
*/


// Blue Left
/*
TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(9.47, 65.39, Math.toRadians(-80.54)))
.splineTo(new Vector2d(20.19, 44.13), Math.toRadians(-60.79))
.splineTo(new Vector2d(51.81, 41.09), Math.toRadians(0.00))
.lineTo(new Vector2d(-57.89, 35.91))
.splineTo(new Vector2d(51.63, 36.27), Math.toRadians(-0.09))
.lineTo(new Vector2d(49.85, 60.92))
.splineTo(new Vector2d(62.17, 60.57), Math.toRadians(9.46))
.build();

 */

/*
Blue Middle


TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(12.86, 63.78, Math.toRadians(-88.45)))
.splineTo(new Vector2d(11.61, 33.77), Math.toRadians(270.00))
.lineTo(new Vector2d(40.02, 38.05))
.splineTo(new Vector2d(51.81, 35.55), Math.toRadians(0.00))
.lineTo(new Vector2d(-58.24, 36.09))
.splineTo(new Vector2d(51.81, 35.73), Math.toRadians(-2.05))
.build();

 */


/*
Blue Right

TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(11.08, 62.89, Math.toRadians(270.00)))
.splineTo(new Vector2d(5.72, 39.84), Math.toRadians(236.31))
.lineTo(new Vector2d(29.12, 38.95))
.splineTo(new Vector2d(51.81, 30.37), Math.toRadians(0.00))
.lineTo(new Vector2d(2.68, 10.36))
.lineTo(new Vector2d(-57.71, 11.43))
.splineTo(new Vector2d(31.27, 15.72), Math.toRadians(2.76))
.splineTo(new Vector2d(46.81, 29.12), Math.toRadians(90.00))
.splineTo(new Vector2d(51.63, 28.41), Math.toRadians(0.00))
.build();

 */