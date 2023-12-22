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
                        drive.trajectorySequenceBuilder(new Pose2d(-35.59, -67.97, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(-35.90, -36.51), Math.toRadians(92.86))
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