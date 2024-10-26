package org.firstinspires.ftc.teamcode.auton;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class autonPoints {

    /*


    Making this so we can just call all the points we need
    hopefully we get some use out of this
    I'm just going to set Pose for now, with just x, y according to the
    Pedro Pathing Simulator at https://pedro-path-generator-itd.vercel.app/

    The buckets, on the left side, will be called Score
    And the parking zone will be called Park
    These don't describe actions happening within the program,
    just the location of the start.

    For the specimen locations, I'm going to set them to the point
    on which the specimen rests. In auton, we can just add/subtract the offset on the path
    Other note, the specimen starting inline with the tape changing slope will be called left

    For the yellow specimens on the field, left will represent closest to the wall


     */


    public Pose startRedScore = new Pose(-60, -60, Math.toRadians(-90));
    public Pose startBlueScore = new Pose(60, 60, Math.toRadians(90));

    public Pose startRedPark = new Pose(-60, 12, Math.toRadians(-90));
    public Pose startBluePark = new Pose(60, -12, Math.toRadians(90));

    public Pose leftRedSpecimen = new Pose(-50, 50);
    public Pose middleRedSpecimen = new Pose(-50, 60);
    public Pose rightRedSpecimen = new Pose(-50, 70);

    public Pose leftBlueSpecimen = new Pose(50, -50);
    public Pose middleBlueSpecimen = new Pose(50, -60);
    public Pose rightBlueSpecimen = new Pose(50, -70);

    public Pose leftYellowSpecimenR = new Pose(-26, -70);
    public Pose middleYellowSpecimenR = new Pose(-26, -58);
    public Pose rightYellowSpecimenR = new Pose(-26, -46);

    public Pose leftYellowSpecimenB = new Pose(26, 70);
    public Pose middleYellowSpecimenB = new Pose(26, 58);
    public Pose rightYellowSpecimenB = new Pose(26, 46);

    public Pose redSubmersibleMidpoint = new Pose(0, -16);
    public Pose blueSubmersibleMidpoint = new Pose(0, 16);

    public Pose redRungMidpoint = new Pose(-24, 0);
    public Pose blueRungMidpoint = new Pose(24, 0);

    public Pose redScore = new Pose(-48, -48, Math.toRadians(45));
    public Pose blueScore = new Pose(48, 48, Math.toRadians(-45));

    public Pose redPark = new Pose(-60,48);
    public Pose bluePark = new Pose(60, -48);











}
