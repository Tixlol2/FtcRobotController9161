package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.Pose2d;

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


    public Pose startRedScore = new Pose(60, -36);
    public Pose startBlueScore = new Pose(-60, 36);

    public Pose startRedPark = new Pose(12, 60);
    public Pose startBluePark = new Pose(132, 84);

    public Pose leftRedSpecimen = new Pose(98, 120);
    public Pose middleRedSpecimen = new Pose(98, 132);
    public Pose rightRedSpecimen = new Pose(98, 144);

    public Pose leftBlueSpecimen = new Pose(46, 0);
    public Pose middleBlueSpecimen = new Pose(46, 12);
    public Pose rightBlueSpecimen = new Pose(46, 24);

    public Pose leftYellowSpecimenR = new Pose(98, 0);
    public Pose middleYellowSpecimenR = new Pose(98, 12);
    public Pose rightYellowSpecimenR = new Pose(98, 24);

    public Pose leftYellowSpecimenB = new Pose(46, 144);
    public Pose middleYellowSpecimenB = new Pose(46, 132);
    public Pose rightYellowSpecimenB = new Pose((120-72), 46-72);

    public Pose redSubmersible = new Pose(72, 56);
    public Pose blueSubmersible = new Pose(72, 88);

    public Pose redRungMidpoint = new Pose(96, 72);
    public Pose blueRungMidpoint = new Pose(48, 72);

    public Pose redScore = new Pose(132, 12);
    public Pose blueScore = new Pose(24, 120, -45);

    public Pose redPark = new Pose(132,132);
    public Pose bluePark = new Pose(12, 32);











}
