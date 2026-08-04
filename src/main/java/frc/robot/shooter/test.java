package frc.robot.shooter;

import edu.wpi.first.math.geometry.Pose2d;

public class test {
    public Pose2d p;

    public test(Pose2d pose){
        this.p = pose;
    }

    public static test t(){
        return new test(new Pose2d());
    }
}
