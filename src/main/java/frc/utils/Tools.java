package frc.utils;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public class Tools {
    public record SwerveModConfig(
            int DriveID,
            int SteerID,
            int EncoderID,
            Angle offset,
            Translation2d place
    ) {
    }
}
