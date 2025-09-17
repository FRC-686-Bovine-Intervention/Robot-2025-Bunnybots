package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Predicate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.util.flipping.AllianceFlipped;

public final class FieldConstants {
    public static final Distance fieldLength = Inches.of(57*12 + 6 + 7.0/8.0);
    public static final Distance fieldWidth =  Inches.of(26*12 + 5);

    public static final AllianceFlipped<Predicate<Translation2d>> onAllianceSide = new AllianceFlipped<>(
        new Predicate<>() {
            private final double halfline = fieldLength.div(2).in(Meters);
            @Override
            public boolean test(Translation2d t) {
                return t.getX() <= this.halfline;
            }
        },
        new Predicate<>() {
            private final double halfline = fieldLength.div(2).in(Meters);
            @Override
            public boolean test(Translation2d t) {
                return t.getX() >= this.halfline;
            }
        }
    );

    public static final AprilTagFieldLayout apriltagLayout;
    static {
        AprilTagFieldLayout a = null;
        try {
            a = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        } catch(Exception e) {
            e.printStackTrace();
        }
        apriltagLayout = a;
    }

    public static final Distance luniteLength = Inches.of(7);
    public static final Distance luniteWidth = Inches.of(4);
}