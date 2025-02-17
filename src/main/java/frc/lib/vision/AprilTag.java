package frc.lib.vision;

/**
 * Apriltag
 */
public class AprilTag {
    public double id;
    public double heightFromGround;
    public double[] fieldPosition;

    public AprilTag(double id, double heightFromGround, double[] fieldPosition) {
        this.id = id;
        this.heightFromGround = heightFromGround;
        this.fieldPosition = fieldPosition;
    }

    public double getId() {
        return id;
    }

    // This method returns the height of the tag from the ground
    // @return heightFromGround
    public double getHeightFromGround() {
        return heightFromGround;
    }

    // This method returns the field position of the tag
    // 0 is the x position
    // 1 is the y position
    // @return fieldPosition
    public double[] getFieldPosition() {
        return fieldPosition;
    }
}