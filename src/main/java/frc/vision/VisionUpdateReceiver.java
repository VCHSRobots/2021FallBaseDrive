package frc.vision;

/**
 * A basic interface for classes that get VisionUpdates. Classes that implement this interface specify what to do when
 * VisionUpdates are received.
 * 
 * @see VisionUpdate.java
 */
public interface VisionUpdateReceiver {
    void gotUpdate(VisionUpdate update);
}
