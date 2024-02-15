/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public final class VisionModule implements Constants.PhotonVision, Constants.Swerve {

    private final PhotonCamera aprilTagsFrontRight;
    private final PhotonCamera aprilTagsRearLeft;
    private PhotonPoseEstimator photonEstimatorFrontRight;
    private PhotonPoseEstimator photonEstimatorRearLeft;


    private final PhotonCamera notesIndexer;
    private PhotonPipelineResult lastNoteResult = null;
    private boolean startTrackingNotes = false;


    private static VisionModule instance;

    private VisionModule() {
        aprilTagsFrontRight = new PhotonCamera(APRIL_TAGS_FRONT_RIGHT_CAMERA_NAME);
        aprilTagsRearLeft = new PhotonCamera(APRIL_TAGS_FRONT_RIGHT_CAMERA_NAME);

        photonEstimatorFrontRight =
                new PhotonPoseEstimator(
                        tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagsFrontRight, ROBOT_TO_TAG_FRONT_RIGHT_CAM_POS);
        photonEstimatorFrontRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonEstimatorRearLeft =
                new PhotonPoseEstimator(
                        tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagsRearLeft, ROBOT_TO_TAG_REAR_LEFT_CAM_POS);
        photonEstimatorRearLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        notesIndexer = new PhotonCamera(NOTES_INDEXER_CAMERA_NAME);

    }

    public static synchronized VisionModule getInstance() {
        if (instance == null) {
            instance = new VisionModule();
        }
        return instance;
    }

    public PhotonCamera getAprilTagsFrontRightCamera() {
        return aprilTagsFrontRight;
    }

    public PhotonCamera getAprilTagsRearLeftCamera() {
        return aprilTagsRearLeft;
    }

    public PhotonPoseEstimator getPhotonEstimatorFrontRight() {
        return photonEstimatorFrontRight;
    }

    public PhotonPoseEstimator getPhotonEstimatorRearLeft() {
        return photonEstimatorRearLeft;
    }

    public PhotonCamera getNotesIndexerCamera() {
        return notesIndexer;
    }

    public PhotonPipelineResult getLastNoteResult() {
        return lastNoteResult;
    }

    protected void setLastNoteResult(PhotonPipelineResult lastNoteResult) {
        this.lastNoteResult = lastNoteResult;
    }

    public boolean isStartTrackingNotes() {
        return startTrackingNotes;
    }

    public void setStartTrackingNotes(boolean startTrackingNotes) {
        this.startTrackingNotes = startTrackingNotes;
    }

    public void trackPipelineResults() {
    }

}