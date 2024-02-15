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

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public final class PoseEstimator implements Constants.PhotonVision, Constants.Swerve {

    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private PhotonCamera currentCamera;
    private PhotonPoseEstimator photonEstimator1;
    private PhotonPoseEstimator photonEstimator2;
    private PhotonPoseEstimator currentPhotonEstimator;

    private double lastEstTimestamp = 0;

    private final SwerveDrivePoseEstimator swervePoseEstimator;

    private static PoseEstimator instance;

    private PoseEstimator() {
        camera1 = VisionModule.getInstance().getAprilTagsFrontRightCamera();
        camera2 = VisionModule.getInstance().getAprilTagsRearLeftCamera();
        currentCamera = camera1;

        photonEstimator1 = VisionModule.getInstance().getPhotonEstimatorFrontRight();
        photonEstimator2 = VisionModule.getInstance().getPhotonEstimatorRearLeft();
        currentPhotonEstimator = photonEstimator1;

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1); // TODO: Tune the standard deviations
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        swervePoseEstimator =
                new SwerveDrivePoseEstimator(
                        swerveKinematics,
                        new Rotation2d(),
                        new SwerveModulePosition[] {
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition(),
                                new SwerveModulePosition()
                        },
                        new Pose2d(),
                        stateStdDevs,
                        visionStdDevs);
    }

    public static synchronized PoseEstimator getInstance() {
        if (instance == null) {
            instance = new PoseEstimator();
        }
        return instance;
    }

    /*
    private void setCamera(PhotonCamera camera) {
        currentCamera = camera;
    }
    
    private void setPhotonEstimator(Transform3d robotToCamera) {
        photonEstimator = new PhotonPoseEstimator(
                tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, currentCamera, robotToCamera
        );
    }
    */

    private void useBestCamera() {
        var cam1Targets = camera1.getLatestResult().getTargets();
        var cam2Targets = camera2.getLatestResult().getTargets();
        int cam1numTags = 0;
        int cam2numTags = 0;
        double cam1AvgDist = 0;
        double cam2AvgDist = 0;

        Transform3d robotToCamera;

        for (var tgt : cam1Targets) {
            var tagPose = currentPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            cam1numTags++;
            cam1AvgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(swervePoseEstimator.getEstimatedPosition().getTranslation());
        }
        cam1AvgDist /= cam1numTags;
        for (var tgt : cam2Targets) {
            var tagPose = currentPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            cam2numTags++;
            cam2AvgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(swervePoseEstimator.getEstimatedPosition().getTranslation());
        }
        cam2AvgDist /= cam2numTags;

        if (cam2numTags > cam1numTags) {
            currentCamera = camera2;
            currentPhotonEstimator = photonEstimator2;
        } else if (cam2AvgDist < cam1AvgDist && cam2numTags == cam1numTags) {
            currentCamera = camera2;
            currentPhotonEstimator = photonEstimator2;
        } else  {
            currentCamera = camera1;
            currentPhotonEstimator = photonEstimator1;
        }
    }

    public PhotonPipelineResult getLatestResult() {
        return currentCamera.getLatestResult();
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedVisionGlobalPose() {
        useBestCamera();
        var visionEst = currentPhotonEstimator.update();
        double latestTimestamp = currentCamera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedVisionGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getVisionEstimationStdDevs(Pose2d estimatedPose) {
        if (DriverStation.isDisabled()) {
            return VecBuilder.fill(0.05d, 0.05d, 0.05d);
        }
        var estStdDevs = singleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = currentPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = multiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    /**
     * @return The {@link Pose2d} of the robot according to the {@link SwerveDrivePoseEstimator}
     */
    public synchronized Pose2d getPose() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    /**
     * @param pose The {@link Pose2d} to set the {@link SwerveDrivePoseEstimator} to
     */
    public synchronized void setPose(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions, Pose2d pose) {
        swervePoseEstimator.resetPosition(gyroYaw, modulePositions, pose);
    }

    /**
     * Update the {@link SwerveDrivePoseEstimator} with the yaw and module positions
     *
     * @param gyroYaw The yaw of the gyro as a {@link Rotation2d}
     * @param modulePositions The {@link SwerveModulePosition} of each swerve module in an array
     */
    public synchronized void updateSwerveEstimator(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
        swervePoseEstimator.update(gyroYaw, modulePositions);
    }

    public synchronized void updateWithVision() {
        var visionEst = getEstimatedVisionGlobalPose();
        visionEst.ifPresent(
                est -> {
                    var estPose = est.estimatedPose.toPose2d();
                    // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getVisionEstimationStdDevs(estPose);

                    swervePoseEstimator.addVisionMeasurement(
                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
    }
}