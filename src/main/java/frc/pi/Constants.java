
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.pi;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
	public static final class PhysicalCameraConstants {
		public static final int CAMERA_ID = 0;

		public static final Transform3d CAMERA_POSITION_FROM_CENTER_CENTER = new Transform3d(
			new Translation3d(0, 0.5, 0.5),
			new Rotation3d());

			
		public static final int CAMERA_FPS = 10;
		public static final int CAMERA_FPS_MILLIS = 1000 / CAMERA_FPS;

		// Understading the focal length values: https://en.wikipedia.org/wiki/Camera_resectioning, https://en.wikipedia.org/wiki/Cardinal_point_(optics)

		// Microsoft LifeCam HD-3000 stuff https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/TeamCode/src/main/res/xml/teamwebcamcalibrations.xml
		public static final int CAMERA_RESOLUTION_WIDTH = 640;
		public static final int CAMERA_RESOLUTION_HEIGHT = 480;

		public static final double CAMERA_FOCAL_LENGTH_X = 678.154f;
		public static final double CAMERA_FOCAL_LENGTH_Y = 678.17;

		public static final double CAMERA_FOCAL_CENTER_X = 318.135;
		public static final double CAMERA_FOCAL_CENTER_Y = 228.374;

		// LimeLight stuff https://docs.limelightvision.io/en/latest/vision_pipeline_tuning.html
		// public static final int CAMERA_RESOLUTION_WIDTH = 960;
		// public static final int CAMERA_RESOLUTION_HEIGHT = 720;

		// public static final double CAMERA_FOCAL_LENGTH_X = 772.53876202;
		// public static final double CAMERA_FOCAL_LENGTH_Y = 769.052151477;

		// public static final double CAMERA_FOCAL_CENTER_X = 479.132337442;
		// public static final double CAMERA_FOCAL_CENTER_Y = 359.143001808;
	}

	public static final class AprilTagValueConstants {
		// width and height of April Tag
		public static final double APRIL_TAG_SIZE_MM = 152.4;

		// Valid tag ID range
		public static final int MIN_TAG_NUMBER = 1;
		public static final int MAX_TAG_NUMBER = 8;

		public static final int MIN_APRIL_TAG_DECISION_MARGIN = 50;
	}

	public static final class DetectionConfigConstants {
		// ---------- AprilTagDetector.Config ----------:

		// how much image should be decimated when finding quads
		public static final float QUAD_DECIMATE = 2.0f;

		// What Gaussian blur should be applied to the segmented image. Important for noisy images
		public static final float QUAD_SIGMA = 0.0f;

		// should edges snap to strong gradients 
		public static final boolean REFINE_EDGES = true;

		// How much sharpening should be done to decoded images.
		public static final double DECODE_SHARPENING = 0.25;

		public static final int NUM_THREADS = 4;

		// ---------- AprilTagDetector.QuadThresholdParameters ---------- 

		// if a cluster of pixels is smaller than this value it is ignored
		public static final int MIN_CLUSTER_PIXELS = 0;

		// The yaw rotation of an object before it is ignored
		public static final double CRITICAL_ANGLE = 10 * Math.PI / 180.0;

		// How square an object needs to be to be considered
		public static final float MAX_LINE_FIT_MSE = 10.0f;

		// Minimum brightness offset
		public static final int MIN_WHITE_BLACK_DIFF = 5;

		// Whether the thresholded image be should be deglitched. Important for noisy images
		public static final boolean DEGLITCH = false;

		// the April Tag family we are using
		public static final String TAG_FAMILY = "tag16h5";
	}

	public static final class VideoDisplayConstants {
		public static final Scalar RED = new Scalar(0, 0, 255);
		public static final Scalar GREEN = new Scalar(0, 255, 0);
		public static final Scalar BLUE = new Scalar(255, 0, 0);
		public static final Scalar WHITE = new Scalar(0, 0, 0);
		public static final Scalar BLACK = new Scalar(255, 255, 255);

		public static final Scalar TEXT_COLOR = VideoDisplayConstants.GREEN;
		public static final Scalar BOX_OUTLINE_COLOR = VideoDisplayConstants.RED;

		public static final int FONT_TYPE = Imgproc.FONT_HERSHEY_DUPLEX;

	}
}