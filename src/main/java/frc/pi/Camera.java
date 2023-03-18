
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.pi;

import java.util.Arrays;
import java.util.HashMap;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.VideoWriter;
import org.opencv.videoio.Videoio;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;
import frc.pi.Constants.AprilTagValueConstants;
import frc.pi.Constants.DetectionConfigConstants;
import frc.pi.Constants.PhysicalCameraConstants;
import frc.pi.Constants.VideoDisplayConstants;

public class Camera {
	private final NetworkTableInstance inst;
	private final NetworkTable table;

	public static void main(String[] args) throws InterruptedException {

		// Loads correct libraries
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
		WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
		WPIMathJNI.Helper.setExtractOnStaticLoad(false);
		CameraServerJNI.Helper.setExtractOnStaticLoad(false);
		try {
			CombinedRuntimeLoader.loadLibraries(Camera.class, "wpiutiljni", "wpimathjni", "ntcorejni",
					"cscorejnicvstatic");
		} catch (Exception e) {
			System.out.println(e.getStackTrace().toString());
		}

		final Camera camera = new Camera(PhysicalCameraConstants.CAMERA_ID);

		while (true) {
			camera.periodic();
		}
	}

	private VideoCapture capture;

	// Create mats here because they are quite exspensive to make
	private final Mat mat;
	private final Mat grayMat;

	private final AprilTagDetector aprilTagDetector;
	private final AprilTagPoseEstimator aprilTagPoseEstimator;

	private final CvSource outputStream;

	/**
	 * Constructor for Camera
	 * Creates a camera with VideoCapture and sets it to server
	 * Creates and configures a AprilTagDetector and AprilTagPoseEstimator
	 * 
	 * @param cameraID ID of the camera
	 */
	public Camera(int cameraID) {

		inst = NetworkTableInstance.getDefault();

		// Sets up network tables
		inst.startClient4("localpiclient");
		inst.setServerTeam(8032);
		inst.startDSClient();
		// Ip address of the roboRio: 10.80.32.158
		inst.setServer("cpepin.ceramica.wifi");

		table = inst.getTable(String.format("camera-%s-tags", cameraID));

		// -------------- Set up Camera --------------

		capture = new VideoCapture(cameraID);

		capture.set(Videoio.CAP_PROP_FOURCC, VideoWriter.fourcc('M', 'J', 'P', 'G'));
		capture.set(Videoio.CAP_PROP_FRAME_WIDTH, PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH);
		capture.set(Videoio.CAP_PROP_FRAME_HEIGHT, PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT);
		capture.set(Videoio.CAP_PROP_FPS, PhysicalCameraConstants.CAMERA_FPS);

		// Sets up source, the thing that gives the image
		final String ConfigJson = "{\"fps\":%fps%,\"height\":%height%,\"pixelformat\":\"mjpeg\",\"properties\":[{\"name\":\"connect_verbose\",\"value\":1},{\"name\":\"raw_brightness\",\"value\":133},{\"name\":\"brightness\",\"value\":45},{\"name\":\"raw_contrast\",\"value\":5},{\"name\":\"contrast\",\"value\":50},{\"name\":\"raw_saturation\",\"value\":83},{\"name\":\"saturation\",\"value\":41},{\"name\":\"white_balance_temperature_auto\",\"value\":true},{\"name\":\"power_line_frequency\",\"value\":2},{\"name\":\"white_balance_temperature\",\"value\":4500},{\"name\":\"raw_sharpness\",\"value\":25},{\"name\":\"sharpness\",\"value\":50},{\"name\":\"backlight_compensation\",\"value\":0},{\"name\":\"exposure_auto\",\"value\":3},{\"name\":\"raw_exposure_absolute\",\"value\":156},{\"name\":\"exposure_absolute\",\"value\":46},{\"name\":\"pan_absolute\",\"value\":0},{\"name\":\"tilt_absolute\",\"value\":0},{\"name\":\"zoom_absolute\",\"value\":0}],\"width\":%width%}"
				.replace("%fps%", Integer.toString(PhysicalCameraConstants.CAMERA_FPS))
				.replace("%width%", Integer.toString(PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH))
				.replace("%height%", Integer.toString(PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH));

		CvSource source = new CvSource("LifeCam", PixelFormat.kMJPEG, PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH,
				PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT, 15);
		source.setConfigJson(ConfigJson);

		// Sets up server, the thing that recieves the image
		MjpegServer server = new MjpegServer("serve_0", CameraServer.kBasePort + 1);
		server.setConfigJson(ConfigJson);
		server.setSource(source);
		CameraServer.addServer(server);

		outputStream = CameraServer.putVideo("AprilTags", PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH,
				PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT);

		// -------------- Set up Mats --------------

		// Create mat with color (8 bits, 3 channels)
		mat = new Mat(
				PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH,
				PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT,
				CvType.CV_8UC3);

		// Create black and white mat (8 bits, 1 channel)
		grayMat = new Mat(
				PhysicalCameraConstants.CAMERA_RESOLUTION_WIDTH,
				PhysicalCameraConstants.CAMERA_RESOLUTION_HEIGHT,
				CvType.CV_8UC1);

		// ---------- Set up aprilTagDetector ----------

		aprilTagDetector = new AprilTagDetector();

		final AprilTagDetector.Config detectorConfig = aprilTagDetector.getConfig();
		// Set config, see CameraConstants for comments what each setting does
		detectorConfig.quadDecimate = DetectionConfigConstants.QUAD_DECIMATE;
		detectorConfig.quadSigma = DetectionConfigConstants.QUAD_SIGMA;
		detectorConfig.refineEdges = DetectionConfigConstants.REFINE_EDGES;
		detectorConfig.decodeSharpening = DetectionConfigConstants.DECODE_SHARPENING;
		detectorConfig.numThreads = DetectionConfigConstants.NUM_THREADS;
		aprilTagDetector.setConfig(detectorConfig);

		final AprilTagDetector.QuadThresholdParameters quadThresholdParameters = aprilTagDetector
				.getQuadThresholdParameters();
		// Set quad config, see DetectionConfigConstants for comments what each setting
		// does
		quadThresholdParameters.minClusterPixels = DetectionConfigConstants.MIN_CLUSTER_PIXELS;
		quadThresholdParameters.criticalAngle = DetectionConfigConstants.CRITICAL_ANGLE;
		quadThresholdParameters.maxLineFitMSE = DetectionConfigConstants.MAX_LINE_FIT_MSE;
		quadThresholdParameters.minWhiteBlackDiff = DetectionConfigConstants.MIN_WHITE_BLACK_DIFF;
		quadThresholdParameters.deglitch = DetectionConfigConstants.DEGLITCH;
		aprilTagDetector.setQuadThresholdParameters(quadThresholdParameters);

		aprilTagDetector.addFamily(DetectionConfigConstants.TAG_FAMILY);

		// ---------- Set up aprilTagPoseEstimator ----------

		final AprilTagPoseEstimator.Config poseConfig = new AprilTagPoseEstimator.Config(
				AprilTagValueConstants.APRIL_TAG_SIZE_MM, PhysicalCameraConstants.CAMERA_FOCAL_LENGTH_X,
				PhysicalCameraConstants.CAMERA_FOCAL_LENGTH_Y, PhysicalCameraConstants.CAMERA_FOCAL_CENTER_X,
				PhysicalCameraConstants.CAMERA_FOCAL_CENTER_Y);

		aprilTagPoseEstimator = new AprilTagPoseEstimator(poseConfig);
	}

	/** Gets position of tag relative to camera.
	 * A 3d pose of a tag is a Transform3d(Translation3d(x: -right to +left, y: -up
	 * to +down, z: +foward), Rotation3d(Quaternion(...)))
	 * @param tag a AprilTagDetection
	 * @return 3d pose of tag.
	 */
	private Transform3d getTagPose(AprilTagDetection tag) {
		// Translation3d
		// https://www.researchgate.net/profile/Ilya-Afanasyev-3/publication/325819721/figure/fig3/AS:638843548094468@1529323579246/3D-Point-Cloud-ModelXYZ-generated-from-disparity-map-where-Y-and-Z-represent-objects.png
		// Rotation3d Quaternion
		// https://upload.wikimedia.org/wikipedia/commons/thumb/5/51/Euler_AxisAngle.png/220px-Euler_AxisAngle.png
		return aprilTagPoseEstimator.estimate(tag);
	}

	/** Outlines and adds helpful data to diplay about a tag in Mat
	 * @param mat the mat to draw the decorations on
	 * @param tag the april tag you want to draw stuff for
	 * @param tagPose3d the 3d pose of that tag
	 */
	private void decorateTagInImage(Mat mat, AprilTagDetection tag, Transform3d tagPose3d) {
		// Get distance to tag and convert it to feet

		// final double distanceMM = makePose2d(tagPose3d).getNorm();
		final double distanceMM = getDistance(tagPose3d);

		// Create point for each corner of tag
		final Point pt0 = new Point(tag.getCornerX(0), tag.getCornerY(0));
		final Point pt1 = new Point(tag.getCornerX(1), tag.getCornerY(1));
		final Point pt2 = new Point(tag.getCornerX(2), tag.getCornerY(2));
		final Point pt3 = new Point(tag.getCornerX(3), tag.getCornerY(3));

		// Draw lines around box with corner points
		Imgproc.line(mat, pt0, pt1, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt1, pt2, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt2, pt3, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, pt3, pt0, VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);

		// Draw tag Id on tag
		Size tagIdTextSize = addCenteredText(mat, Integer.toString(tag.getId()), 4, 4,
				new Point(tag.getCenterX(), tag.getCenterY()));

		// Draw distance to tag under tag Id

		final double distanceFeet = distanceMM / 304.8;
		addCenteredText(mat, String.format("%.1f ft.", distanceFeet), 1, 2,
				new Point(tag.getCenterX(), tag.getCenterY() + (tagIdTextSize.height * 0.7)));

		Imgproc.line(mat, new Point(tag.getCenterX(), tag.getCenterY()),
				new Point(tag.getCenterX() + tagPose3d.getX(), tag.getCenterY()),
				VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, new Point(tag.getCenterX(), tag.getCenterY()),
				new Point(tag.getCenterX(), tag.getCenterY() + tagPose3d.getY()),
				VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
		Imgproc.line(mat, new Point(tag.getCenterX(), tag.getCenterY()),
				new Point(tag.getCenterX() - tagPose3d.getZ() / 10, tag.getCenterY() + tagPose3d.getZ() / 10),
				VideoDisplayConstants.BOX_OUTLINE_COLOR, 5);
	}

	/** Add text centerd to point to mat */
	private Size addCenteredText(Mat mat, String text, int fontScale, int thickness, Point org) {
		// Get width and height of text
		final Size textSize = Imgproc.getTextSize(text, VideoDisplayConstants.FONT_TYPE, fontScale, thickness, null);

		// Find point where the text goes if it were centered
		// Y is added because Point(0, 0) is top left of Mat
		final Point textCenter = new Point(org.x - (textSize.width / 2), org.y + (textSize.height / 2));

		// Add text outline
		Imgproc.putText(mat, text, textCenter, VideoDisplayConstants.FONT_TYPE,
				fontScale, VideoDisplayConstants.WHITE, thickness + 2);
		// Add text
		Imgproc.putText(mat, text, textCenter, VideoDisplayConstants.FONT_TYPE,
				fontScale, VideoDisplayConstants.TEXT_COLOR, thickness);

		return textSize;
	}

	/** Returns distance to pose in mm */
	private double getDistance(Transform3d pose3d) {
		return pose3d.getTranslation().getNorm();
	}

	/** Check if tag should be consideded by checking if its ID is possible and if
	 * its decision margin is high enough
	 */
	private boolean isTagValid(AprilTagDetection tag) {
		return tag.getId() >= AprilTagValueConstants.MIN_TAG_NUMBER
				&& tag.getId() <= AprilTagValueConstants.MAX_TAG_NUMBER
				&& tag.getDecisionMargin() > AprilTagValueConstants.MIN_APRIL_TAG_DECISION_MARGIN;
	}

	/** Convert tag location transform with origin at center of robot to a transform
	 * with origin at very center front of robot
	 * @param tranformFromCamera transform3d with origin at camera
	 * @return transform3d with origin at very center of robot
	 */
	private Transform3d adjustTransformToRobotFrontCenter(Transform3d tranformFromCamera) {
		return tranformFromCamera.plus(PhysicalCameraConstants.CAMERA_POSITION_FROM_CENTER_CENTER);
	}
	
	/** compress tag location transform to array of doubles */
	private double[] compressTransform3d(Transform3d tagLocation) {
		return new double[] { tagLocation.getX(), tagLocation.getY(), tagLocation.getZ(),
			tagLocation.getRotation().getQuaternion().getW(),
			tagLocation.getRotation().getX(), tagLocation.getRotation().getY(),
			tagLocation.getRotation().getZ() };
	}

	/**
	 * @param aprilTags list of april tags
	 * @param mat Mat to decorate with found tags
	 * @return Map with keys as tag id and values as 3D pose of tag. HashMap<tagId, pose3d>.
	 */
	private HashMap<Integer, Transform3d> makeAprilTagLookup(AprilTagDetection[] apirlTags) {
		final HashMap<Integer, Transform3d> tagLocations = new HashMap<>();
		for (AprilTagDetection tag : apirlTags) {

			// Gets position of tag and puts it in hash map with tagId as key
			final Transform3d pose3d = getTagPose(tag);
			tagLocations.put(tag.getId(), pose3d);

			decorateTagInImage(mat, tag, pose3d);
		}
		return tagLocations;
	}

	public void periodic() {

		// Boolean to see if code is recieving an image, capture.read(mat) will be false
		// if no image
		final boolean success = capture.read(mat);

		AprilTagDetection[] detectedAprilTags = {};

		if (success) {
			// Convert mat to gray scale and store it in grayMat
			Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

			// Detects all AprilTags in grayMat and store them
			detectedAprilTags = aprilTagDetector.detect(grayMat);
		}

		// filter out invalid tags
		detectedAprilTags = Arrays
				.stream(detectedAprilTags)
				.filter(this::isTagValid)
				.toArray(AprilTagDetection[]::new);

		// Makes hash map of found AprilTags. Tag Id is key and location is value.
		HashMap<Integer, Transform3d> tagLocations = makeAprilTagLookup(detectedAprilTags);

		// adjust april tag transforms to have origin at very center of robot
		tagLocations.replaceAll(
				(tagId, tagLocation) -> (tagLocation != null) ? adjustTransformToRobotFrontCenter(tagLocation) : null);

		// decorate all tags on the mat by outlining it, putting tag id on it, and
		// putting distance on it
		for (AprilTagDetection tag : detectedAprilTags) {
			decorateTagInImage(mat, tag, tagLocations.get(tag.getId()));
		}

		boolean tagFound = false;

		// Sends data to network tables by looping through tagIds and putting data
		// gotten from hash map into network table entry
		for (int tagId = AprilTagValueConstants.MIN_TAG_NUMBER; tagId <= AprilTagValueConstants.MAX_TAG_NUMBER; tagId++) {

			// Creates a new entry in correct topic
			NetworkTableEntry entry = table.getEntry(String.valueOf(tagId));

			if (tagLocations.containsKey(tagId)) {
				// Gets pose stored in hash map
				Transform3d tagLocation = tagLocations.get(tagId);

				// Creates double array network table entry with Transform3d propoerties
				entry.setDoubleArray(compressTransform3d(tagLocation));

				System.out.println(String.format("Pi: Found April Tag #%s at distance of %.1f feet [%s]",
						tagId, getDistance(tagLocation) / 304.8, System.currentTimeMillis()));

				tagFound = true;
			} else {
				entry.setDoubleArray(new double[] {});
			}
		}

		if (!tagFound) {
			System.out.println(String.format("Pi: No Tags [%s]", System.currentTimeMillis()));
		}

		outputStream.putFrame(mat);
	}
}