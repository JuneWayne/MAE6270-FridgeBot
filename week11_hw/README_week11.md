# FridgeBot RGB-D perception — ROS 2 Jazzy

A standalone Python version of the perception workflow developed during the lab session: detect a yellow-taped box, estimate its 3D position, express that position in robot coordinates, and display its position and orientation in RViz2.

These files organize the session's scripts into reusable programs, with imports, entry points, comments, and shutdown handling. This is a cleaned reconstruction of the workflow, not a byte-for-byte terminal transcript. The packaged files must be checked once on the robot after download; packaging checks do not replace a hardware test.

**Current project: ROS_DOMAIN_ID=3.** The robot-side computer is Victor's ThinkPad, using Ubuntu 24.04 / ROS 2 Jazzy. The camera is a RealSense D435-family RGB-D camera. All distances in published messages are **meters**.

## 1. Files and what to run

| File | Purpose | When to use |
|---|---|---|
| `check_dependencies.py` | Check Python imports | Once after sourcing ROS |
| `yellow_detector_2d.py` | Yellow contour, green outline, red pixel centroid, binary mask | Optional first learning/debugging stage |
| `box_detector_3d.py` | Same detection plus depth-derived XYZ in the camera optical frame | Final point pipeline |
| `robot_point_transform.py` | Transform camera point to `base_link`; print readable XYZ and report stale data | Run with the 3D detector |
| `box_pose_rgbd.py` | Estimate full box pose using RGB-D, yellow corners, and a white corner mark; publish a cube | Final pose pipeline |
| `README.md` | Startup, theory, display configuration, validation, and troubleshooting | This document |

**Do not run `yellow_detector_2d.py` and `box_detector_3d.py` together.** They intentionally publish the same debug image and mask topics. The 3D detector supersedes the 2D-only version.

For the full demonstration, run the camera driver, static mount transform, `box_detector_3d.py`, `robot_point_transform.py`, `box_pose_rgbd.py`, and RViz. The pose node can also run independently of the two point-processing scripts if you only need its cube and pose image.

These are standalone scripts, not an `ament_python` package. You do not need `colcon build` to run them. They publish perception data only; they do not command robot movement.

## 2. Copy into the project

Extract the ZIP. It contains a folder named `fridgebot_perception`. Copy that folder into your repository, for example:

```bash
cd ~/MAE6270-FridgeBot/fridgebot_perception
```

All `python3 ...` commands below assume this is your working directory. If you put the folder elsewhere, change the `cd` command accordingly. To upload to GitHub, upload this entire folder, including the README and all five `.py` files. Do not upload generated `__pycache__` directories.

## 3. Environment and dependencies

Run this in **every new terminal** used for this project:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
```

If your RealSense driver or other dependencies are in a source-built workspace, also source that workspace's actual `install/setup.bash`. Do not guess a workspace path. Avoid a Python virtual environment that hides Ubuntu's ROS packages.

Check imports:

```bash
cd ~/MAE6270-FridgeBot/fridgebot_perception
python3 check_dependencies.py
```

Expected: an `OK` line for each of `rclpy`, `cv2`, `numpy`, `cv_bridge`, `sensor_msgs.msg`, `geometry_msgs.msg`, `tf2_ros`, `message_filters`, `tf2_geometry_msgs`, and `visualization_msgs.msg`.

If ROS is installed but these additional packages are missing, install only what you need:

```bash
sudo apt update
sudo apt install python3-opencv python3-numpy \
  ros-jazzy-cv-bridge ros-jazzy-message-filters \
  ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-visualization-msgs ros-jazzy-rviz2
```

The camera launch command also requires a working `realsense2_camera` installation. Preserve the working driver installation from the lab session. The detector does not directly import `pyrealsense2`; it consumes ROS topics.

## 4. Prepare the physical target

- Box dimensions: **0.420 × 0.325 × 0.050 m** (42 × 32.5 × 5 cm).
- Yellow tape forms a closed border around the visible face.
- For the pose script, use a unique **white patch near the top-left corner**, inside the yellow border, in the reference portrait orientation.
- In that reference orientation, width across the image is 0.325 m and height is 0.420 m.
- Keep the entire border and white patch visible. Start with the face roughly toward the camera, then test moderate rotations.
- Keep other yellow rectangles out of the view where possible. The code chooses the largest qualifying yellow quadrilateral.

The white patch disambiguates which corner is corner 0. It is not a depth marker. Its detection uses low saturation and high brightness, so glare or other white patches inside the box can confuse it. Yellow border detection itself does not require the white patch; full pose does.

## 5. Start the camera — terminal 1

If USB detection is uncertain:

```bash
lsusb
rs-enumerate-devices
```

Close RealSense Viewer and any duplicate camera driver before starting ROS. A `/dev/video*` entry alone does not identify which camera it belongs to. If RealSense enumeration sees the device, investigate driver access/configuration rather than assuming the cable is disconnected.

Launch:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true
```

Leave this terminal running. Wait until the driver reports that the camera node is up. The launch uses your driver's default stream profiles; it does not force the lab's resolutions.

The working session used approximately 30 FPS, color 1280×720, native depth 848×480, and color-aligned depth 1280×720. The scripts read calibration at runtime and do not hardcode these resolutions.

### Verify the data — a separate terminal

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
ros2 topic list -t
```

Required input topics:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
```

**Use aligned depth for RGB pixel lookup.** `/camera/camera/depth/image_rect_raw` is the native depth stream and is not a substitute for color-aligned depth.

```bash
ros2 topic echo /camera/camera/aligned_depth_to_color/image_raw \
  --once --field encoding --qos-reliability best_effort

ros2 topic echo /camera/camera/color/camera_info \
  --once --qos-reliability best_effort
```

The session returned `16UC1`, meaning this RealSense depth stream supplies millimeters. The scripts convert it by multiplying by 0.001. They also accept `32FC1` depth in meters. Zero, NaN, and infinite depth values are excluded.

The observed color calibration was:

```text
frame_id: camera_color_optical_frame
width: 1280, height: 720
fx = 925.4647216796875
fy = 925.03125
cx = 651.4511108398438
cy = 353.1671447753906
distortion coefficients: all zero
```

This is a reference record, not a calibration to paste into code. The scripts subscribe to `CameraInfo`. They require matching color/depth/calibration dimensions and frame IDs, and reject nonzero distortion rather than silently using the wrong pinhole model.

To measure rates, run each command separately for several seconds and press **Ctrl+C** before the next:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

An initial “does not appear to be published yet” warning can clear when discovery completes. Sustained rates are more informative. The session measured roughly 29 Hz color and 29.6 Hz aligned depth.

## 6. Optional first stage: image-only detection

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
cd ~/MAE6270-FridgeBot/fridgebot_perception
python3 yellow_detector_2d.py
```

Display `/eriche/target/debug_image` and `/eriche/target/mask` as RViz **Image** displays. A green outline should surround the yellow border and a red dot should mark the image centroid. In the mask, the detected tape is white and the rest is black; that white outline is expected.

Stop this script with Ctrl+C before starting `box_detector_3d.py`.

The image pipeline converts BGR to HSV, thresholds yellow with OpenCV bounds `(18,70,60)` to `(40,255,255)`, closes small gaps with a 5×5 kernel, and finds the largest convex four-corner contour above the minimum area. OpenCV hue is in the range 0–179. These thresholds may need tuning under different lighting.

## 7. Publish camera-frame XYZ — terminal 2

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
cd ~/MAE6270-FridgeBot/fridgebot_perception
python3 box_detector_3d.py
```

Leave it running. It synchronizes RGB and aligned depth using a queue of 10 and a maximum timestamp separation of 0.05 seconds. A five-second status log reports synchronized callback count and whether calibration has arrived.

The detector takes the median valid depth in a 7×7 patch around the contour's image centroid. At least 10 valid depth samples are required. It computes:

```text
Z = median depth in meters
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

The point describes the visible surface near the projected contour centroid. It is not necessarily the exact physical face center under perspective, and it is not the center of the box's volume.

View live values in another sourced terminal:

```bash
ros2 topic echo /eriche/target/point --field point
```

For the frame and timestamp as well:

```bash
ros2 topic echo /eriche/target/point --once
```

Camera optical axes are **X right, Y down, Z forward**. A small X does not mean the box is close; forward depth is Z. Euclidean range is `sqrt(X²+Y²+Z²)`. For example, the session's `(0.2426, 0.1383, 1.154)` is about 1.154 m forward and 1.187 m from the optical origin.

## 8. Publish the camera mounting transform — terminal 3

We defined `base_link` at the horizontal center of the robot, at the bottom-of-base height:

- +X: forward
- +Y: left, from the robot's own perspective
- +Z: up

The user-provided measurements were:

| Measurement | Calculation | Translation |
|---|---|---|
| Camera forward of robot center | 9 in × 0.0254 | X = 0.2286 m |
| Robot width / camera inward from left edge | (14.5/2 − 1.75) in × 0.0254 | Y = +0.1397 m |
| Camera above bottom of base | 22.5 in × 0.0254 | Z = 0.5715 m |

The camera is assumed upright, level, and facing forward. These are approximate mount measurements. The reference point must correspond to `camera_link`; measurements to the housing or color lens can introduce small offsets. If a robot URDF already defines a different `base_link` origin, reconcile that definition before reusing these numbers. Do not publish competing transforms for the same child frame.

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
ros2 run tf2_ros static_transform_publisher \
  --x 0.2286 --y 0.1397 --z 0.5715 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id camera_link
```

Leave it running. “Spinning until stopped” means success, not a frozen terminal. This command is the only additional mount transform needed: the RealSense driver already publishes the internal transforms connecting `camera_link` to the optical frames.

Check in another terminal:

```bash
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
```

Press Ctrl+C, then:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

The observed combined translation was approximately `[0.228, 0.155, 0.572]` m. Y differs from the mount's 0.1397 m because the driver adds the internal camera baseline offset. The optical frame also has approximately −90° roll and −90° yaw relative to a forward-facing body frame; this is expected axis conversion, not evidence that the camera is mounted sideways.

A short initial “frame does not exist” during discovery is acceptable if the transform then appears. `At time 0.0` is expected for a static transform. Press Ctrl+C to stop `tf2_echo`; that does not stop the publisher in terminal 3.

## 9. Transform and print robot-frame XYZ — terminal 4

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
cd ~/MAE6270-FridgeBot/fridgebot_perception
python3 robot_point_transform.py
```

Expected readable output, about twice per second:

```text
BASE_LINK  X=+1.316 m  Y=-0.062 m  Z=+0.411 m
```

This example means 1.316 m forward, 6.2 cm to the robot's right, and 0.411 m above our defined base origin. Z is **not automatically height above the floor**, since the origin was defined at the bottom of the base.

The output topic publishes at the incoming detection rate when transforms are available; the terminal print is throttled for readability. The node transforms at the source image timestamp and preserves that timestamp on the output.

```bash
ros2 topic echo /eriche/target/point_base --field point
```

If no successfully transformed detection arrives for more than one second, the terminal reports `STALE`. It does not publish a fake zero point. A consumer must use the timestamp and its own freshness timeout; the last numeric value is not a guarantee that the target remains visible.

## 10. Estimate full box pose — terminal 5

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
cd ~/MAE6270-FridgeBot/fridgebot_perception
python3 box_pose_rgbd.py
```

The status should become something like:

```text
POSE OK | measured face 0.325 x 0.420 m
```

The actual measured dimensions will fluctuate. The code accepts width within 0.10 m and height within 0.12 m of the configured dimensions; these are loose rejection gates, not a precision claim.

### How pose estimation works

1. Detect the yellow quadrilateral and order its corners cyclically.
2. Find a white blob inside the polygon near a corner; rotate the corner order so that marked corner is 0.
3. Sample valid depth inside a shrunken face polygon, excluding much of the tape and background.
4. Convert those depth samples to 3D camera-frame points using the inverse intrinsic matrix.
5. Fit a plane with SVD; trim large residuals three times and refit.
6. Intersect each corner's camera ray with that plane to recover four 3D corners.
7. Use corner 0→1 for box +X, the inward plane normal for +Z, and a cross product for +Y.
8. Average the face corners, then move inward by half the 0.050 m thickness to estimate the volume center.
9. Publish a quaternion pose and a 0.325 × 0.420 × 0.050 m cube marker.

This implementation uses **depth-plane fitting and ray intersections**, not `solvePnP`. The known dimensions validate the estimated face and set the cube size. It does not infer depth solely from apparent image size.

The box frame starts at its volume center; in the reference front view its axes point right, down, and inward. This is a right-handed coordinate system. Camera pose and base-frame point are different outputs: the point samples the visible surface; the pose locates the estimated box volume center.

The pose image includes numbered corners, a purple circle around the detected white patch, and projected XYZ axes. **The purple circle is the corner-marker detection, not the midpoint.** The 3D RViz point is a separate display.

Inspect raw pose values:

```bash
ros2 topic echo /eriche/target/pose_camera --once
```

The pose is published in `camera_color_optical_frame`. RViz transforms it into `base_link` for display when the mount TF is available. This script does not publish a separate `/pose_base` topic or a box TF frame.

## 11. RViz setup — terminal 6

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=3
rviz2
```

Set **Global Options → Fixed Frame → `base_link`** once the mount transform is running. Before that, `camera_color_optical_frame` is suitable for camera-only testing.

Use **Add → By display type** and select the exact display type below. Expand the display to set its Topic.

| RViz display type | Topic | Meaning |
|---|---|---|
| Image | `/camera/camera/color/image_raw` | Raw color, no detection overlay |
| Image | `/camera/camera/aligned_depth_to_color/image_raw` | Aligned depth visualization |
| Image | `/eriche/target/debug_image` | Green outline, red centroid, camera XYZ |
| Image | `/eriche/target/mask` | White yellow-tape mask on black background |
| PointStamped | `/eriche/target/point` | Surface point in camera optical coordinates |
| PointStamped | `/eriche/target/point_base` | Same surface point expressed in base coordinates |
| Image | `/eriche/target/pose_debug` | Pose status, corners, white marker, projected axes |
| Marker | `/eriche/target/box_marker` | Green translucent oriented 3D cube |
| Pose | `/eriche/target/pose_camera` | Optional pose arrows/axes |

For image subscriptions, use **Reliability Policy → Best Effort** and **Durability → Volatile** where these controls appear. If a Transport Hint option exists, select `raw`; some RViz versions do not expose that option, and you do not need to find or install it.

For PointStamped, set **History Length = 1** if offered, so the view does not accumulate a trail. Increase the point's visual radius if it is hard to see. Use a comfortable camera view distance, roughly a few meters, and add Grid or TF only when useful. A detection image is an Image display; a point is not an Image or PointCloud2 display.

For a lightweight final demo, show just the pose image, cube marker, and one point display. Several live images and TF labels can make the GUI slower.

Save the configuration using **File → Save Config As**, for example `fridgebot_perception.rviz`. Then restart it later with:

```bash
rviz2 -d fridgebot_perception.rviz
```

That `.rviz` file is created by you in RViz; it is not included in this bundle.

## 12. Topic and frame reference

| Topic | ROS message type | Publisher |
|---|---|---|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | Camera driver |
| `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera driver |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Camera driver |
| `/eriche/target/debug_image` | `sensor_msgs/msg/Image` | 2D or 3D detector |
| `/eriche/target/mask` | `sensor_msgs/msg/Image` | 2D or 3D detector |
| `/eriche/target/point` | `geometry_msgs/msg/PointStamped` | 3D detector |
| `/eriche/target/point_base` | `geometry_msgs/msg/PointStamped` | Point transformer |
| `/eriche/target/pose_debug` | `sensor_msgs/msg/Image` | Pose estimator |
| `/eriche/target/pose_camera` | `geometry_msgs/msg/PoseStamped` | Pose estimator |
| `/eriche/target/box_marker` | `visualization_msgs/msg/Marker` | Pose estimator |

`base_link` is a coordinate frame, **not the base-station computer**. You do not have to copy or display everything on another computer merely to use that frame. This entire workflow can run on the robot computer. For remote viewing, the other computer needs compatible ROS networking, domain 3, and access to the relevant topics and TF; matching domain IDs alone does not guarantee network discovery or data delivery. Do not launch a duplicate detector just to view the output remotely.

## 13. Validation before recording the demo

Keep the robot stationary during these perception checks.

1. Show the entire target with the white patch visible. Confirm the green outline and `POSE OK` status.
2. Read camera XYZ and base XYZ. Confirm units and signs, then compare with a tape-measured reference to the corresponding coordinate origin.
3. Move the target farther from the camera: camera Z and robot forward X should generally increase for this forward-facing mount.
4. Move it toward the robot's left: base Y should increase. Move it upward: base Z should increase.
5. Tilt and rotate the target moderately. The cube should rotate with it while corner 0 stays tied to the white patch.
6. Remove the target from view. The point transformer should report stale data; the cube should disappear approximately 0.5 seconds after its last successful update. RViz PointStamped may retain its last point.
7. Return the target. Valid points and pose updates should resume.
8. Hold it still and observe jitter. Record actual errors or variability rather than assuming the three decimal places imply millimeter accuracy.

Compare the surface-point and pose-center outputs with their definitions in mind. They should not be expected to match exactly. The pose center includes a half-thickness offset and uses plane-intersected corners.

To check throughput:

```bash
ros2 topic hz /eriche/target/point
ros2 topic hz /eriche/target/point_base
ros2 topic hz /eriche/target/pose_camera
```

Run one rate command at a time. During the session, the point topic was approximately 25–27 Hz, with occasional larger gaps; that was not evidence of a camera running at only a few FPS. The pose pipeline has additional processing cost. Rate alone also does not measure end-to-end latency.

## 14. Troubleshooting

### Command prompt seems stuck / cannot enter another command

Camera launch, `python3` nodes, `ros2 topic echo`, `ros2 topic hz`, and TF utilities run continuously. Open another terminal for the next task, or press Ctrl+C if you intend to stop the current task. Do not paste a new shell command into a process that is still running. Copy complete topic names: accidental suffixes such as `image_rawaw` or `pointnt` are different names.

### Camera is connected but ROS cannot find or open it

Check `rs-enumerate-devices`, cable/USB connection, duplicate drivers, and RealSense Viewer. A transient format or reconnect error that is followed by a running stream is different from a persistent startup failure. Confirm live messages before changing the working driver installation.

### Aligned-depth topic is absent

Check that the active camera launch has `align_depth.enable:=true`, and inspect `ros2 topic list -t` for the actual namespace. Stop only the old camera launch before restarting it; do not start a second driver for the same device.

### Images exist but the detector publishes nothing

Check both RGB and aligned-depth rates. The callback requires a synchronized pair. Look at the five-second status log of `box_detector_3d.py`: no increasing pair count points toward subscription/QoS/timestamps; missing CameraInfo blocks XYZ; no visible rectangle or invalid depth blocks point output even if images are flowing. A 0.05-second synchronization tolerance is not permission to use unrelated frames.

### `base_link` does not exist

The camera driver normally supplies camera frames only. Start the mount static publisher on domain 3 and leave it running. Allow a few seconds for discovery. Use `tf2_echo base_link camera_color_optical_frame` to verify the complete chain. If another robot configuration already owns this transform, use that configuration instead of publishing a duplicate.

### `/tf_static` echo waits forever

Try an explicit message type and transient-local durability:

```bash
ros2 topic echo /tf_static tf2_msgs/msg/TFMessage \
  --once --qos-durability transient_local --qos-reliability best_effort
```

Inspect endpoints:

```bash
ros2 topic info /tf_static --verbose
```

A waiting echo alone does not prove that TF is absent; `tf2_echo` successfully found the camera transforms during the session. Large apparent delays or implausible rates in `tf2_monitor` for static transforms are not equivalent to image-stream latency.

### RViz crashes with incompatible type / invalid allocator

The reported error included an existing `/eriche/target/point` topic with an incompatible type. Check the actual type:

```bash
ros2 topic list -t
ros2 topic info /eriche/target/point --verbose
```

It must be `geometry_msgs/msg/PointStamped`. Remove displays that subscribe to that name as another type, and use a PointStamped display. Check for another node publishing a different message type under the same name. The later “invalid allocator” message may be secondary to the failed subscription; it is not by itself a diagnosis of memory exhaustion.

If RViz is unresponsive, close it or, from another terminal:

```bash
pkill -TERM -x rviz2
```

This stops matching RViz instances, not the camera or detector nodes. Start RViz again from a terminal with ROS sourced and domain 3. Rebuild a minimal configuration first instead of reopening a configuration that triggers the same problem.

### The 3D dot does not move

Check `/eriche/target/point` and `/eriche/target/point_base` in the terminal while moving the box. If messages stop, check whether the full border left the camera frame or depth became invalid. PointStamped has no marker lifetime, so RViz can leave the last point on screen after detection stops. Set History Length to 1, check the display's topic, and distinguish a fresh point from an old one. The pose cube has a 0.5-second lifetime, but the PointStamped dot does not.

### Pose says no white marker / size mismatch

Make sure the white patch is inside the border near the correct physical corner and is large/bright enough to pass the white threshold. The image's purple circle marks the detected patch. Keep unrelated white regions and glare out of the face. For a different object, update `BOX_WIDTH`, `BOX_HEIGHT`, `BOX_DEPTH`, the physical corner-mark convention, and dimension rejection tolerances in `box_pose_rgbd.py`. Merely rotating the same marked object should not require changing its dimensions.

### XYZ seems too small or uses the wrong axis

Check `header.frame_id` and units. In the camera optical frame Z is forward; in `base_link` X is forward. A depth value such as 1154 in the raw 16-bit image represents 1.154 m for this RealSense stream. Do not divide coordinates already published in meters by 1000 again.

### Pose or point is noisy

Use valid depth at a sensible distance, avoid grazing angles and occlusion, and inspect the segmentation. The pose node uses simple residual trimming, not a full robust RANSAC estimator. There is no temporal smoothing or tracking. Measurements of the mount are approximate, and the white-corner heuristic can fail. The loose dimension gates can accept imperfect detections. Validate before using these estimates in a robot controller.

## 15. Shutdown and repeatable startup

Press Ctrl+C in each Python-node terminal, close RViz, then stop the mount publisher and camera launch. The mount publisher is not permanently installed; restart it next session unless you integrate the transform into the robot's URDF/launch configuration.

Next session's order:

1. Source Jazzy and set domain 3 in every terminal.
2. Camera launch with alignment enabled.
3. Static `base_link → camera_link` publisher.
4. `python3 box_detector_3d.py`.
5. `python3 robot_point_transform.py`.
6. `python3 box_pose_rgbd.py`.
7. RViz with the correct displays and fixed frame.
8. Check detection freshness, measured position, and orientation.

For the lab hand-in, retain a short demo showing the real target moving, the detection image, and the RViz result, and document the method and measured limitations in the report. This bundle supplies the code and tutorial; it does not include a recorded demo or a completed lab report.

## 16. References

- [RealSense ROS driver](https://github.com/realsenseai/realsense-ros)
- [ROS 2 Jazzy: Python TF2 listener](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html)
- [ROS 2 Jazzy: static TF broadcaster](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html)
- [OpenCV HSV thresholding](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html)
- [OpenCV contour features](https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html)
- [REP 118: depth image representation](https://reps.openrobotics.org/rep-0118/)

## 17. Packaging verification

All five Python files passed syntax compilation. Synthetic checks exercised the actual 3D detector's localization method with lightweight message substitutes: millimeter and meter depth conversion, pinhole projection, and rejection of missing depth, nonfinite depth, mismatched dimensions, mismatched frames, and nonzero distortion. The mount's inch-to-meter calculations were also checked.

The packaging environment did not have ROS 2 or OpenCV installed. Live subscriptions, image segmentation, plane-based pose estimation, TF transport, and RViz were not rerun there. The session's observed results describe the earlier interactive workflow; check these packaged scripts on the robot before recording or submitting your final demonstration.
