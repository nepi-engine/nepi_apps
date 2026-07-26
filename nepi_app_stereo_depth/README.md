# NEPI App Stereo Depth

Produces a depth map and point cloud from two independent USB cameras mounted as a
rigid stereo pair. The app consumes the color image topics published by the standard
IDX drivers (e.g. `idx_v4l2` for UVC webcams), pairs left/right frames by timestamp,
computes disparity with OpenCV StereoSGBM, and publishes the result through the
standard NEPI `DepthMapIF` and `PointcloudIF` interfaces.

The app does not touch driver code. The first target hardware is two identical
NexiGo 1080p 110-degree UVC webcams, which enumerate as ordinary IDX color-image
devices.
