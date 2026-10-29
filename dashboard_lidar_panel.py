import math
import os
import sys
import time

import numpy as np
from PyQt5.QtGui import QVector3D
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QSizePolicy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from read_cloud_xyz import read_cloud_xyz


# Display the complete XYZ point cloud in an interactive 3D view
class DashboardLidarPanel:
    def __init__(self, node, placeholder, group):
        # Match OpenGL to the active Qt display before importing the graphics library
        # WSL and Ubuntu can expose both X11 and Wayland environment variables
        platform = QApplication.platformName()
        if sys.platform.startswith('linux') and 'OpenGL.platform' not in sys.modules:
            if platform == 'xcb':
                os.environ.setdefault('PYOPENGL_PLATFORM', 'glx')
            elif platform.startswith('wayland'):
                os.environ.setdefault('PYOPENGL_PLATFORM', 'egl')
        import pyqtgraph.opengl as gl

        self.group = group
        self.pending_cloud = None
        self.last_cloud_time = None
        self.status = 'Waiting for LiDAR'
        self.point_count = 0

        # Keep the existing parameter as the starting grid and camera scale
        node.declare_parameter('lidar_view_radius_m', 5.0)
        self.radius_m = float(node.get_parameter('lidar_view_radius_m').value)
        if not math.isfinite(self.radius_m) or self.radius_m <= 0:
            raise ValueError('lidar_view_radius_m must be finite and greater than zero')

        # Replace the Designer placeholder without changing the saved UI file
        self.view = gl.GLViewWidget(parent=group)
        self.view.setObjectName('lidarView3D')
        self.view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.view.setBackgroundColor('#18212b')
        layout = group.layout()
        layout.replaceWidget(placeholder, self.view)
        placeholder.hide()
        placeholder.deleteLater()
        self.message_label = QLabel('Waiting for LiDAR', group)
        self.message_label.setWordWrap(True)
        self.message_label.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        layout.addWidget(self.message_label)
        self.reset_button = QPushButton('Reset 3D view', group)
        self.reset_button.clicked.connect(self.reset_view)
        layout.addWidget(self.reset_button)
        layout.setStretch(0, 1)
        self.view.setToolTip('Left drag: rotate | Middle drag: move view | Wheel: zoom')

        # Grid coordinates use metres in the point cloud message frame
        self.grid = gl.GLGridItem()
        self.grid.setSize(x=2 * self.radius_m, y=2 * self.radius_m)
        self.grid.setSpacing(x=self.radius_m / 5, y=self.radius_m / 5)
        self.view.addItem(self.grid)
        self.axes = []
        for end, color in (
            ((1, 0, 0), (1, 0.25, 0.25, 1)),
            ((0, 1, 0), (0.25, 1, 0.25, 1)),
            ((0, 0, 1), (0.3, 0.6, 1, 1)),
        ):
            axis = gl.GLLinePlotItem(pos=np.array([(0, 0, 0), end], dtype=np.float32),
                                     color=color, width=2, antialias=True)
            self.axes.append(axis)
            self.view.addItem(axis)

        # Keep Z coordinates so points above each other remain separate in 3D
        self.points_item = gl.GLScatterPlotItem(
            pos=np.empty((0, 3), dtype=np.float32),
            color=(0.35, 0.96, 0.54, 1), size=2, pxMode=True,
        )
        self.points_item.setGLOptions('opaque')
        self.view.addItem(self.points_item)
        self.reset_view()

        # Best effort reception accepts best effort and reliable point cloud publishers
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = node.create_subscription(
            PointCloud2, '/velodyne_points', self.receive_cloud, qos
        )

    def reset_view(self):
        # Start above the XY plane so height is visible without moving the mouse
        self.view.setCameraPosition(pos=QVector3D(0, 0, 0), distance=3 * self.radius_m,
                                    elevation=25, azimuth=45)

    def receive_cloud(self, message):
        # Keep the latest point cloud until the next display update
        self.pending_cloud = message
        self.last_cloud_time = time.monotonic()

    def update(self):
        if self.pending_cloud is not None:
            message = self.pending_cloud
            self.pending_cloud = None
            self.draw_cloud(message)
        if self.last_cloud_time is not None and time.monotonic() - self.last_cloud_time > 2.0:
            self.clear_cloud('No LiDAR point cloud for 2 seconds')
        return self.status

    def draw_cloud(self, message):
        try:
            points = read_cloud_xyz(message)
        except (ValueError, TypeError, BufferError, OverflowError) as error:
            self.clear_cloud(f'Invalid LiDAR point cloud: {error}')
            return

        # Remove invalid coordinates but keep valid points outside the initial grid
        valid = np.isfinite(points).all(axis=1)
        valid &= (np.abs(points) <= np.finfo(np.float32).max).all(axis=1)
        points = np.ascontiguousarray(points[valid], dtype=np.float32)
        self.point_count = len(points)
        self.points_item.setData(pos=points)
        self.group.setTitle(f'LiDAR 3D | {message.header.frame_id or "frame not set"}')
        self.status = f'LiDAR: {self.point_count} 3D points'
        self.message_label.setText(
            f'{self.point_count} points | X: red | Y: green | Z: blue | Units: metres'
            if self.point_count else 'No valid points in the current cloud'
        )

    def clear_cloud(self, status):
        self.points_item.setData(pos=np.empty((0, 3), dtype=np.float32))
        self.point_count = 0
        self.status = status
        self.message_label.setText(status)
