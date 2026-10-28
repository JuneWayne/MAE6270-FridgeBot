import math
import time

import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QImage, QPainter, QPainterPath, QPen, QPixmap
from PyQt5.QtWidgets import QGraphicsScene
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from read_cloud_xyz import read_cloud_xyz


# Project XYZ point clouds onto the XY plane of the message coordinate frame
class DashboardLidarPanel:
    def __init__(self, node, view, group):
        self.view = view
        self.group = group
        self.pending_cloud = None
        self.last_cloud_time = None
        self.status = 'Waiting for LiDAR'
        self.point_count = 0

        # Set the visible distance from the sensor to each edge of the plot
        node.declare_parameter('lidar_view_radius_m', 5.0)
        self.radius_m = float(node.get_parameter('lidar_view_radius_m').value)
        if not math.isfinite(self.radius_m) or self.radius_m <= 0:
            raise ValueError('lidar_view_radius_m must be finite and greater than zero')

        # Use a fixed scene size so labels and points keep useful sizes
        self.plot_radius = 300.0
        self.pixels_per_metre = self.plot_radius / self.radius_m
        self.scene = QGraphicsScene(view)
        self.scene.setSceneRect(-345, -345, 690, 690)
        view.setScene(self.scene)
        view.setBackgroundBrush(QColor('#18212b'))
        view.setRenderHint(QPainter.Antialiasing)
        view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        view.setToolTip('Current point cloud: +X up, +Y left, all heights shown')
        self.draw_grid()

        # Use one image for large clouds so thousands of graphics items are not needed
        self.points_item = self.scene.addPixmap(QPixmap())
        self.points_item.setPos(-self.plot_radius, -self.plot_radius)
        self.message_item = self.scene.addText('Waiting for LiDAR')
        self.message_item.setDefaultTextColor(QColor('white'))
        self.message_item.setPos(-110, 40)

        # Best effort reception accepts best effort and reliable point cloud publishers
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = node.create_subscription(
            PointCloud2, '/velodyne_points', self.receive_cloud, qos
        )

    def draw_grid(self):
        grid_pen = QPen(QColor('#455565'))
        grid_pen.setCosmetic(True)
        for ring in range(1, 6):
            radius = self.plot_radius * ring / 5
            self.scene.addEllipse(-radius, -radius, 2 * radius, 2 * radius, grid_pen)
            label = self.scene.addText(f'{self.radius_m * ring / 5:g} m')
            label.setDefaultTextColor(QColor('#b5c4d3'))
            label.setPos(radius - 35, 4)
        self.scene.addLine(-300, 0, 300, 0, grid_pen)
        self.scene.addLine(0, -300, 0, 300, grid_pen)
        heading = self.scene.addText('+X up | +Y left | all heights')
        heading.setDefaultTextColor(QColor('white'))
        heading.setPos(-100, -338)

        # The centre marker shows the message frame origin and its positive X direction
        marker = QPainterPath()
        marker.moveTo(0, -10)
        marker.lineTo(-7, 7)
        marker.lineTo(7, 7)
        marker.closeSubpath()
        self.scene.addPath(marker, QPen(Qt.NoPen), QColor('#ffcf5c')).setZValue(1)

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

        # Remove invalid coordinates even if the publisher marks the cloud as dense
        valid = np.isfinite(points).all(axis=1)
        valid &= (np.abs(points[:, :2]) <= self.radius_m).all(axis=1)
        points = points[valid]
        self.point_count = len(points)

        # Place positive X at the top and positive Y at the left of the plot
        size = int(2 * self.plot_radius + 1)
        pixels = np.zeros((size, size, 4), dtype=np.uint8)
        columns = np.rint(self.plot_radius - points[:, 1] * self.pixels_per_metre).astype(int)
        rows = np.rint(self.plot_radius - points[:, 0] * self.pixels_per_metre).astype(int)
        for dx, dy in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
            pixels[np.clip(rows + dy, 0, size - 1), np.clip(columns + dx, 0, size - 1)] = (
                89, 245, 138, 255
            )
        image = QImage(pixels.data, size, size, pixels.strides[0], QImage.Format_RGBA8888).copy()

        # Replace the previous cloud without accumulating a map or applying a TF transform
        self.points_item.setPixmap(QPixmap.fromImage(image))
        self.group.setTitle(f'LiDAR | {message.header.frame_id or "frame not set"}')
        self.status = f'LiDAR: {self.point_count} points in view'
        self.message_item.setPlainText('No cloud points in view')
        self.message_item.setVisible(self.point_count == 0)

    def clear_cloud(self, status):
        self.points_item.setPixmap(QPixmap())
        self.point_count = 0
        self.status = status
        self.message_item.setPlainText(status)
        self.message_item.setVisible(True)

    def fit_view(self):
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)
