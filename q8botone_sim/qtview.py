'''
The model drawn into a Qt widget by MuJoCo's own renderer, the camera
following the chassis - for a window that wants the robot in a panel of its
own rather than in MuJoCo's viewer:

    view = ModelView(sim)
    sim.step(...)
    view.refresh()                  # draw the sim as it is now

Drag to turn the view, scroll to zoom. Where MuJoCo's renderer can't run
(no usable OpenGL - a virtual machine, a remote desktop, a build server), the
first refresh() raises why, and the view shows a note instead from then on.
'''

import mujoco
import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtGui import QImage, QPainter
from PySide6.QtWidgets import QSizePolicy, QWidget

MAX_PIXELS = (1600, 1200)           # drawn no larger than this; a bigger widget scales it up
CAMERA = {'distance': 0.33, 'azimuth': 135, 'elevation': -22}
TURN_PER_PIXEL = 0.4                # deg of camera turn per pixel dragged
ZOOM_PER_NOTCH = 0.9                # distance x this per scroll-wheel notch in


class ModelView(QWidget):
    def __init__(self, sim, parent=None):
        super().__init__(parent)
        self.sim = sim
        vis = sim.model.vis.global_
        vis.offwidth, vis.offheight = max(vis.offwidth, MAX_PIXELS[0]), max(vis.offheight, MAX_PIXELS[1])
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.camera.trackbodyid = sim.model.body('chassis').id
        self.camera.distance, self.camera.azimuth, self.camera.elevation = (
            CAMERA['distance'], CAMERA['azimuth'], CAMERA['elevation'])
        self._renderer = None
        self._size = None
        self._pixels = None                 # kept alive while the image below points into it
        self._image = None
        self._drag = None
        self.unavailable = None             # why it can't draw here, once the renderer has failed to start
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(160, 120)
        self.setCursor(Qt.OpenHandCursor)

    def refresh(self):
        '''Draw the sim as it is now.'''
        if self.unavailable is not None or not self.isVisible():
            return
        ratio = self.devicePixelRatioF()
        w = max(1, min(round(self.width() * ratio), MAX_PIXELS[0]))
        h = max(1, min(round(self.height() * ratio), MAX_PIXELS[1]))
        if (w, h) != self._size:            # a renderer draws one size: a new one for a new size
            self.close_renderer()
            try:
                self._renderer = mujoco.Renderer(self.sim.model, h, w)
            except Exception as exc:
                self.unavailable = f'{type(exc).__name__}: {exc}'
                self.update()
                raise
            self._size = (w, h)
        self._renderer.update_scene(self.sim.data, self.camera)
        self._pixels = np.ascontiguousarray(self._renderer.render())
        self._image = QImage(self._pixels.data, w, h, 3 * w, QImage.Format_RGB888)
        self.update()

    def close_renderer(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer, self._size = None, None

    def paintEvent(self, _event):
        if self.unavailable is not None:
            QPainter(self).drawText(self.rect().adjusted(12, 12, -12, -12), Qt.AlignCenter | Qt.TextWordWrap,
                                    "No 3D view on this computer: MuJoCo draws with OpenGL, which its graphics "
                                    f"don't offer ({self.unavailable}). The simulation still runs.")
        elif self._image is not None:
            QPainter(self).drawImage(self.rect(), self._image)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._drag = event.position()
            self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._drag is not None:
            delta, self._drag = event.position() - self._drag, event.position()
            self.camera.azimuth -= delta.x() * TURN_PER_PIXEL
            self.camera.elevation = min(-2.0, max(-89.0, self.camera.elevation - delta.y() * TURN_PER_PIXEL))
            self.refresh()

    def mouseReleaseEvent(self, _event):
        self._drag = None
        self.setCursor(Qt.OpenHandCursor)

    def wheelEvent(self, event):
        notches = event.angleDelta().y() / 120
        self.camera.distance = min(3.0, max(0.08, self.camera.distance * ZOOM_PER_NOTCH ** notches))
        self.refresh()

    def closeEvent(self, event):
        self.close_renderer()
        super().closeEvent(event)
