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


# The sky and floor: (rgb1, rgb2) of each texture, 0-1 - light as
# model/q8botone.xml makes them (keep these the same), and dark for a dark
# window around the view (set_dark). The robot stays as it is.
SCENE = {
    'sky': {'light': ((0.80, 0.85, 0.90), (0.98, 0.98, 1.00)),
            'dark': ((0.17, 0.18, 0.20), (0.11, 0.11, 0.12))},
    'grid': {'light': ((0.92, 0.93, 0.94), (0.84, 0.86, 0.88)),
             'dark': ((0.33, 0.34, 0.36), (0.27, 0.28, 0.30))},
}
DARK_HAZE = (0.14, 0.15, 0.16, 1.0)  # the glow along the horizon - MuJoCo's own is white


def render_size(width, height, pixel_ratio=1.0):
    '''
    The pixels to draw a widget of this size in: its own, on a scaled
    display too, shrunk to fit MAX_PIXELS - both sides by the same factor,
    so the picture keeps the widget's shape when it's stretched over it.
    '''
    w, h = width * pixel_ratio, height * pixel_ratio
    fit = min(1.0, MAX_PIXELS[0] / max(w, 1), MAX_PIXELS[1] / max(h, 1))
    return max(1, round(w * fit)), max(1, round(h * fit))


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
        self._light_textures = None         # the sky's and floor's own pixels, once set_dark() has changed them
        self.unavailable = None             # why it can't draw here, once the renderer has failed to start
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(160, 120)
        self.setCursor(Qt.OpenHandCursor)

    def refresh(self):
        '''Draw the sim as it is now.'''
        if self.unavailable is not None or not self.isVisible():
            return
        w, h = render_size(self.width(), self.height(), self.devicePixelRatioF())
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

    def set_dark(self, dark):
        '''
        The sky and floor dark, or back to the model's own light ones. Each
        texture's pixels are mapped from its light pair of colours to its
        dark pair, channel by channel, so the sky stays a gradient and the
        floor a checker.
        '''
        model = self.sim.model
        if self._light_textures is None:
            self._light_haze = model.vis.rgba.haze.copy()
            self._light_textures = {}
            for name in SCENE:
                i = model.texture(name).id
                start = model.tex_adr[i]
                end = start + model.tex_width[i] * model.tex_height[i] * model.tex_nchannel[i]
                self._light_textures[name] = (start, end, model.tex_data[start:end].copy())
        for name, colors in SCENE.items():
            start, end, light = self._light_textures[name]
            if not dark:
                model.tex_data[start:end] = light
                continue
            pixels = light.reshape(-1, 3).astype(np.float32)
            (l1, l2), (d1, d2) = (np.array(c) * 255 for c in colors['light']), (np.array(c) * 255 for c in colors['dark'])
            span = np.where(l2 != l1, l2 - l1, 1.0)
            mapped = d1 + (pixels - l1) / span * (d2 - d1)
            model.tex_data[start:end] = np.clip(np.rint(mapped), 0, 255).astype(np.uint8).ravel()
        model.vis.rgba.haze[:] = DARK_HAZE if dark else self._light_haze
        self.close_renderer()               # a new renderer uploads the textures afresh
        self.refresh()

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
