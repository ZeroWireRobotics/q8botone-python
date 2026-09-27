'''
Written by yufeng.wu0902@gmail.com

The 3D viewer panel: shows the robot as a single STL you can orbit.

STL rather than STEP on purpose. STEP is BREP/NURBS - a CAD exchange format
that no real-time viewer can draw directly; it has to be tessellated into
triangles first, and doing that at runtime means dragging in OpenCASCADE.
Exporting a mesh once from CAD is the cheaper trade by a wide margin.

The model is one rigid body here. Articulating it (legs following the robot)
needs the assembly exported as separate parts with known pivots, plus the
five-bar linkage solved for the intermediate joints - deliberately not
attempted yet.

Drop a .stl into q8bot/assets/ and it gets picked up; without one the panel
falls back to a message saying so.
'''

import struct
import sys
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QGroupBox, QLabel, QVBoxLayout

from ..helpers import Q8Logger


def _shaded_from_above():
    """
    Replaces pyqtgraph's stock 'shaded' shader, which lights from below and
    behind - from any sensible camera angle almost every visible face lands
    on its 0.2 ambient floor, so the model reads as a flat silhouette no
    matter what colour it is. This lights mainly from above with a much
    higher ambient, so faces separate instead of crushing together.
    """
    from pyqtgraph.opengl.shaders import FragmentShader, ShaderProgram, VertexShader

    return ShaderProgram('q8shaded', [
        VertexShader("""
            uniform mat4 u_mvp;
            uniform mat3 u_normal;
            attribute vec4 a_position;
            attribute vec3 a_normal;
            attribute vec4 a_color;
            varying vec4 v_color;
            varying vec3 v_normal;
            void main() {
                v_normal = normalize(u_normal * a_normal);
                v_color = a_color;
                gl_Position = u_mvp * a_position;
            }
        """),
        FragmentShader("""
            #ifdef GL_ES
            precision mediump float;
            #endif
            varying vec4 v_color;
            varying vec3 v_normal;
            void main() {
                float key = max(dot(v_normal, normalize(vec3(0.35, 0.35, 0.87))), 0.0);
                float fill = max(dot(v_normal, normalize(vec3(-0.6, -0.3, 0.2))), 0.0);
                float light = 0.40 + 0.50 * key + 0.14 * fill;
                gl_FragColor = vec4(v_color.rgb * light, v_color.a);
            }
        """),
    ])


def _asset_dir():
    # PyInstaller unpacks datas under _MEIPASS rather than beside the module.
    bundled = getattr(sys, '_MEIPASS', None)
    if bundled:
        return Path(bundled) / 'q8bot' / 'assets'
    return Path(__file__).resolve().parent.parent / 'assets'


ASSET_DIR = _asset_dir()
PREFERRED_NAME = 'q8bot.stl'

# Two different colour conventions in the same library, so mind which is
# which: GLMeshItem unpacks its colour straight into glVertexAttrib4f and
# needs 0-1 floats, while setBackgroundColor and GLGridItem go through
# mkColor(), which reads floats as 0-255 ints and turns them all black.
MODEL_COLOR = (0.60, 0.66, 0.75, 1.0)
BACKGROUND = '#eef1f5'
GRID_COLOR = (120, 132, 148, 90)


def find_model(asset_dir=None):
    """The STL to display: q8bot.stl if present, else any other in the folder."""
    folder = Path(asset_dir) if asset_dir else ASSET_DIR
    if not folder.is_dir():
        return None
    preferred = folder / PREFERRED_NAME
    if preferred.is_file():
        return preferred
    return next(iter(sorted(folder.glob('*.stl'))), None)


def load_stl(path):
    """
    Read an STL into (vertices, faces) arrays.

    Handles both binary and ASCII. Binary is the common case and is a fixed
    50-byte record per triangle, so it needs no parser to speak of; ASCII is
    slower but rare enough not to matter.

    Returns:
        (vertices Nx3 float32, faces Mx3 int32)
    """
    data = Path(path).read_bytes()

    if _is_ascii_stl(data):
        points = [
            [float(n) for n in line.split()[1:4]]
            for line in data.decode('utf-8', 'replace').splitlines()
            if line.strip().startswith('vertex')
        ]
        vertices = np.array(points, dtype=np.float32)
    else:
        triangle_count = struct.unpack('<I', data[80:84])[0]
        records = np.frombuffer(data, dtype=np.uint8, count=triangle_count * 50, offset=84)
        records = records.reshape(triangle_count, 50)
        # Each record is normal(3f) + 3 vertices(3f each) + attribute(uint16);
        # skip the normal and the trailing attribute, keep the 9 floats.
        vertices = records[:, 12:48].copy().view(np.float32).reshape(-1, 3)

    faces = np.arange(len(vertices), dtype=np.int32).reshape(-1, 3)
    return vertices, faces


def _is_ascii_stl(data):
    # A binary STL's size is exactly the header plus its declared triangles;
    # checking that beats trusting the "solid" prefix, which binary files
    # written by some CAD tools also carry.
    if len(data) < 84:
        return True
    triangle_count = struct.unpack('<I', data[80:84])[0]
    return len(data) != 84 + triangle_count * 50


class ViewerPanel(QGroupBox):
    """Orbits the robot model. Click-drag to rotate, scroll to zoom."""

    def __init__(self, parent=None):
        super().__init__("", parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.view = None
        model_path = find_model()

        if model_path is None:
            layout.addWidget(self._placeholder(
                "Interactive 3D Viewer\n\n"
                f"Drop a .stl into {ASSET_DIR} to show the robot here."))
            return

        try:
            widget = self._build_view(model_path)
        except Exception as exc:
            Q8Logger.error(f"Could not show {model_path.name}: {exc}")
            layout.addWidget(self._placeholder(
                f"Could not load {model_path.name}\n\n{exc}"))
            return

        layout.addWidget(widget)

    @staticmethod
    def _placeholder(text):
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setWordWrap(True)
        return label

    def _build_view(self, model_path):
        # Imported here so a missing/broken OpenGL stack degrades to the
        # message above instead of stopping the whole window from opening.
        import pyqtgraph.opengl as gl

        vertices, faces = load_stl(model_path)

        # Centre on the model's bounding box and frame it, so any export
        # lands in view whatever its origin or units.
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        centre = (low + high) / 2
        size = float(np.linalg.norm(high - low))

        view = gl.GLViewWidget()
        view.setBackgroundColor(BACKGROUND)
        view.setCameraPosition(distance=size * 1.6, elevation=22, azimuth=45)

        mesh = gl.GLMeshItem(
            meshdata=gl.MeshData(vertexes=vertices - centre, faces=faces),
            smooth=False,
            shader=_shaded_from_above(),
            color=MODEL_COLOR,
        )
        view.addItem(mesh)

        grid = gl.GLGridItem(color=GRID_COLOR)
        grid.setSize(size * 1.4, size * 1.4)
        grid.setSpacing(size / 12, size / 12)
        grid.translate(0, 0, -(high[2] - low[2]) / 2)
        view.addItem(grid)

        self.view = view
        Q8Logger.info(f"Loaded {model_path.name} ({len(faces):,} triangles).")
        return view
