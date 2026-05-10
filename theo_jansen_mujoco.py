"""
jansen_mujoco.py — Theo Jansen Mechanism in MuJoCo (kinematic replay)
======================================================================
Every joint AND every link is a separate mocap body, so both joints
and links move correctly each frame. Link bodies are repositioned and
reoriented analytically — no constraint solver involved.

Outputs
-------
  jansen_mujoco.gif   — animated GIF (matplotlib renderer, always works)
  jansen_mujoco.png   — static composite figure
  jansen_mujoco_3d.gif — MuJoCo off-screen render (if OpenGL available)

And opens an interactive MuJoCo viewer window.
"""

import os
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import imageio.v2 as imageio

# ---------------------------------------------------------------------------
# Kinematics — Jansen holy numbers, circle-circle intersection
# ---------------------------------------------------------------------------
LINKS = {
    'a': 38.0,  'b': 41.5,  'c': 39.3,  'd': 40.1,
    'e': 55.8,  'f': 39.4,  'g': 36.7,  'h': 65.7,
    'i': 49.0,  'j': 50.0,  'k': 61.9,  'l': 7.8,
    'm': 15.0,
}
MM2M = 1e-3


def cci(c1, r1, c2, r2, branch='+'):
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    dv = c2 - c1
    d = np.linalg.norm(dv)
    if d < 1e-12 or d > r1 + r2 or d < abs(r1 - r2):
        return None
    p = (r1*r1 - r2*r2 + d*d) / (2*d)
    h2 = r1*r1 - p*p
    if h2 < 0:
        return None
    h = np.sqrt(max(h2, 0.0))
    mid = c1 + p * dv / d
    perp = np.array([-dv[1]/d, dv[0]/d])
    return mid + h*perp if branch == '+' else mid - h*perp


def solve_pose_2d(theta, L=None):
    if L is None:
        L = LINKS
    a, b, c, d_ = L['a'], L['b'], L['c'], L['d']
    e, f, g, h_ = L['e'], L['f'], L['g'], L['h']
    i_, j, k    = L['i'], L['j'], L['k']
    l, m        = L['l'], L['m']

    O = np.array([0.0, 0.0])
    P = np.array([-a, -l])
    C = O + m * np.array([-np.cos(theta), np.sin(theta)])

    B_up = cci(C, j, P, b, '-')
    if B_up is None: return None
    D_up = cci(B_up, c, P, d_, '-')
    if D_up is None: return None
    B_lo = cci(C, e, P, f, '+')
    if B_lo is None: return None
    D_lo = cci(B_lo, g, D_up, h_, '+')
    if D_lo is None: return None
    F = cci(B_lo, i_, D_lo, k, '+')
    if F is None: return None

    return {'O': O, 'P': P, 'C': C,
            'B_up': B_up, 'D_up': D_up,
            'B_lo': B_lo, 'D_lo': D_lo,
            'F': F}


# ---------------------------------------------------------------------------
# Link / joint definitions
# ---------------------------------------------------------------------------
JOINT_NAMES = ['O', 'P', 'C', 'B_up', 'D_up', 'B_lo', 'D_lo', 'F']

LINK_EDGES = [
    ('O',    'C',    [0.90, 0.20, 0.10, 1]),
    ('O',    'P',    [0.15, 0.45, 0.85, 1]),
    ('C',    'B_up', [1.00, 0.65, 0.00, 1]),
    ('P',    'B_up', [0.10, 0.70, 0.85, 1]),
    ('B_up', 'D_up', [0.18, 0.63, 0.18, 1]),
    ('P',    'D_up', [0.58, 0.30, 0.75, 1]),
    ('C',    'B_lo', [0.91, 0.12, 0.39, 1]),
    ('P',    'B_lo', [1.00, 0.65, 0.00, 1]),
    ('B_lo', 'D_lo', [0.90, 0.20, 0.10, 1]),
    ('D_up', 'D_lo', [0.10, 0.70, 0.85, 1]),
    ('B_lo', 'F',    [0.18, 0.63, 0.18, 1]),
    ('D_lo', 'F',    [0.10, 0.70, 0.85, 1]),
]


def link_body_name(idx):
    return 'link_body_%d' % idx


# ---------------------------------------------------------------------------
# Link pose: midpoint + quaternion that rotates MuJoCo capsule (Z-axis)
# to align with the direction pa -> pb in the XY plane.
#
# MuJoCo capsule default axis: Z.
# We need to rotate Z -> (dx, dy, 0) in XY.
# Steps:  q = Rz(angle) * Ry(90deg)
#   Ry(90): rotates Z -> X
#   Rz(angle): rotates X -> link direction
# ---------------------------------------------------------------------------
def _qmul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def compute_link_pose(pa, pb):
    """
    pa, pb : (x, y) in metres.
    Returns (midpoint_xyz, quaternion_wxyz, half_length).
    """
    dx = pb[0] - pa[0]
    dy = pb[1] - pa[1]
    length = np.sqrt(dx*dx + dy*dy)
    mid = np.array([(pa[0]+pb[0])/2, (pa[1]+pb[1])/2, 0.0])
    angle = np.arctan2(dy, dx)
    # Ry(90°): w=cos45, x=0, y=sin45, z=0
    qy90 = np.array([np.cos(np.pi/4), 0.0, np.sin(np.pi/4), 0.0])
    # Rz(angle): w=cos(a/2), x=0, y=0, z=sin(a/2)
    qz   = np.array([np.cos(angle/2), 0.0, 0.0, np.sin(angle/2)])
    q = _qmul(qz, qy90)
    return mid, q, length / 2.0


# ---------------------------------------------------------------------------
# MuJoCo XML — all bodies are mocap so we can drive them kinematically
# ---------------------------------------------------------------------------
def build_xml(first_pose_mm):
    """Build XML from one reference pose (dict of joint_name -> (x,y) in mm)."""
    joint_r = 0.005   # sphere radius m
    link_r  = 0.0022  # capsule radius m

    def fmt3(xyz):
        return '%.6f %.6f %.6f' % (xyz[0], xyz[1], xyz[2] if len(xyz) > 2 else 0.0)

    def fmtq(q):
        return '%.6f %.6f %.6f %.6f' % tuple(q)

    def fmtc(c):
        return '%.2f %.2f %.2f %.2f' % tuple(c)

    lines = [
        '<mujoco model="jansen">',
        '  <option gravity="0 0 0" timestep="0.002"/>',
        '  <visual>',
        '    <rgba haze="0.15 0.25 0.35 1"/>',
        '    <quality shadowsize="2048"/>',
        '    <global offheight="800" offwidth="800"/>',
        '  </visual>',
        '  <worldbody>',
        '    <light pos="0 0 3" dir="0 0 -1" diffuse="1 1 1"/>',
        '    <light pos="0.2 0 1" dir="-0.1 0 -1" diffuse="0.6 0.6 0.6"/>',
        '    <camera name="main" pos="0 0 2.5" xyaxes="1 0 0 0 1 0"/>',
    ]

    # Joint bodies (spheres)
    for name in JOINT_NAMES:
        xy = first_pose_mm[name] * MM2M
        col = '0.05 0.05 0.05 1' if name in ('O', 'P') else '0.95 0.95 0.20 1'
        pos = '%.6f %.6f 0.000000' % (xy[0], xy[1])
        lines.append(
            '    <body name="%s" pos="%s" mocap="true">'
            '<geom type="sphere" size="%.4f" rgba="%s"/>'
            '</body>' % (name, pos, joint_r, col)
        )

    # Link bodies (capsules)
    for idx, (a, b, color) in enumerate(LINK_EDGES):
        pa = first_pose_mm[a] * MM2M
        pb = first_pose_mm[b] * MM2M
        mid, q, half = compute_link_pose(pa, pb)
        bname = link_body_name(idx)
        lines.append(
            '    <body name="%s" pos="%s" quat="%s" mocap="true">'
            '<geom type="capsule" size="%.4f %.6f" rgba="%s"/>'
            '</body>' % (bname, fmt3(mid), fmtq(q), link_r, half, fmtc(color))
        )

    lines += ['  </worldbody>', '</mujoco>']
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Update all mocap bodies for a given pose
# ---------------------------------------------------------------------------
def make_mocap_lookup(model, mujoco):
    """Return dicts: joint_name -> mocap_id, link_idx -> mocap_id."""
    jnt = {}
    for name in JOINT_NAMES:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid >= 0:
            mid = model.body_mocapid[bid]
            if mid >= 0:
                jnt[name] = mid

    lnk = {}
    for idx in range(len(LINK_EDGES)):
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link_body_name(idx))
        if bid >= 0:
            mid = model.body_mocapid[bid]
            if mid >= 0:
                lnk[idx] = mid

    return jnt, lnk


def apply_pose(data, pose_mm, jnt_ids, lnk_ids):
    """Write joint positions and link poses into data.mocap_pos/quat."""
    for name, mid in jnt_ids.items():
        xy = pose_mm[name] * MM2M
        data.mocap_pos[mid] = [xy[0], xy[1], 0.0]

    for idx, (a, b, _) in enumerate(LINK_EDGES):
        if idx not in lnk_ids:
            continue
        pa = pose_mm[a] * MM2M
        pb = pose_mm[b] * MM2M
        mid_xyz, q, _ = compute_link_pose(pa, pb)
        data.mocap_pos[lnk_ids[idx]]  = mid_xyz
        data.mocap_quat[lnk_ids[idx]] = q


# ---------------------------------------------------------------------------
# Matplotlib frame renderer
# ---------------------------------------------------------------------------
def render_frame_mpl(pose_mm, foot_history_mm, theta_deg, fig_size=(8, 8)):
    fig, ax = plt.subplots(figsize=fig_size, dpi=100)
    fig.patch.set_facecolor('#0d1117')
    ax.set_facecolor('#0d1117')

    if len(foot_history_mm) > 1:
        fh = np.array(foot_history_mm)
        ax.plot(fh[:, 0], fh[:, 1], '-', color='magenta', lw=1.8, alpha=0.7, zorder=2)

    for (a, b, color) in LINK_EDGES:
        pa, pb = pose_mm[a], pose_mm[b]
        ax.plot([pa[0], pb[0]], [pa[1], pb[1]], '-',
                color=color[:3], lw=5, solid_capstyle='round', zorder=3, alpha=0.92)

    for name in JOINT_NAMES:
        pt = pose_mm[name]
        m = 's' if name in ('O', 'P') else 'o'
        ms = 10 if name in ('O', 'P') else 6
        ax.plot(pt[0], pt[1], m, color='white', ms=ms, zorder=6,
                markeredgecolor='#444', markeredgewidth=0.8)
        ax.text(pt[0]+3, pt[1]+3, name, color='white', fontsize=7, zorder=7, alpha=0.75)

    F = pose_mm['F']
    ax.plot(F[0], F[1], 'D', color='magenta', ms=9, zorder=8)

    ax.set_aspect('equal')
    ax.set_title('Theo Jansen Mechanism  —  crank = %.1f°' % theta_deg,
                 color='white', fontsize=12, pad=10)
    ax.tick_params(colors='#888')
    for spine in ax.spines.values():
        spine.set_edgecolor('#444')
    ax.set_xlabel('X (mm)', color='#aaa')
    ax.set_ylabel('Y (mm)', color='#aaa')
    ax.grid(alpha=0.15, color='#555')

    legend_items = [
        mpatches.Patch(color=[0.90, 0.20, 0.10], label='crank (m)'),
        mpatches.Patch(color=[0.15, 0.45, 0.85], label='ground (a-l)'),
        mpatches.Patch(color=[0.10, 0.70, 0.85], label='connectors'),
        mpatches.Patch(color=[0.18, 0.63, 0.18], label='foot links'),
    ]
    ax.legend(handles=legend_items, loc='upper right',
              facecolor='#1a1f2b', edgecolor='#444', labelcolor='white', fontsize=8)

    fig.canvas.draw()
    img = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
    plt.close(fig)
    return img


# ---------------------------------------------------------------------------
# Static composite figure
# ---------------------------------------------------------------------------
def plot_static(thetas, poses_mm, out_path):
    foot = np.array([p['F'] for p in poses_mm if p is not None])
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7), facecolor='#0d1117')
    for ax in (ax1, ax2):
        ax.set_facecolor('#0d1117')
        ax.tick_params(colors='#888')
        for sp in ax.spines.values():
            sp.set_edgecolor('#444')

    ax1.plot(foot[:, 0], foot[:, 1], color='magenta', lw=2.5, label='Foot tip (F)')
    y_med = np.median(foot[:, 1])
    stance = foot[:, 1] < y_med
    ax1.plot(foot[stance, 0], foot[stance, 1], color='red', lw=5, alpha=0.45, label='Stance phase')
    ax1.plot(foot[0, 0], foot[0, 1], 'go', ms=10, label='Start')
    stride, height = np.ptp(foot[:, 0]), np.ptp(foot[:, 1])
    info = 'Stride: %.1f mm\nStep height: %.1f mm\nRatio: %.2f' % (stride, height, stride/height)
    ax1.text(0.03, 0.97, info, transform=ax1.transAxes, va='top', color='white', fontsize=10,
             bbox=dict(boxstyle='round', fc='#1a1f2b', ec='#555', alpha=0.9))
    ax1.set_aspect('equal')
    ax1.grid(alpha=0.15, color='#555')
    ax1.set_xlabel('X (mm)', color='#aaa')
    ax1.set_ylabel('Y (mm)', color='#aaa')
    ax1.set_title('Foot Trajectory — One Full Crank Cycle', color='white', fontsize=12)
    ax1.legend(facecolor='#1a1f2b', edgecolor='#444', labelcolor='white', fontsize=9)

    idx = int(np.argmin(np.abs(thetas - np.pi/2)))
    pose = poses_mm[idx]
    for (a, b, color) in LINK_EDGES:
        pa, pb = pose[a], pose[b]
        ax2.plot([pa[0], pb[0]], [pa[1], pb[1]], '-',
                 color=color[:3], lw=5, solid_capstyle='round', alpha=0.92, zorder=3)
    for name in JOINT_NAMES:
        pt = pose[name]
        mk = 's' if name in ('O', 'P') else 'o'
        ax2.plot(pt[0], pt[1], mk, color='white', ms=9, zorder=6,
                 markeredgecolor='#444', markeredgewidth=0.8)
        ax2.text(pt[0]+3, pt[1]+3, name, color='white', fontsize=8, zorder=7)
    ax2.plot(foot[:, 0], foot[:, 1], color='magenta', lw=1.2, alpha=0.35, zorder=1)
    ax2.plot(pose['F'][0], pose['F'][1], 'D', color='magenta', ms=10, zorder=8)
    ax2.set_aspect('equal')
    ax2.grid(alpha=0.15, color='#555')
    ax2.set_xlabel('X (mm)', color='#aaa')
    ax2.set_ylabel('Y (mm)', color='#aaa')
    ax2.set_title('Mechanism Pose at Crank = 90°', color='white', fontsize=12)

    plt.tight_layout(pad=2)
    plt.savefig(out_path, dpi=130, bbox_inches='tight', facecolor='#0d1117')
    plt.close()
    print('  saved: %s' % out_path)


# ---------------------------------------------------------------------------
# MuJoCo off-screen GIF
# ---------------------------------------------------------------------------
def try_mujoco_render(poses_mm, n_frames=60, out_path=None):
    try:
        import mujoco
    except ImportError:
        print('  MuJoCo not importable.')
        return None

    first = next(p for p in poses_mm if p is not None)
    xml = build_xml(first)

    try:
        model = mujoco.MjModel.from_xml_string(xml)
        data  = mujoco.MjData(model)
    except Exception as ex:
        print('  MuJoCo model build failed: %s' % ex)
        return None

    try:
        renderer = mujoco.Renderer(model, height=480, width=480)
    except Exception as ex:
        print('  MuJoCo renderer unavailable (%s)' % ex)
        return None

    jnt_ids, lnk_ids = make_mocap_lookup(model, mujoco)

    frames = []
    step = max(1, len(poses_mm) // n_frames)
    for pose in poses_mm[::step][:n_frames]:
        if pose is None:
            continue
        apply_pose(data, pose, jnt_ids, lnk_ids)
        mujoco.mj_step(model, data)
        renderer.update_scene(data, camera='main')
        frames.append(renderer.render())

    renderer.close()
    return frames


# ---------------------------------------------------------------------------
# Interactive viewer
# ---------------------------------------------------------------------------
def launch_interactive_viewer(poses_mm, thetas):
    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        print('  MuJoCo not available.')
        return

    first = next(p for p in poses_mm if p is not None)
    xml = build_xml(first)

    try:
        model = mujoco.MjModel.from_xml_string(xml)
        data  = mujoco.MjData(model)
    except Exception as ex:
        print('  Model build failed: %s' % ex)
        return

    jnt_ids, lnk_ids = make_mocap_lookup(model, mujoco)
    apply_pose(data, first, jnt_ids, lnk_ids)
    mujoco.mj_step(model, data)

    # Compute centre of mechanism for camera lookat
    all_xy = np.array([first[n] for n in JOINT_NAMES]) * MM2M
    cx, cy = all_xy[:, 0].mean(), all_xy[:, 1].mean()

    print('  Viewer open — rotate/zoom with mouse. Close window to exit.')

    n = len(poses_mm)
    frame_idx = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [cx, cy, 0.0]
        viewer.cam.distance  = 0.45
        viewer.cam.elevation = -5
        viewer.cam.azimuth   = 0

        while viewer.is_running():
            pose = poses_mm[frame_idx % n]
            if pose is not None:
                apply_pose(data, pose, jnt_ids, lnk_ids)
                mujoco.mj_step(model, data)
            viewer.sync()
            frame_idx += 1
            time.sleep(0.025)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print('=' * 60)
    print('Theo Jansen Mechanism — MuJoCo Kinematic Replay')
    print('=' * 60)

    n_samples = 360
    thetas = np.linspace(0, 2*np.pi, n_samples, endpoint=False)

    print('\n[1/3] Solving kinematics (%d poses)...' % n_samples)
    poses_mm = [solve_pose_2d(th) for th in thetas]
    valid_n = sum(p is not None for p in poses_mm)
    print('      Valid: %d/%d' % (valid_n, n_samples))

    foot_all = np.array([p['F'] for p in poses_mm if p is not None])
    stride, height = np.ptp(foot_all[:, 0]), np.ptp(foot_all[:, 1])
    print('      Stride: %.1f mm   Step height: %.1f mm' % (stride, height))

    out_dir = os.path.dirname(os.path.abspath(__file__))

    print('\n[2/3] Saving static figure...')
    plot_static(thetas, poses_mm, os.path.join(out_dir, 'jansen_mujoco.png'))

    print('\n[3/3] Rendering matplotlib animation...')
    n_frames = 80
    step = max(1, n_samples // n_frames)
    foot_history = []
    gif_frames = []
    for th, pose in zip(thetas[::step][:n_frames], poses_mm[::step][:n_frames]):
        if pose is None:
            continue
        foot_history.append(pose['F'].copy())
        gif_frames.append(render_frame_mpl(pose, foot_history, np.degrees(th)))

    gif_path = os.path.join(out_dir, 'jansen_mujoco.gif')
    imageio.mimsave(gif_path, gif_frames, duration=0.06, loop=0)
    print('      saved: %s  (%d frames)' % (gif_path, len(gif_frames)))

    print('\n[bonus] Attempting MuJoCo off-screen render...')
    mj_frames = try_mujoco_render(poses_mm, n_frames=60)
    if mj_frames:
        mj_gif = os.path.join(out_dir, 'jansen_mujoco_3d.gif')
        imageio.mimsave(mj_gif, mj_frames, duration=0.07, loop=0)
        print('         saved: %s' % mj_gif)
    else:
        print('         (skipped)')

    print('\n[viewer] Launching interactive MuJoCo window...')
    launch_interactive_viewer(poses_mm, thetas)

    print('\nDone.')
    print('  jansen_mujoco.png  — static figure')
    print('  jansen_mujoco.gif  — matplotlib animation')


if __name__ == '__main__':
    main()
