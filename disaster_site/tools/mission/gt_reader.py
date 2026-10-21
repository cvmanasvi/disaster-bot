"""Gazebo ground-truth pose of burger_cam -> gt.csv (sim_t, x, y, yaw)."""
import json, math, subprocess, sys
out = open(sys.argv[1] + '/gt.csv', 'w'); out.write('t,x,y,yaw\n'); last = -1.0
p = subprocess.Popen(['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info', '--json-output'],
                     stdout=subprocess.PIPE, text=True, bufsize=1)
for line in p.stdout:
    try:
        m = json.loads(line)
    except ValueError:
        continue
    st = m.get('header', {}).get('stamp', {})
    t = float(st.get('sec', 0)) + float(st.get('nsec', 0)) * 1e-9
    for ps in m.get('pose', []):
        if ps.get('name') != 'burger_cam':
            continue
        if t - last < 0.05:
            break
        o = ps.get('orientation', {}); x, y, z, w = (float(o.get(k, 0.0)) for k in ('x', 'y', 'z', 'w'))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)); pos = ps.get('position', {})
        out.write('%.4f,%.4f,%.4f,%.6f\n' % (t, float(pos.get('x', 0)), float(pos.get('y', 0)), yaw)); out.flush(); last = t
        break
