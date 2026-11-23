"""Grab one fresh JPEG from the control server's MJPEG stream + the joint state at that moment.
usage: python3 -m robot.grab NAME  -> NAME.jpg, NAME.json"""
import json, sys, time, urllib.request
URL = 'http://127.0.0.1:8000'
def state():
    return json.load(urllib.request.urlopen(URL + '/api/state', timeout=3))
def frame(skip=4):
    r = urllib.request.urlopen(URL + '/api/camera/stream', timeout=8)
    buf, n = b'', 0
    while True:
        buf += r.read(4096)
        a = buf.find(b'\xff\xd8'); b = buf.find(b'\xff\xd9', a + 2)
        if a >= 0 and b >= 0:
            jpg, buf, n = buf[a:b + 2], buf[b + 2:], n + 1
            if n > skip:   # drop buffered frames so the image is newer than the state read below
                r.close(); return jpg
if __name__ == '__main__':
    name = sys.argv[1]
    s0 = state(); jpg = frame(); s1 = state()
    open(name + '.jpg', 'wb').write(jpg)
    json.dump({'t': time.time(), 'q_before': s0['q'], 'q_after': s1['q'], 'busy': s0['busy'] or s1['busy'],
               'gripper': s1['gripper'], 'position_m': s1.get('position_m'), 'rpy_rad': s1.get('rpy_rad')}, open(name + '.json', 'w'))
    print(name, len(jpg), 'bytes; q', [round(v, 3) for v in s1['q']], 'busy', s0['busy'] or s1['busy'])
