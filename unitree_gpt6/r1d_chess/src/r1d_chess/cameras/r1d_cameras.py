"""Subscribe only to requested R1-D streams; every capture uses new SUB sockets."""
from datetime import datetime
from pathlib import Path
import time
import cv2
import numpy as np
import zmq
from r1d_chess.robot.r1d_config import load_profile

PORTS = {'cam_left_high': 55555, 'cam_left_wrist': 55556, 'cam_right_wrist': 55557}


def capture(host, cameras, output_dir, *, timeout=8, profile=None):
    if not cameras or len(set(cameras)) != len(cameras) or not set(cameras) <= set(PORTS):
        raise ValueError('invalid selected cameras')
    p = load_profile() if profile is None else profile
    root = Path(output_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    paths, meta, sockets = {}, {}, {}
    context = zmq.Context()
    poller = zmq.Poller()
    try:
        for name in cameras:
            sock = context.socket(zmq.SUB)
            sock.setsockopt(zmq.LINGER, 0)
            sock.setsockopt(zmq.SUBSCRIBE, b'')
            sock.setsockopt(zmq.CONFLATE, 1)
            sock.connect(f'tcp://{host}:{PORTS[name]}')
            poller.register(sock, zmq.POLLIN)
            sockets[sock] = name
        end = time.monotonic() + timeout
        while sockets and time.monotonic() < end:
            for sock, _ in poller.poll(100):
                raw = sock.recv()
                image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    continue
                name = sockets[sock]
                expected = (480, 1280) if name == 'cam_left_high' else (480, 640)
                if image.shape[:2] != expected:
                    raise ValueError(f'{name} unexpected image shape {image.shape}')
                now = datetime.now().astimezone()
                directory = root / ('observation.images.' + name)
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / (now.strftime('%Y%m%d_%H%M%S_%f') + '.jpg')
                if name == 'cam_left_high':
                    if p['head_eye'] not in ('left', 'right'):
                        raise ValueError('head_eye must be left or right')
                    # Legacy planner sees one eye, never a resized stereo pair.
                    eye = image[:, :640] if p['head_eye'] == 'left' else image[:, 640:]
                    if not cv2.imwrite(str(path), eye):
                        raise OSError('head image write failed')
                    stereo = path.with_name(path.stem + '_stereo.jpg')
                    stereo.write_bytes(raw)
                else:
                    path.write_bytes(raw)
                paths[name] = str(path)
                meta[name] = {'received_at': now.isoformat(), 'port': PORTS[name],
                              'source_shape': list(image.shape),
                              'head_eye': p['head_eye'] if name == 'cam_left_high' else None,
                              'freshness_scope': 'received on a new subscription; no source timestamp in raw JPEG'}
                poller.unregister(sock)
                sock.close()
                del sockets[sock]
        if sockets:
            raise TimeoutError(f'no new JPEG for {list(sockets.values())}; partial files retained in {root}')
    finally:
        for sock in sockets:
            sock.close()
        context.term()
    return {'timestamp': datetime.now().astimezone().isoformat(), 'paths': paths,
            'camera_order': cameras, 'camera_metadata': meta}
