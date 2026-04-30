import sys
import queue
import signal
import threading
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from flask import Flask, jsonify, request
from rclpy.node import Node
from sensor_msgs.msg import Image
from waitress import serve
from typing import Optional


class ImagePub(Node):
    def __init__(self, node_name: str, is_test: bool, path: Optional[str]):
        super().__init__(node_name)
        # Use an absolute topic name to make the published topic explicit.
        self.publisher = self.create_publisher(Image, '/techman_image', 10)
        self.bridge = CvBridge()

        self.con = threading.Condition()
        self.imageQ: queue.Queue[object] = queue.Queue()
        self.leaveThread = False
        self.is_test = is_test

        self.worker = threading.Thread(target=self.pub_data_thread, args=(not is_test,), daemon=True)
        self.worker.start()

        self.img = None
        self.tmr = None
        if is_test:
            self.img = cv2.imread(path) if path else None
            if self.img is None:
                raise RuntimeError(f'Failed to load test image: {path}')
            self.tmr = self.create_timer(1.0, self.publish_test_image)

    def set_image_and_notify_send(self, img):
        with self.con:
            self.imageQ.put(img)
            self.con.notify()

    def signal_handler(self, *_args):
        self.get_logger().info('Shutting down image publisher...')
        self.close_thread()
        try:
            self.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    def publish_test_image(self):
        if self.img is None:
            return
        flipped = cv2.flip(self.img, 1)
        self.set_image_and_notify_send(flipped)

    def image_publisher(self, image):
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.get_logger().info(
            f'Publishing /techman_image, queue size is {self.imageQ.qsize()}'
        )
        self.publisher.publish(msg)

    def close_thread(self):
        with self.con:
            self.leaveThread = True
            self.con.notify_all()

    def pub_data_thread(self, is_request_data: bool):
        while True:
            with self.con:
                while self.imageQ.empty() and not self.leaveThread:
                    self.con.wait()
                if self.leaveThread and self.imageQ.empty():
                    break
                item = self.imageQ.get()

            try:
                if is_request_data:
                    file2np = np.frombuffer(item, np.uint8)
                    img = cv2.imdecode(file2np, cv2.IMREAD_COLOR)
                    if img is not None:
                        self.image_publisher(img)
                    else:
                        self.get_logger().warning('Failed to decode incoming image bytes.')
                else:
                    self.image_publisher(item)
            except Exception as exc:
                self.get_logger().error(f'Failed to publish image: {exc}')

    def fake_result(self, m_method):
        if m_method == 'CLS':
            result = {
                'message': 'success',
                'result': 'NG',
                'score': 0.987,
            }
        elif m_method == 'DET':
            result = {
                'message': 'success',
                'annotations': [
                    {
                        'box_cx': 150,
                        'box_cy': 150,
                        'box_w': 100,
                        'box_h': 100,
                        'label': 'apple',
                        'score': 0.964,
                        'rotate': -45,
                    },
                    {
                        'box_cx': 550,
                        'box_cy': 550,
                        'box_w': 100,
                        'box_h': 100,
                        'label': 'car',
                        'score': 1.000,
                        'rotation': 0,
                    },
                    {
                        'box_cx': 350,
                        'box_cy': 350,
                        'box_w': 150,
                        'box_h': 150,
                        'label': 'mobilephone',
                        'score': 0.886,
                        'rotation': 135,
                    },
                ],
                'result': None,
            }
        else:
            result = {
                'message': 'no method',
                'result': None,
            }
        return result

    def get_none(self):
        print(f"\n[{request.environ['REMOTE_ADDR']}] [{datetime.now()}] -> Get()")
        return jsonify({'result': 'api', 'message': 'running'})

    def get(self, m_method):
        print(f"\n[{request.environ['REMOTE_ADDR']}] [{datetime.now()}] -> Get({m_method})")
        if m_method == 'status':
            result = {'result': 'status', 'message': 'im ok'}
        else:
            result = {'result': 'fail', 'message': 'wrong request'}
        return jsonify(result)

    def post(self, m_method):
        print(f"\n[{request.environ['REMOTE_ADDR']}] [{datetime.now()}] -> Post({m_method})")
        model_id = request.args.get('model_id')
        print(f'model_id: {model_id}')

        if model_id is None:
            print('model_id is not set')
            return jsonify({'message': 'fail', 'result': 'model_id required'})

        if 'file' not in request.files:
            return jsonify({'message': 'fail', 'result': 'file required'})

        self.set_image_and_notify_send(request.files['file'].read())
        result = self.fake_result(m_method)
        return jsonify(result)


def set_route(app: Flask, node: ImagePub):
    app.route('/api/<string:m_method>', methods=['POST'])(node.post)
    app.route('/api/<string:m_method>', methods=['GET'])(node.get)
    app.route('/api', methods=['GET'])(node.get_none)


def main():
    rclpy.init(args=None)
    is_test = False
    app = Flask(__name__)

    if is_test:
        if len(sys.argv) < 2:
            print('Usage: image_pub.py <image_path>')
            return
        node = ImagePub('image_pub', True, sys.argv[1])
    else:
        node = ImagePub('image_pub', False, None)
        set_route(app, node)

    signal.signal(signal.SIGINT, node.signal_handler)

    if not is_test:
        server_thread = threading.Thread(
            target=lambda: serve(app, host='0.0.0.0', port=6189),
            daemon=True,
        )
        server_thread.start()
        node.get_logger().info('Listening on 0.0.0.0:6189 and publishing to /techman_image')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_thread()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
