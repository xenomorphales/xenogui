from time import sleep
import json
import os

import rclpy

from std_msgs.msg import String

import tornado.ioloop
import tornado.web

class BaseHandler(tornado.web.RequestHandler):
    node = None

class NodesHandler(BaseHandler):
    def get(self):
        self.write(json.dumps(BaseHandler.node.get_node_names()))

class TopicsHandler(BaseHandler):
    def get(self):
        self.write(json.dumps(BaseHandler.node.get_topic_names_and_types()))

def make_app():
    client_path = os.getenv('XENOGUI_CLIENT', ".")
    index_path = os.path.join(client_path, "index.html")
    api_handlers = [
        (r"/nodes", NodesHandler),
        (r"/topics", TopicsHandler),
    ]
    web_handlers = [
        (r"/()", tornado.web.StaticFileHandler, {'path': index_path}),
        (r"/(.*)", tornado.web.StaticFileHandler, {'path': client_path}),
    ]
    if not os.path.isfile(index_path):
        print("WARNING : " + index_path + " file not found")
        return tornado.web.Application(api_handlers)
    return tornado.web.Application(api_handlers + web_handlers)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('xenogui')
    BaseHandler.node = node
    
    app = make_app()
    app.listen(8888)
    tornado.ioloop.IOLoop.current().start()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
