import time
import json
import os

import rclpy

from std_msgs.msg import String

import tornado.ioloop
import tornado.web
import tornado.websocket

class BaseHandler(tornado.web.RequestHandler):
    node = None

class WebsocketHandler(tornado.websocket.WebSocketHandler):
    def sub_cb(self, tname, msg):
        print("["+tname+"]:"+str(msg))
        self.write_message(json.dumps({'event':'recv','data':{'name':tname,'msg':str(msg)}}))
    def open(self):
        print("WebSocket opened")
        self.topics = []
        for t in BaseHandler.node.get_topic_names_and_types():
            self.topics.append({'name':t[0],'type':t[1]})
        self.write_message(json.dumps({'event':'list','data':self.topics}))
        self.subs = []
        for t in self.topics:
            self.subs.append(BaseHandler.node.create_subscription(String, t['name'], lambda msg, t=t['name']: self.sub_cb(t, msg)))
    def on_message(self, message):
        self.write_message(u"You said: " + message)
    def on_close(self):
        print("WebSocket closed")
        for sub in self.subs:
            BaseHandler.node.destroy_subscription(sub)


class NodesHandler(BaseHandler):
    def get(self):
        self.write(json.dumps(BaseHandler.node.get_node_names()))

class TopicsHandler(BaseHandler):
    def get(self):
        res = []
        for t in BaseHandler.node.get_topic_names_and_types():
            res.append({'name':t[0],'type':t[1]})
        self.write(json.dumps(res))

class TopicSubscribeHandler(BaseHandler):
    def sub_cb(self, msg):
        self.res = msg
    def get(self, topic):
        self.res = None
        sub = BaseHandler.node.create_subscription(String, topic, self.sub_cb)
        t0 = time.time()
        while not self.res and time.time()-t0 < 0.5:
            rclpy.spin_once(BaseHandler.node, timeout_sec=0.1)
        BaseHandler.node.destroy_subscription(sub)
        self.write(str(self.res))

def make_app():
    client_path = os.getenv('XENOGUI_CLIENT', ".")
    index_path = os.path.join(client_path, "index.html")
    api_handlers = [
        (r"/nodes", NodesHandler),
        (r"/topics", TopicsHandler),
        (r"/topic/(.*)", TopicSubscribeHandler),
    ]
    web_handlers = [
        (r"/websocket", WebsocketHandler),
        (r"/()", tornado.web.StaticFileHandler, {'path': index_path}),
        (r"/(.*)", tornado.web.StaticFileHandler, {'path': client_path}),
    ]
    if not os.path.isfile(index_path):
        print("WARNING : " + index_path + " file not found")
        return tornado.web.Application(api_handlers)
    return tornado.web.Application(api_handlers + web_handlers)

def main(args=None):
    rclpy.init(args=args)
    BaseHandler.node = rclpy.create_node('xenogui')
    
    app = make_app()
    app.listen(8888)
    tornado.ioloop.PeriodicCallback(lambda: rclpy.spin_once(BaseHandler.node, timeout_sec=0.1), 100).start()
    tornado.ioloop.IOLoop.current().start()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
