import cv2
import numpy as np
import time
from libxrobot import *
from teaching_task import *
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import simplejson
import threading

class HTTPRequestHandler(BaseHTTPRequestHandler):
	def _set_headers(self):
		self.send_response(200)
		self.send_header('Content-type', 'application/json')
		self.end_headers()

	def do_GET(self):
		pos_json = self.server.fetch_handler()
		# self._set_headers()
		self.wfile.write(pos_json)

	def do_POST(self):
		# self._set_headers()
		self.wfile.write("Update")
		content_len = int(self.headers.getheader('content-length', 0))
		post_body = self.rfile.read(content_len)
		data = simplejson.loads(post_body)

		uid = data['UID']
		pos_x = data['Position']['x']
		pos_y = data['Position']['y']
		pos_z = data['Position']['z']
		orn_x = data['Orientation']['x']
		orn_y = data['Orientation']['y']
		orn_z = data['Orientation']['z']
		orn_w = data['Orientation']['w']

		self.server.update_handler(uid, [pos_x, pos_y, pos_z], \
								 [orn_x, orn_y, orn_z, orn_w])

class XHTTPServer(HTTPServer):
	def __init__(self, address, RequestHandlerClass, \
			update_handler, fetch_handler):
		HTTPServer.__init__(self, address,RequestHandlerClass)
		self.update_handler = update_handler
		self.fetch_handler = fetch_handler

class XWorldServer:
	def __init__(self, host, port, env):
		self.update_handler = self.update;
		self.fetch_handler = self.fetch;
		self.host = host
		self.port = port
		self.server = None
		self.thread = None
		self.env = env
		
	def fetch(self):
		agent = self.env.env.GetAgent()
		a_pos = agent.GetPosition()

		pos = dict()
		pos['x'] = a_pos[0]
		pos['y'] = a_pos[1]
		pos['z'] = a_pos[2]
		pos_json = simplejson.dumps(pos)

		return pos_json

	def update(self, uid, pos, orn):
		if uid != 0:
			scene = self.env.env.GetXWorldScene()
			self.env.env.Teleport(scene[uid], pos, orn)

	def start(self):
		self.server = XHTTPServer((self.host, self.port), HTTPRequestHandler, \
			self.update, self.fetch)
		self.thread = threading.Thread(target = self.server.serve_forever)
		self.thread.daemon = True
		self.thread.start()

	def stop(self):
		if self.server:
			self.server.shutdown()
