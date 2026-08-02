import sys
import socket
import selectors
import types

import json

from usbStreamerClass import CameraPipelineStreamer, CAMERAS_DEFAULTS
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

#HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
#PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

"""
NOTE: if testing on local host, you must set the IP of both srt uri's to 127.0.0.1 !
"""

"""
packet = {
    "state" : 
    "camera" :

    "width" :
    "height" :
    "fps" :
    bitrate" :
}
"""

def handle_client_command(commandPacket, ip):
    command = commandPacket.get("state")
    camera_selection = commandPacket.get("camera")
    width = commandPacket.get("width")
    height = commandPacket.get("height")
    fps = commandPacket.get("fps")
    bitrate = commandPacket.get("bitrate")

    if command not in ['play', 'stop']:
        print(f"Invalid command: {command}")
        return

    if camera_selection not in CAMERAS_DEFAULTS:
        print(f"Invalid camera selection: {camera_selection}")
        return

    streamer = camera_pipelines.get(camera_selection)

    if command == 'stop':
        if streamer is not None:
            streamer.stop_pipeline()
            camera_pipelines.pop(camera_selection)
            print(f"Stopped pipeline for camera: {camera_selection}")
        else:
            print(f"No active pipeline for camera: {camera_selection}")

    elif command == 'play':
        if streamer is None:
            camera_pipelines[camera_selection] = CameraPipelineStreamer(camera_selection, latency=200)
            streamer = camera_pipelines[camera_selection]

        if width and height:
            streamer.set_width(width)
            streamer.set_height(height)

        if fps:
            streamer.set_framerate(fps)

        if bitrate:
            streamer.set_bitrate(bitrate)

        try:
            streamer.start_pipeline()
            print(f"Started/Updated pipeline for camera: {camera_selection}")
        except RuntimeError as e:
            print(e)


def accept_wrapper(sock):
    conn, addr = sock.accept()  # Should be ready to read
    print(f"Accepted connection from {addr}")
    conn.setblocking(False)
    data = types.SimpleNamespace(addr=addr, inb=b"", outb=b"")
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    sel.register(conn, events, data=data)


def service_connection(key, mask):
    sock = key.fileobj
    data = key.data
    if mask & selectors.EVENT_READ:
        recv_data = sock.recv(4096)  # Should be ready to read
        if recv_data:
            #data.outb += recv_data
            try:
                commandPacket = json.loads(recv_data.decode("utf-8").strip())
                print(f"Received command: {commandPacket}")
                handle_client_command(commandPacket, data.addr[0])
            except json.JSONDecodeError:
                print(f"Malformed message.")
                return {}
        else:
            print(f"Closing connection to {data.addr}")
            sel.unregister(sock)
            sock.close()
    if mask & selectors.EVENT_WRITE:
        if data.outb:
            print(f"Echoing {data.outb!r} to {data.addr}")
            sent = sock.send(data.outb)  # Should be ready to write
            data.outb = data.outb[sent:]


def quit():
    print("Quitting server.")
    for key in sel.get_map().values():
        sock = key.fileobj
        sel.unregister(sock)
        sock.close()
    sys.exit(0)



sel = selectors.DefaultSelector()

# IP and PORT are from command line arguments
host, port = sys.argv[1], int(sys.argv[2])

# TCP socket setup - server
lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
lsock.bind((host, port))
lsock.listen() # listen for requests
print(f"Listening on {(host, port)}")
lsock.setblocking(False) # don't block any client connection requests

# monitor the socket for incoming connections
sel.register(lsock, selectors.EVENT_READ, data=None)

# set up gstreamer pipelin variables
Gst.init(None)

camera_pipelines = {}

try:
    while True:
        events = sel.select(timeout=None) # blocks until socket is ready
        for key, mask in events:
            # from listening socket/port, accept the connection and register it with the selector
            if key.data is None:
                accept_wrapper(key.fileobj)
            # existing client conenction, proces it
            else:
                service_connection(key, mask)
except KeyboardInterrupt:
    print("Caught keyboard interrupt, exiting")
finally:
    sel.close()

