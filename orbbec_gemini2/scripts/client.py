#!/usr/bin/env python3
"""
Interactive test client for initializev2.py.

Mimics what the real GUI's QTcpSocket does: connect once, then send one
JSON control message per line for as long as the connection stays open.
This exercises the same "wait to receive data" loop the diagram shows,
across multiple state changes on a single connection.

Usage:
    python3 client.py [--host 127.0.0.1] [--port 6000]

At the prompt:
    play back                  -> play with default settings
    play back 640x360 15 300   -> play with res, fps, bitrate overrides
    pause back
    kill back
    quit
"""

import argparse
import json
import socket


def build_message(parts):
    state = parts[0]
    camera = parts[1]
    message = {"state": state, "camera": camera}
    if len(parts) > 2:
        message["res"] = parts[2]
    if len(parts) > 3:
        message["fps"] = int(parts[3])
    if len(parts) > 4:
        message["bitrate"] = int(parts[4])
    return message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.1.7")
    parser.add_argument("--port", type=int, default=6000)
    args = parser.parse_args()

    print(f"Connecting to {args.host}:{args.port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((args.host, args.port))
    print("Connected.")
    print("Commands: play <camera> [res] [fps] [bitrate] | pause <camera> | kill <camera> | quit")

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue
            if line == "quit":
                break

            parts = line.split()
            try:
                message = build_message(parts)
            except (IndexError, ValueError):
                print("Couldn't parse that. Format: <state> <camera> [res] [fps] [bitrate]")
                continue

            payload = (json.dumps(message) + "\n").encode("utf-8")
            sock.sendall(payload)
            print(f"Sent: {message}")
    finally:
        sock.close()
        print("Connection closed.")


if __name__ == "__main__":
    main()
