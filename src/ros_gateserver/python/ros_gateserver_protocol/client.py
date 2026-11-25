"""Small standard-library WebSocket client for gateway protobuf frames."""

import argparse
import base64
import hashlib
import json
import math
import os
import socket
import struct
import sys

from google.protobuf.message import DecodeError

from .codec import decode_status


class ProtobufWebSocketClient:
    """Read binary status frames from one gateway connection."""

    def __init__(self, host='127.0.0.1', port=8765, timeout=5.0):
        self.socket = socket.create_connection((host, port), timeout=timeout)
        try:
            self.handshake(host, port)
        except Exception:
            self.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        self.close()

    def exact(self, count):
        data = bytearray()
        while len(data) < count:
            chunk = self.socket.recv(count - len(data))
            if not chunk:
                raise ConnectionError('Gateway closed the WebSocket connection')
            data.extend(chunk)
        return bytes(data)

    def handshake(self, host, port):
        key = base64.b64encode(os.urandom(16)).decode()
        request = (
            f'GET / HTTP/1.1\r\nHost: {host}:{port}\r\n'
            'Upgrade: websocket\r\nConnection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n'
        )
        self.socket.sendall(request.encode())
        header = bytearray()
        while not header.endswith(b'\r\n\r\n'):
            header.extend(self.exact(1))
            if len(header) > 16384:
                raise ValueError('Oversized WebSocket handshake')
        lines = header.decode('ascii').split('\r\n')
        if not lines[0].startswith('HTTP/1.1 101 '):
            raise ValueError(f'WebSocket upgrade failed: {lines[0]}')
        fields = {}
        for line in lines[1:]:
            if ':' in line:
                name, value = line.split(':', 1)
                fields[name.lower()] = value.strip()
        digest = hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()
        if fields.get('sec-websocket-accept') != base64.b64encode(digest).decode():
            raise ValueError('Invalid WebSocket accept key')

    def receive_payload(self):
        while True:
            first, second = self.exact(2)
            if not first & 0x80 or first & 0x70 or second & 0x80:
                raise ValueError('Expected a complete, unmasked WebSocket frame')
            length = second & 0x7f
            if length == 126:
                length = struct.unpack('!H', self.exact(2))[0]
            elif length == 127:
                length = struct.unpack('!Q', self.exact(8))[0]
            if length > 1024 * 1024:
                raise ValueError('Oversized gateway status frame')
            payload = self.exact(length)
            opcode = first & 0xf
            if opcode == 8:
                raise ConnectionError('Gateway sent a WebSocket close frame')
            if opcode == 9:
                if length > 125:
                    raise ValueError('Oversized WebSocket ping')
                mask = os.urandom(4)
                masked = bytes(value ^ mask[index % 4] for index, value in enumerate(payload))
                self.socket.sendall(bytes([0x8a, 0x80 | length]) + mask + masked)
                continue
            if opcode != 2:
                raise ValueError('Expected a binary frame; use Gateway ws_format:=protobuf')
            return payload

    def receive(self):
        return decode_status(self.receive_payload())

    def close(self):
        self.socket.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=5.0)
    arguments = parser.parse_args()
    if not 1 <= arguments.port <= 65535 or arguments.count < 1:
        parser.error('Port must be 1..65535 and count must be positive.')
    if not math.isfinite(arguments.timeout) or arguments.timeout <= 0:
        parser.error('Timeout must be finite and positive.')
    if not arguments.host or '\r' in arguments.host or '\n' in arguments.host:
        parser.error('Invalid host.')

    try:
        with ProtobufWebSocketClient(arguments.host, arguments.port, arguments.timeout) as client:
            for _ in range(arguments.count):
                print(json.dumps(client.receive(), ensure_ascii=False, allow_nan=False), flush=True)
        return 0
    except (OSError, ValueError, DecodeError) as error:
        print(f'Protobuf client failed: {error}', file=sys.stderr)
        return 1
