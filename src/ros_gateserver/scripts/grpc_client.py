#!/usr/bin/env python3
"""Read gateway gRPC status snapshots as JSON lines."""

import argparse
import json
import math
import sys

import grpc

from ros_gateserver_protocol.codec import decode_status_message
from ros_gateserver_protocol.gateway_status_pb2 import WatchStatusRequest
from ros_gateserver_protocol.gateway_status_pb2_grpc import GatewayStatusServiceStub


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='127.0.0.1:50051')
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=5.0, help='Deadline for the whole stream.')
    arguments = parser.parse_args()
    if arguments.count < 1:
        parser.error('Count must be positive.')
    if not math.isfinite(arguments.timeout) or arguments.timeout <= 0:
        parser.error('Timeout must be finite and positive.')

    try:
        with grpc.insecure_channel(arguments.target) as channel:
            stream = GatewayStatusServiceStub(channel).WatchStatus(
                WatchStatusRequest(), timeout=arguments.timeout, wait_for_ready=True,
            )
            try:
                for _ in range(arguments.count):
                    status = decode_status_message(next(stream))
                    print(json.dumps(status, ensure_ascii=False, allow_nan=False), flush=True)
            finally:
                stream.cancel()
    except grpc.RpcError as error:
        print(f'gRPC stream failed ({error.code().name}): {error.details()}', file=sys.stderr)
        return 1
    except StopIteration:
        print('gRPC stream ended before the requested number of snapshots.', file=sys.stderr)
        return 1
    except ValueError as error:
        print(f'Invalid gateway status: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
