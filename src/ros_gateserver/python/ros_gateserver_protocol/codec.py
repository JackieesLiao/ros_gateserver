"""Decode typed protobuf messages into the existing gateway JSON structure."""

import math

from .gateway_status_pb2 import GatewayStatus


def optional_number(message, name):
    if not message.HasField(name):
        return None
    value = getattr(message, name)
    if not math.isfinite(value):
        raise ValueError(f'Non-finite protobuf number: {name}')
    return value


def coordinates(message, quaternion=False):
    names = ('x', 'y', 'z', 'w') if quaternion else ('x', 'y', 'z')
    return {name: optional_number(message, name) for name in names}


def metadata(message):
    if not message.HasField('stamp'):
        raise ValueError('Sensor data is missing its timestamp')
    return {
        'frame_id': message.frame_id,
        'stamp': {'sec': message.stamp.sec, 'nanosec': message.stamp.nanosec},
    }


def point(message):
    return {'frame_id': message.frame_id, **coordinates(message)}


def optional_point(message, name):
    if not message.HasField(name):
        return None
    return point(getattr(message, name))


def image_data(message):
    result = {
        **metadata(message),
        'width': message.width,
        'height': message.height,
        'encoding': message.encoding,
        'bytes': message.byte_count,
    }
    if message.HasField('quality'):
        quality = message.quality
        result['quality'] = {
            'status': quality.status,
            'reason': quality.reason,
            'analysis_width': quality.analysis_width,
            'analysis_height': quality.analysis_height,
            'mean_brightness': optional_number(quality, 'mean_brightness'),
            'contrast_stddev': optional_number(quality, 'contrast_stddev'),
            'laplacian_variance': optional_number(quality, 'laplacian_variance'),
            'flags': list(quality.flags),
        }
    return result


def scan_data(message):
    return {
        **metadata(message),
        'samples': message.samples,
        'valid_samples': message.valid_samples,
        'nearest_m': optional_number(message, 'nearest_m'),
        'nearest_point_sensor': optional_point(message, 'nearest_point_sensor'),
        'nearest_point': optional_point(message, 'nearest_point'),
        'transform_status': message.transform_status,
    }


def odometry_data(message):
    return {
        **metadata(message),
        'child_frame_id': message.child_frame_id,
        'position': coordinates(message.position),
        'orientation': coordinates(message.orientation, quaternion=True),
        'linear_x': optional_number(message, 'linear_x'),
        'angular_z': optional_number(message, 'angular_z'),
    }


def diagnostics_data(message):
    fields = (
        'state', 'topic', 'reliability', 'publishers', 'compatible_publishers',
        'incompatible_publishers', 'uncertain_publishers', 'type_mismatches',
        'incompatible_events', 'last_policy', 'event_supported', 'graph_error',
    )
    return {
        **{name: getattr(message, name) for name in fields},
        'receive_hz': optional_number(message, 'receive_hz'),
        'reasons': list(message.reasons),
    }


def decode_status(payload):
    message = GatewayStatus()
    message.ParseFromString(payload)
    return decode_status_message(message)


def decode_status_message(message):
    if message.schema_version != 1:
        raise ValueError(f'Unsupported protobuf schema: {message.schema_version}')

    decoders = {'image': image_data, 'scan': scan_data, 'odom': odometry_data}
    sensors = {}
    for name, decoder in decoders.items():
        if not message.HasField(name):
            raise ValueError(f'Missing protobuf sensor status: {name}')
        sensor = getattr(message, name)
        selected = sensor.WhichOneof('data')
        expected = f'{name}_data'
        if selected is not None and selected != expected:
            raise ValueError(f'Unexpected payload {selected} in sensor {name}')
        sensors[name] = {
            'received': sensor.received,
            'fresh': sensor.fresh,
            'age_sec': optional_number(sensor, 'age_sec'),
            'data': None if selected is None else decoder(getattr(sensor, selected)),
        }
        # 新字段按消息存在性解码，仍兼容没有诊断信息的旧版载荷。
        if sensor.HasField('diagnostics'):
            sensors[name]['diagnostics'] = diagnostics_data(sensor.diagnostics)

    result = {
        'schema_version': message.schema_version,
        'enabled': message.enabled,
        'stamp_ns': message.stamp_ns,
        'sensors': sensors,
    }
    if message.HasField('clock'):
        result['clock'] = {
            'source': message.clock.source,
            'initialized': message.clock.initialized,
            'epoch': message.clock.epoch,
        }
    if message.HasField('sequence'):
        result['sequence'] = message.sequence
    return result
