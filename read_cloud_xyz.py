import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import dtype_from_fields


# Read XYZ coordinates while respecting point offsets, row padding and byte order
def read_cloud_xyz(message):
    if message.width == 0 or message.height == 0:
        return np.empty((0, 3), dtype=np.float32)

    fields = {field.name: field for field in message.fields}
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if (field is None or field.count != 1
                or field.datatype not in (PointField.FLOAT32, PointField.FLOAT64)):
            raise ValueError('The point cloud must contain scalar x, y and z float fields')

    # Do not read a cloud whose byte buffer is smaller than its declared dimensions
    if (message.point_step <= 0
            or message.row_step < message.width * message.point_step
            or len(message.data) < message.height * message.row_step):
        raise ValueError('The point cloud data size is invalid')

    dtype = dtype_from_fields(
        [fields[name] for name in ('x', 'y', 'z')], point_step=message.point_step
    ).newbyteorder('>' if message.is_bigendian else '<')
    points = np.ndarray(
        shape=(message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step),
    )

    # Return a separate array so drawing cannot change the received message
    return np.stack([points[name] for name in ('x', 'y', 'z')], axis=-1).reshape(-1, 3)
