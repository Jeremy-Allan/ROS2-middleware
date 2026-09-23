"""The simple one-call recipe actions: 'home', 'move_arm', 'relative_move', 'gripper'."""


def home(ctx, params):
    motion_params = ctx.build_motion_params(params.get('speed'))
    result = ctx.call_home_service(motion_params)
    return result is not None and result['success']


def move_arm(ctx, params):
    target_name = params['target']
    coords = ctx.get_static_object_coords(target_name)
    if not coords:
        return False

    orientation = ctx.resolve_orientation(params.get('orientation'))
    if orientation is None:
        ctx.get_logger().error(f"Unknown orientation preset '{params.get('orientation')}'")
        return False
    has_orientation, roll, pitch, yaw = orientation

    motion_params = ctx.build_motion_params(params.get('speed'))
    result = ctx.call_move_service(coords['x'], coords['y'], coords['z'], has_orientation, roll, pitch, yaw, motion_params)
    return result is not None and result['success']


def relative_move(ctx, params):
    vector_name = params['vector']
    vector = ctx.get_relative_movement_vector(vector_name)
    if not vector:
        return False

    orientation = ctx.resolve_orientation(params.get('orientation'))
    if orientation is None:
        ctx.get_logger().error(f"Unknown orientation preset '{params.get('orientation')}'")
        return False
    has_orientation, roll_delta, pitch_delta, yaw_delta = orientation

    motion_params = ctx.build_motion_params(params.get('speed'))
    result = ctx.call_relative_move_service(vector['x'], vector['y'], vector['z'], has_orientation, roll_delta, pitch_delta, yaw_delta, motion_params)
    return result is not None and result['success']


def gripper(ctx, params):
    gripper = float(params['position'])
    result = ctx.call_move_gripper_service(gripper)
    return result is not None and result['success']
