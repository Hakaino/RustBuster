################  AUTOMATICALLY CREATED FILE DO NOT MODIFY ############

from .manual_conversions import *

def convert_bosdyn_msgs_mobility_params_to_any_proto(ros_msg, proto):
    from bosdyn.api.spot.robot_command_pb2 import MobilityParams
    raw_proto = MobilityParams()
    convert_bosdyn_msgs_mobility_params_to_proto(ros_msg, raw_proto)
    proto.Pack(raw_proto)

def convert_any_proto_to_bosdyn_msgs_mobility_params(proto, ros_msg):
    from bosdyn.api.spot.robot_command_pb2 import MobilityParams
    raw_proto = MobilityParams()
    proto.Unpack(raw_proto)
    convert_proto_to_bosdyn_msgs_mobility_params(raw_proto, ros_msg)

def convert_proto_to_serialized_bosdyn_msgs_resource_tree(proto, ros_msg):
    import rclpy.serialization
    from bosdyn_msgs.msg import ResourceTree
    full_msg = ResourceTree()
    convert_proto_to_bosdyn_msgs_resource_tree(proto, full_msg)
    ros_msg.serialized_msg = list(rclpy.serialization.serialize_message(full_msg))

def convert_serialized_bosdyn_msgs_resource_tree_to_proto(ros_msg, proto):
    proto.Clear()
    import rclpy.serialization
    from bosdyn_msgs.msg import ResourceTree
    full_msg = rclpy.serialization.deserialize_message(bytes(ros_msg.serialized_msg), ResourceTree)
    convert_bosdyn_msgs_resource_tree_to_proto(full_msg, proto)

def convert_proto_to_serialized_bosdyn_msgs_node_info(proto, ros_msg):
    import rclpy.serialization
    from bosdyn_msgs.msg import NodeInfo
    full_msg = NodeInfo()
    convert_proto_to_bosdyn_msgs_node_info(proto, full_msg)
    ros_msg.serialized_msg = list(rclpy.serialization.serialize_message(full_msg))

def convert_serialized_bosdyn_msgs_node_info_to_proto(ros_msg, proto):
    proto.Clear()
    import rclpy.serialization
    from bosdyn_msgs.msg import NodeInfo
    full_msg = rclpy.serialization.deserialize_message(bytes(ros_msg.serialized_msg), NodeInfo)
    convert_bosdyn_msgs_node_info_to_proto(full_msg, proto)

def convert_proto_to_serialized_bosdyn_msgs_action_wrapper_gripper_camera_params(proto, ros_msg):
    import rclpy.serialization
    from bosdyn_msgs.msg import ActionWrapperGripperCameraParams
    full_msg = ActionWrapperGripperCameraParams()
    convert_proto_to_bosdyn_msgs_action_wrapper_gripper_camera_params(proto, full_msg)
    ros_msg.serialized_msg = list(rclpy.serialization.serialize_message(full_msg))

def convert_serialized_bosdyn_msgs_action_wrapper_gripper_camera_params_to_proto(ros_msg, proto):
    proto.Clear()
    import rclpy.serialization
    from bosdyn_msgs.msg import ActionWrapperGripperCameraParams
    full_msg = rclpy.serialization.deserialize_message(bytes(ros_msg.serialized_msg), ActionWrapperGripperCameraParams)
    convert_bosdyn_msgs_action_wrapper_gripper_camera_params_to_proto(full_msg, proto)

def convert_proto_to_bosdyn_msgs_world_object(proto, ros_msg):
    ros_msg.id = proto.id
    ros_msg.name = proto.name
    convert_proto_to_builtin_interfaces_time(proto.acquisition_time, ros_msg.acquisition_time)
    ros_msg.acquisition_time_is_set = proto.HasField("acquisition_time")
    from bosdyn_msgs.msg import DrawableProperties
    ros_msg.drawable_properties = []
    for _item in proto.drawable_properties:
        ros_msg.drawable_properties.append(DrawableProperties())
        convert_proto_to_bosdyn_msgs_drawable_properties(_item, ros_msg.drawable_properties[-1])
    convert_proto_to_bosdyn_msgs_april_tag_properties(proto.apriltag_properties, ros_msg.apriltag_properties)
    ros_msg.apriltag_properties_is_set = proto.HasField("apriltag_properties")
    convert_proto_to_bosdyn_msgs_image_properties(proto.image_properties, ros_msg.image_properties)
    ros_msg.image_properties_is_set = proto.HasField("image_properties")
    convert_proto_to_bosdyn_msgs_dock_properties(proto.dock_properties, ros_msg.dock_properties)
    ros_msg.dock_properties_is_set = proto.HasField("dock_properties")
    convert_proto_to_bosdyn_msgs_ray_properties(proto.ray_properties, ros_msg.ray_properties)
    ros_msg.ray_properties_is_set = proto.HasField("ray_properties")
    convert_proto_to_bosdyn_msgs_bounding_box_properties(proto.bounding_box_properties, ros_msg.bounding_box_properties)
    ros_msg.bounding_box_properties_is_set = proto.HasField("bounding_box_properties")

def convert_bosdyn_msgs_world_object_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    proto.name = ros_msg.name
    if ros_msg.acquisition_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.acquisition_time, proto.acquisition_time)
    del proto.drawable_properties[:]
    for _item in ros_msg.drawable_properties:
        convert_bosdyn_msgs_drawable_properties_to_proto(_item, proto.drawable_properties.add())
    if ros_msg.apriltag_properties_is_set:
        convert_bosdyn_msgs_april_tag_properties_to_proto(ros_msg.apriltag_properties, proto.apriltag_properties)
    if ros_msg.image_properties_is_set:
        convert_bosdyn_msgs_image_properties_to_proto(ros_msg.image_properties, proto.image_properties)
    if ros_msg.dock_properties_is_set:
        convert_bosdyn_msgs_dock_properties_to_proto(ros_msg.dock_properties, proto.dock_properties)
    if ros_msg.ray_properties_is_set:
        convert_bosdyn_msgs_ray_properties_to_proto(ros_msg.ray_properties, proto.ray_properties)
    if ros_msg.bounding_box_properties_is_set:
        convert_bosdyn_msgs_bounding_box_properties_to_proto(ros_msg.bounding_box_properties, proto.bounding_box_properties)

def convert_proto_to_bosdyn_msgs_list_world_object_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import WorldObjectType
    ros_msg.object_type = []
    for _item in proto.object_type:
        ros_msg.object_type.append(WorldObjectType())
        ros_msg.object_type[-1].value = _item
    convert_proto_to_builtin_interfaces_time(proto.timestamp_filter, ros_msg.timestamp_filter)
    ros_msg.timestamp_filter_is_set = proto.HasField("timestamp_filter")

def convert_bosdyn_msgs_list_world_object_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.object_type[:]
    for _item in ros_msg.object_type:
        proto.object_type.add(_item.value)
    if ros_msg.timestamp_filter_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp_filter, proto.timestamp_filter)

def convert_proto_to_bosdyn_msgs_list_world_object_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import WorldObject
    ros_msg.world_objects = []
    for _item in proto.world_objects:
        ros_msg.world_objects.append(WorldObject())
        convert_proto_to_bosdyn_msgs_world_object(_item, ros_msg.world_objects[-1])

def convert_bosdyn_msgs_list_world_object_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.world_objects[:]
    for _item in ros_msg.world_objects:
        convert_bosdyn_msgs_world_object_to_proto(_item, proto.world_objects.add())

def convert_proto_to_bosdyn_msgs_mutate_world_object_request_mutation(proto, ros_msg):
    ros_msg.action.value = proto.action
    convert_proto_to_bosdyn_msgs_world_object(proto.object, ros_msg.object)
    ros_msg.object_is_set = proto.HasField("object")

def convert_bosdyn_msgs_mutate_world_object_request_mutation_to_proto(ros_msg, proto):
    proto.Clear()
    proto.action = ros_msg.action.value
    if ros_msg.object_is_set:
        convert_bosdyn_msgs_world_object_to_proto(ros_msg.object, proto.object)

def convert_proto_to_bosdyn_msgs_mutate_world_object_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_mutate_world_object_request_mutation(proto.mutation, ros_msg.mutation)
    ros_msg.mutation_is_set = proto.HasField("mutation")

def convert_bosdyn_msgs_mutate_world_object_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.mutation_is_set:
        convert_bosdyn_msgs_mutate_world_object_request_mutation_to_proto(ros_msg.mutation, proto.mutation)

def convert_proto_to_bosdyn_msgs_mutate_world_object_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.mutated_object_id = proto.mutated_object_id

def convert_bosdyn_msgs_mutate_world_object_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.mutated_object_id = ros_msg.mutated_object_id

def convert_proto_to_bosdyn_msgs_image_properties_one_of_image_data(proto, ros_msg):
    if proto.HasField("coordinates"):
        ros_msg.image_data_choice = ros_msg.IMAGE_DATA_COORDINATES_SET
        convert_proto_to_bosdyn_msgs_polygon(proto.coordinates, ros_msg.coordinates)
    if proto.HasField("keypoints"):
        ros_msg.image_data_choice = ros_msg.IMAGE_DATA_KEYPOINTS_SET
        convert_proto_to_bosdyn_msgs_keypoint_set(proto.keypoints, ros_msg.keypoints)

def convert_bosdyn_msgs_image_properties_one_of_image_data_to_proto(ros_msg, proto):
    proto.ClearField("image_data")
    if ros_msg.image_data_choice == ros_msg.IMAGE_DATA_COORDINATES_SET:
        convert_bosdyn_msgs_polygon_to_proto(ros_msg.coordinates, proto.coordinates)
    if ros_msg.image_data_choice == ros_msg.IMAGE_DATA_KEYPOINTS_SET:
        convert_bosdyn_msgs_keypoint_set_to_proto(ros_msg.keypoints, proto.keypoints)

def convert_proto_to_bosdyn_msgs_image_properties(proto, ros_msg):
    ros_msg.camera_source = proto.camera_source
    convert_proto_to_bosdyn_msgs_image_properties_one_of_image_data(proto, ros_msg.image_data)
    convert_proto_to_bosdyn_msgs_image_source(proto.image_source, ros_msg.image_source)
    ros_msg.image_source_is_set = proto.HasField("image_source")
    convert_proto_to_bosdyn_msgs_image_capture(proto.image_capture, ros_msg.image_capture)
    ros_msg.image_capture_is_set = proto.HasField("image_capture")
    ros_msg.frame_name_image_coordinates = proto.frame_name_image_coordinates

def convert_bosdyn_msgs_image_properties_to_proto(ros_msg, proto):
    proto.Clear()
    proto.camera_source = ros_msg.camera_source
    convert_bosdyn_msgs_image_properties_one_of_image_data_to_proto(ros_msg.image_data, proto)
    if ros_msg.image_source_is_set:
        convert_bosdyn_msgs_image_source_to_proto(ros_msg.image_source, proto.image_source)
    if ros_msg.image_capture_is_set:
        convert_bosdyn_msgs_image_capture_to_proto(ros_msg.image_capture, proto.image_capture)
    proto.frame_name_image_coordinates = ros_msg.frame_name_image_coordinates

def convert_proto_to_bosdyn_msgs_dock_properties(proto, ros_msg):
    ros_msg.dock_id = proto.dock_id
    ros_msg.type.value = proto.type
    ros_msg.frame_name_dock = proto.frame_name_dock
    ros_msg.unavailable = proto.unavailable
    ros_msg.from_prior = proto.from_prior

def convert_bosdyn_msgs_dock_properties_to_proto(ros_msg, proto):
    proto.Clear()
    proto.dock_id = ros_msg.dock_id
    proto.type = ros_msg.type.value
    proto.frame_name_dock = ros_msg.frame_name_dock
    proto.unavailable = ros_msg.unavailable
    proto.from_prior = ros_msg.from_prior

def convert_proto_to_bosdyn_msgs_april_tag_properties(proto, ros_msg):
    ros_msg.tag_id = proto.tag_id
    convert_proto_to_bosdyn_msgs_vec2(proto.dimensions, ros_msg.dimensions)
    ros_msg.dimensions_is_set = proto.HasField("dimensions")
    ros_msg.frame_name_fiducial = proto.frame_name_fiducial
    ros_msg.fiducial_pose_status.value = proto.fiducial_pose_status
    ros_msg.frame_name_fiducial_filtered = proto.frame_name_fiducial_filtered
    ros_msg.fiducial_filtered_pose_status.value = proto.fiducial_filtered_pose_status
    ros_msg.frame_name_camera = proto.frame_name_camera
    convert_proto_to_bosdyn_msgs_se3_covariance(proto.detection_covariance, ros_msg.detection_covariance)
    ros_msg.detection_covariance_is_set = proto.HasField("detection_covariance")
    ros_msg.detection_covariance_reference_frame = proto.detection_covariance_reference_frame

def convert_bosdyn_msgs_april_tag_properties_to_proto(ros_msg, proto):
    proto.Clear()
    proto.tag_id = ros_msg.tag_id
    if ros_msg.dimensions_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.dimensions, proto.dimensions)
    proto.frame_name_fiducial = ros_msg.frame_name_fiducial
    proto.fiducial_pose_status = ros_msg.fiducial_pose_status.value
    proto.frame_name_fiducial_filtered = ros_msg.frame_name_fiducial_filtered
    proto.fiducial_filtered_pose_status = ros_msg.fiducial_filtered_pose_status.value
    proto.frame_name_camera = ros_msg.frame_name_camera
    if ros_msg.detection_covariance_is_set:
        convert_bosdyn_msgs_se3_covariance_to_proto(ros_msg.detection_covariance, proto.detection_covariance)
    proto.detection_covariance_reference_frame = ros_msg.detection_covariance_reference_frame

def convert_proto_to_bosdyn_msgs_ray_properties(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_ray(proto.ray, ros_msg.ray)
    ros_msg.ray_is_set = proto.HasField("ray")
    ros_msg.frame = proto.frame

def convert_bosdyn_msgs_ray_properties_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.ray_is_set:
        convert_bosdyn_msgs_ray_to_proto(ros_msg.ray, proto.ray)
    proto.frame = ros_msg.frame

def convert_proto_to_bosdyn_msgs_bounding_box_properties(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.size_ewrt_frame, ros_msg.size_ewrt_frame)
    ros_msg.size_ewrt_frame_is_set = proto.HasField("size_ewrt_frame")
    ros_msg.frame = proto.frame

def convert_bosdyn_msgs_bounding_box_properties_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.size_ewrt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.size_ewrt_frame, proto.size_ewrt_frame)
    proto.frame = ros_msg.frame

def convert_proto_to_bosdyn_msgs_drawable_properties_color(proto, ros_msg):
    ros_msg.r = proto.r
    ros_msg.g = proto.g
    ros_msg.b = proto.b
    ros_msg.a = proto.a

def convert_bosdyn_msgs_drawable_properties_color_to_proto(ros_msg, proto):
    proto.Clear()
    proto.r = ros_msg.r
    proto.g = ros_msg.g
    proto.b = ros_msg.b
    proto.a = ros_msg.a

def convert_proto_to_bosdyn_msgs_drawable_properties_one_of_drawable(proto, ros_msg):
    if proto.HasField("frame"):
        ros_msg.drawable_choice = ros_msg.DRAWABLE_FRAME_SET
        convert_proto_to_bosdyn_msgs_drawable_frame(proto.frame, ros_msg.frame)
    if proto.HasField("sphere"):
        ros_msg.drawable_choice = ros_msg.DRAWABLE_SPHERE_SET
        convert_proto_to_bosdyn_msgs_drawable_sphere(proto.sphere, ros_msg.sphere)
    if proto.HasField("box"):
        ros_msg.drawable_choice = ros_msg.DRAWABLE_BOX_SET
        convert_proto_to_bosdyn_msgs_drawable_box(proto.box, ros_msg.box)
    if proto.HasField("arrow"):
        ros_msg.drawable_choice = ros_msg.DRAWABLE_ARROW_SET
        convert_proto_to_bosdyn_msgs_drawable_arrow(proto.arrow, ros_msg.arrow)
    if proto.HasField("capsule"):
        ros_msg.drawable_choice = ros_msg.DRAWABLE_CAPSULE_SET
        convert_proto_to_bosdyn_msgs_drawable_capsule(proto.capsule, ros_msg.capsule)
    if proto.HasField("cylinder"):
        ros_msg.drawable_choice = ros_msg.DRAWABLE_CYLINDER_SET
        convert_proto_to_bosdyn_msgs_drawable_cylinder(proto.cylinder, ros_msg.cylinder)
    if proto.HasField("linestrip"):
        ros_msg.drawable_choice = ros_msg.DRAWABLE_LINESTRIP_SET
        convert_proto_to_bosdyn_msgs_drawable_line_strip(proto.linestrip, ros_msg.linestrip)
    if proto.HasField("points"):
        ros_msg.drawable_choice = ros_msg.DRAWABLE_POINTS_SET
        convert_proto_to_bosdyn_msgs_drawable_points(proto.points, ros_msg.points)

def convert_bosdyn_msgs_drawable_properties_one_of_drawable_to_proto(ros_msg, proto):
    proto.ClearField("drawable")
    if ros_msg.drawable_choice == ros_msg.DRAWABLE_FRAME_SET:
        convert_bosdyn_msgs_drawable_frame_to_proto(ros_msg.frame, proto.frame)
    if ros_msg.drawable_choice == ros_msg.DRAWABLE_SPHERE_SET:
        convert_bosdyn_msgs_drawable_sphere_to_proto(ros_msg.sphere, proto.sphere)
    if ros_msg.drawable_choice == ros_msg.DRAWABLE_BOX_SET:
        convert_bosdyn_msgs_drawable_box_to_proto(ros_msg.box, proto.box)
    if ros_msg.drawable_choice == ros_msg.DRAWABLE_ARROW_SET:
        convert_bosdyn_msgs_drawable_arrow_to_proto(ros_msg.arrow, proto.arrow)
    if ros_msg.drawable_choice == ros_msg.DRAWABLE_CAPSULE_SET:
        convert_bosdyn_msgs_drawable_capsule_to_proto(ros_msg.capsule, proto.capsule)
    if ros_msg.drawable_choice == ros_msg.DRAWABLE_CYLINDER_SET:
        convert_bosdyn_msgs_drawable_cylinder_to_proto(ros_msg.cylinder, proto.cylinder)
    if ros_msg.drawable_choice == ros_msg.DRAWABLE_LINESTRIP_SET:
        convert_bosdyn_msgs_drawable_line_strip_to_proto(ros_msg.linestrip, proto.linestrip)
    if ros_msg.drawable_choice == ros_msg.DRAWABLE_POINTS_SET:
        convert_bosdyn_msgs_drawable_points_to_proto(ros_msg.points, proto.points)

def convert_proto_to_bosdyn_msgs_drawable_properties(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_drawable_properties_color(proto.color, ros_msg.color)
    ros_msg.color_is_set = proto.HasField("color")
    ros_msg.label = proto.label
    ros_msg.wireframe = proto.wireframe
    convert_proto_to_bosdyn_msgs_drawable_properties_one_of_drawable(proto, ros_msg.drawable)
    ros_msg.frame_name_drawable = proto.frame_name_drawable

def convert_bosdyn_msgs_drawable_properties_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.color_is_set:
        convert_bosdyn_msgs_drawable_properties_color_to_proto(ros_msg.color, proto.color)
    proto.label = ros_msg.label
    proto.wireframe = ros_msg.wireframe
    convert_bosdyn_msgs_drawable_properties_one_of_drawable_to_proto(ros_msg.drawable, proto)
    proto.frame_name_drawable = ros_msg.frame_name_drawable

def convert_proto_to_bosdyn_msgs_drawable_frame(proto, ros_msg):
    ros_msg.arrow_length = proto.arrow_length
    ros_msg.arrow_radius = proto.arrow_radius

def convert_bosdyn_msgs_drawable_frame_to_proto(ros_msg, proto):
    proto.Clear()
    proto.arrow_length = ros_msg.arrow_length
    proto.arrow_radius = ros_msg.arrow_radius

def convert_proto_to_bosdyn_msgs_drawable_sphere(proto, ros_msg):
    ros_msg.radius = proto.radius

def convert_bosdyn_msgs_drawable_sphere_to_proto(ros_msg, proto):
    proto.Clear()
    proto.radius = ros_msg.radius

def convert_proto_to_bosdyn_msgs_drawable_box(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.size, ros_msg.size)
    ros_msg.size_is_set = proto.HasField("size")

def convert_bosdyn_msgs_drawable_box_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.size_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.size, proto.size)

def convert_proto_to_bosdyn_msgs_drawable_arrow(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.direction, ros_msg.direction)
    ros_msg.direction_is_set = proto.HasField("direction")
    ros_msg.radius = proto.radius

def convert_bosdyn_msgs_drawable_arrow_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.direction_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.direction, proto.direction)
    proto.radius = ros_msg.radius

def convert_proto_to_bosdyn_msgs_drawable_capsule(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.direction, ros_msg.direction)
    ros_msg.direction_is_set = proto.HasField("direction")
    ros_msg.radius = proto.radius

def convert_bosdyn_msgs_drawable_capsule_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.direction_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.direction, proto.direction)
    proto.radius = ros_msg.radius

def convert_proto_to_bosdyn_msgs_drawable_cylinder(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.direction, ros_msg.direction)
    ros_msg.direction_is_set = proto.HasField("direction")
    ros_msg.radius = proto.radius

def convert_bosdyn_msgs_drawable_cylinder_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.direction_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.direction, proto.direction)
    proto.radius = ros_msg.radius

def convert_proto_to_bosdyn_msgs_drawable_line_strip(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.points, ros_msg.points)
    ros_msg.points_is_set = proto.HasField("points")

def convert_bosdyn_msgs_drawable_line_strip_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.points_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.points, proto.points)

def convert_proto_to_bosdyn_msgs_drawable_points(proto, ros_msg):
    from geometry_msgs.msg import Vector3
    ros_msg.points = []
    for _item in proto.points:
        ros_msg.points.append(Vector3())
        convert_proto_to_geometry_msgs_vector3(_item, ros_msg.points[-1])

def convert_bosdyn_msgs_drawable_points_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.points[:]
    for _item in ros_msg.points:
        convert_geometry_msgs_vector3_to_proto(_item, proto.points.add())

def convert_proto_to_bosdyn_msgs_time_sync_round_trip(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.client_tx, ros_msg.client_tx)
    ros_msg.client_tx_is_set = proto.HasField("client_tx")
    convert_proto_to_builtin_interfaces_time(proto.server_rx, ros_msg.server_rx)
    ros_msg.server_rx_is_set = proto.HasField("server_rx")
    convert_proto_to_builtin_interfaces_time(proto.server_tx, ros_msg.server_tx)
    ros_msg.server_tx_is_set = proto.HasField("server_tx")
    convert_proto_to_builtin_interfaces_time(proto.client_rx, ros_msg.client_rx)
    ros_msg.client_rx_is_set = proto.HasField("client_rx")

def convert_bosdyn_msgs_time_sync_round_trip_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.client_tx_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.client_tx, proto.client_tx)
    if ros_msg.server_rx_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.server_rx, proto.server_rx)
    if ros_msg.server_tx_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.server_tx, proto.server_tx)
    if ros_msg.client_rx_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.client_rx, proto.client_rx)

def convert_proto_to_bosdyn_msgs_time_sync_estimate(proto, ros_msg):
    convert_proto_to_builtin_interfaces_duration(proto.round_trip_time, ros_msg.round_trip_time)
    ros_msg.round_trip_time_is_set = proto.HasField("round_trip_time")
    convert_proto_to_builtin_interfaces_duration(proto.clock_skew, ros_msg.clock_skew)
    ros_msg.clock_skew_is_set = proto.HasField("clock_skew")

def convert_bosdyn_msgs_time_sync_estimate_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.round_trip_time_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.round_trip_time, proto.round_trip_time)
    if ros_msg.clock_skew_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.clock_skew, proto.clock_skew)

def convert_proto_to_bosdyn_msgs_time_sync_state(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_time_sync_estimate(proto.best_estimate, ros_msg.best_estimate)
    ros_msg.best_estimate_is_set = proto.HasField("best_estimate")
    ros_msg.status.value = proto.status
    convert_proto_to_builtin_interfaces_time(proto.measurement_time, ros_msg.measurement_time)
    ros_msg.measurement_time_is_set = proto.HasField("measurement_time")

def convert_bosdyn_msgs_time_sync_state_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.best_estimate_is_set:
        convert_bosdyn_msgs_time_sync_estimate_to_proto(ros_msg.best_estimate, proto.best_estimate)
    proto.status = ros_msg.status.value
    if ros_msg.measurement_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.measurement_time, proto.measurement_time)

def convert_proto_to_bosdyn_msgs_time_sync_update_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_time_sync_round_trip(proto.previous_round_trip, ros_msg.previous_round_trip)
    ros_msg.previous_round_trip_is_set = proto.HasField("previous_round_trip")
    ros_msg.clock_identifier = proto.clock_identifier

def convert_bosdyn_msgs_time_sync_update_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.previous_round_trip_is_set:
        convert_bosdyn_msgs_time_sync_round_trip_to_proto(ros_msg.previous_round_trip, proto.previous_round_trip)
    proto.clock_identifier = ros_msg.clock_identifier

def convert_proto_to_bosdyn_msgs_time_sync_update_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_time_sync_estimate(proto.previous_estimate, ros_msg.previous_estimate)
    ros_msg.previous_estimate_is_set = proto.HasField("previous_estimate")
    convert_proto_to_bosdyn_msgs_time_sync_state(proto.state, ros_msg.state)
    ros_msg.state_is_set = proto.HasField("state")
    ros_msg.clock_identifier = proto.clock_identifier

def convert_bosdyn_msgs_time_sync_update_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.previous_estimate_is_set:
        convert_bosdyn_msgs_time_sync_estimate_to_proto(ros_msg.previous_estimate, proto.previous_estimate)
    if ros_msg.state_is_set:
        convert_bosdyn_msgs_time_sync_state_to_proto(ros_msg.state, proto.state)
    proto.clock_identifier = ros_msg.clock_identifier

def convert_proto_to_bosdyn_msgs_estop_endpoint(proto, ros_msg):
    ros_msg.role = proto.role
    ros_msg.name = proto.name
    ros_msg.unique_id = proto.unique_id
    convert_proto_to_builtin_interfaces_duration(proto.timeout, ros_msg.timeout)
    ros_msg.timeout_is_set = proto.HasField("timeout")
    convert_proto_to_builtin_interfaces_duration(proto.cut_power_timeout, ros_msg.cut_power_timeout)
    ros_msg.cut_power_timeout_is_set = proto.HasField("cut_power_timeout")

def convert_bosdyn_msgs_estop_endpoint_to_proto(ros_msg, proto):
    proto.Clear()
    proto.role = ros_msg.role
    proto.name = ros_msg.name
    proto.unique_id = ros_msg.unique_id
    if ros_msg.timeout_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.timeout, proto.timeout)
    if ros_msg.cut_power_timeout_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.cut_power_timeout, proto.cut_power_timeout)

def convert_proto_to_bosdyn_msgs_estop_config(proto, ros_msg):
    from bosdyn_msgs.msg import EstopEndpoint
    ros_msg.endpoints = []
    for _item in proto.endpoints:
        ros_msg.endpoints.append(EstopEndpoint())
        convert_proto_to_bosdyn_msgs_estop_endpoint(_item, ros_msg.endpoints[-1])
    ros_msg.unique_id = proto.unique_id

def convert_bosdyn_msgs_estop_config_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.endpoints[:]
    for _item in ros_msg.endpoints:
        convert_bosdyn_msgs_estop_endpoint_to_proto(_item, proto.endpoints.add())
    proto.unique_id = ros_msg.unique_id

def convert_proto_to_bosdyn_msgs_estop_endpoint_with_status(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_estop_endpoint(proto.endpoint, ros_msg.endpoint)
    ros_msg.endpoint_is_set = proto.HasField("endpoint")
    ros_msg.stop_level.value = proto.stop_level
    convert_proto_to_builtin_interfaces_duration(proto.time_since_valid_response, ros_msg.time_since_valid_response)
    ros_msg.time_since_valid_response_is_set = proto.HasField("time_since_valid_response")

def convert_bosdyn_msgs_estop_endpoint_with_status_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.endpoint_is_set:
        convert_bosdyn_msgs_estop_endpoint_to_proto(ros_msg.endpoint, proto.endpoint)
    proto.stop_level = ros_msg.stop_level.value
    if ros_msg.time_since_valid_response_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.time_since_valid_response, proto.time_since_valid_response)

def convert_proto_to_bosdyn_msgs_estop_system_status(proto, ros_msg):
    from bosdyn_msgs.msg import EstopEndpointWithStatus
    ros_msg.endpoints = []
    for _item in proto.endpoints:
        ros_msg.endpoints.append(EstopEndpointWithStatus())
        convert_proto_to_bosdyn_msgs_estop_endpoint_with_status(_item, ros_msg.endpoints[-1])
    ros_msg.stop_level.value = proto.stop_level
    ros_msg.stop_level_details = proto.stop_level_details

def convert_bosdyn_msgs_estop_system_status_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.endpoints[:]
    for _item in ros_msg.endpoints:
        convert_bosdyn_msgs_estop_endpoint_with_status_to_proto(_item, proto.endpoints.add())
    proto.stop_level = ros_msg.stop_level.value
    proto.stop_level_details = ros_msg.stop_level_details

def convert_proto_to_bosdyn_msgs_estop_check_in_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_estop_endpoint(proto.endpoint, ros_msg.endpoint)
    ros_msg.endpoint_is_set = proto.HasField("endpoint")
    ros_msg.challenge = proto.challenge
    ros_msg.response = proto.response
    ros_msg.stop_level.value = proto.stop_level

def convert_bosdyn_msgs_estop_check_in_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.endpoint_is_set:
        convert_bosdyn_msgs_estop_endpoint_to_proto(ros_msg.endpoint, proto.endpoint)
    proto.challenge = ros_msg.challenge
    proto.response = ros_msg.response
    proto.stop_level = ros_msg.stop_level.value

def convert_proto_to_bosdyn_msgs_estop_check_in_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_estop_check_in_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")
    ros_msg.challenge = proto.challenge
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_estop_check_in_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_estop_check_in_request_to_proto(ros_msg.request, proto.request)
    proto.challenge = ros_msg.challenge
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_register_estop_endpoint_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_estop_endpoint(proto.target_endpoint, ros_msg.target_endpoint)
    ros_msg.target_endpoint_is_set = proto.HasField("target_endpoint")
    ros_msg.target_config_id = proto.target_config_id
    convert_proto_to_bosdyn_msgs_estop_endpoint(proto.new_endpoint, ros_msg.new_endpoint)
    ros_msg.new_endpoint_is_set = proto.HasField("new_endpoint")

def convert_bosdyn_msgs_register_estop_endpoint_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.target_endpoint_is_set:
        convert_bosdyn_msgs_estop_endpoint_to_proto(ros_msg.target_endpoint, proto.target_endpoint)
    proto.target_config_id = ros_msg.target_config_id
    if ros_msg.new_endpoint_is_set:
        convert_bosdyn_msgs_estop_endpoint_to_proto(ros_msg.new_endpoint, proto.new_endpoint)

def convert_proto_to_bosdyn_msgs_register_estop_endpoint_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_register_estop_endpoint_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")
    convert_proto_to_bosdyn_msgs_estop_endpoint(proto.new_endpoint, ros_msg.new_endpoint)
    ros_msg.new_endpoint_is_set = proto.HasField("new_endpoint")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_register_estop_endpoint_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_register_estop_endpoint_request_to_proto(ros_msg.request, proto.request)
    if ros_msg.new_endpoint_is_set:
        convert_bosdyn_msgs_estop_endpoint_to_proto(ros_msg.new_endpoint, proto.new_endpoint)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_deregister_estop_endpoint_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_estop_endpoint(proto.target_endpoint, ros_msg.target_endpoint)
    ros_msg.target_endpoint_is_set = proto.HasField("target_endpoint")
    ros_msg.target_config_id = proto.target_config_id

def convert_bosdyn_msgs_deregister_estop_endpoint_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.target_endpoint_is_set:
        convert_bosdyn_msgs_estop_endpoint_to_proto(ros_msg.target_endpoint, proto.target_endpoint)
    proto.target_config_id = ros_msg.target_config_id

def convert_proto_to_bosdyn_msgs_deregister_estop_endpoint_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_deregister_estop_endpoint_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_deregister_estop_endpoint_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_deregister_estop_endpoint_request_to_proto(ros_msg.request, proto.request)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_get_estop_config_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.target_config_id = proto.target_config_id

def convert_bosdyn_msgs_get_estop_config_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.target_config_id = ros_msg.target_config_id

def convert_proto_to_bosdyn_msgs_get_estop_config_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_get_estop_config_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")
    convert_proto_to_bosdyn_msgs_estop_config(proto.active_config, ros_msg.active_config)
    ros_msg.active_config_is_set = proto.HasField("active_config")

def convert_bosdyn_msgs_get_estop_config_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_get_estop_config_request_to_proto(ros_msg.request, proto.request)
    if ros_msg.active_config_is_set:
        convert_bosdyn_msgs_estop_config_to_proto(ros_msg.active_config, proto.active_config)

def convert_proto_to_bosdyn_msgs_set_estop_config_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_estop_config(proto.config, ros_msg.config)
    ros_msg.config_is_set = proto.HasField("config")
    ros_msg.target_config_id = proto.target_config_id

def convert_bosdyn_msgs_set_estop_config_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.config_is_set:
        convert_bosdyn_msgs_estop_config_to_proto(ros_msg.config, proto.config)
    proto.target_config_id = ros_msg.target_config_id

def convert_proto_to_bosdyn_msgs_set_estop_config_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_set_estop_config_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")
    convert_proto_to_bosdyn_msgs_estop_config(proto.active_config, ros_msg.active_config)
    ros_msg.active_config_is_set = proto.HasField("active_config")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_set_estop_config_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_set_estop_config_request_to_proto(ros_msg.request, proto.request)
    if ros_msg.active_config_is_set:
        convert_bosdyn_msgs_estop_config_to_proto(ros_msg.active_config, proto.active_config)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_get_estop_system_status_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_estop_system_status_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_estop_system_status_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_estop_system_status(proto.status, ros_msg.status)
    ros_msg.status_is_set = proto.HasField("status")

def convert_bosdyn_msgs_get_estop_system_status_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.status_is_set:
        convert_bosdyn_msgs_estop_system_status_to_proto(ros_msg.status, proto.status)

def convert_proto_to_bosdyn_msgs_list_available_models_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_network_compute_server_configuration(proto.server_config, ros_msg.server_config)
    ros_msg.server_config_is_set = proto.HasField("server_config")

def convert_bosdyn_msgs_list_available_models_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.server_config_is_set:
        convert_bosdyn_msgs_network_compute_server_configuration_to_proto(ros_msg.server_config, proto.server_config)

def convert_proto_to_bosdyn_msgs_list_available_models_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.available_models = []
    for _item in proto.available_models:
        ros_msg.available_models.append(_item)
    from bosdyn_msgs.msg import ModelLabels
    ros_msg.labels = []
    for _item in proto.labels:
        ros_msg.labels.append(ModelLabels())
        convert_proto_to_bosdyn_msgs_model_labels(_item, ros_msg.labels[-1])
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_list_available_models_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.available_models[:]
    for _item in ros_msg.available_models:
        proto.available_models.add(_item)
    del proto.labels[:]
    for _item in ros_msg.labels:
        convert_bosdyn_msgs_model_labels_to_proto(_item, proto.labels.add())
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_model_labels(proto, ros_msg):
    ros_msg.model_name = proto.model_name
    ros_msg.available_labels = []
    for _item in proto.available_labels:
        ros_msg.available_labels.append(_item)

def convert_bosdyn_msgs_model_labels_to_proto(ros_msg, proto):
    proto.Clear()
    proto.model_name = ros_msg.model_name
    del proto.available_labels[:]
    for _item in ros_msg.available_labels:
        proto.available_labels.add(_item)

def convert_proto_to_bosdyn_msgs_network_compute_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_network_compute_input_data(proto.input_data, ros_msg.input_data)
    ros_msg.input_data_is_set = proto.HasField("input_data")
    convert_proto_to_bosdyn_msgs_network_compute_server_configuration(proto.server_config, ros_msg.server_config)
    ros_msg.server_config_is_set = proto.HasField("server_config")

def convert_bosdyn_msgs_network_compute_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.input_data_is_set:
        convert_bosdyn_msgs_network_compute_input_data_to_proto(ros_msg.input_data, proto.input_data)
    if ros_msg.server_config_is_set:
        convert_bosdyn_msgs_network_compute_server_configuration_to_proto(ros_msg.server_config, proto.server_config)

def convert_proto_to_bosdyn_msgs_image_source_and_service_one_of_request_data(proto, ros_msg):
    if proto.HasField("image_source"):
        ros_msg.request_data_choice = ros_msg.REQUEST_DATA_IMAGE_SOURCE_SET
        ros_msg.image_source = proto.image_source
    if proto.HasField("image_request"):
        ros_msg.request_data_choice = ros_msg.REQUEST_DATA_IMAGE_REQUEST_SET
        convert_proto_to_bosdyn_msgs_image_request(proto.image_request, ros_msg.image_request)

def convert_bosdyn_msgs_image_source_and_service_one_of_request_data_to_proto(ros_msg, proto):
    proto.ClearField("request_data")
    if ros_msg.request_data_choice == ros_msg.REQUEST_DATA_IMAGE_SOURCE_SET:
        proto.image_source = ros_msg.image_source
    if ros_msg.request_data_choice == ros_msg.REQUEST_DATA_IMAGE_REQUEST_SET:
        convert_bosdyn_msgs_image_request_to_proto(ros_msg.image_request, proto.image_request)

def convert_proto_to_bosdyn_msgs_image_source_and_service(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_image_source_and_service_one_of_request_data(proto, ros_msg.request_data)
    ros_msg.image_service = proto.image_service

def convert_bosdyn_msgs_image_source_and_service_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_image_source_and_service_one_of_request_data_to_proto(ros_msg.request_data, proto)
    proto.image_service = ros_msg.image_service

def convert_proto_to_bosdyn_msgs_network_compute_input_data_one_of_input(proto, ros_msg):
    if proto.HasField("image_source_and_service"):
        ros_msg.input_choice = ros_msg.INPUT_IMAGE_SOURCE_AND_SERVICE_SET
        convert_proto_to_bosdyn_msgs_image_source_and_service(proto.image_source_and_service, ros_msg.image_source_and_service)
    if proto.HasField("image"):
        ros_msg.input_choice = ros_msg.INPUT_IMAGE_SET
        convert_proto_to_bosdyn_msgs_image(proto.image, ros_msg.image)

def convert_bosdyn_msgs_network_compute_input_data_one_of_input_to_proto(ros_msg, proto):
    proto.ClearField("input")
    if ros_msg.input_choice == ros_msg.INPUT_IMAGE_SOURCE_AND_SERVICE_SET:
        convert_bosdyn_msgs_image_source_and_service_to_proto(ros_msg.image_source_and_service, proto.image_source_and_service)
    if ros_msg.input_choice == ros_msg.INPUT_IMAGE_SET:
        convert_bosdyn_msgs_image_to_proto(ros_msg.image, proto.image)

def convert_proto_to_bosdyn_msgs_network_compute_input_data(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_network_compute_input_data_one_of_input(proto, ros_msg.input)
    ros_msg.model_name = proto.model_name
    ros_msg.min_confidence = proto.min_confidence
    ros_msg.rotate_image.value = proto.rotate_image

def convert_bosdyn_msgs_network_compute_input_data_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_network_compute_input_data_one_of_input_to_proto(ros_msg.input, proto)
    proto.model_name = ros_msg.model_name
    proto.min_confidence = ros_msg.min_confidence
    proto.rotate_image = ros_msg.rotate_image.value

def convert_proto_to_bosdyn_msgs_network_compute_server_configuration(proto, ros_msg):
    ros_msg.service_name = proto.service_name

def convert_bosdyn_msgs_network_compute_server_configuration_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name

def convert_proto_to_bosdyn_msgs_output_image(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_image_response(proto.image_response, ros_msg.image_response)
    ros_msg.image_response_is_set = proto.HasField("image_response")
    from bosdyn_msgs.msg import WorldObject
    ros_msg.object_in_image = []
    for _item in proto.object_in_image:
        ros_msg.object_in_image.append(WorldObject())
        convert_proto_to_bosdyn_msgs_world_object(_item, ros_msg.object_in_image[-1])
    convert_proto_to_bosdyn_msgs_alert_data(proto.alert_data, ros_msg.alert_data)
    ros_msg.alert_data_is_set = proto.HasField("alert_data")

def convert_bosdyn_msgs_output_image_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.image_response_is_set:
        convert_bosdyn_msgs_image_response_to_proto(ros_msg.image_response, proto.image_response)
    del proto.object_in_image[:]
    for _item in ros_msg.object_in_image:
        convert_bosdyn_msgs_world_object_to_proto(_item, proto.object_in_image.add())
    if ros_msg.alert_data_is_set:
        convert_bosdyn_msgs_alert_data_to_proto(ros_msg.alert_data, proto.alert_data)

def convert_proto_to_bosdyn_msgs_network_compute_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import WorldObject
    ros_msg.object_in_image = []
    for _item in proto.object_in_image:
        ros_msg.object_in_image.append(WorldObject())
        convert_proto_to_bosdyn_msgs_world_object(_item, ros_msg.object_in_image[-1])
    convert_proto_to_bosdyn_msgs_image_response(proto.image_response, ros_msg.image_response)
    ros_msg.image_response_is_set = proto.HasField("image_response")
    ros_msg.image_rotation_angle = proto.image_rotation_angle
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_alert_data(proto.alert_data, ros_msg.alert_data)
    ros_msg.alert_data_is_set = proto.HasField("alert_data")
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsOutputImage
    ros_msg.output_images = []
    for _item in proto.output_images:
        ros_msg.output_images.append(KeyStringValueBosdynMsgsOutputImage())
        ros_msg.output_images[-1].key = _item
        convert_proto_to_bosdyn_msgs_output_image(proto.output_images[_item], ros_msg.output_images[-1].value)

def convert_bosdyn_msgs_network_compute_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.object_in_image[:]
    for _item in ros_msg.object_in_image:
        convert_bosdyn_msgs_world_object_to_proto(_item, proto.object_in_image.add())
    if ros_msg.image_response_is_set:
        convert_bosdyn_msgs_image_response_to_proto(ros_msg.image_response, proto.image_response)
    proto.image_rotation_angle = ros_msg.image_rotation_angle
    proto.status = ros_msg.status.value
    if ros_msg.alert_data_is_set:
        convert_bosdyn_msgs_alert_data_to_proto(ros_msg.alert_data, proto.alert_data)
    for _item in ros_msg.output_images:
        convert_bosdyn_msgs_output_image_to_proto(_item.value, proto.output_images[_item.key])

def convert_proto_to_bosdyn_msgs_raycast_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.ray_frame_name = proto.ray_frame_name
    convert_proto_to_bosdyn_msgs_ray(proto.ray, ros_msg.ray)
    ros_msg.ray_is_set = proto.HasField("ray")
    ros_msg.min_intersection_distance = proto.min_intersection_distance
    from bosdyn_msgs.msg import RayIntersectionType
    ros_msg.intersection_types = []
    for _item in proto.intersection_types:
        ros_msg.intersection_types.append(RayIntersectionType())
        ros_msg.intersection_types[-1].value = _item

def convert_bosdyn_msgs_raycast_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.ray_frame_name = ros_msg.ray_frame_name
    if ros_msg.ray_is_set:
        convert_bosdyn_msgs_ray_to_proto(ros_msg.ray, proto.ray)
    proto.min_intersection_distance = ros_msg.min_intersection_distance
    del proto.intersection_types[:]
    for _item in ros_msg.intersection_types:
        proto.intersection_types.add(_item.value)

def convert_proto_to_bosdyn_msgs_ray_intersection(proto, ros_msg):
    ros_msg.type.value = proto.type
    convert_proto_to_geometry_msgs_vector3(proto.hit_position_in_hit_frame, ros_msg.hit_position_in_hit_frame)
    ros_msg.hit_position_in_hit_frame_is_set = proto.HasField("hit_position_in_hit_frame")
    ros_msg.distance_meters = proto.distance_meters

def convert_bosdyn_msgs_ray_intersection_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    if ros_msg.hit_position_in_hit_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.hit_position_in_hit_frame, proto.hit_position_in_hit_frame)
    proto.distance_meters = ros_msg.distance_meters

def convert_proto_to_bosdyn_msgs_raycast_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.message = proto.message
    ros_msg.hit_frame_name = proto.hit_frame_name
    from bosdyn_msgs.msg import RayIntersection
    ros_msg.hits = []
    for _item in proto.hits:
        ros_msg.hits.append(RayIntersection())
        convert_proto_to_bosdyn_msgs_ray_intersection(_item, ros_msg.hits[-1])

def convert_bosdyn_msgs_raycast_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.message = ros_msg.message
    proto.hit_frame_name = ros_msg.hit_frame_name
    del proto.hits[:]
    for _item in ros_msg.hits:
        convert_bosdyn_msgs_ray_intersection_to_proto(_item, proto.hits.add())

def convert_proto_to_bosdyn_msgs_action_id_query(proto, ros_msg):
    from bosdyn_msgs.msg import CaptureActionId
    ros_msg.action_ids = []
    for _item in proto.action_ids:
        ros_msg.action_ids.append(CaptureActionId())
        convert_proto_to_bosdyn_msgs_capture_action_id(_item, ros_msg.action_ids[-1])

def convert_bosdyn_msgs_action_id_query_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.action_ids[:]
    for _item in ros_msg.action_ids:
        convert_bosdyn_msgs_capture_action_id_to_proto(_item, proto.action_ids.add())

def convert_proto_to_bosdyn_msgs_time_range_query(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.from_timestamp, ros_msg.from_timestamp)
    ros_msg.from_timestamp_is_set = proto.HasField("from_timestamp")
    convert_proto_to_builtin_interfaces_time(proto.to_timestamp, ros_msg.to_timestamp)
    ros_msg.to_timestamp_is_set = proto.HasField("to_timestamp")

def convert_bosdyn_msgs_time_range_query_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.from_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.from_timestamp, proto.from_timestamp)
    if ros_msg.to_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.to_timestamp, proto.to_timestamp)

def convert_proto_to_bosdyn_msgs_data_query_params_one_of_query(proto, ros_msg):
    if proto.HasField("time_range"):
        ros_msg.query_choice = ros_msg.QUERY_TIME_RANGE_SET
        convert_proto_to_bosdyn_msgs_time_range_query(proto.time_range, ros_msg.time_range)
    if proto.HasField("action_ids"):
        ros_msg.query_choice = ros_msg.QUERY_ACTION_IDS_SET
        convert_proto_to_bosdyn_msgs_action_id_query(proto.action_ids, ros_msg.action_ids)

def convert_bosdyn_msgs_data_query_params_one_of_query_to_proto(ros_msg, proto):
    proto.ClearField("query")
    if ros_msg.query_choice == ros_msg.QUERY_TIME_RANGE_SET:
        convert_bosdyn_msgs_time_range_query_to_proto(ros_msg.time_range, proto.time_range)
    if ros_msg.query_choice == ros_msg.QUERY_ACTION_IDS_SET:
        convert_bosdyn_msgs_action_id_query_to_proto(ros_msg.action_ids, proto.action_ids)

def convert_proto_to_bosdyn_msgs_data_query_params(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_data_query_params_one_of_query(proto, ros_msg.query)

def convert_bosdyn_msgs_data_query_params_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_data_query_params_one_of_query_to_proto(ros_msg.query, proto)

def convert_proto_to_bosdyn_msgs_store_image_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_image_capture(proto.image, ros_msg.image)
    ros_msg.image_is_set = proto.HasField("image")
    convert_proto_to_bosdyn_msgs_data_identifier(proto.data_id, ros_msg.data_id)
    ros_msg.data_id_is_set = proto.HasField("data_id")

def convert_bosdyn_msgs_store_image_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.image_is_set:
        convert_bosdyn_msgs_image_capture_to_proto(ros_msg.image, proto.image)
    if ros_msg.data_id_is_set:
        convert_bosdyn_msgs_data_identifier_to_proto(ros_msg.data_id, proto.data_id)

def convert_proto_to_bosdyn_msgs_store_image_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_store_image_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_store_metadata_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_associated_metadata(proto.metadata, ros_msg.metadata)
    ros_msg.metadata_is_set = proto.HasField("metadata")
    convert_proto_to_bosdyn_msgs_data_identifier(proto.data_id, ros_msg.data_id)
    ros_msg.data_id_is_set = proto.HasField("data_id")

def convert_bosdyn_msgs_store_metadata_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.metadata_is_set:
        convert_bosdyn_msgs_associated_metadata_to_proto(ros_msg.metadata, proto.metadata)
    if ros_msg.data_id_is_set:
        convert_bosdyn_msgs_data_identifier_to_proto(ros_msg.data_id, proto.data_id)

def convert_proto_to_bosdyn_msgs_store_metadata_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_store_metadata_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_store_alert_data_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_associated_alert_data(proto.alert_data, ros_msg.alert_data)
    ros_msg.alert_data_is_set = proto.HasField("alert_data")
    convert_proto_to_bosdyn_msgs_data_identifier(proto.data_id, ros_msg.data_id)
    ros_msg.data_id_is_set = proto.HasField("data_id")

def convert_bosdyn_msgs_store_alert_data_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.alert_data_is_set:
        convert_bosdyn_msgs_associated_alert_data_to_proto(ros_msg.alert_data, proto.alert_data)
    if ros_msg.data_id_is_set:
        convert_bosdyn_msgs_data_identifier_to_proto(ros_msg.data_id, proto.data_id)

def convert_proto_to_bosdyn_msgs_store_alert_data_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_store_alert_data_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_store_data_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.data = proto.data
    convert_proto_to_bosdyn_msgs_data_identifier(proto.data_id, ros_msg.data_id)
    ros_msg.data_id_is_set = proto.HasField("data_id")
    ros_msg.file_extension = proto.file_extension

def convert_bosdyn_msgs_store_data_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.data = ros_msg.data
    if ros_msg.data_id_is_set:
        convert_bosdyn_msgs_data_identifier_to_proto(ros_msg.data_id, proto.data_id)
    proto.file_extension = ros_msg.file_extension

def convert_proto_to_bosdyn_msgs_store_data_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_store_data_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_capture_actions_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_query_params(proto.query, ros_msg.query)
    ros_msg.query_is_set = proto.HasField("query")

def convert_bosdyn_msgs_list_capture_actions_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.query_is_set:
        convert_bosdyn_msgs_data_query_params_to_proto(ros_msg.query, proto.query)

def convert_proto_to_bosdyn_msgs_list_capture_actions_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import CaptureActionId
    ros_msg.action_ids = []
    for _item in proto.action_ids:
        ros_msg.action_ids.append(CaptureActionId())
        convert_proto_to_bosdyn_msgs_capture_action_id(_item, ros_msg.action_ids[-1])

def convert_bosdyn_msgs_list_capture_actions_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.action_ids[:]
    for _item in ros_msg.action_ids:
        convert_bosdyn_msgs_capture_action_id_to_proto(_item, proto.action_ids.add())

def convert_proto_to_bosdyn_msgs_list_stored_images_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_query_params(proto.query, ros_msg.query)
    ros_msg.query_is_set = proto.HasField("query")

def convert_bosdyn_msgs_list_stored_images_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.query_is_set:
        convert_bosdyn_msgs_data_query_params_to_proto(ros_msg.query, proto.query)

def convert_proto_to_bosdyn_msgs_list_stored_images_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import DataIdentifier
    ros_msg.data_ids = []
    for _item in proto.data_ids:
        ros_msg.data_ids.append(DataIdentifier())
        convert_proto_to_bosdyn_msgs_data_identifier(_item, ros_msg.data_ids[-1])

def convert_bosdyn_msgs_list_stored_images_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.data_ids[:]
    for _item in ros_msg.data_ids:
        convert_bosdyn_msgs_data_identifier_to_proto(_item, proto.data_ids.add())

def convert_proto_to_bosdyn_msgs_list_stored_metadata_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_query_params(proto.query, ros_msg.query)
    ros_msg.query_is_set = proto.HasField("query")

def convert_bosdyn_msgs_list_stored_metadata_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.query_is_set:
        convert_bosdyn_msgs_data_query_params_to_proto(ros_msg.query, proto.query)

def convert_proto_to_bosdyn_msgs_list_stored_metadata_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import DataIdentifier
    ros_msg.data_ids = []
    for _item in proto.data_ids:
        ros_msg.data_ids.append(DataIdentifier())
        convert_proto_to_bosdyn_msgs_data_identifier(_item, ros_msg.data_ids[-1])

def convert_bosdyn_msgs_list_stored_metadata_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.data_ids[:]
    for _item in ros_msg.data_ids:
        convert_bosdyn_msgs_data_identifier_to_proto(_item, proto.data_ids.add())

def convert_proto_to_bosdyn_msgs_list_stored_alert_data_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_query_params(proto.query, ros_msg.query)
    ros_msg.query_is_set = proto.HasField("query")

def convert_bosdyn_msgs_list_stored_alert_data_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.query_is_set:
        convert_bosdyn_msgs_data_query_params_to_proto(ros_msg.query, proto.query)

def convert_proto_to_bosdyn_msgs_list_stored_alert_data_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import DataIdentifier
    ros_msg.data_ids = []
    for _item in proto.data_ids:
        ros_msg.data_ids.append(DataIdentifier())
        convert_proto_to_bosdyn_msgs_data_identifier(_item, ros_msg.data_ids[-1])

def convert_bosdyn_msgs_list_stored_alert_data_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.data_ids[:]
    for _item in ros_msg.data_ids:
        convert_bosdyn_msgs_data_identifier_to_proto(_item, proto.data_ids.add())

def convert_proto_to_bosdyn_msgs_list_stored_data_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_query_params(proto.query, ros_msg.query)
    ros_msg.query_is_set = proto.HasField("query")

def convert_bosdyn_msgs_list_stored_data_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.query_is_set:
        convert_bosdyn_msgs_data_query_params_to_proto(ros_msg.query, proto.query)

def convert_proto_to_bosdyn_msgs_list_stored_data_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import DataIdentifier
    ros_msg.data_ids = []
    for _item in proto.data_ids:
        ros_msg.data_ids.append(DataIdentifier())
        convert_proto_to_bosdyn_msgs_data_identifier(_item, ros_msg.data_ids[-1])

def convert_bosdyn_msgs_list_stored_data_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.data_ids[:]
    for _item in ros_msg.data_ids:
        convert_bosdyn_msgs_data_identifier_to_proto(_item, proto.data_ids.add())

def convert_proto_to_bosdyn_msgs_register_service_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_endpoint(proto.endpoint, ros_msg.endpoint)
    ros_msg.endpoint_is_set = proto.HasField("endpoint")
    convert_proto_to_bosdyn_msgs_service_entry(proto.service_entry, ros_msg.service_entry)
    ros_msg.service_entry_is_set = proto.HasField("service_entry")

def convert_bosdyn_msgs_register_service_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.endpoint_is_set:
        convert_bosdyn_msgs_endpoint_to_proto(ros_msg.endpoint, proto.endpoint)
    if ros_msg.service_entry_is_set:
        convert_bosdyn_msgs_service_entry_to_proto(ros_msg.service_entry, proto.service_entry)

def convert_proto_to_bosdyn_msgs_register_service_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_register_service_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_unregister_service_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.service_name = proto.service_name

def convert_bosdyn_msgs_unregister_service_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.service_name = ros_msg.service_name

def convert_proto_to_bosdyn_msgs_unregister_service_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_unregister_service_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_update_service_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_endpoint(proto.endpoint, ros_msg.endpoint)
    ros_msg.endpoint_is_set = proto.HasField("endpoint")
    convert_proto_to_bosdyn_msgs_service_entry(proto.service_entry, ros_msg.service_entry)
    ros_msg.service_entry_is_set = proto.HasField("service_entry")

def convert_bosdyn_msgs_update_service_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.endpoint_is_set:
        convert_bosdyn_msgs_endpoint_to_proto(ros_msg.endpoint, proto.endpoint)
    if ros_msg.service_entry_is_set:
        convert_bosdyn_msgs_service_entry_to_proto(ros_msg.service_entry, proto.service_entry)

def convert_proto_to_bosdyn_msgs_update_service_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_update_service_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_vec2(proto, ros_msg):
    ros_msg.x = proto.x
    ros_msg.y = proto.y

def convert_bosdyn_msgs_vec2_to_proto(ros_msg, proto):
    proto.Clear()
    proto.x = ros_msg.x
    proto.y = ros_msg.y

def convert_proto_to_bosdyn_msgs_vec3(proto, ros_msg):
    ros_msg.x = proto.x
    ros_msg.y = proto.y
    ros_msg.z = proto.z

def convert_bosdyn_msgs_vec3_to_proto(ros_msg, proto):
    proto.Clear()
    proto.x = ros_msg.x
    proto.y = ros_msg.y
    proto.z = ros_msg.z

def convert_proto_to_bosdyn_msgs_cylindrical_coordinate(proto, ros_msg):
    ros_msg.r = proto.r
    ros_msg.theta = proto.theta
    ros_msg.z = proto.z

def convert_bosdyn_msgs_cylindrical_coordinate_to_proto(ros_msg, proto):
    proto.Clear()
    proto.r = ros_msg.r
    proto.theta = ros_msg.theta
    proto.z = ros_msg.z

def convert_proto_to_bosdyn_msgs_quaternion(proto, ros_msg):
    ros_msg.x = proto.x
    ros_msg.y = proto.y
    ros_msg.z = proto.z
    ros_msg.w = proto.w

def convert_bosdyn_msgs_quaternion_to_proto(ros_msg, proto):
    proto.Clear()
    proto.x = ros_msg.x
    proto.y = ros_msg.y
    proto.z = ros_msg.z
    proto.w = ros_msg.w

def convert_proto_to_bosdyn_msgs_plane(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.point, ros_msg.point)
    ros_msg.point_is_set = proto.HasField("point")
    convert_proto_to_geometry_msgs_vector3(proto.normal, ros_msg.normal)
    ros_msg.normal_is_set = proto.HasField("normal")

def convert_bosdyn_msgs_plane_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.point_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.point, proto.point)
    if ros_msg.normal_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.normal, proto.normal)

def convert_proto_to_bosdyn_msgs_quad(proto, ros_msg):
    convert_proto_to_geometry_msgs_pose(proto.pose, ros_msg.pose)
    ros_msg.pose_is_set = proto.HasField("pose")
    ros_msg.size = proto.size

def convert_bosdyn_msgs_quad_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.pose_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.pose, proto.pose)
    proto.size = ros_msg.size

def convert_proto_to_bosdyn_msgs_ray(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.origin, ros_msg.origin)
    ros_msg.origin_is_set = proto.HasField("origin")
    convert_proto_to_geometry_msgs_vector3(proto.direction, ros_msg.direction)
    ros_msg.direction_is_set = proto.HasField("direction")

def convert_bosdyn_msgs_ray_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.origin_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.origin, proto.origin)
    if ros_msg.direction_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.direction, proto.direction)

def convert_proto_to_bosdyn_msgs_se2_pose(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.position, ros_msg.position)
    ros_msg.position_is_set = proto.HasField("position")
    ros_msg.angle = proto.angle

def convert_bosdyn_msgs_se2_pose_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.position_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.position, proto.position)
    proto.angle = ros_msg.angle

def convert_proto_to_bosdyn_msgs_se2_velocity(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.linear, ros_msg.linear)
    ros_msg.linear_is_set = proto.HasField("linear")
    ros_msg.angular = proto.angular

def convert_bosdyn_msgs_se2_velocity_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.linear_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.linear, proto.linear)
    proto.angular = ros_msg.angular

def convert_proto_to_bosdyn_msgs_se2_velocity_limit(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_se2_velocity(proto.max_vel, ros_msg.max_vel)
    ros_msg.max_vel_is_set = proto.HasField("max_vel")
    convert_proto_to_bosdyn_msgs_se2_velocity(proto.min_vel, ros_msg.min_vel)
    ros_msg.min_vel_is_set = proto.HasField("min_vel")

def convert_bosdyn_msgs_se2_velocity_limit_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.max_vel_is_set:
        convert_bosdyn_msgs_se2_velocity_to_proto(ros_msg.max_vel, proto.max_vel)
    if ros_msg.min_vel_is_set:
        convert_bosdyn_msgs_se2_velocity_to_proto(ros_msg.min_vel, proto.min_vel)

def convert_proto_to_bosdyn_msgs_se3_pose(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.position, ros_msg.position)
    ros_msg.position_is_set = proto.HasField("position")
    convert_proto_to_geometry_msgs_quaternion(proto.rotation, ros_msg.rotation)
    ros_msg.rotation_is_set = proto.HasField("rotation")

def convert_bosdyn_msgs_se3_pose_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.position_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.position, proto.position)
    if ros_msg.rotation_is_set:
        convert_geometry_msgs_quaternion_to_proto(ros_msg.rotation, proto.rotation)

def convert_proto_to_bosdyn_msgs_se3_velocity(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.linear, ros_msg.linear)
    ros_msg.linear_is_set = proto.HasField("linear")
    convert_proto_to_geometry_msgs_vector3(proto.angular, ros_msg.angular)
    ros_msg.angular_is_set = proto.HasField("angular")

def convert_bosdyn_msgs_se3_velocity_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.linear_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.linear, proto.linear)
    if ros_msg.angular_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.angular, proto.angular)

def convert_proto_to_bosdyn_msgs_wrench(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.force, ros_msg.force)
    ros_msg.force_is_set = proto.HasField("force")
    convert_proto_to_geometry_msgs_vector3(proto.torque, ros_msg.torque)
    ros_msg.torque_is_set = proto.HasField("torque")

def convert_bosdyn_msgs_wrench_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.force_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.force, proto.force)
    if ros_msg.torque_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.torque, proto.torque)

def convert_proto_to_bosdyn_msgs_frame_tree_snapshot_parent_edge(proto, ros_msg):
    ros_msg.parent_frame_name = proto.parent_frame_name
    convert_proto_to_geometry_msgs_pose(proto.parent_tform_child, ros_msg.parent_tform_child)
    ros_msg.parent_tform_child_is_set = proto.HasField("parent_tform_child")

def convert_bosdyn_msgs_frame_tree_snapshot_parent_edge_to_proto(ros_msg, proto):
    proto.Clear()
    proto.parent_frame_name = ros_msg.parent_frame_name
    if ros_msg.parent_tform_child_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.parent_tform_child, proto.parent_tform_child)

def convert_proto_to_bosdyn_msgs_frame_tree_snapshot(proto, ros_msg):
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsParentEdge
    ros_msg.child_to_parent_edge_map = []
    for _item in proto.child_to_parent_edge_map:
        ros_msg.child_to_parent_edge_map.append(KeyStringValueBosdynMsgsParentEdge())
        ros_msg.child_to_parent_edge_map[-1].key = _item
        convert_proto_to_bosdyn_msgs_frame_tree_snapshot_parent_edge(proto.child_to_parent_edge_map[_item], ros_msg.child_to_parent_edge_map[-1].value)

def convert_bosdyn_msgs_frame_tree_snapshot_to_proto(ros_msg, proto):
    proto.Clear()
    for _item in ros_msg.child_to_parent_edge_map:
        convert_bosdyn_msgs_frame_tree_snapshot_parent_edge_to_proto(_item.value, proto.child_to_parent_edge_map[_item.key])

def convert_proto_to_bosdyn_msgs_box2(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.size, ros_msg.size)
    ros_msg.size_is_set = proto.HasField("size")

def convert_bosdyn_msgs_box2_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.size_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.size, proto.size)

def convert_proto_to_bosdyn_msgs_box2_with_frame(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_box2(proto.box, ros_msg.box)
    ros_msg.box_is_set = proto.HasField("box")
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_pose(proto.frame_name_tform_box, ros_msg.frame_name_tform_box)
    ros_msg.frame_name_tform_box_is_set = proto.HasField("frame_name_tform_box")

def convert_bosdyn_msgs_box2_with_frame_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.box_is_set:
        convert_bosdyn_msgs_box2_to_proto(ros_msg.box, proto.box)
    proto.frame_name = ros_msg.frame_name
    if ros_msg.frame_name_tform_box_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.frame_name_tform_box, proto.frame_name_tform_box)

def convert_proto_to_bosdyn_msgs_box3(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.size, ros_msg.size)
    ros_msg.size_is_set = proto.HasField("size")

def convert_bosdyn_msgs_box3_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.size_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.size, proto.size)

def convert_proto_to_bosdyn_msgs_box3_with_frame(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_box3(proto.box, ros_msg.box)
    ros_msg.box_is_set = proto.HasField("box")
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_pose(proto.frame_name_tform_box, ros_msg.frame_name_tform_box)
    ros_msg.frame_name_tform_box_is_set = proto.HasField("frame_name_tform_box")

def convert_bosdyn_msgs_box3_with_frame_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.box_is_set:
        convert_bosdyn_msgs_box3_to_proto(ros_msg.box, proto.box)
    proto.frame_name = ros_msg.frame_name
    if ros_msg.frame_name_tform_box_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.frame_name_tform_box, proto.frame_name_tform_box)

def convert_proto_to_bosdyn_msgs_matrix(proto, ros_msg):
    ros_msg.rows = proto.rows
    ros_msg.cols = proto.cols
    ros_msg.values = []
    for _item in proto.values:
        ros_msg.values.append(_item)

def convert_bosdyn_msgs_matrix_to_proto(ros_msg, proto):
    proto.Clear()
    proto.rows = ros_msg.rows
    proto.cols = ros_msg.cols
    del proto.values[:]
    for _item in ros_msg.values:
        proto.values.add(_item)

def convert_proto_to_bosdyn_msgs_matrixf(proto, ros_msg):
    ros_msg.rows = proto.rows
    ros_msg.cols = proto.cols
    ros_msg.values = []
    for _item in proto.values:
        ros_msg.values.append(_item)

def convert_bosdyn_msgs_matrixf_to_proto(ros_msg, proto):
    proto.Clear()
    proto.rows = ros_msg.rows
    proto.cols = ros_msg.cols
    del proto.values[:]
    for _item in ros_msg.values:
        proto.values.add(_item)

def convert_proto_to_bosdyn_msgs_matrix_int64(proto, ros_msg):
    ros_msg.rows = proto.rows
    ros_msg.cols = proto.cols
    ros_msg.values = []
    for _item in proto.values:
        ros_msg.values.append(_item)

def convert_bosdyn_msgs_matrix_int64_to_proto(ros_msg, proto):
    proto.Clear()
    proto.rows = ros_msg.rows
    proto.cols = ros_msg.cols
    del proto.values[:]
    for _item in ros_msg.values:
        proto.values.add(_item)

def convert_proto_to_bosdyn_msgs_matrix_int32(proto, ros_msg):
    ros_msg.rows = proto.rows
    ros_msg.cols = proto.cols
    ros_msg.values = []
    for _item in proto.values:
        ros_msg.values.append(_item)

def convert_bosdyn_msgs_matrix_int32_to_proto(ros_msg, proto):
    proto.Clear()
    proto.rows = ros_msg.rows
    proto.cols = ros_msg.cols
    del proto.values[:]
    for _item in ros_msg.values:
        proto.values.add(_item)

def convert_proto_to_bosdyn_msgs_vector(proto, ros_msg):
    ros_msg.values = []
    for _item in proto.values:
        ros_msg.values.append(_item)

def convert_bosdyn_msgs_vector_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.values[:]
    for _item in ros_msg.values:
        proto.values.add(_item)

def convert_proto_to_bosdyn_msgs_se3_covariance(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_matrix(proto.matrix, ros_msg.matrix)
    ros_msg.matrix_is_set = proto.HasField("matrix")

def convert_bosdyn_msgs_se3_covariance_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.matrix_is_set:
        convert_bosdyn_msgs_matrix_to_proto(ros_msg.matrix, proto.matrix)

def convert_proto_to_bosdyn_msgs_poly_line(proto, ros_msg):
    from bosdyn_msgs.msg import Vec2
    ros_msg.points = []
    for _item in proto.points:
        ros_msg.points.append(Vec2())
        convert_proto_to_bosdyn_msgs_vec2(_item, ros_msg.points[-1])

def convert_bosdyn_msgs_poly_line_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.points[:]
    for _item in ros_msg.points:
        convert_bosdyn_msgs_vec2_to_proto(_item, proto.points.add())

def convert_proto_to_bosdyn_msgs_polygon(proto, ros_msg):
    from bosdyn_msgs.msg import Vec2
    ros_msg.vertexes = []
    for _item in proto.vertexes:
        ros_msg.vertexes.append(Vec2())
        convert_proto_to_bosdyn_msgs_vec2(_item, ros_msg.vertexes[-1])

def convert_bosdyn_msgs_polygon_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.vertexes[:]
    for _item in ros_msg.vertexes:
        convert_bosdyn_msgs_vec2_to_proto(_item, proto.vertexes.add())

def convert_proto_to_bosdyn_msgs_polygon_with_exclusions(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_polygon(proto.inclusion, ros_msg.inclusion)
    ros_msg.inclusion_is_set = proto.HasField("inclusion")
    from bosdyn_msgs.msg import Polygon
    ros_msg.exclusions = []
    for _item in proto.exclusions:
        ros_msg.exclusions.append(Polygon())
        convert_proto_to_bosdyn_msgs_polygon(_item, ros_msg.exclusions[-1])

def convert_bosdyn_msgs_polygon_with_exclusions_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.inclusion_is_set:
        convert_bosdyn_msgs_polygon_to_proto(ros_msg.inclusion, proto.inclusion)
    del proto.exclusions[:]
    for _item in ros_msg.exclusions:
        convert_bosdyn_msgs_polygon_to_proto(_item, proto.exclusions.add())

def convert_proto_to_bosdyn_msgs_circle(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.center_pt, ros_msg.center_pt)
    ros_msg.center_pt_is_set = proto.HasField("center_pt")
    ros_msg.radius = proto.radius

def convert_bosdyn_msgs_circle_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.center_pt_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.center_pt, proto.center_pt)
    proto.radius = ros_msg.radius

def convert_proto_to_bosdyn_msgs_area_one_of_geometry(proto, ros_msg):
    if proto.HasField("polygon"):
        ros_msg.geometry_choice = ros_msg.GEOMETRY_POLYGON_SET
        convert_proto_to_bosdyn_msgs_polygon(proto.polygon, ros_msg.polygon)
    if proto.HasField("circle"):
        ros_msg.geometry_choice = ros_msg.GEOMETRY_CIRCLE_SET
        convert_proto_to_bosdyn_msgs_circle(proto.circle, ros_msg.circle)

def convert_bosdyn_msgs_area_one_of_geometry_to_proto(ros_msg, proto):
    proto.ClearField("geometry")
    if ros_msg.geometry_choice == ros_msg.GEOMETRY_POLYGON_SET:
        convert_bosdyn_msgs_polygon_to_proto(ros_msg.polygon, proto.polygon)
    if ros_msg.geometry_choice == ros_msg.GEOMETRY_CIRCLE_SET:
        convert_bosdyn_msgs_circle_to_proto(ros_msg.circle, proto.circle)

def convert_proto_to_bosdyn_msgs_area(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_area_one_of_geometry(proto, ros_msg.geometry)

def convert_bosdyn_msgs_area_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_area_one_of_geometry_to_proto(ros_msg.geometry, proto)

def convert_proto_to_bosdyn_msgs_volume_one_of_geometry(proto, ros_msg):
    if proto.HasField("box"):
        ros_msg.geometry_choice = ros_msg.GEOMETRY_BOX_SET
        convert_proto_to_geometry_msgs_vector3(proto.box, ros_msg.box)

def convert_bosdyn_msgs_volume_one_of_geometry_to_proto(ros_msg, proto):
    proto.ClearField("geometry")
    if ros_msg.geometry_choice == ros_msg.GEOMETRY_BOX_SET:
        convert_geometry_msgs_vector3_to_proto(ros_msg.box, proto.box)

def convert_proto_to_bosdyn_msgs_volume(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_volume_one_of_geometry(proto, ros_msg.geometry)

def convert_bosdyn_msgs_volume_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_volume_one_of_geometry_to_proto(ros_msg.geometry, proto)

def convert_proto_to_bosdyn_msgs_bounds(proto, ros_msg):
    ros_msg.lower = proto.lower
    ros_msg.upper = proto.upper

def convert_bosdyn_msgs_bounds_to_proto(ros_msg, proto):
    proto.Clear()
    proto.lower = ros_msg.lower
    proto.upper = ros_msg.upper

def convert_proto_to_bosdyn_msgs_vec2_value(proto, ros_msg):
    ros_msg.x = proto.x.value
    ros_msg.x_is_set = proto.HasField("x")
    ros_msg.y = proto.y.value
    ros_msg.y_is_set = proto.HasField("y")

def convert_bosdyn_msgs_vec2_value_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.x_is_set:
        convert_float64_to_proto(ros_msg.x, proto.x)
    if ros_msg.y_is_set:
        convert_float64_to_proto(ros_msg.y, proto.y)

def convert_proto_to_bosdyn_msgs_vec3_value(proto, ros_msg):
    ros_msg.x = proto.x.value
    ros_msg.x_is_set = proto.HasField("x")
    ros_msg.y = proto.y.value
    ros_msg.y_is_set = proto.HasField("y")
    ros_msg.z = proto.z.value
    ros_msg.z_is_set = proto.HasField("z")

def convert_bosdyn_msgs_vec3_value_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.x_is_set:
        convert_float64_to_proto(ros_msg.x, proto.x)
    if ros_msg.y_is_set:
        convert_float64_to_proto(ros_msg.y, proto.y)
    if ros_msg.z_is_set:
        convert_float64_to_proto(ros_msg.z, proto.z)

def convert_proto_to_bosdyn_msgs_request_header(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.request_timestamp, ros_msg.request_timestamp)
    ros_msg.request_timestamp_is_set = proto.HasField("request_timestamp")
    ros_msg.client_name = proto.client_name
    ros_msg.disable_rpc_logging = proto.disable_rpc_logging

def convert_bosdyn_msgs_request_header_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.request_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.request_timestamp, proto.request_timestamp)
    proto.client_name = ros_msg.client_name
    proto.disable_rpc_logging = ros_msg.disable_rpc_logging

def convert_proto_to_bosdyn_msgs_common_error(proto, ros_msg):
    ros_msg.code.value = proto.code
    ros_msg.message = proto.message

def convert_bosdyn_msgs_common_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.code = ros_msg.code.value
    proto.message = ros_msg.message

def convert_proto_to_bosdyn_msgs_response_header(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.request_header, ros_msg.request_header)
    ros_msg.request_header_is_set = proto.HasField("request_header")
    convert_proto_to_builtin_interfaces_time(proto.request_received_timestamp, ros_msg.request_received_timestamp)
    ros_msg.request_received_timestamp_is_set = proto.HasField("request_received_timestamp")
    convert_proto_to_builtin_interfaces_time(proto.response_timestamp, ros_msg.response_timestamp)
    ros_msg.response_timestamp_is_set = proto.HasField("response_timestamp")
    convert_proto_to_bosdyn_msgs_common_error(proto.error, ros_msg.error)
    ros_msg.error_is_set = proto.HasField("error")

def convert_bosdyn_msgs_response_header_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.request_header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.request_header, proto.request_header)
    if ros_msg.request_received_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.request_received_timestamp, proto.request_received_timestamp)
    if ros_msg.response_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.response_timestamp, proto.response_timestamp)
    if ros_msg.error_is_set:
        convert_bosdyn_msgs_common_error_to_proto(ros_msg.error, proto.error)

def convert_proto_to_bosdyn_msgs_mobility_command_request_one_of_command(proto, ros_msg):
    if proto.HasField("se2_trajectory_request"):
        ros_msg.command_choice = ros_msg.COMMAND_SE2_TRAJECTORY_REQUEST_SET
        convert_proto_to_bosdyn_msgs_se2_trajectory_command_request(proto.se2_trajectory_request, ros_msg.se2_trajectory_request)
    if proto.HasField("se2_velocity_request"):
        ros_msg.command_choice = ros_msg.COMMAND_SE2_VELOCITY_REQUEST_SET
        convert_proto_to_bosdyn_msgs_se2_velocity_command_request(proto.se2_velocity_request, ros_msg.se2_velocity_request)
    if proto.HasField("sit_request"):
        ros_msg.command_choice = ros_msg.COMMAND_SIT_REQUEST_SET
        convert_proto_to_bosdyn_msgs_sit_command_request(proto.sit_request, ros_msg.sit_request)
    if proto.HasField("stand_request"):
        ros_msg.command_choice = ros_msg.COMMAND_STAND_REQUEST_SET
        convert_proto_to_bosdyn_msgs_stand_command_request(proto.stand_request, ros_msg.stand_request)
    if proto.HasField("stance_request"):
        ros_msg.command_choice = ros_msg.COMMAND_STANCE_REQUEST_SET
        convert_proto_to_bosdyn_msgs_stance_command_request(proto.stance_request, ros_msg.stance_request)
    if proto.HasField("stop_request"):
        ros_msg.command_choice = ros_msg.COMMAND_STOP_REQUEST_SET
        convert_proto_to_bosdyn_msgs_stop_command_request(proto.stop_request, ros_msg.stop_request)
    if proto.HasField("follow_arm_request"):
        ros_msg.command_choice = ros_msg.COMMAND_FOLLOW_ARM_REQUEST_SET
        convert_proto_to_bosdyn_msgs_follow_arm_command_request(proto.follow_arm_request, ros_msg.follow_arm_request)

def convert_bosdyn_msgs_mobility_command_request_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_SE2_TRAJECTORY_REQUEST_SET:
        convert_bosdyn_msgs_se2_trajectory_command_request_to_proto(ros_msg.se2_trajectory_request, proto.se2_trajectory_request)
    if ros_msg.command_choice == ros_msg.COMMAND_SE2_VELOCITY_REQUEST_SET:
        convert_bosdyn_msgs_se2_velocity_command_request_to_proto(ros_msg.se2_velocity_request, proto.se2_velocity_request)
    if ros_msg.command_choice == ros_msg.COMMAND_SIT_REQUEST_SET:
        convert_bosdyn_msgs_sit_command_request_to_proto(ros_msg.sit_request, proto.sit_request)
    if ros_msg.command_choice == ros_msg.COMMAND_STAND_REQUEST_SET:
        convert_bosdyn_msgs_stand_command_request_to_proto(ros_msg.stand_request, proto.stand_request)
    if ros_msg.command_choice == ros_msg.COMMAND_STANCE_REQUEST_SET:
        convert_bosdyn_msgs_stance_command_request_to_proto(ros_msg.stance_request, proto.stance_request)
    if ros_msg.command_choice == ros_msg.COMMAND_STOP_REQUEST_SET:
        convert_bosdyn_msgs_stop_command_request_to_proto(ros_msg.stop_request, proto.stop_request)
    if ros_msg.command_choice == ros_msg.COMMAND_FOLLOW_ARM_REQUEST_SET:
        convert_bosdyn_msgs_follow_arm_command_request_to_proto(ros_msg.follow_arm_request, proto.follow_arm_request)

def convert_proto_to_bosdyn_msgs_mobility_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_mobility_command_request_one_of_command(proto, ros_msg.command)
    convert_any_proto_to_bosdyn_msgs_mobility_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")

def convert_bosdyn_msgs_mobility_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_mobility_command_request_one_of_command_to_proto(ros_msg.command, proto)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_mobility_params_to_any_proto(ros_msg.params, proto.params)

def convert_proto_to_bosdyn_msgs_mobility_command_feedback_one_of_feedback(proto, ros_msg):
    if proto.HasField("se2_trajectory_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_se2_trajectory_command_feedback(proto.se2_trajectory_feedback, ros_msg.se2_trajectory_feedback)
    if proto.HasField("se2_velocity_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_SE2_VELOCITY_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_se2_velocity_command_feedback(proto.se2_velocity_feedback, ros_msg.se2_velocity_feedback)
    if proto.HasField("sit_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_SIT_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_sit_command_feedback(proto.sit_feedback, ros_msg.sit_feedback)
    if proto.HasField("stand_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_STAND_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_stand_command_feedback(proto.stand_feedback, ros_msg.stand_feedback)
    if proto.HasField("stance_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_STANCE_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_stance_command_feedback(proto.stance_feedback, ros_msg.stance_feedback)
    if proto.HasField("stop_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_STOP_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_stop_command_feedback(proto.stop_feedback, ros_msg.stop_feedback)
    if proto.HasField("follow_arm_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_FOLLOW_ARM_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_follow_arm_command_feedback(proto.follow_arm_feedback, ros_msg.follow_arm_feedback)

def convert_bosdyn_msgs_mobility_command_feedback_one_of_feedback_to_proto(ros_msg, proto):
    proto.ClearField("feedback")
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_SE2_TRAJECTORY_FEEDBACK_SET:
        convert_bosdyn_msgs_se2_trajectory_command_feedback_to_proto(ros_msg.se2_trajectory_feedback, proto.se2_trajectory_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_SE2_VELOCITY_FEEDBACK_SET:
        convert_bosdyn_msgs_se2_velocity_command_feedback_to_proto(ros_msg.se2_velocity_feedback, proto.se2_velocity_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_SIT_FEEDBACK_SET:
        convert_bosdyn_msgs_sit_command_feedback_to_proto(ros_msg.sit_feedback, proto.sit_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_STAND_FEEDBACK_SET:
        convert_bosdyn_msgs_stand_command_feedback_to_proto(ros_msg.stand_feedback, proto.stand_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_STANCE_FEEDBACK_SET:
        convert_bosdyn_msgs_stance_command_feedback_to_proto(ros_msg.stance_feedback, proto.stance_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_STOP_FEEDBACK_SET:
        convert_bosdyn_msgs_stop_command_feedback_to_proto(ros_msg.stop_feedback, proto.stop_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_FOLLOW_ARM_FEEDBACK_SET:
        convert_bosdyn_msgs_follow_arm_command_feedback_to_proto(ros_msg.follow_arm_feedback, proto.follow_arm_feedback)

def convert_proto_to_bosdyn_msgs_mobility_command_feedback(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_mobility_command_feedback_one_of_feedback(proto, ros_msg.feedback)
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_mobility_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_mobility_command_feedback_one_of_feedback_to_proto(ros_msg.feedback, proto)
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_mobility_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_se2_trajectory(proto, ros_msg):
    from bosdyn_msgs.msg import SE2TrajectoryPoint
    ros_msg.points = []
    for _item in proto.points:
        ros_msg.points.append(SE2TrajectoryPoint())
        convert_proto_to_bosdyn_msgs_se2_trajectory_point(_item, ros_msg.points[-1])
    convert_proto_to_builtin_interfaces_time(proto.reference_time, ros_msg.reference_time)
    ros_msg.reference_time_is_set = proto.HasField("reference_time")
    ros_msg.interpolation.value = proto.interpolation

def convert_bosdyn_msgs_se2_trajectory_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.points[:]
    for _item in ros_msg.points:
        convert_bosdyn_msgs_se2_trajectory_point_to_proto(_item, proto.points.add())
    if ros_msg.reference_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.reference_time, proto.reference_time)
    proto.interpolation = ros_msg.interpolation.value

def convert_proto_to_bosdyn_msgs_se2_trajectory_point(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_se2_pose(proto.pose, ros_msg.pose)
    ros_msg.pose_is_set = proto.HasField("pose")
    convert_proto_to_builtin_interfaces_duration(proto.time_since_reference, ros_msg.time_since_reference)
    ros_msg.time_since_reference_is_set = proto.HasField("time_since_reference")

def convert_bosdyn_msgs_se2_trajectory_point_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.pose_is_set:
        convert_bosdyn_msgs_se2_pose_to_proto(ros_msg.pose, proto.pose)
    if ros_msg.time_since_reference_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.time_since_reference, proto.time_since_reference)

def convert_proto_to_bosdyn_msgs_se3_trajectory(proto, ros_msg):
    from bosdyn_msgs.msg import SE3TrajectoryPoint
    ros_msg.points = []
    for _item in proto.points:
        ros_msg.points.append(SE3TrajectoryPoint())
        convert_proto_to_bosdyn_msgs_se3_trajectory_point(_item, ros_msg.points[-1])
    convert_proto_to_builtin_interfaces_time(proto.reference_time, ros_msg.reference_time)
    ros_msg.reference_time_is_set = proto.HasField("reference_time")
    ros_msg.pos_interpolation.value = proto.pos_interpolation
    ros_msg.ang_interpolation.value = proto.ang_interpolation

def convert_bosdyn_msgs_se3_trajectory_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.points[:]
    for _item in ros_msg.points:
        convert_bosdyn_msgs_se3_trajectory_point_to_proto(_item, proto.points.add())
    if ros_msg.reference_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.reference_time, proto.reference_time)
    proto.pos_interpolation = ros_msg.pos_interpolation.value
    proto.ang_interpolation = ros_msg.ang_interpolation.value

def convert_proto_to_bosdyn_msgs_se3_trajectory_point(proto, ros_msg):
    convert_proto_to_geometry_msgs_pose(proto.pose, ros_msg.pose)
    ros_msg.pose_is_set = proto.HasField("pose")
    convert_proto_to_geometry_msgs_twist(proto.velocity, ros_msg.velocity)
    ros_msg.velocity_is_set = proto.HasField("velocity")
    convert_proto_to_builtin_interfaces_duration(proto.time_since_reference, ros_msg.time_since_reference)
    ros_msg.time_since_reference_is_set = proto.HasField("time_since_reference")

def convert_bosdyn_msgs_se3_trajectory_point_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.pose_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.pose, proto.pose)
    if ros_msg.velocity_is_set:
        convert_geometry_msgs_twist_to_proto(ros_msg.velocity, proto.velocity)
    if ros_msg.time_since_reference_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.time_since_reference, proto.time_since_reference)

def convert_proto_to_bosdyn_msgs_vec3_trajectory(proto, ros_msg):
    from bosdyn_msgs.msg import Vec3TrajectoryPoint
    ros_msg.points = []
    for _item in proto.points:
        ros_msg.points.append(Vec3TrajectoryPoint())
        convert_proto_to_bosdyn_msgs_vec3_trajectory_point(_item, ros_msg.points[-1])
    convert_proto_to_builtin_interfaces_time(proto.reference_time, ros_msg.reference_time)
    ros_msg.reference_time_is_set = proto.HasField("reference_time")
    ros_msg.pos_interpolation.value = proto.pos_interpolation
    convert_proto_to_geometry_msgs_vector3(proto.starting_velocity, ros_msg.starting_velocity)
    ros_msg.starting_velocity_is_set = proto.HasField("starting_velocity")
    convert_proto_to_geometry_msgs_vector3(proto.ending_velocity, ros_msg.ending_velocity)
    ros_msg.ending_velocity_is_set = proto.HasField("ending_velocity")

def convert_bosdyn_msgs_vec3_trajectory_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.points[:]
    for _item in ros_msg.points:
        convert_bosdyn_msgs_vec3_trajectory_point_to_proto(_item, proto.points.add())
    if ros_msg.reference_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.reference_time, proto.reference_time)
    proto.pos_interpolation = ros_msg.pos_interpolation.value
    if ros_msg.starting_velocity_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.starting_velocity, proto.starting_velocity)
    if ros_msg.ending_velocity_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.ending_velocity, proto.ending_velocity)

def convert_proto_to_bosdyn_msgs_vec3_trajectory_point(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.point, ros_msg.point)
    ros_msg.point_is_set = proto.HasField("point")
    ros_msg.linear_speed = proto.linear_speed
    convert_proto_to_builtin_interfaces_duration(proto.time_since_reference, ros_msg.time_since_reference)
    ros_msg.time_since_reference_is_set = proto.HasField("time_since_reference")

def convert_bosdyn_msgs_vec3_trajectory_point_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.point_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.point, proto.point)
    proto.linear_speed = ros_msg.linear_speed
    if ros_msg.time_since_reference_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.time_since_reference, proto.time_since_reference)

def convert_proto_to_bosdyn_msgs_wrench_trajectory(proto, ros_msg):
    from bosdyn_msgs.msg import WrenchTrajectoryPoint
    ros_msg.points = []
    for _item in proto.points:
        ros_msg.points.append(WrenchTrajectoryPoint())
        convert_proto_to_bosdyn_msgs_wrench_trajectory_point(_item, ros_msg.points[-1])
    convert_proto_to_builtin_interfaces_time(proto.reference_time, ros_msg.reference_time)
    ros_msg.reference_time_is_set = proto.HasField("reference_time")

def convert_bosdyn_msgs_wrench_trajectory_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.points[:]
    for _item in ros_msg.points:
        convert_bosdyn_msgs_wrench_trajectory_point_to_proto(_item, proto.points.add())
    if ros_msg.reference_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.reference_time, proto.reference_time)

def convert_proto_to_bosdyn_msgs_wrench_trajectory_point(proto, ros_msg):
    convert_proto_to_geometry_msgs_wrench(proto.wrench, ros_msg.wrench)
    ros_msg.wrench_is_set = proto.HasField("wrench")
    convert_proto_to_builtin_interfaces_duration(proto.time_since_reference, ros_msg.time_since_reference)
    ros_msg.time_since_reference_is_set = proto.HasField("time_since_reference")

def convert_bosdyn_msgs_wrench_trajectory_point_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.wrench_is_set:
        convert_geometry_msgs_wrench_to_proto(ros_msg.wrench, proto.wrench)
    if ros_msg.time_since_reference_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.time_since_reference, proto.time_since_reference)

def convert_proto_to_bosdyn_msgs_scalar_trajectory(proto, ros_msg):
    from bosdyn_msgs.msg import ScalarTrajectoryPoint
    ros_msg.points = []
    for _item in proto.points:
        ros_msg.points.append(ScalarTrajectoryPoint())
        convert_proto_to_bosdyn_msgs_scalar_trajectory_point(_item, ros_msg.points[-1])
    convert_proto_to_builtin_interfaces_time(proto.reference_time, ros_msg.reference_time)
    ros_msg.reference_time_is_set = proto.HasField("reference_time")
    ros_msg.interpolation.value = proto.interpolation

def convert_bosdyn_msgs_scalar_trajectory_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.points[:]
    for _item in ros_msg.points:
        convert_bosdyn_msgs_scalar_trajectory_point_to_proto(_item, proto.points.add())
    if ros_msg.reference_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.reference_time, proto.reference_time)
    proto.interpolation = ros_msg.interpolation.value

def convert_proto_to_bosdyn_msgs_scalar_trajectory_point(proto, ros_msg):
    ros_msg.point = proto.point
    ros_msg.velocity = proto.velocity.value
    ros_msg.velocity_is_set = proto.HasField("velocity")
    convert_proto_to_builtin_interfaces_duration(proto.time_since_reference, ros_msg.time_since_reference)
    ros_msg.time_since_reference_is_set = proto.HasField("time_since_reference")

def convert_bosdyn_msgs_scalar_trajectory_point_to_proto(ros_msg, proto):
    proto.Clear()
    proto.point = ros_msg.point
    if ros_msg.velocity_is_set:
        convert_float64_to_proto(ros_msg.velocity, proto.velocity)
    if ros_msg.time_since_reference_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.time_since_reference, proto.time_since_reference)

def convert_bosdyn_msgs_robot_command_feedback_status_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_battery_change_pose_command_request(proto, ros_msg):
    ros_msg.direction_hint.value = proto.direction_hint

def convert_bosdyn_msgs_battery_change_pose_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.direction_hint = ros_msg.direction_hint.value

def convert_proto_to_bosdyn_msgs_battery_change_pose_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_battery_change_pose_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_battery_change_pose_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_self_right_command_request_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_self_right_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_self_right_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_self_right_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_stop_command_request_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_stop_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_stop_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_freeze_command_request_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_freeze_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_freeze_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_safe_power_off_command_request(proto, ros_msg):
    ros_msg.unsafe_action.value = proto.unsafe_action

def convert_bosdyn_msgs_safe_power_off_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.unsafe_action = ros_msg.unsafe_action.value

def convert_proto_to_bosdyn_msgs_safe_power_off_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_safe_power_off_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_safe_power_off_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_se2_trajectory_command_request(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.se2_frame_name = proto.se2_frame_name
    convert_proto_to_bosdyn_msgs_se2_trajectory(proto.trajectory, ros_msg.trajectory)
    ros_msg.trajectory_is_set = proto.HasField("trajectory")

def convert_bosdyn_msgs_se2_trajectory_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    proto.se2_frame_name = ros_msg.se2_frame_name
    if ros_msg.trajectory_is_set:
        convert_bosdyn_msgs_se2_trajectory_to_proto(ros_msg.trajectory, proto.trajectory)

def convert_proto_to_bosdyn_msgs_se2_trajectory_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.body_movement_status.value = proto.body_movement_status

def convert_bosdyn_msgs_se2_trajectory_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.body_movement_status = ros_msg.body_movement_status.value

def convert_bosdyn_msgs_se2_trajectory_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_se2_velocity_command_request(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.se2_frame_name = proto.se2_frame_name
    convert_proto_to_bosdyn_msgs_se2_velocity(proto.velocity, ros_msg.velocity)
    ros_msg.velocity_is_set = proto.HasField("velocity")
    convert_proto_to_bosdyn_msgs_se2_velocity(proto.slew_rate_limit, ros_msg.slew_rate_limit)
    ros_msg.slew_rate_limit_is_set = proto.HasField("slew_rate_limit")

def convert_bosdyn_msgs_se2_velocity_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    proto.se2_frame_name = ros_msg.se2_frame_name
    if ros_msg.velocity_is_set:
        convert_bosdyn_msgs_se2_velocity_to_proto(ros_msg.velocity, proto.velocity)
    if ros_msg.slew_rate_limit_is_set:
        convert_bosdyn_msgs_se2_velocity_to_proto(ros_msg.slew_rate_limit, proto.slew_rate_limit)

def convert_bosdyn_msgs_se2_velocity_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_se2_velocity_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_sit_command_request_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_sit_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_sit_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_sit_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_stand_command_request_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_stand_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.standing_state.value = proto.standing_state

def convert_bosdyn_msgs_stand_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.standing_state = ros_msg.standing_state.value

def convert_bosdyn_msgs_stand_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_stance_command_request(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    convert_proto_to_bosdyn_msgs_stance(proto.stance, ros_msg.stance)
    ros_msg.stance_is_set = proto.HasField("stance")

def convert_bosdyn_msgs_stance_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    if ros_msg.stance_is_set:
        convert_bosdyn_msgs_stance_to_proto(ros_msg.stance, proto.stance)

def convert_proto_to_bosdyn_msgs_stance_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_stance_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_stance_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_stance(proto, ros_msg):
    ros_msg.se2_frame_name = proto.se2_frame_name
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsVec2
    ros_msg.foot_positions = []
    for _item in proto.foot_positions:
        ros_msg.foot_positions.append(KeyStringValueBosdynMsgsVec2())
        ros_msg.foot_positions[-1].key = _item
        convert_proto_to_bosdyn_msgs_vec2(proto.foot_positions[_item], ros_msg.foot_positions[-1].value)
    ros_msg.accuracy = proto.accuracy

def convert_bosdyn_msgs_stance_to_proto(ros_msg, proto):
    proto.Clear()
    proto.se2_frame_name = ros_msg.se2_frame_name
    for _item in ros_msg.foot_positions:
        convert_bosdyn_msgs_vec2_to_proto(_item.value, proto.foot_positions[_item.key])
    proto.accuracy = ros_msg.accuracy

def convert_proto_to_bosdyn_msgs_follow_arm_command_request(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.body_offset_from_hand, ros_msg.body_offset_from_hand)
    ros_msg.body_offset_from_hand_is_set = proto.HasField("body_offset_from_hand")

def convert_bosdyn_msgs_follow_arm_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.body_offset_from_hand_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.body_offset_from_hand, proto.body_offset_from_hand)

def convert_bosdyn_msgs_follow_arm_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_follow_arm_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_arm_drag_command_request_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_arm_drag_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_arm_drag_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_arm_drag_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_constrained_manipulation_command_request_one_of_task_speed(proto, ros_msg):
    if proto.HasField("tangential_speed"):
        ros_msg.task_speed_choice = ros_msg.TASK_SPEED_TANGENTIAL_SPEED_SET
        ros_msg.tangential_speed = proto.tangential_speed
    if proto.HasField("rotational_speed"):
        ros_msg.task_speed_choice = ros_msg.TASK_SPEED_ROTATIONAL_SPEED_SET
        ros_msg.rotational_speed = proto.rotational_speed

def convert_bosdyn_msgs_constrained_manipulation_command_request_one_of_task_speed_to_proto(ros_msg, proto):
    proto.ClearField("task_speed")
    if ros_msg.task_speed_choice == ros_msg.TASK_SPEED_TANGENTIAL_SPEED_SET:
        proto.tangential_speed = ros_msg.tangential_speed
    if ros_msg.task_speed_choice == ros_msg.TASK_SPEED_ROTATIONAL_SPEED_SET:
        proto.rotational_speed = ros_msg.rotational_speed

def convert_proto_to_bosdyn_msgs_constrained_manipulation_command_request(proto, ros_msg):
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_wrench(proto.init_wrench_direction_in_frame_name, ros_msg.init_wrench_direction_in_frame_name)
    ros_msg.init_wrench_direction_in_frame_name_is_set = proto.HasField("init_wrench_direction_in_frame_name")
    convert_proto_to_bosdyn_msgs_constrained_manipulation_command_request_one_of_task_speed(proto, ros_msg.task_speed)
    ros_msg.force_limit = proto.force_limit.value
    ros_msg.force_limit_is_set = proto.HasField("force_limit")
    ros_msg.torque_limit = proto.torque_limit.value
    ros_msg.torque_limit_is_set = proto.HasField("torque_limit")
    ros_msg.task_type.value = proto.task_type
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.enable_robot_locomotion = proto.enable_robot_locomotion.value
    ros_msg.enable_robot_locomotion_is_set = proto.HasField("enable_robot_locomotion")

def convert_bosdyn_msgs_constrained_manipulation_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.frame_name = ros_msg.frame_name
    if ros_msg.init_wrench_direction_in_frame_name_is_set:
        convert_geometry_msgs_wrench_to_proto(ros_msg.init_wrench_direction_in_frame_name, proto.init_wrench_direction_in_frame_name)
    convert_bosdyn_msgs_constrained_manipulation_command_request_one_of_task_speed_to_proto(ros_msg.task_speed, proto)
    if ros_msg.force_limit_is_set:
        convert_float64_to_proto(ros_msg.force_limit, proto.force_limit)
    if ros_msg.torque_limit_is_set:
        convert_float64_to_proto(ros_msg.torque_limit, proto.torque_limit)
    proto.task_type = ros_msg.task_type.value
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    if ros_msg.enable_robot_locomotion_is_set:
        convert_bool_to_proto(ros_msg.enable_robot_locomotion, proto.enable_robot_locomotion)

def convert_proto_to_bosdyn_msgs_constrained_manipulation_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status
    convert_proto_to_geometry_msgs_wrench(proto.desired_wrench_odom_frame, ros_msg.desired_wrench_odom_frame)
    ros_msg.desired_wrench_odom_frame_is_set = proto.HasField("desired_wrench_odom_frame")

def convert_bosdyn_msgs_constrained_manipulation_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    if ros_msg.desired_wrench_odom_frame_is_set:
        convert_geometry_msgs_wrench_to_proto(ros_msg.desired_wrench_odom_frame, proto.desired_wrench_odom_frame)

def convert_bosdyn_msgs_constrained_manipulation_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_association(proto, ros_msg):
    ros_msg.mac_address = proto.mac_address
    convert_proto_to_builtin_interfaces_duration(proto.connected_time, ros_msg.connected_time)
    ros_msg.connected_time_is_set = proto.HasField("connected_time")
    ros_msg.rx_signal_dbm = proto.rx_signal_dbm
    ros_msg.rx_signal_avg_dbm = proto.rx_signal_avg_dbm
    ros_msg.rx_beacon_signal_avg_dbm = proto.rx_beacon_signal_avg_dbm
    ros_msg.expected_bits_per_second = proto.expected_bits_per_second
    ros_msg.rx_bytes = proto.rx_bytes
    ros_msg.rx_packets = proto.rx_packets
    ros_msg.rx_bits_per_second = proto.rx_bits_per_second
    ros_msg.tx_bytes = proto.tx_bytes
    ros_msg.tx_packets = proto.tx_packets
    ros_msg.tx_bits_per_second = proto.tx_bits_per_second
    ros_msg.tx_retries = proto.tx_retries
    ros_msg.tx_failed = proto.tx_failed
    ros_msg.beacons_received = proto.beacons_received
    ros_msg.beacon_loss_count = proto.beacon_loss_count

def convert_bosdyn_msgs_association_to_proto(ros_msg, proto):
    proto.Clear()
    proto.mac_address = ros_msg.mac_address
    if ros_msg.connected_time_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.connected_time, proto.connected_time)
    proto.rx_signal_dbm = ros_msg.rx_signal_dbm
    proto.rx_signal_avg_dbm = ros_msg.rx_signal_avg_dbm
    proto.rx_beacon_signal_avg_dbm = ros_msg.rx_beacon_signal_avg_dbm
    proto.expected_bits_per_second = ros_msg.expected_bits_per_second
    proto.rx_bytes = ros_msg.rx_bytes
    proto.rx_packets = ros_msg.rx_packets
    proto.rx_bits_per_second = ros_msg.rx_bits_per_second
    proto.tx_bytes = ros_msg.tx_bytes
    proto.tx_packets = ros_msg.tx_packets
    proto.tx_bits_per_second = ros_msg.tx_bits_per_second
    proto.tx_retries = ros_msg.tx_retries
    proto.tx_failed = ros_msg.tx_failed
    proto.beacons_received = ros_msg.beacons_received
    proto.beacon_loss_count = ros_msg.beacon_loss_count

def convert_proto_to_bosdyn_msgs_wifi_device(proto, ros_msg):
    ros_msg.type.value = proto.type
    ros_msg.name = proto.name
    ros_msg.mac_address = proto.mac_address
    ros_msg.ssid = proto.ssid
    ros_msg.tx_power_dbm = proto.tx_power_dbm
    from bosdyn_msgs.msg import Association
    ros_msg.associations = []
    for _item in proto.associations:
        ros_msg.associations.append(Association())
        convert_proto_to_bosdyn_msgs_association(_item, ros_msg.associations[-1])

def convert_bosdyn_msgs_wifi_device_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    proto.name = ros_msg.name
    proto.mac_address = ros_msg.mac_address
    proto.ssid = ros_msg.ssid
    proto.tx_power_dbm = ros_msg.tx_power_dbm
    del proto.associations[:]
    for _item in ros_msg.associations:
        convert_bosdyn_msgs_association_to_proto(_item, proto.associations.add())

def convert_proto_to_bosdyn_msgs_wifi_stats(proto, ros_msg):
    ros_msg.hostname = proto.hostname
    from bosdyn_msgs.msg import WifiDevice
    ros_msg.devices = []
    for _item in proto.devices:
        ros_msg.devices.append(WifiDevice())
        convert_proto_to_bosdyn_msgs_wifi_device(_item, ros_msg.devices[-1])

def convert_bosdyn_msgs_wifi_stats_to_proto(ros_msg, proto):
    proto.Clear()
    proto.hostname = ros_msg.hostname
    del proto.devices[:]
    for _item in ros_msg.devices:
        convert_bosdyn_msgs_wifi_device_to_proto(_item, proto.devices.add())

def convert_proto_to_bosdyn_msgs_skeleton_link_obj_model(proto, ros_msg):
    ros_msg.file_name = proto.file_name
    ros_msg.file_contents = proto.file_contents

def convert_bosdyn_msgs_skeleton_link_obj_model_to_proto(ros_msg, proto):
    proto.Clear()
    proto.file_name = ros_msg.file_name
    proto.file_contents = ros_msg.file_contents

def convert_proto_to_bosdyn_msgs_skeleton_link(proto, ros_msg):
    ros_msg.name = proto.name
    convert_proto_to_bosdyn_msgs_skeleton_link_obj_model(proto.obj_model, ros_msg.obj_model)
    ros_msg.obj_model_is_set = proto.HasField("obj_model")

def convert_bosdyn_msgs_skeleton_link_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    if ros_msg.obj_model_is_set:
        convert_bosdyn_msgs_skeleton_link_obj_model_to_proto(ros_msg.obj_model, proto.obj_model)

def convert_proto_to_bosdyn_msgs_skeleton(proto, ros_msg):
    from bosdyn_msgs.msg import Link
    ros_msg.links = []
    for _item in proto.links:
        ros_msg.links.append(Link())
        convert_proto_to_bosdyn_msgs_skeleton_link(_item, ros_msg.links[-1])
    ros_msg.urdf = proto.urdf

def convert_bosdyn_msgs_skeleton_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.links[:]
    for _item in ros_msg.links:
        convert_bosdyn_msgs_skeleton_link_to_proto(_item, proto.links.add())
    proto.urdf = ros_msg.urdf

def convert_proto_to_bosdyn_msgs_hardware_configuration(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_skeleton(proto.skeleton, ros_msg.skeleton)
    ros_msg.skeleton_is_set = proto.HasField("skeleton")
    ros_msg.can_power_command_request_off_robot = proto.can_power_command_request_off_robot
    ros_msg.can_power_command_request_cycle_robot = proto.can_power_command_request_cycle_robot
    ros_msg.can_power_command_request_payload_ports = proto.can_power_command_request_payload_ports
    ros_msg.can_power_command_request_wifi_radio = proto.can_power_command_request_wifi_radio

def convert_bosdyn_msgs_hardware_configuration_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.skeleton_is_set:
        convert_bosdyn_msgs_skeleton_to_proto(ros_msg.skeleton, proto.skeleton)
    proto.can_power_command_request_off_robot = ros_msg.can_power_command_request_off_robot
    proto.can_power_command_request_cycle_robot = ros_msg.can_power_command_request_cycle_robot
    proto.can_power_command_request_payload_ports = ros_msg.can_power_command_request_payload_ports
    proto.can_power_command_request_wifi_radio = ros_msg.can_power_command_request_wifi_radio

def convert_proto_to_bosdyn_msgs_robot_state(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_power_state(proto.power_state, ros_msg.power_state)
    ros_msg.power_state_is_set = proto.HasField("power_state")
    from bosdyn_msgs.msg import BatteryState
    ros_msg.battery_states = []
    for _item in proto.battery_states:
        ros_msg.battery_states.append(BatteryState())
        convert_proto_to_bosdyn_msgs_battery_state(_item, ros_msg.battery_states[-1])
    from bosdyn_msgs.msg import CommsState
    ros_msg.comms_states = []
    for _item in proto.comms_states:
        ros_msg.comms_states.append(CommsState())
        convert_proto_to_bosdyn_msgs_comms_state(_item, ros_msg.comms_states[-1])
    convert_proto_to_bosdyn_msgs_system_fault_state(proto.system_fault_state, ros_msg.system_fault_state)
    ros_msg.system_fault_state_is_set = proto.HasField("system_fault_state")
    from bosdyn_msgs.msg import EStopState
    ros_msg.estop_states = []
    for _item in proto.estop_states:
        ros_msg.estop_states.append(EStopState())
        convert_proto_to_bosdyn_msgs_e_stop_state(_item, ros_msg.estop_states[-1])
    convert_proto_to_bosdyn_msgs_kinematic_state(proto.kinematic_state, ros_msg.kinematic_state)
    ros_msg.kinematic_state_is_set = proto.HasField("kinematic_state")
    convert_proto_to_bosdyn_msgs_behavior_fault_state(proto.behavior_fault_state, ros_msg.behavior_fault_state)
    ros_msg.behavior_fault_state_is_set = proto.HasField("behavior_fault_state")
    from bosdyn_msgs.msg import FootState
    ros_msg.foot_state = []
    for _item in proto.foot_state:
        ros_msg.foot_state.append(FootState())
        convert_proto_to_bosdyn_msgs_foot_state(_item, ros_msg.foot_state[-1])
    convert_proto_to_bosdyn_msgs_manipulator_state(proto.manipulator_state, ros_msg.manipulator_state)
    ros_msg.manipulator_state_is_set = proto.HasField("manipulator_state")
    convert_proto_to_bosdyn_msgs_service_fault_state(proto.service_fault_state, ros_msg.service_fault_state)
    ros_msg.service_fault_state_is_set = proto.HasField("service_fault_state")
    convert_proto_to_bosdyn_msgs_terrain_state(proto.terrain_state, ros_msg.terrain_state)
    ros_msg.terrain_state_is_set = proto.HasField("terrain_state")

def convert_bosdyn_msgs_robot_state_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.power_state_is_set:
        convert_bosdyn_msgs_power_state_to_proto(ros_msg.power_state, proto.power_state)
    del proto.battery_states[:]
    for _item in ros_msg.battery_states:
        convert_bosdyn_msgs_battery_state_to_proto(_item, proto.battery_states.add())
    del proto.comms_states[:]
    for _item in ros_msg.comms_states:
        convert_bosdyn_msgs_comms_state_to_proto(_item, proto.comms_states.add())
    if ros_msg.system_fault_state_is_set:
        convert_bosdyn_msgs_system_fault_state_to_proto(ros_msg.system_fault_state, proto.system_fault_state)
    del proto.estop_states[:]
    for _item in ros_msg.estop_states:
        convert_bosdyn_msgs_e_stop_state_to_proto(_item, proto.estop_states.add())
    if ros_msg.kinematic_state_is_set:
        convert_bosdyn_msgs_kinematic_state_to_proto(ros_msg.kinematic_state, proto.kinematic_state)
    if ros_msg.behavior_fault_state_is_set:
        convert_bosdyn_msgs_behavior_fault_state_to_proto(ros_msg.behavior_fault_state, proto.behavior_fault_state)
    del proto.foot_state[:]
    for _item in ros_msg.foot_state:
        convert_bosdyn_msgs_foot_state_to_proto(_item, proto.foot_state.add())
    if ros_msg.manipulator_state_is_set:
        convert_bosdyn_msgs_manipulator_state_to_proto(ros_msg.manipulator_state, proto.manipulator_state)
    if ros_msg.service_fault_state_is_set:
        convert_bosdyn_msgs_service_fault_state_to_proto(ros_msg.service_fault_state, proto.service_fault_state)
    if ros_msg.terrain_state_is_set:
        convert_bosdyn_msgs_terrain_state_to_proto(ros_msg.terrain_state, proto.terrain_state)

def convert_proto_to_bosdyn_msgs_power_state(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.motor_power_state.value = proto.motor_power_state
    ros_msg.shore_power_state.value = proto.shore_power_state
    ros_msg.robot_power_state.value = proto.robot_power_state
    ros_msg.payload_ports_power_state.value = proto.payload_ports_power_state
    ros_msg.wifi_radio_power_state.value = proto.wifi_radio_power_state
    ros_msg.locomotion_charge_percentage = proto.locomotion_charge_percentage.value
    ros_msg.locomotion_charge_percentage_is_set = proto.HasField("locomotion_charge_percentage")
    convert_proto_to_builtin_interfaces_duration(proto.locomotion_estimated_runtime, ros_msg.locomotion_estimated_runtime)
    ros_msg.locomotion_estimated_runtime_is_set = proto.HasField("locomotion_estimated_runtime")

def convert_bosdyn_msgs_power_state_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.motor_power_state = ros_msg.motor_power_state.value
    proto.shore_power_state = ros_msg.shore_power_state.value
    proto.robot_power_state = ros_msg.robot_power_state.value
    proto.payload_ports_power_state = ros_msg.payload_ports_power_state.value
    proto.wifi_radio_power_state = ros_msg.wifi_radio_power_state.value
    if ros_msg.locomotion_charge_percentage_is_set:
        convert_float64_to_proto(ros_msg.locomotion_charge_percentage, proto.locomotion_charge_percentage)
    if ros_msg.locomotion_estimated_runtime_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.locomotion_estimated_runtime, proto.locomotion_estimated_runtime)

def convert_proto_to_bosdyn_msgs_system_fault_state(proto, ros_msg):
    from bosdyn_msgs.msg import SystemFault
    ros_msg.faults = []
    for _item in proto.faults:
        ros_msg.faults.append(SystemFault())
        convert_proto_to_bosdyn_msgs_system_fault(_item, ros_msg.faults[-1])
    from bosdyn_msgs.msg import SystemFault
    ros_msg.historical_faults = []
    for _item in proto.historical_faults:
        ros_msg.historical_faults.append(SystemFault())
        convert_proto_to_bosdyn_msgs_system_fault(_item, ros_msg.historical_faults[-1])
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsSystemFaultSeverity
    ros_msg.aggregated = []
    for _item in proto.aggregated:
        ros_msg.aggregated.append(KeyStringValueBosdynMsgsSystemFaultSeverity())
        ros_msg.aggregated[-1].key = _item
        convert_proto_to_bosdyn_msgs_system_fault_severity(proto.aggregated[_item], ros_msg.aggregated[-1].value)

def convert_bosdyn_msgs_system_fault_state_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.faults[:]
    for _item in ros_msg.faults:
        convert_bosdyn_msgs_system_fault_to_proto(_item, proto.faults.add())
    del proto.historical_faults[:]
    for _item in ros_msg.historical_faults:
        convert_bosdyn_msgs_system_fault_to_proto(_item, proto.historical_faults.add())
    for _item in ros_msg.aggregated:
        convert_bosdyn_msgs_system_fault_severity_to_proto(_item.value, proto.aggregated[_item.key])

def convert_proto_to_bosdyn_msgs_system_fault(proto, ros_msg):
    ros_msg.name = proto.name
    convert_proto_to_builtin_interfaces_time(proto.onset_timestamp, ros_msg.onset_timestamp)
    ros_msg.onset_timestamp_is_set = proto.HasField("onset_timestamp")
    convert_proto_to_builtin_interfaces_duration(proto.duration, ros_msg.duration)
    ros_msg.duration_is_set = proto.HasField("duration")
    ros_msg.code = proto.code
    ros_msg.uid = proto.uid
    ros_msg.error_message = proto.error_message
    ros_msg.attributes = []
    for _item in proto.attributes:
        ros_msg.attributes.append(_item)
    ros_msg.severity.value = proto.severity

def convert_bosdyn_msgs_system_fault_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    if ros_msg.onset_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.onset_timestamp, proto.onset_timestamp)
    if ros_msg.duration_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.duration, proto.duration)
    proto.code = ros_msg.code
    proto.uid = ros_msg.uid
    proto.error_message = ros_msg.error_message
    del proto.attributes[:]
    for _item in ros_msg.attributes:
        proto.attributes.add(_item)
    proto.severity = ros_msg.severity.value

def convert_proto_to_bosdyn_msgs_e_stop_state(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.name = proto.name
    ros_msg.type.value = proto.type
    ros_msg.state.value = proto.state
    ros_msg.state_description = proto.state_description

def convert_bosdyn_msgs_e_stop_state_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.name = ros_msg.name
    proto.type = ros_msg.type.value
    proto.state = ros_msg.state.value
    proto.state_description = ros_msg.state_description

def convert_proto_to_bosdyn_msgs_battery_state(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.identifier = proto.identifier
    ros_msg.charge_percentage = proto.charge_percentage.value
    ros_msg.charge_percentage_is_set = proto.HasField("charge_percentage")
    convert_proto_to_builtin_interfaces_duration(proto.estimated_runtime, ros_msg.estimated_runtime)
    ros_msg.estimated_runtime_is_set = proto.HasField("estimated_runtime")
    ros_msg.current = proto.current.value
    ros_msg.current_is_set = proto.HasField("current")
    ros_msg.voltage = proto.voltage.value
    ros_msg.voltage_is_set = proto.HasField("voltage")
    ros_msg.temperatures = []
    for _item in proto.temperatures:
        ros_msg.temperatures.append(_item)
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_battery_state_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.identifier = ros_msg.identifier
    if ros_msg.charge_percentage_is_set:
        convert_float64_to_proto(ros_msg.charge_percentage, proto.charge_percentage)
    if ros_msg.estimated_runtime_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.estimated_runtime, proto.estimated_runtime)
    if ros_msg.current_is_set:
        convert_float64_to_proto(ros_msg.current, proto.current)
    if ros_msg.voltage_is_set:
        convert_float64_to_proto(ros_msg.voltage, proto.voltage)
    del proto.temperatures[:]
    for _item in ros_msg.temperatures:
        proto.temperatures.add(_item)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_kinematic_state(proto, ros_msg):
    from bosdyn_msgs.msg import JointState
    ros_msg.joint_states = []
    for _item in proto.joint_states:
        ros_msg.joint_states.append(JointState())
        convert_proto_to_bosdyn_msgs_joint_state(_item, ros_msg.joint_states[-1])
    convert_proto_to_builtin_interfaces_time(proto.acquisition_timestamp, ros_msg.acquisition_timestamp)
    ros_msg.acquisition_timestamp_is_set = proto.HasField("acquisition_timestamp")
    convert_proto_to_geometry_msgs_twist(proto.velocity_of_body_in_vision, ros_msg.velocity_of_body_in_vision)
    ros_msg.velocity_of_body_in_vision_is_set = proto.HasField("velocity_of_body_in_vision")
    convert_proto_to_geometry_msgs_twist(proto.velocity_of_body_in_odom, ros_msg.velocity_of_body_in_odom)
    ros_msg.velocity_of_body_in_odom_is_set = proto.HasField("velocity_of_body_in_odom")

def convert_bosdyn_msgs_kinematic_state_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.joint_states[:]
    for _item in ros_msg.joint_states:
        convert_bosdyn_msgs_joint_state_to_proto(_item, proto.joint_states.add())
    if ros_msg.acquisition_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.acquisition_timestamp, proto.acquisition_timestamp)
    if ros_msg.velocity_of_body_in_vision_is_set:
        convert_geometry_msgs_twist_to_proto(ros_msg.velocity_of_body_in_vision, proto.velocity_of_body_in_vision)
    if ros_msg.velocity_of_body_in_odom_is_set:
        convert_geometry_msgs_twist_to_proto(ros_msg.velocity_of_body_in_odom, proto.velocity_of_body_in_odom)

def convert_proto_to_bosdyn_msgs_joint_state(proto, ros_msg):
    ros_msg.name = proto.name
    ros_msg.position = proto.position.value
    ros_msg.position_is_set = proto.HasField("position")
    ros_msg.velocity = proto.velocity.value
    ros_msg.velocity_is_set = proto.HasField("velocity")
    ros_msg.acceleration = proto.acceleration.value
    ros_msg.acceleration_is_set = proto.HasField("acceleration")
    ros_msg.load = proto.load.value
    ros_msg.load_is_set = proto.HasField("load")

def convert_bosdyn_msgs_joint_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    if ros_msg.position_is_set:
        convert_float64_to_proto(ros_msg.position, proto.position)
    if ros_msg.velocity_is_set:
        convert_float64_to_proto(ros_msg.velocity, proto.velocity)
    if ros_msg.acceleration_is_set:
        convert_float64_to_proto(ros_msg.acceleration, proto.acceleration)
    if ros_msg.load_is_set:
        convert_float64_to_proto(ros_msg.load, proto.load)

def convert_proto_to_bosdyn_msgs_behavior_fault_state(proto, ros_msg):
    from bosdyn_msgs.msg import BehaviorFault
    ros_msg.faults = []
    for _item in proto.faults:
        ros_msg.faults.append(BehaviorFault())
        convert_proto_to_bosdyn_msgs_behavior_fault(_item, ros_msg.faults[-1])

def convert_bosdyn_msgs_behavior_fault_state_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.faults[:]
    for _item in ros_msg.faults:
        convert_bosdyn_msgs_behavior_fault_to_proto(_item, proto.faults.add())

def convert_proto_to_bosdyn_msgs_behavior_fault(proto, ros_msg):
    ros_msg.behavior_fault_id = proto.behavior_fault_id
    convert_proto_to_builtin_interfaces_time(proto.onset_timestamp, ros_msg.onset_timestamp)
    ros_msg.onset_timestamp_is_set = proto.HasField("onset_timestamp")
    ros_msg.cause.value = proto.cause
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_behavior_fault_to_proto(ros_msg, proto):
    proto.Clear()
    proto.behavior_fault_id = ros_msg.behavior_fault_id
    if ros_msg.onset_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.onset_timestamp, proto.onset_timestamp)
    proto.cause = ros_msg.cause.value
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_robot_metrics(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    from bosdyn_msgs.msg import Parameter
    ros_msg.metrics = []
    for _item in proto.metrics:
        ros_msg.metrics.append(Parameter())
        convert_proto_to_bosdyn_msgs_parameter(_item, ros_msg.metrics[-1])

def convert_bosdyn_msgs_robot_metrics_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    del proto.metrics[:]
    for _item in ros_msg.metrics:
        convert_bosdyn_msgs_parameter_to_proto(_item, proto.metrics.add())

def convert_proto_to_bosdyn_msgs_comms_state_one_of_state(proto, ros_msg):
    if proto.HasField("wifi_state"):
        ros_msg.state_choice = ros_msg.STATE_WIFI_STATE_SET
        convert_proto_to_bosdyn_msgs_wi_fi_state(proto.wifi_state, ros_msg.wifi_state)

def convert_bosdyn_msgs_comms_state_one_of_state_to_proto(ros_msg, proto):
    proto.ClearField("state")
    if ros_msg.state_choice == ros_msg.STATE_WIFI_STATE_SET:
        convert_bosdyn_msgs_wi_fi_state_to_proto(ros_msg.wifi_state, proto.wifi_state)

def convert_proto_to_bosdyn_msgs_comms_state(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    convert_proto_to_bosdyn_msgs_comms_state_one_of_state(proto, ros_msg.state)

def convert_bosdyn_msgs_comms_state_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    convert_bosdyn_msgs_comms_state_one_of_state_to_proto(ros_msg.state, proto)

def convert_proto_to_bosdyn_msgs_wi_fi_state(proto, ros_msg):
    ros_msg.current_mode.value = proto.current_mode
    ros_msg.essid = proto.essid

def convert_bosdyn_msgs_wi_fi_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.current_mode = ros_msg.current_mode.value
    proto.essid = ros_msg.essid

def convert_proto_to_bosdyn_msgs_foot_state_terrain_state(proto, ros_msg):
    ros_msg.ground_mu_est = proto.ground_mu_est
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_vector3(proto.foot_slip_distance_rt_frame, ros_msg.foot_slip_distance_rt_frame)
    ros_msg.foot_slip_distance_rt_frame_is_set = proto.HasField("foot_slip_distance_rt_frame")
    convert_proto_to_geometry_msgs_vector3(proto.foot_slip_velocity_rt_frame, ros_msg.foot_slip_velocity_rt_frame)
    ros_msg.foot_slip_velocity_rt_frame_is_set = proto.HasField("foot_slip_velocity_rt_frame")
    convert_proto_to_geometry_msgs_vector3(proto.ground_contact_normal_rt_frame, ros_msg.ground_contact_normal_rt_frame)
    ros_msg.ground_contact_normal_rt_frame_is_set = proto.HasField("ground_contact_normal_rt_frame")
    ros_msg.visual_surface_ground_penetration_mean = proto.visual_surface_ground_penetration_mean
    ros_msg.visual_surface_ground_penetration_std = proto.visual_surface_ground_penetration_std

def convert_bosdyn_msgs_foot_state_terrain_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.ground_mu_est = ros_msg.ground_mu_est
    proto.frame_name = ros_msg.frame_name
    if ros_msg.foot_slip_distance_rt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.foot_slip_distance_rt_frame, proto.foot_slip_distance_rt_frame)
    if ros_msg.foot_slip_velocity_rt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.foot_slip_velocity_rt_frame, proto.foot_slip_velocity_rt_frame)
    if ros_msg.ground_contact_normal_rt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.ground_contact_normal_rt_frame, proto.ground_contact_normal_rt_frame)
    proto.visual_surface_ground_penetration_mean = ros_msg.visual_surface_ground_penetration_mean
    proto.visual_surface_ground_penetration_std = ros_msg.visual_surface_ground_penetration_std

def convert_proto_to_bosdyn_msgs_foot_state(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.foot_position_rt_body, ros_msg.foot_position_rt_body)
    ros_msg.foot_position_rt_body_is_set = proto.HasField("foot_position_rt_body")
    ros_msg.contact.value = proto.contact
    convert_proto_to_bosdyn_msgs_foot_state_terrain_state(proto.terrain, ros_msg.terrain)
    ros_msg.terrain_is_set = proto.HasField("terrain")

def convert_bosdyn_msgs_foot_state_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.foot_position_rt_body_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.foot_position_rt_body, proto.foot_position_rt_body)
    proto.contact = ros_msg.contact.value
    if ros_msg.terrain_is_set:
        convert_bosdyn_msgs_foot_state_terrain_state_to_proto(ros_msg.terrain, proto.terrain)

def convert_proto_to_bosdyn_msgs_manipulator_state(proto, ros_msg):
    ros_msg.gripper_open_percentage = proto.gripper_open_percentage
    ros_msg.is_gripper_holding_item = proto.is_gripper_holding_item
    convert_proto_to_geometry_msgs_vector3(proto.estimated_end_effector_force_in_hand, ros_msg.estimated_end_effector_force_in_hand)
    ros_msg.estimated_end_effector_force_in_hand_is_set = proto.HasField("estimated_end_effector_force_in_hand")
    ros_msg.stow_state.value = proto.stow_state
    convert_proto_to_geometry_msgs_twist(proto.velocity_of_hand_in_vision, ros_msg.velocity_of_hand_in_vision)
    ros_msg.velocity_of_hand_in_vision_is_set = proto.HasField("velocity_of_hand_in_vision")
    convert_proto_to_geometry_msgs_twist(proto.velocity_of_hand_in_odom, ros_msg.velocity_of_hand_in_odom)
    ros_msg.velocity_of_hand_in_odom_is_set = proto.HasField("velocity_of_hand_in_odom")
    ros_msg.carry_state.value = proto.carry_state

def convert_bosdyn_msgs_manipulator_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.gripper_open_percentage = ros_msg.gripper_open_percentage
    proto.is_gripper_holding_item = ros_msg.is_gripper_holding_item
    if ros_msg.estimated_end_effector_force_in_hand_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.estimated_end_effector_force_in_hand, proto.estimated_end_effector_force_in_hand)
    proto.stow_state = ros_msg.stow_state.value
    if ros_msg.velocity_of_hand_in_vision_is_set:
        convert_geometry_msgs_twist_to_proto(ros_msg.velocity_of_hand_in_vision, proto.velocity_of_hand_in_vision)
    if ros_msg.velocity_of_hand_in_odom_is_set:
        convert_geometry_msgs_twist_to_proto(ros_msg.velocity_of_hand_in_odom, proto.velocity_of_hand_in_odom)
    proto.carry_state = ros_msg.carry_state.value

def convert_proto_to_bosdyn_msgs_service_fault_state(proto, ros_msg):
    from bosdyn_msgs.msg import ServiceFault
    ros_msg.faults = []
    for _item in proto.faults:
        ros_msg.faults.append(ServiceFault())
        convert_proto_to_bosdyn_msgs_service_fault(_item, ros_msg.faults[-1])
    from bosdyn_msgs.msg import ServiceFault
    ros_msg.historical_faults = []
    for _item in proto.historical_faults:
        ros_msg.historical_faults.append(ServiceFault())
        convert_proto_to_bosdyn_msgs_service_fault(_item, ros_msg.historical_faults[-1])
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsServiceFaultSeverity
    ros_msg.aggregated = []
    for _item in proto.aggregated:
        ros_msg.aggregated.append(KeyStringValueBosdynMsgsServiceFaultSeverity())
        ros_msg.aggregated[-1].key = _item
        convert_proto_to_bosdyn_msgs_service_fault_severity(proto.aggregated[_item], ros_msg.aggregated[-1].value)

def convert_bosdyn_msgs_service_fault_state_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.faults[:]
    for _item in ros_msg.faults:
        convert_bosdyn_msgs_service_fault_to_proto(_item, proto.faults.add())
    del proto.historical_faults[:]
    for _item in ros_msg.historical_faults:
        convert_bosdyn_msgs_service_fault_to_proto(_item, proto.historical_faults.add())
    for _item in ros_msg.aggregated:
        convert_bosdyn_msgs_service_fault_severity_to_proto(_item.value, proto.aggregated[_item.key])

def convert_proto_to_bosdyn_msgs_terrain_state(proto, ros_msg):
    ros_msg.is_unsafe_to_sit = proto.is_unsafe_to_sit

def convert_bosdyn_msgs_terrain_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.is_unsafe_to_sit = ros_msg.is_unsafe_to_sit

def convert_proto_to_bosdyn_msgs_robot_state_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_robot_state_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_robot_state_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_robot_state(proto.robot_state, ros_msg.robot_state)
    ros_msg.robot_state_is_set = proto.HasField("robot_state")

def convert_bosdyn_msgs_robot_state_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.robot_state_is_set:
        convert_bosdyn_msgs_robot_state_to_proto(ros_msg.robot_state, proto.robot_state)

def convert_proto_to_bosdyn_msgs_robot_metrics_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_robot_metrics_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_robot_metrics_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_robot_metrics(proto.robot_metrics, ros_msg.robot_metrics)
    ros_msg.robot_metrics_is_set = proto.HasField("robot_metrics")

def convert_bosdyn_msgs_robot_metrics_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.robot_metrics_is_set:
        convert_bosdyn_msgs_robot_metrics_to_proto(ros_msg.robot_metrics, proto.robot_metrics)

def convert_proto_to_bosdyn_msgs_robot_hardware_configuration_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_robot_hardware_configuration_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_robot_hardware_configuration_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_hardware_configuration(proto.hardware_configuration, ros_msg.hardware_configuration)
    ros_msg.hardware_configuration_is_set = proto.HasField("hardware_configuration")

def convert_bosdyn_msgs_robot_hardware_configuration_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.hardware_configuration_is_set:
        convert_bosdyn_msgs_hardware_configuration_to_proto(ros_msg.hardware_configuration, proto.hardware_configuration)

def convert_proto_to_bosdyn_msgs_robot_link_model_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.link_name = proto.link_name

def convert_bosdyn_msgs_robot_link_model_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.link_name = ros_msg.link_name

def convert_proto_to_bosdyn_msgs_robot_link_model_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_skeleton_link_obj_model(proto.link_model, ros_msg.link_model)
    ros_msg.link_model_is_set = proto.HasField("link_model")

def convert_bosdyn_msgs_robot_link_model_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.link_model_is_set:
        convert_bosdyn_msgs_skeleton_link_obj_model_to_proto(ros_msg.link_model, proto.link_model)

def convert_proto_to_bosdyn_msgs_robot_impaired_state(proto, ros_msg):
    ros_msg.impaired_status.value = proto.impaired_status
    from bosdyn_msgs.msg import SystemFault
    ros_msg.system_faults = []
    for _item in proto.system_faults:
        ros_msg.system_faults.append(SystemFault())
        convert_proto_to_bosdyn_msgs_system_fault(_item, ros_msg.system_faults[-1])
    from bosdyn_msgs.msg import ServiceFault
    ros_msg.service_faults = []
    for _item in proto.service_faults:
        ros_msg.service_faults.append(ServiceFault())
        convert_proto_to_bosdyn_msgs_service_fault(_item, ros_msg.service_faults[-1])
    from bosdyn_msgs.msg import BehaviorFault
    ros_msg.behavior_faults = []
    for _item in proto.behavior_faults:
        ros_msg.behavior_faults.append(BehaviorFault())
        convert_proto_to_bosdyn_msgs_behavior_fault(_item, ros_msg.behavior_faults[-1])

def convert_bosdyn_msgs_robot_impaired_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.impaired_status = ros_msg.impaired_status.value
    del proto.system_faults[:]
    for _item in ros_msg.system_faults:
        convert_bosdyn_msgs_system_fault_to_proto(_item, proto.system_faults.add())
    del proto.service_faults[:]
    for _item in ros_msg.service_faults:
        convert_bosdyn_msgs_service_fault_to_proto(_item, proto.service_faults.add())
    del proto.behavior_faults[:]
    for _item in ros_msg.behavior_faults:
        convert_bosdyn_msgs_behavior_fault_to_proto(_item, proto.behavior_faults.add())

def convert_proto_to_bosdyn_msgs_point_cloud_source(proto, ros_msg):
    ros_msg.name = proto.name
    ros_msg.frame_name_sensor = proto.frame_name_sensor
    convert_proto_to_builtin_interfaces_time(proto.acquisition_time, ros_msg.acquisition_time)
    ros_msg.acquisition_time_is_set = proto.HasField("acquisition_time")

def convert_bosdyn_msgs_point_cloud_source_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    proto.frame_name_sensor = ros_msg.frame_name_sensor
    if ros_msg.acquisition_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.acquisition_time, proto.acquisition_time)

def convert_proto_to_bosdyn_msgs_point_cloud_encoding_parameters(proto, ros_msg):
    ros_msg.scale_factor = proto.scale_factor
    ros_msg.max_x = proto.max_x
    ros_msg.max_y = proto.max_y
    ros_msg.max_z = proto.max_z
    ros_msg.remapping_constant = proto.remapping_constant
    ros_msg.bytes_per_point = proto.bytes_per_point

def convert_bosdyn_msgs_point_cloud_encoding_parameters_to_proto(ros_msg, proto):
    proto.Clear()
    proto.scale_factor = ros_msg.scale_factor
    proto.max_x = ros_msg.max_x
    proto.max_y = ros_msg.max_y
    proto.max_z = ros_msg.max_z
    proto.remapping_constant = ros_msg.remapping_constant
    proto.bytes_per_point = ros_msg.bytes_per_point

def convert_proto_to_bosdyn_msgs_point_cloud(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_point_cloud_source(proto.source, ros_msg.source)
    ros_msg.source_is_set = proto.HasField("source")
    ros_msg.num_points = proto.num_points
    ros_msg.encoding.value = proto.encoding
    convert_proto_to_bosdyn_msgs_point_cloud_encoding_parameters(proto.encoding_parameters, ros_msg.encoding_parameters)
    ros_msg.encoding_parameters_is_set = proto.HasField("encoding_parameters")
    ros_msg.data = proto.data

def convert_bosdyn_msgs_point_cloud_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.source_is_set:
        convert_bosdyn_msgs_point_cloud_source_to_proto(ros_msg.source, proto.source)
    proto.num_points = ros_msg.num_points
    proto.encoding = ros_msg.encoding.value
    if ros_msg.encoding_parameters_is_set:
        convert_bosdyn_msgs_point_cloud_encoding_parameters_to_proto(ros_msg.encoding_parameters, proto.encoding_parameters)
    proto.data = ros_msg.data

def convert_proto_to_bosdyn_msgs_list_point_cloud_sources_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_point_cloud_sources_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_point_cloud_sources_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import PointCloudSource
    ros_msg.point_cloud_sources = []
    for _item in proto.point_cloud_sources:
        ros_msg.point_cloud_sources.append(PointCloudSource())
        convert_proto_to_bosdyn_msgs_point_cloud_source(_item, ros_msg.point_cloud_sources[-1])

def convert_bosdyn_msgs_list_point_cloud_sources_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.point_cloud_sources[:]
    for _item in ros_msg.point_cloud_sources:
        convert_bosdyn_msgs_point_cloud_source_to_proto(_item, proto.point_cloud_sources.add())

def convert_proto_to_bosdyn_msgs_point_cloud_request(proto, ros_msg):
    ros_msg.point_cloud_source_name = proto.point_cloud_source_name

def convert_bosdyn_msgs_point_cloud_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.point_cloud_source_name = ros_msg.point_cloud_source_name

def convert_proto_to_bosdyn_msgs_get_point_cloud_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import PointCloudRequest
    ros_msg.point_cloud_requests = []
    for _item in proto.point_cloud_requests:
        ros_msg.point_cloud_requests.append(PointCloudRequest())
        convert_proto_to_bosdyn_msgs_point_cloud_request(_item, ros_msg.point_cloud_requests[-1])

def convert_bosdyn_msgs_get_point_cloud_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.point_cloud_requests[:]
    for _item in ros_msg.point_cloud_requests:
        convert_bosdyn_msgs_point_cloud_request_to_proto(_item, proto.point_cloud_requests.add())

def convert_proto_to_bosdyn_msgs_point_cloud_response(proto, ros_msg):
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_point_cloud(proto.point_cloud, ros_msg.point_cloud)
    ros_msg.point_cloud_is_set = proto.HasField("point_cloud")

def convert_bosdyn_msgs_point_cloud_response_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    if ros_msg.point_cloud_is_set:
        convert_bosdyn_msgs_point_cloud_to_proto(ros_msg.point_cloud, proto.point_cloud)

def convert_proto_to_bosdyn_msgs_get_point_cloud_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import PointCloudResponse
    ros_msg.point_cloud_responses = []
    for _item in proto.point_cloud_responses:
        ros_msg.point_cloud_responses.append(PointCloudResponse())
        convert_proto_to_bosdyn_msgs_point_cloud_response(_item, ros_msg.point_cloud_responses[-1])

def convert_bosdyn_msgs_get_point_cloud_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.point_cloud_responses[:]
    for _item in ros_msg.point_cloud_responses:
        convert_bosdyn_msgs_point_cloud_response_to_proto(_item, proto.point_cloud_responses.add())

def convert_proto_to_bosdyn_msgs_walk_to_object_ray_in_world(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.ray_start_rt_frame, ros_msg.ray_start_rt_frame)
    ros_msg.ray_start_rt_frame_is_set = proto.HasField("ray_start_rt_frame")
    convert_proto_to_geometry_msgs_vector3(proto.ray_end_rt_frame, ros_msg.ray_end_rt_frame)
    ros_msg.ray_end_rt_frame_is_set = proto.HasField("ray_end_rt_frame")
    ros_msg.frame_name = proto.frame_name
    ros_msg.offset_distance = proto.offset_distance.value
    ros_msg.offset_distance_is_set = proto.HasField("offset_distance")

def convert_bosdyn_msgs_walk_to_object_ray_in_world_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.ray_start_rt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.ray_start_rt_frame, proto.ray_start_rt_frame)
    if ros_msg.ray_end_rt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.ray_end_rt_frame, proto.ray_end_rt_frame)
    proto.frame_name = ros_msg.frame_name
    if ros_msg.offset_distance_is_set:
        convert_float32_to_proto(ros_msg.offset_distance, proto.offset_distance)

def convert_proto_to_bosdyn_msgs_walk_to_object_in_image(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.pixel_xy, ros_msg.pixel_xy)
    ros_msg.pixel_xy_is_set = proto.HasField("pixel_xy")
    ros_msg.frame_name_image_sensor = proto.frame_name_image_sensor
    convert_proto_to_bosdyn_msgs_image_source_pinhole_model(proto.camera_model, ros_msg.camera_model)
    ros_msg.camera_model_is_set = proto.HasField("camera_model")
    ros_msg.offset_distance = proto.offset_distance.value
    ros_msg.offset_distance_is_set = proto.HasField("offset_distance")

def convert_bosdyn_msgs_walk_to_object_in_image_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.pixel_xy_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.pixel_xy, proto.pixel_xy)
    proto.frame_name_image_sensor = ros_msg.frame_name_image_sensor
    if ros_msg.camera_model_is_set:
        convert_bosdyn_msgs_image_source_pinhole_model_to_proto(ros_msg.camera_model, proto.camera_model)
    if ros_msg.offset_distance_is_set:
        convert_float32_to_proto(ros_msg.offset_distance, proto.offset_distance)

def convert_proto_to_bosdyn_msgs_pick_object_ray_in_world(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.ray_start_rt_frame, ros_msg.ray_start_rt_frame)
    ros_msg.ray_start_rt_frame_is_set = proto.HasField("ray_start_rt_frame")
    convert_proto_to_geometry_msgs_vector3(proto.ray_end_rt_frame, ros_msg.ray_end_rt_frame)
    ros_msg.ray_end_rt_frame_is_set = proto.HasField("ray_end_rt_frame")
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_bosdyn_msgs_grasp_params(proto.grasp_params, ros_msg.grasp_params)
    ros_msg.grasp_params_is_set = proto.HasField("grasp_params")
    ros_msg.walk_gaze_mode.value = proto.walk_gaze_mode

def convert_bosdyn_msgs_pick_object_ray_in_world_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.ray_start_rt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.ray_start_rt_frame, proto.ray_start_rt_frame)
    if ros_msg.ray_end_rt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.ray_end_rt_frame, proto.ray_end_rt_frame)
    proto.frame_name = ros_msg.frame_name
    if ros_msg.grasp_params_is_set:
        convert_bosdyn_msgs_grasp_params_to_proto(ros_msg.grasp_params, proto.grasp_params)
    proto.walk_gaze_mode = ros_msg.walk_gaze_mode.value

def convert_bosdyn_msgs_pick_object_execute_plan_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_pick_object(proto, ros_msg):
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_vector3(proto.object_rt_frame, ros_msg.object_rt_frame)
    ros_msg.object_rt_frame_is_set = proto.HasField("object_rt_frame")
    convert_proto_to_bosdyn_msgs_grasp_params(proto.grasp_params, ros_msg.grasp_params)
    ros_msg.grasp_params_is_set = proto.HasField("grasp_params")

def convert_bosdyn_msgs_pick_object_to_proto(ros_msg, proto):
    proto.Clear()
    proto.frame_name = ros_msg.frame_name
    if ros_msg.object_rt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.object_rt_frame, proto.object_rt_frame)
    if ros_msg.grasp_params_is_set:
        convert_bosdyn_msgs_grasp_params_to_proto(ros_msg.grasp_params, proto.grasp_params)

def convert_proto_to_bosdyn_msgs_pick_object_in_image(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.pixel_xy, ros_msg.pixel_xy)
    ros_msg.pixel_xy_is_set = proto.HasField("pixel_xy")
    ros_msg.frame_name_image_sensor = proto.frame_name_image_sensor
    convert_proto_to_bosdyn_msgs_image_source_pinhole_model(proto.camera_model, ros_msg.camera_model)
    ros_msg.camera_model_is_set = proto.HasField("camera_model")
    convert_proto_to_bosdyn_msgs_grasp_params(proto.grasp_params, ros_msg.grasp_params)
    ros_msg.grasp_params_is_set = proto.HasField("grasp_params")
    ros_msg.walk_gaze_mode.value = proto.walk_gaze_mode

def convert_bosdyn_msgs_pick_object_in_image_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.pixel_xy_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.pixel_xy, proto.pixel_xy)
    proto.frame_name_image_sensor = ros_msg.frame_name_image_sensor
    if ros_msg.camera_model_is_set:
        convert_bosdyn_msgs_image_source_pinhole_model_to_proto(ros_msg.camera_model, proto.camera_model)
    if ros_msg.grasp_params_is_set:
        convert_bosdyn_msgs_grasp_params_to_proto(ros_msg.grasp_params, proto.grasp_params)
    proto.walk_gaze_mode = ros_msg.walk_gaze_mode.value

def convert_proto_to_bosdyn_msgs_grasp_params(proto, ros_msg):
    ros_msg.grasp_palm_to_fingertip = proto.grasp_palm_to_fingertip
    ros_msg.grasp_params_frame_name = proto.grasp_params_frame_name
    from bosdyn_msgs.msg import AllowableOrientation
    ros_msg.allowable_orientation = []
    for _item in proto.allowable_orientation:
        ros_msg.allowable_orientation.append(AllowableOrientation())
        convert_proto_to_bosdyn_msgs_allowable_orientation(_item, ros_msg.allowable_orientation[-1])
    ros_msg.position_constraint.value = proto.position_constraint
    ros_msg.manipulation_camera_source.value = proto.manipulation_camera_source

def convert_bosdyn_msgs_grasp_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.grasp_palm_to_fingertip = ros_msg.grasp_palm_to_fingertip
    proto.grasp_params_frame_name = ros_msg.grasp_params_frame_name
    del proto.allowable_orientation[:]
    for _item in ros_msg.allowable_orientation:
        convert_bosdyn_msgs_allowable_orientation_to_proto(_item, proto.allowable_orientation.add())
    proto.position_constraint = ros_msg.position_constraint.value
    proto.manipulation_camera_source = ros_msg.manipulation_camera_source.value

def convert_proto_to_bosdyn_msgs_allowable_orientation_one_of_constraint(proto, ros_msg):
    if proto.HasField("rotation_with_tolerance"):
        ros_msg.constraint_choice = ros_msg.CONSTRAINT_ROTATION_WITH_TOLERANCE_SET
        convert_proto_to_bosdyn_msgs_rotation_with_tolerance(proto.rotation_with_tolerance, ros_msg.rotation_with_tolerance)
    if proto.HasField("vector_alignment_with_tolerance"):
        ros_msg.constraint_choice = ros_msg.CONSTRAINT_VECTOR_ALIGNMENT_WITH_TOLERANCE_SET
        convert_proto_to_bosdyn_msgs_vector_alignment_with_tolerance(proto.vector_alignment_with_tolerance, ros_msg.vector_alignment_with_tolerance)
    if proto.HasField("squeeze_grasp"):
        ros_msg.constraint_choice = ros_msg.CONSTRAINT_SQUEEZE_GRASP_SET
        convert_proto_to_bosdyn_msgs_squeeze_grasp(proto.squeeze_grasp, ros_msg.squeeze_grasp)

def convert_bosdyn_msgs_allowable_orientation_one_of_constraint_to_proto(ros_msg, proto):
    proto.ClearField("constraint")
    if ros_msg.constraint_choice == ros_msg.CONSTRAINT_ROTATION_WITH_TOLERANCE_SET:
        convert_bosdyn_msgs_rotation_with_tolerance_to_proto(ros_msg.rotation_with_tolerance, proto.rotation_with_tolerance)
    if ros_msg.constraint_choice == ros_msg.CONSTRAINT_VECTOR_ALIGNMENT_WITH_TOLERANCE_SET:
        convert_bosdyn_msgs_vector_alignment_with_tolerance_to_proto(ros_msg.vector_alignment_with_tolerance, proto.vector_alignment_with_tolerance)
    if ros_msg.constraint_choice == ros_msg.CONSTRAINT_SQUEEZE_GRASP_SET:
        convert_bosdyn_msgs_squeeze_grasp_to_proto(ros_msg.squeeze_grasp, proto.squeeze_grasp)

def convert_proto_to_bosdyn_msgs_allowable_orientation(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_allowable_orientation_one_of_constraint(proto, ros_msg.constraint)

def convert_bosdyn_msgs_allowable_orientation_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_allowable_orientation_one_of_constraint_to_proto(ros_msg.constraint, proto)

def convert_proto_to_bosdyn_msgs_rotation_with_tolerance(proto, ros_msg):
    convert_proto_to_geometry_msgs_quaternion(proto.rotation_ewrt_frame, ros_msg.rotation_ewrt_frame)
    ros_msg.rotation_ewrt_frame_is_set = proto.HasField("rotation_ewrt_frame")
    ros_msg.threshold_radians = proto.threshold_radians

def convert_bosdyn_msgs_rotation_with_tolerance_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.rotation_ewrt_frame_is_set:
        convert_geometry_msgs_quaternion_to_proto(ros_msg.rotation_ewrt_frame, proto.rotation_ewrt_frame)
    proto.threshold_radians = ros_msg.threshold_radians

def convert_proto_to_bosdyn_msgs_vector_alignment_with_tolerance(proto, ros_msg):
    convert_proto_to_geometry_msgs_vector3(proto.axis_on_gripper_ewrt_gripper, ros_msg.axis_on_gripper_ewrt_gripper)
    ros_msg.axis_on_gripper_ewrt_gripper_is_set = proto.HasField("axis_on_gripper_ewrt_gripper")
    convert_proto_to_geometry_msgs_vector3(proto.axis_to_align_with_ewrt_frame, ros_msg.axis_to_align_with_ewrt_frame)
    ros_msg.axis_to_align_with_ewrt_frame_is_set = proto.HasField("axis_to_align_with_ewrt_frame")
    ros_msg.threshold_radians = proto.threshold_radians

def convert_bosdyn_msgs_vector_alignment_with_tolerance_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.axis_on_gripper_ewrt_gripper_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.axis_on_gripper_ewrt_gripper, proto.axis_on_gripper_ewrt_gripper)
    if ros_msg.axis_to_align_with_ewrt_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.axis_to_align_with_ewrt_frame, proto.axis_to_align_with_ewrt_frame)
    proto.threshold_radians = ros_msg.threshold_radians

def convert_proto_to_bosdyn_msgs_squeeze_grasp(proto, ros_msg):
    ros_msg.squeeze_grasp_disallowed = proto.squeeze_grasp_disallowed

def convert_bosdyn_msgs_squeeze_grasp_to_proto(ros_msg, proto):
    proto.Clear()
    proto.squeeze_grasp_disallowed = ros_msg.squeeze_grasp_disallowed

def convert_proto_to_bosdyn_msgs_manipulation_api_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.manipulation_cmd_id = proto.manipulation_cmd_id

def convert_bosdyn_msgs_manipulation_api_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.manipulation_cmd_id = ros_msg.manipulation_cmd_id

def convert_proto_to_bosdyn_msgs_manipulation_api_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.manipulation_cmd_id = proto.manipulation_cmd_id
    ros_msg.current_state.value = proto.current_state

def convert_bosdyn_msgs_manipulation_api_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.manipulation_cmd_id = ros_msg.manipulation_cmd_id
    proto.current_state = ros_msg.current_state.value

def convert_proto_to_bosdyn_msgs_manipulation_api_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.manipulation_cmd_id = proto.manipulation_cmd_id
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_manipulation_api_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.manipulation_cmd_id = ros_msg.manipulation_cmd_id
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_manipulation_api_request_one_of_manipulation_cmd(proto, ros_msg):
    if proto.HasField("walk_to_object_ray_in_world"):
        ros_msg.manipulation_cmd_choice = ros_msg.MANIPULATION_CMD_WALK_TO_OBJECT_RAY_IN_WORLD_SET
        convert_proto_to_bosdyn_msgs_walk_to_object_ray_in_world(proto.walk_to_object_ray_in_world, ros_msg.walk_to_object_ray_in_world)
    if proto.HasField("walk_to_object_in_image"):
        ros_msg.manipulation_cmd_choice = ros_msg.MANIPULATION_CMD_WALK_TO_OBJECT_IN_IMAGE_SET
        convert_proto_to_bosdyn_msgs_walk_to_object_in_image(proto.walk_to_object_in_image, ros_msg.walk_to_object_in_image)
    if proto.HasField("pick_object"):
        ros_msg.manipulation_cmd_choice = ros_msg.MANIPULATION_CMD_PICK_OBJECT_SET
        convert_proto_to_bosdyn_msgs_pick_object(proto.pick_object, ros_msg.pick_object)
    if proto.HasField("pick_object_in_image"):
        ros_msg.manipulation_cmd_choice = ros_msg.MANIPULATION_CMD_PICK_OBJECT_IN_IMAGE_SET
        convert_proto_to_bosdyn_msgs_pick_object_in_image(proto.pick_object_in_image, ros_msg.pick_object_in_image)
    if proto.HasField("pick_object_ray_in_world"):
        ros_msg.manipulation_cmd_choice = ros_msg.MANIPULATION_CMD_PICK_OBJECT_RAY_IN_WORLD_SET
        convert_proto_to_bosdyn_msgs_pick_object_ray_in_world(proto.pick_object_ray_in_world, ros_msg.pick_object_ray_in_world)
    if proto.HasField("pick_object_execute_plan"):
        ros_msg.manipulation_cmd_choice = ros_msg.MANIPULATION_CMD_PICK_OBJECT_EXECUTE_PLAN_SET
        convert_proto_to_bosdyn_msgs_pick_object_execute_plan(proto.pick_object_execute_plan, ros_msg.pick_object_execute_plan)

def convert_bosdyn_msgs_manipulation_api_request_one_of_manipulation_cmd_to_proto(ros_msg, proto):
    proto.ClearField("manipulation_cmd")
    if ros_msg.manipulation_cmd_choice == ros_msg.MANIPULATION_CMD_WALK_TO_OBJECT_RAY_IN_WORLD_SET:
        convert_bosdyn_msgs_walk_to_object_ray_in_world_to_proto(ros_msg.walk_to_object_ray_in_world, proto.walk_to_object_ray_in_world)
    if ros_msg.manipulation_cmd_choice == ros_msg.MANIPULATION_CMD_WALK_TO_OBJECT_IN_IMAGE_SET:
        convert_bosdyn_msgs_walk_to_object_in_image_to_proto(ros_msg.walk_to_object_in_image, proto.walk_to_object_in_image)
    if ros_msg.manipulation_cmd_choice == ros_msg.MANIPULATION_CMD_PICK_OBJECT_SET:
        convert_bosdyn_msgs_pick_object_to_proto(ros_msg.pick_object, proto.pick_object)
    if ros_msg.manipulation_cmd_choice == ros_msg.MANIPULATION_CMD_PICK_OBJECT_IN_IMAGE_SET:
        convert_bosdyn_msgs_pick_object_in_image_to_proto(ros_msg.pick_object_in_image, proto.pick_object_in_image)
    if ros_msg.manipulation_cmd_choice == ros_msg.MANIPULATION_CMD_PICK_OBJECT_RAY_IN_WORLD_SET:
        convert_bosdyn_msgs_pick_object_ray_in_world_to_proto(ros_msg.pick_object_ray_in_world, proto.pick_object_ray_in_world)
    if ros_msg.manipulation_cmd_choice == ros_msg.MANIPULATION_CMD_PICK_OBJECT_EXECUTE_PLAN_SET:
        convert_bosdyn_msgs_pick_object_execute_plan_to_proto(ros_msg.pick_object_execute_plan, proto.pick_object_execute_plan)

def convert_proto_to_bosdyn_msgs_manipulation_api_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    convert_proto_to_bosdyn_msgs_manipulation_api_request_one_of_manipulation_cmd(proto, ros_msg.manipulation_cmd)

def convert_bosdyn_msgs_manipulation_api_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    convert_bosdyn_msgs_manipulation_api_request_one_of_manipulation_cmd_to_proto(ros_msg.manipulation_cmd, proto)

def convert_proto_to_bosdyn_msgs_api_grasp_override(proto, ros_msg):
    ros_msg.override_request.value = proto.override_request

def convert_bosdyn_msgs_api_grasp_override_to_proto(ros_msg, proto):
    proto.Clear()
    proto.override_request = ros_msg.override_request.value

def convert_proto_to_bosdyn_msgs_api_grasped_carry_state_override(proto, ros_msg):
    ros_msg.override_request.value = proto.override_request

def convert_bosdyn_msgs_api_grasped_carry_state_override_to_proto(ros_msg, proto):
    proto.Clear()
    proto.override_request = ros_msg.override_request.value

def convert_proto_to_bosdyn_msgs_api_grasp_override_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_api_grasp_override(proto.api_grasp_override, ros_msg.api_grasp_override)
    ros_msg.api_grasp_override_is_set = proto.HasField("api_grasp_override")
    convert_proto_to_bosdyn_msgs_api_grasped_carry_state_override(proto.carry_state_override, ros_msg.carry_state_override)
    ros_msg.carry_state_override_is_set = proto.HasField("carry_state_override")

def convert_bosdyn_msgs_api_grasp_override_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.api_grasp_override_is_set:
        convert_bosdyn_msgs_api_grasp_override_to_proto(ros_msg.api_grasp_override, proto.api_grasp_override)
    if ros_msg.carry_state_override_is_set:
        convert_bosdyn_msgs_api_grasped_carry_state_override_to_proto(ros_msg.carry_state_override, proto.carry_state_override)

def convert_proto_to_bosdyn_msgs_api_grasp_override_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_api_grasp_override_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_grpc_spec(proto, ros_msg):
    ros_msg.service_name = proto.service_name

def convert_bosdyn_msgs_grpc_spec_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name

def convert_proto_to_bosdyn_msgs_blob_spec(proto, ros_msg):
    ros_msg.source = proto.source
    ros_msg.message_type = proto.message_type
    ros_msg.channel = proto.channel
    ros_msg.channel_glob = proto.channel_glob

def convert_bosdyn_msgs_blob_spec_to_proto(ros_msg, proto):
    proto.Clear()
    proto.source = ros_msg.source
    proto.message_type = ros_msg.message_type
    proto.channel = ros_msg.channel
    proto.channel_glob = ros_msg.channel_glob

def convert_proto_to_bosdyn_msgs_event_spec(proto, ros_msg):
    ros_msg.source = proto.source
    ros_msg.type = proto.type
    ros_msg.level = proto.level.value
    ros_msg.level_is_set = proto.HasField("level")
    ros_msg.log_preserve_hint.value = proto.log_preserve_hint

def convert_bosdyn_msgs_event_spec_to_proto(ros_msg, proto):
    proto.Clear()
    proto.source = ros_msg.source
    proto.type = ros_msg.type
    if ros_msg.level_is_set:
        convert_int32_to_proto(ros_msg.level, proto.level)
    proto.log_preserve_hint = ros_msg.log_preserve_hint.value

def convert_proto_to_bosdyn_msgs_page_info(proto, ros_msg):
    ros_msg.id = proto.id
    ros_msg.path = proto.path
    ros_msg.source = proto.source
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    ros_msg.num_ticks = proto.num_ticks
    ros_msg.total_bytes = proto.total_bytes
    ros_msg.format.value = proto.format
    ros_msg.compression.value = proto.compression
    ros_msg.is_open = proto.is_open
    ros_msg.is_downloaded = proto.is_downloaded
    convert_proto_to_builtin_interfaces_time(proto.deleted_timestamp, ros_msg.deleted_timestamp)
    ros_msg.deleted_timestamp_is_set = proto.HasField("deleted_timestamp")
    convert_proto_to_builtin_interfaces_time(proto.download_started_timestamp, ros_msg.download_started_timestamp)
    ros_msg.download_started_timestamp_is_set = proto.HasField("download_started_timestamp")
    ros_msg.request_preserve = proto.request_preserve

def convert_bosdyn_msgs_page_info_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    proto.path = ros_msg.path
    proto.source = ros_msg.source
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    proto.num_ticks = ros_msg.num_ticks
    proto.total_bytes = ros_msg.total_bytes
    proto.format = ros_msg.format.value
    proto.compression = ros_msg.compression.value
    proto.is_open = ros_msg.is_open
    proto.is_downloaded = ros_msg.is_downloaded
    if ros_msg.deleted_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.deleted_timestamp, proto.deleted_timestamp)
    if ros_msg.download_started_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.download_started_timestamp, proto.download_started_timestamp)
    proto.request_preserve = ros_msg.request_preserve

def convert_proto_to_bosdyn_msgs_grpc_pages(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    convert_proto_to_bosdyn_msgs_grpc_spec(proto.spec, ros_msg.spec)
    ros_msg.spec_is_set = proto.HasField("spec")
    from bosdyn_msgs.msg import PageInfo
    ros_msg.pages = []
    for _item in proto.pages:
        ros_msg.pages.append(PageInfo())
        convert_proto_to_bosdyn_msgs_page_info(_item, ros_msg.pages[-1])

def convert_bosdyn_msgs_grpc_pages_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    if ros_msg.spec_is_set:
        convert_bosdyn_msgs_grpc_spec_to_proto(ros_msg.spec, proto.spec)
    del proto.pages[:]
    for _item in ros_msg.pages:
        convert_bosdyn_msgs_page_info_to_proto(_item, proto.pages.add())

def convert_proto_to_bosdyn_msgs_blob_page(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_blob_spec(proto.spec, ros_msg.spec)
    ros_msg.spec_is_set = proto.HasField("spec")
    convert_proto_to_bosdyn_msgs_page_info(proto.page, ros_msg.page)
    ros_msg.page_is_set = proto.HasField("page")

def convert_bosdyn_msgs_blob_page_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.spec_is_set:
        convert_bosdyn_msgs_blob_spec_to_proto(ros_msg.spec, proto.spec)
    if ros_msg.page_is_set:
        convert_bosdyn_msgs_page_info_to_proto(ros_msg.page, proto.page)

def convert_proto_to_bosdyn_msgs_blob_pages(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    from bosdyn_msgs.msg import BlobPage
    ros_msg.pages = []
    for _item in proto.pages:
        ros_msg.pages.append(BlobPage())
        convert_proto_to_bosdyn_msgs_blob_page(_item, ros_msg.pages[-1])

def convert_bosdyn_msgs_blob_pages_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    del proto.pages[:]
    for _item in ros_msg.pages:
        convert_bosdyn_msgs_blob_page_to_proto(_item, proto.pages.add())

def convert_proto_to_bosdyn_msgs_pages_and_timestamp(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    from bosdyn_msgs.msg import PageInfo
    ros_msg.pages = []
    for _item in proto.pages:
        ros_msg.pages.append(PageInfo())
        convert_proto_to_bosdyn_msgs_page_info(_item, ros_msg.pages[-1])

def convert_bosdyn_msgs_pages_and_timestamp_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    del proto.pages[:]
    for _item in ros_msg.pages:
        convert_bosdyn_msgs_page_info_to_proto(_item, proto.pages.add())

def convert_proto_to_bosdyn_msgs_data_query(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    from bosdyn_msgs.msg import BlobSpec
    ros_msg.blobs = []
    for _item in proto.blobs:
        ros_msg.blobs.append(BlobSpec())
        convert_proto_to_bosdyn_msgs_blob_spec(_item, ros_msg.blobs[-1])
    ros_msg.text_messages = proto.text_messages
    ros_msg.events = proto.events
    ros_msg.comments = proto.comments

def convert_bosdyn_msgs_data_query_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    del proto.blobs[:]
    for _item in ros_msg.blobs:
        convert_bosdyn_msgs_blob_spec_to_proto(_item, proto.blobs.add())
    proto.text_messages = ros_msg.text_messages
    proto.events = ros_msg.events
    proto.comments = ros_msg.comments

def convert_proto_to_bosdyn_msgs_data_index(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    from bosdyn_msgs.msg import BlobPages
    ros_msg.blobs = []
    for _item in proto.blobs:
        ros_msg.blobs.append(BlobPages())
        convert_proto_to_bosdyn_msgs_blob_pages(_item, ros_msg.blobs[-1])
    convert_proto_to_bosdyn_msgs_pages_and_timestamp(proto.text_messages, ros_msg.text_messages)
    ros_msg.text_messages_is_set = proto.HasField("text_messages")
    convert_proto_to_bosdyn_msgs_pages_and_timestamp(proto.events, ros_msg.events)
    ros_msg.events_is_set = proto.HasField("events")
    convert_proto_to_bosdyn_msgs_pages_and_timestamp(proto.comments, ros_msg.comments)
    ros_msg.comments_is_set = proto.HasField("comments")

def convert_bosdyn_msgs_data_index_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    del proto.blobs[:]
    for _item in ros_msg.blobs:
        convert_bosdyn_msgs_blob_pages_to_proto(_item, proto.blobs.add())
    if ros_msg.text_messages_is_set:
        convert_bosdyn_msgs_pages_and_timestamp_to_proto(ros_msg.text_messages, proto.text_messages)
    if ros_msg.events_is_set:
        convert_bosdyn_msgs_pages_and_timestamp_to_proto(ros_msg.events, proto.events)
    if ros_msg.comments_is_set:
        convert_bosdyn_msgs_pages_and_timestamp_to_proto(ros_msg.comments, proto.comments)

def convert_proto_to_bosdyn_msgs_events_comments_spec(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    from bosdyn_msgs.msg import EventSpec
    ros_msg.events = []
    for _item in proto.events:
        ros_msg.events.append(EventSpec())
        convert_proto_to_bosdyn_msgs_event_spec(_item, ros_msg.events[-1])
    ros_msg.comments = proto.comments
    ros_msg.max_events = proto.max_events
    ros_msg.max_comments = proto.max_comments

def convert_bosdyn_msgs_events_comments_spec_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    del proto.events[:]
    for _item in ros_msg.events:
        convert_bosdyn_msgs_event_spec_to_proto(_item, proto.events.add())
    proto.comments = ros_msg.comments
    proto.max_events = ros_msg.max_events
    proto.max_comments = ros_msg.max_comments

def convert_proto_to_bosdyn_msgs_events_comments(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    from bosdyn_msgs.msg import Event
    ros_msg.events = []
    for _item in proto.events:
        ros_msg.events.append(Event())
        convert_proto_to_bosdyn_msgs_event(_item, ros_msg.events[-1])
    from bosdyn_msgs.msg import OperatorComment
    ros_msg.operator_comments = []
    for _item in proto.operator_comments:
        ros_msg.operator_comments.append(OperatorComment())
        convert_proto_to_bosdyn_msgs_operator_comment(_item, ros_msg.operator_comments[-1])
    ros_msg.events_limited = proto.events_limited
    ros_msg.operator_comments_limited = proto.operator_comments_limited

def convert_bosdyn_msgs_events_comments_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    del proto.events[:]
    for _item in ros_msg.events:
        convert_bosdyn_msgs_event_to_proto(_item, proto.events.add())
    del proto.operator_comments[:]
    for _item in ros_msg.operator_comments:
        convert_bosdyn_msgs_operator_comment_to_proto(_item, proto.operator_comments.add())
    proto.events_limited = ros_msg.events_limited
    proto.operator_comments_limited = ros_msg.operator_comments_limited

def convert_proto_to_bosdyn_msgs_data_buffer_status(proto, ros_msg):
    ros_msg.num_data_buffer_pages = proto.num_data_buffer_pages
    ros_msg.data_buffer_total_bytes = proto.data_buffer_total_bytes
    ros_msg.num_comments = proto.num_comments
    ros_msg.num_events = proto.num_events
    from bosdyn_msgs.msg import BlobSpec
    ros_msg.blob_specs = []
    for _item in proto.blob_specs:
        ros_msg.blob_specs.append(BlobSpec())
        convert_proto_to_bosdyn_msgs_blob_spec(_item, ros_msg.blob_specs[-1])

def convert_bosdyn_msgs_data_buffer_status_to_proto(ros_msg, proto):
    proto.Clear()
    proto.num_data_buffer_pages = ros_msg.num_data_buffer_pages
    proto.data_buffer_total_bytes = ros_msg.data_buffer_total_bytes
    proto.num_comments = ros_msg.num_comments
    proto.num_events = ros_msg.num_events
    del proto.blob_specs[:]
    for _item in ros_msg.blob_specs:
        convert_bosdyn_msgs_blob_spec_to_proto(_item, proto.blob_specs.add())

def convert_proto_to_bosdyn_msgs_get_data_index_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_index(proto.data_index, ros_msg.data_index)
    ros_msg.data_index_is_set = proto.HasField("data_index")

def convert_bosdyn_msgs_get_data_index_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.data_index_is_set:
        convert_bosdyn_msgs_data_index_to_proto(ros_msg.data_index, proto.data_index)

def convert_proto_to_bosdyn_msgs_get_data_index_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_query(proto.data_query, ros_msg.data_query)
    ros_msg.data_query_is_set = proto.HasField("data_query")

def convert_bosdyn_msgs_get_data_index_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.data_query_is_set:
        convert_bosdyn_msgs_data_query_to_proto(ros_msg.data_query, proto.data_query)

def convert_proto_to_bosdyn_msgs_get_events_comments_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_events_comments_spec(proto.event_comment_request, ros_msg.event_comment_request)
    ros_msg.event_comment_request_is_set = proto.HasField("event_comment_request")

def convert_bosdyn_msgs_get_events_comments_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.event_comment_request_is_set:
        convert_bosdyn_msgs_events_comments_spec_to_proto(ros_msg.event_comment_request, proto.event_comment_request)

def convert_proto_to_bosdyn_msgs_get_events_comments_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_events_comments(proto.events_comments, ros_msg.events_comments)
    ros_msg.events_comments_is_set = proto.HasField("events_comments")

def convert_bosdyn_msgs_get_events_comments_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.events_comments_is_set:
        convert_bosdyn_msgs_events_comments_to_proto(ros_msg.events_comments, proto.events_comments)

def convert_proto_to_bosdyn_msgs_get_data_buffer_status_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.get_blob_specs = proto.get_blob_specs

def convert_bosdyn_msgs_get_data_buffer_status_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.get_blob_specs = ros_msg.get_blob_specs

def convert_proto_to_bosdyn_msgs_get_data_buffer_status_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_buffer_status(proto.data_buffer_status, ros_msg.data_buffer_status)
    ros_msg.data_buffer_status_is_set = proto.HasField("data_buffer_status")

def convert_bosdyn_msgs_get_data_buffer_status_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.data_buffer_status_is_set:
        convert_bosdyn_msgs_data_buffer_status_to_proto(ros_msg.data_buffer_status, proto.data_buffer_status)

def convert_proto_to_bosdyn_msgs_get_data_pages_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")

def convert_bosdyn_msgs_get_data_pages_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)

def convert_proto_to_bosdyn_msgs_get_data_pages_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import PageInfo
    ros_msg.pages = []
    for _item in proto.pages:
        ros_msg.pages.append(PageInfo())
        convert_proto_to_bosdyn_msgs_page_info(_item, ros_msg.pages[-1])

def convert_bosdyn_msgs_get_data_pages_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.pages[:]
    for _item in ros_msg.pages:
        convert_bosdyn_msgs_page_info_to_proto(_item, proto.pages.add())

def convert_proto_to_bosdyn_msgs_delete_data_pages_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_time_range(proto.time_range, ros_msg.time_range)
    ros_msg.time_range_is_set = proto.HasField("time_range")
    ros_msg.page_ids = []
    for _item in proto.page_ids:
        ros_msg.page_ids.append(_item)

def convert_bosdyn_msgs_delete_data_pages_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.time_range_is_set:
        convert_bosdyn_msgs_time_range_to_proto(ros_msg.time_range, proto.time_range)
    del proto.page_ids[:]
    for _item in ros_msg.page_ids:
        proto.page_ids.add(_item)

def convert_proto_to_bosdyn_msgs_delete_page_status(proto, ros_msg):
    ros_msg.page_id = proto.page_id
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_delete_page_status_to_proto(ros_msg, proto):
    proto.Clear()
    proto.page_id = ros_msg.page_id
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_delete_data_pages_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.bytes_deleted = proto.bytes_deleted
    from bosdyn_msgs.msg import DeletePageStatus
    ros_msg.status = []
    for _item in proto.status:
        ros_msg.status.append(DeletePageStatus())
        convert_proto_to_bosdyn_msgs_delete_page_status(_item, ros_msg.status[-1])

def convert_bosdyn_msgs_delete_data_pages_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.bytes_deleted = ros_msg.bytes_deleted
    del proto.status[:]
    for _item in ros_msg.status:
        convert_bosdyn_msgs_delete_page_status_to_proto(_item, proto.status.add())

def convert_proto_to_bosdyn_msgs_image(proto, ros_msg):
    ros_msg.cols = proto.cols
    ros_msg.rows = proto.rows
    ros_msg.data = proto.data
    ros_msg.format.value = proto.format
    ros_msg.pixel_format.value = proto.pixel_format

def convert_bosdyn_msgs_image_to_proto(ros_msg, proto):
    proto.Clear()
    proto.cols = ros_msg.cols
    proto.rows = ros_msg.rows
    proto.data = ros_msg.data
    proto.format = ros_msg.format.value
    proto.pixel_format = ros_msg.pixel_format.value

def convert_proto_to_bosdyn_msgs_capture_parameters(proto, ros_msg):
    convert_proto_to_builtin_interfaces_duration(proto.exposure_duration, ros_msg.exposure_duration)
    ros_msg.exposure_duration_is_set = proto.HasField("exposure_duration")
    ros_msg.gain = proto.gain

def convert_bosdyn_msgs_capture_parameters_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.exposure_duration_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.exposure_duration, proto.exposure_duration)
    proto.gain = ros_msg.gain

def convert_proto_to_bosdyn_msgs_image_capture(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.acquisition_time, ros_msg.acquisition_time)
    ros_msg.acquisition_time_is_set = proto.HasField("acquisition_time")
    ros_msg.frame_name_image_sensor = proto.frame_name_image_sensor
    convert_proto_to_bosdyn_msgs_image(proto.image, ros_msg.image)
    ros_msg.image_is_set = proto.HasField("image")
    convert_proto_to_bosdyn_msgs_capture_parameters(proto.capture_params, ros_msg.capture_params)
    ros_msg.capture_params_is_set = proto.HasField("capture_params")

def convert_bosdyn_msgs_image_capture_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.acquisition_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.acquisition_time, proto.acquisition_time)
    proto.frame_name_image_sensor = ros_msg.frame_name_image_sensor
    if ros_msg.image_is_set:
        convert_bosdyn_msgs_image_to_proto(ros_msg.image, proto.image)
    if ros_msg.capture_params_is_set:
        convert_bosdyn_msgs_capture_parameters_to_proto(ros_msg.capture_params, proto.capture_params)

def convert_proto_to_bosdyn_msgs_image_source_pinhole_model_camera_intrinsics(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.focal_length, ros_msg.focal_length)
    ros_msg.focal_length_is_set = proto.HasField("focal_length")
    convert_proto_to_bosdyn_msgs_vec2(proto.principal_point, ros_msg.principal_point)
    ros_msg.principal_point_is_set = proto.HasField("principal_point")
    convert_proto_to_bosdyn_msgs_vec2(proto.skew, ros_msg.skew)
    ros_msg.skew_is_set = proto.HasField("skew")

def convert_bosdyn_msgs_image_source_pinhole_model_camera_intrinsics_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.focal_length_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.focal_length, proto.focal_length)
    if ros_msg.principal_point_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.principal_point, proto.principal_point)
    if ros_msg.skew_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.skew, proto.skew)

def convert_proto_to_bosdyn_msgs_image_source_pinhole_model(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_image_source_pinhole_model_camera_intrinsics(proto.intrinsics, ros_msg.intrinsics)
    ros_msg.intrinsics_is_set = proto.HasField("intrinsics")

def convert_bosdyn_msgs_image_source_pinhole_model_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.intrinsics_is_set:
        convert_bosdyn_msgs_image_source_pinhole_model_camera_intrinsics_to_proto(ros_msg.intrinsics, proto.intrinsics)

def convert_proto_to_bosdyn_msgs_image_source_one_of_camera_models(proto, ros_msg):
    if proto.HasField("pinhole"):
        ros_msg.camera_models_choice = ros_msg.CAMERA_MODELS_PINHOLE_SET
        convert_proto_to_bosdyn_msgs_image_source_pinhole_model(proto.pinhole, ros_msg.pinhole)

def convert_bosdyn_msgs_image_source_one_of_camera_models_to_proto(ros_msg, proto):
    proto.ClearField("camera_models")
    if ros_msg.camera_models_choice == ros_msg.CAMERA_MODELS_PINHOLE_SET:
        convert_bosdyn_msgs_image_source_pinhole_model_to_proto(ros_msg.pinhole, proto.pinhole)

def convert_proto_to_bosdyn_msgs_image_source(proto, ros_msg):
    ros_msg.name = proto.name
    ros_msg.cols = proto.cols
    ros_msg.rows = proto.rows
    ros_msg.depth_scale = proto.depth_scale
    convert_proto_to_bosdyn_msgs_image_source_one_of_camera_models(proto, ros_msg.camera_models)
    ros_msg.image_type.value = proto.image_type
    from bosdyn_msgs.msg import ImagePixelFormat
    ros_msg.pixel_formats = []
    for _item in proto.pixel_formats:
        ros_msg.pixel_formats.append(ImagePixelFormat())
        ros_msg.pixel_formats[-1].value = _item
    from bosdyn_msgs.msg import ImageFormat
    ros_msg.image_formats = []
    for _item in proto.image_formats:
        ros_msg.image_formats.append(ImageFormat())
        ros_msg.image_formats[-1].value = _item

def convert_bosdyn_msgs_image_source_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    proto.cols = ros_msg.cols
    proto.rows = ros_msg.rows
    proto.depth_scale = ros_msg.depth_scale
    convert_bosdyn_msgs_image_source_one_of_camera_models_to_proto(ros_msg.camera_models, proto)
    proto.image_type = ros_msg.image_type.value
    del proto.pixel_formats[:]
    for _item in ros_msg.pixel_formats:
        proto.pixel_formats.add(_item.value)
    del proto.image_formats[:]
    for _item in ros_msg.image_formats:
        proto.image_formats.add(_item.value)

def convert_proto_to_bosdyn_msgs_list_image_sources_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_image_sources_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_image_sources_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import ImageSource
    ros_msg.image_sources = []
    for _item in proto.image_sources:
        ros_msg.image_sources.append(ImageSource())
        convert_proto_to_bosdyn_msgs_image_source(_item, ros_msg.image_sources[-1])

def convert_bosdyn_msgs_list_image_sources_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.image_sources[:]
    for _item in ros_msg.image_sources:
        convert_bosdyn_msgs_image_source_to_proto(_item, proto.image_sources.add())

def convert_proto_to_bosdyn_msgs_image_request(proto, ros_msg):
    ros_msg.image_source_name = proto.image_source_name
    ros_msg.quality_percent = proto.quality_percent
    ros_msg.image_format.value = proto.image_format
    ros_msg.resize_ratio = proto.resize_ratio
    ros_msg.pixel_format.value = proto.pixel_format

def convert_bosdyn_msgs_image_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.image_source_name = ros_msg.image_source_name
    proto.quality_percent = ros_msg.quality_percent
    proto.image_format = ros_msg.image_format.value
    proto.resize_ratio = ros_msg.resize_ratio
    proto.pixel_format = ros_msg.pixel_format.value

def convert_proto_to_bosdyn_msgs_get_image_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import ImageRequest
    ros_msg.image_requests = []
    for _item in proto.image_requests:
        ros_msg.image_requests.append(ImageRequest())
        convert_proto_to_bosdyn_msgs_image_request(_item, ros_msg.image_requests[-1])

def convert_bosdyn_msgs_get_image_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.image_requests[:]
    for _item in ros_msg.image_requests:
        convert_bosdyn_msgs_image_request_to_proto(_item, proto.image_requests.add())

def convert_proto_to_bosdyn_msgs_image_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_image_capture(proto.shot, ros_msg.shot)
    ros_msg.shot_is_set = proto.HasField("shot")
    convert_proto_to_bosdyn_msgs_image_source(proto.source, ros_msg.source)
    ros_msg.source_is_set = proto.HasField("source")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_image_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.shot_is_set:
        convert_bosdyn_msgs_image_capture_to_proto(ros_msg.shot, proto.shot)
    if ros_msg.source_is_set:
        convert_bosdyn_msgs_image_source_to_proto(ros_msg.source, proto.source)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_get_image_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import ImageResponse
    ros_msg.image_responses = []
    for _item in proto.image_responses:
        ros_msg.image_responses.append(ImageResponse())
        convert_proto_to_bosdyn_msgs_image_response(_item, ros_msg.image_responses[-1])

def convert_bosdyn_msgs_get_image_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.image_responses[:]
    for _item in ros_msg.image_responses:
        convert_bosdyn_msgs_image_response_to_proto(_item, proto.image_responses.add())

def convert_bosdyn_msgs_payload_estimation_command_request_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_payload_estimation_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.progress = proto.progress
    ros_msg.error.value = proto.error
    convert_proto_to_bosdyn_msgs_payload(proto.estimated_payload, ros_msg.estimated_payload)
    ros_msg.estimated_payload_is_set = proto.HasField("estimated_payload")

def convert_bosdyn_msgs_payload_estimation_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.progress = ros_msg.progress
    proto.error = ros_msg.error.value
    if ros_msg.estimated_payload_is_set:
        convert_bosdyn_msgs_payload_to_proto(ros_msg.estimated_payload, proto.estimated_payload)

def convert_bosdyn_msgs_payload_estimation_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_robot_id(proto, ros_msg):
    ros_msg.serial_number = proto.serial_number
    ros_msg.species = proto.species
    ros_msg.version = proto.version
    convert_proto_to_bosdyn_msgs_robot_software_release(proto.software_release, ros_msg.software_release)
    ros_msg.software_release_is_set = proto.HasField("software_release")
    ros_msg.nickname = proto.nickname
    ros_msg.computer_serial_number = proto.computer_serial_number

def convert_bosdyn_msgs_robot_id_to_proto(ros_msg, proto):
    proto.Clear()
    proto.serial_number = ros_msg.serial_number
    proto.species = ros_msg.species
    proto.version = ros_msg.version
    if ros_msg.software_release_is_set:
        convert_bosdyn_msgs_robot_software_release_to_proto(ros_msg.software_release, proto.software_release)
    proto.nickname = ros_msg.nickname
    proto.computer_serial_number = ros_msg.computer_serial_number

def convert_proto_to_bosdyn_msgs_software_version(proto, ros_msg):
    ros_msg.major_version = proto.major_version
    ros_msg.minor_version = proto.minor_version
    ros_msg.patch_level = proto.patch_level

def convert_bosdyn_msgs_software_version_to_proto(ros_msg, proto):
    proto.Clear()
    proto.major_version = ros_msg.major_version
    proto.minor_version = ros_msg.minor_version
    proto.patch_level = ros_msg.patch_level

def convert_proto_to_bosdyn_msgs_robot_software_release(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_software_version(proto.version, ros_msg.version)
    ros_msg.version_is_set = proto.HasField("version")
    ros_msg.name = proto.name
    ros_msg.type = proto.type
    convert_proto_to_builtin_interfaces_time(proto.changeset_date, ros_msg.changeset_date)
    ros_msg.changeset_date_is_set = proto.HasField("changeset_date")
    ros_msg.changeset = proto.changeset
    ros_msg.api_version = proto.api_version
    ros_msg.build_information = proto.build_information
    convert_proto_to_builtin_interfaces_time(proto.install_date, ros_msg.install_date)
    ros_msg.install_date_is_set = proto.HasField("install_date")
    from bosdyn_msgs.msg import Parameter
    ros_msg.parameters = []
    for _item in proto.parameters:
        ros_msg.parameters.append(Parameter())
        convert_proto_to_bosdyn_msgs_parameter(_item, ros_msg.parameters[-1])

def convert_bosdyn_msgs_robot_software_release_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.version_is_set:
        convert_bosdyn_msgs_software_version_to_proto(ros_msg.version, proto.version)
    proto.name = ros_msg.name
    proto.type = ros_msg.type
    if ros_msg.changeset_date_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.changeset_date, proto.changeset_date)
    proto.changeset = ros_msg.changeset
    proto.api_version = ros_msg.api_version
    proto.build_information = ros_msg.build_information
    if ros_msg.install_date_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.install_date, proto.install_date)
    del proto.parameters[:]
    for _item in ros_msg.parameters:
        convert_bosdyn_msgs_parameter_to_proto(_item, proto.parameters.add())

def convert_proto_to_bosdyn_msgs_robot_id_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_robot_id_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_robot_id_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_robot_id(proto.robot_id, ros_msg.robot_id)
    ros_msg.robot_id_is_set = proto.HasField("robot_id")

def convert_bosdyn_msgs_robot_id_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.robot_id_is_set:
        convert_bosdyn_msgs_robot_id_to_proto(ros_msg.robot_id, proto.robot_id)

def convert_proto_to_bosdyn_msgs_register_payload_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_payload(proto.payload, ros_msg.payload)
    ros_msg.payload_is_set = proto.HasField("payload")
    ros_msg.payload_secret = proto.payload_secret

def convert_bosdyn_msgs_register_payload_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.payload_is_set:
        convert_bosdyn_msgs_payload_to_proto(ros_msg.payload, proto.payload)
    proto.payload_secret = ros_msg.payload_secret

def convert_proto_to_bosdyn_msgs_register_payload_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_register_payload_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_update_payload_version_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_payload_credentials(proto.payload_credentials, ros_msg.payload_credentials)
    ros_msg.payload_credentials_is_set = proto.HasField("payload_credentials")
    convert_proto_to_bosdyn_msgs_software_version(proto.updated_version, ros_msg.updated_version)
    ros_msg.updated_version_is_set = proto.HasField("updated_version")

def convert_bosdyn_msgs_update_payload_version_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.payload_credentials_is_set:
        convert_bosdyn_msgs_payload_credentials_to_proto(ros_msg.payload_credentials, proto.payload_credentials)
    if ros_msg.updated_version_is_set:
        convert_bosdyn_msgs_software_version_to_proto(ros_msg.updated_version, proto.updated_version)

def convert_proto_to_bosdyn_msgs_update_payload_version_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_update_payload_version_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_get_payload_auth_token_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_payload_credentials(proto.payload_credentials, ros_msg.payload_credentials)
    ros_msg.payload_credentials_is_set = proto.HasField("payload_credentials")

def convert_bosdyn_msgs_get_payload_auth_token_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.payload_credentials_is_set:
        convert_bosdyn_msgs_payload_credentials_to_proto(ros_msg.payload_credentials, proto.payload_credentials)

def convert_proto_to_bosdyn_msgs_get_payload_auth_token_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.token = proto.token

def convert_bosdyn_msgs_get_payload_auth_token_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.token = ros_msg.token

def convert_proto_to_bosdyn_msgs_update_payload_attached_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_payload_credentials(proto.payload_credentials, ros_msg.payload_credentials)
    ros_msg.payload_credentials_is_set = proto.HasField("payload_credentials")
    ros_msg.request.value = proto.request

def convert_bosdyn_msgs_update_payload_attached_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.payload_credentials_is_set:
        convert_bosdyn_msgs_payload_credentials_to_proto(ros_msg.payload_credentials, proto.payload_credentials)
    proto.request = ros_msg.request.value

def convert_proto_to_bosdyn_msgs_update_payload_attached_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_update_payload_attached_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_payload_credentials(proto, ros_msg):
    ros_msg.guid = proto.guid
    ros_msg.secret = proto.secret

def convert_bosdyn_msgs_payload_credentials_to_proto(ros_msg, proto):
    proto.Clear()
    proto.guid = ros_msg.guid
    proto.secret = ros_msg.secret

def convert_proto_to_bosdyn_msgs_payload(proto, ros_msg):
    ros_msg.guid = proto.GUID
    ros_msg.name = proto.name
    ros_msg.description = proto.description
    ros_msg.label_prefix = []
    for _item in proto.label_prefix:
        ros_msg.label_prefix.append(_item)
    ros_msg.is_authorized = proto.is_authorized
    ros_msg.is_enabled = proto.is_enabled
    ros_msg.is_noncompute_payload = proto.is_noncompute_payload
    convert_proto_to_bosdyn_msgs_software_version(proto.version, ros_msg.version)
    ros_msg.version_is_set = proto.HasField("version")
    convert_proto_to_geometry_msgs_pose(proto.body_tform_payload, ros_msg.body_tform_payload)
    ros_msg.body_tform_payload_is_set = proto.HasField("body_tform_payload")
    convert_proto_to_geometry_msgs_pose(proto.mount_tform_payload, ros_msg.mount_tform_payload)
    ros_msg.mount_tform_payload_is_set = proto.HasField("mount_tform_payload")
    ros_msg.mount_frame_name.value = proto.mount_frame_name
    convert_proto_to_bosdyn_msgs_payload_mass_volume_properties(proto.mass_volume_properties, ros_msg.mass_volume_properties)
    ros_msg.mass_volume_properties_is_set = proto.HasField("mass_volume_properties")
    from bosdyn_msgs.msg import PayloadPreset
    ros_msg.preset_configurations = []
    for _item in proto.preset_configurations:
        ros_msg.preset_configurations.append(PayloadPreset())
        convert_proto_to_bosdyn_msgs_payload_preset(_item, ros_msg.preset_configurations[-1])

def convert_bosdyn_msgs_payload_to_proto(ros_msg, proto):
    proto.Clear()
    proto.GUID = ros_msg.guid
    proto.name = ros_msg.name
    proto.description = ros_msg.description
    del proto.label_prefix[:]
    for _item in ros_msg.label_prefix:
        proto.label_prefix.add(_item)
    proto.is_authorized = ros_msg.is_authorized
    proto.is_enabled = ros_msg.is_enabled
    proto.is_noncompute_payload = ros_msg.is_noncompute_payload
    if ros_msg.version_is_set:
        convert_bosdyn_msgs_software_version_to_proto(ros_msg.version, proto.version)
    if ros_msg.body_tform_payload_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.body_tform_payload, proto.body_tform_payload)
    if ros_msg.mount_tform_payload_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.mount_tform_payload, proto.mount_tform_payload)
    proto.mount_frame_name = ros_msg.mount_frame_name.value
    if ros_msg.mass_volume_properties_is_set:
        convert_bosdyn_msgs_payload_mass_volume_properties_to_proto(ros_msg.mass_volume_properties, proto.mass_volume_properties)
    del proto.preset_configurations[:]
    for _item in ros_msg.preset_configurations:
        convert_bosdyn_msgs_payload_preset_to_proto(_item, proto.preset_configurations.add())

def convert_proto_to_bosdyn_msgs_payload_preset(proto, ros_msg):
    ros_msg.preset_name = proto.preset_name
    ros_msg.description = proto.description
    convert_proto_to_geometry_msgs_pose(proto.mount_tform_payload, ros_msg.mount_tform_payload)
    ros_msg.mount_tform_payload_is_set = proto.HasField("mount_tform_payload")
    ros_msg.mount_frame_name.value = proto.mount_frame_name
    convert_proto_to_bosdyn_msgs_payload_mass_volume_properties(proto.mass_volume_properties, ros_msg.mass_volume_properties)
    ros_msg.mass_volume_properties_is_set = proto.HasField("mass_volume_properties")
    ros_msg.label_prefix = []
    for _item in proto.label_prefix:
        ros_msg.label_prefix.append(_item)

def convert_bosdyn_msgs_payload_preset_to_proto(ros_msg, proto):
    proto.Clear()
    proto.preset_name = ros_msg.preset_name
    proto.description = ros_msg.description
    if ros_msg.mount_tform_payload_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.mount_tform_payload, proto.mount_tform_payload)
    proto.mount_frame_name = ros_msg.mount_frame_name.value
    if ros_msg.mass_volume_properties_is_set:
        convert_bosdyn_msgs_payload_mass_volume_properties_to_proto(ros_msg.mass_volume_properties, proto.mass_volume_properties)
    del proto.label_prefix[:]
    for _item in ros_msg.label_prefix:
        proto.label_prefix.add(_item)

def convert_proto_to_bosdyn_msgs_payload_mass_volume_properties(proto, ros_msg):
    ros_msg.total_mass = proto.total_mass
    convert_proto_to_geometry_msgs_vector3(proto.com_pos_rt_payload, ros_msg.com_pos_rt_payload)
    ros_msg.com_pos_rt_payload_is_set = proto.HasField("com_pos_rt_payload")
    convert_proto_to_bosdyn_msgs_moment_of_intertia(proto.moi_tensor, ros_msg.moi_tensor)
    ros_msg.moi_tensor_is_set = proto.HasField("moi_tensor")
    from bosdyn_msgs.msg import Box3WithFrame
    ros_msg.bounding_box = []
    for _item in proto.bounding_box:
        ros_msg.bounding_box.append(Box3WithFrame())
        convert_proto_to_bosdyn_msgs_box3_with_frame(_item, ros_msg.bounding_box[-1])
    from bosdyn_msgs.msg import JointLimits
    ros_msg.joint_limits = []
    for _item in proto.joint_limits:
        ros_msg.joint_limits.append(JointLimits())
        convert_proto_to_bosdyn_msgs_joint_limits(_item, ros_msg.joint_limits[-1])

def convert_bosdyn_msgs_payload_mass_volume_properties_to_proto(ros_msg, proto):
    proto.Clear()
    proto.total_mass = ros_msg.total_mass
    if ros_msg.com_pos_rt_payload_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.com_pos_rt_payload, proto.com_pos_rt_payload)
    if ros_msg.moi_tensor_is_set:
        convert_bosdyn_msgs_moment_of_intertia_to_proto(ros_msg.moi_tensor, proto.moi_tensor)
    del proto.bounding_box[:]
    for _item in ros_msg.bounding_box:
        convert_bosdyn_msgs_box3_with_frame_to_proto(_item, proto.bounding_box.add())
    del proto.joint_limits[:]
    for _item in ros_msg.joint_limits:
        convert_bosdyn_msgs_joint_limits_to_proto(_item, proto.joint_limits.add())

def convert_proto_to_bosdyn_msgs_moment_of_intertia(proto, ros_msg):
    ros_msg.xx = proto.xx
    ros_msg.yy = proto.yy
    ros_msg.zz = proto.zz
    ros_msg.xy = proto.xy
    ros_msg.xz = proto.xz
    ros_msg.yz = proto.yz

def convert_bosdyn_msgs_moment_of_intertia_to_proto(ros_msg, proto):
    proto.Clear()
    proto.xx = ros_msg.xx
    proto.yy = ros_msg.yy
    proto.zz = ros_msg.zz
    proto.xy = ros_msg.xy
    proto.xz = ros_msg.xz
    proto.yz = ros_msg.yz

def convert_proto_to_bosdyn_msgs_joint_limits(proto, ros_msg):
    ros_msg.label = proto.label
    ros_msg.hy = []
    for _item in proto.hy:
        ros_msg.hy.append(_item)
    ros_msg.hx = []
    for _item in proto.hx:
        ros_msg.hx.append(_item)

def convert_bosdyn_msgs_joint_limits_to_proto(ros_msg, proto):
    proto.Clear()
    proto.label = ros_msg.label
    del proto.hy[:]
    for _item in ros_msg.hy:
        proto.hy.add(_item)
    del proto.hx[:]
    for _item in ros_msg.hx:
        proto.hx.add(_item)

def convert_proto_to_bosdyn_msgs_list_payloads_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_payloads_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_payloads_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Payload
    ros_msg.payloads = []
    for _item in proto.payloads:
        ros_msg.payloads.append(Payload())
        convert_proto_to_bosdyn_msgs_payload(_item, ros_msg.payloads[-1])

def convert_bosdyn_msgs_list_payloads_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.payloads[:]
    for _item in ros_msg.payloads:
        convert_bosdyn_msgs_payload_to_proto(_item, proto.payloads.add())

def convert_proto_to_bosdyn_msgs_robot_command_one_of_command(proto, ros_msg):
    if proto.HasField("full_body_command"):
        ros_msg.command_choice = ros_msg.COMMAND_FULL_BODY_COMMAND_SET
        convert_proto_to_bosdyn_msgs_full_body_command_request(proto.full_body_command, ros_msg.full_body_command)
    if proto.HasField("synchronized_command"):
        ros_msg.command_choice = ros_msg.COMMAND_SYNCHRONIZED_COMMAND_SET
        convert_proto_to_bosdyn_msgs_synchronized_command_request(proto.synchronized_command, ros_msg.synchronized_command)

def convert_bosdyn_msgs_robot_command_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_FULL_BODY_COMMAND_SET:
        convert_bosdyn_msgs_full_body_command_request_to_proto(ros_msg.full_body_command, proto.full_body_command)
    if ros_msg.command_choice == ros_msg.COMMAND_SYNCHRONIZED_COMMAND_SET:
        convert_bosdyn_msgs_synchronized_command_request_to_proto(ros_msg.synchronized_command, proto.synchronized_command)

def convert_proto_to_bosdyn_msgs_robot_command(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_robot_command_one_of_command(proto, ros_msg.command)

def convert_bosdyn_msgs_robot_command_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_robot_command_one_of_command_to_proto(ros_msg.command, proto)

def convert_proto_to_bosdyn_msgs_robot_command_feedback_one_of_command(proto, ros_msg):
    if proto.HasField("full_body_feedback"):
        ros_msg.command_choice = ros_msg.COMMAND_FULL_BODY_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_full_body_command_feedback(proto.full_body_feedback, ros_msg.full_body_feedback)
    if proto.HasField("synchronized_feedback"):
        ros_msg.command_choice = ros_msg.COMMAND_SYNCHRONIZED_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_synchronized_command_feedback(proto.synchronized_feedback, ros_msg.synchronized_feedback)

def convert_bosdyn_msgs_robot_command_feedback_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_FULL_BODY_FEEDBACK_SET:
        convert_bosdyn_msgs_full_body_command_feedback_to_proto(ros_msg.full_body_feedback, proto.full_body_feedback)
    if ros_msg.command_choice == ros_msg.COMMAND_SYNCHRONIZED_FEEDBACK_SET:
        convert_bosdyn_msgs_synchronized_command_feedback_to_proto(ros_msg.synchronized_feedback, proto.synchronized_feedback)

def convert_proto_to_bosdyn_msgs_robot_command_feedback(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_robot_command_feedback_one_of_command(proto, ros_msg.command)

def convert_bosdyn_msgs_robot_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_robot_command_feedback_one_of_command_to_proto(ros_msg.command, proto)

def convert_proto_to_bosdyn_msgs_robot_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    convert_proto_to_bosdyn_msgs_robot_command(proto.command, ros_msg.command)
    ros_msg.command_is_set = proto.HasField("command")
    ros_msg.clock_identifier = proto.clock_identifier

def convert_bosdyn_msgs_robot_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    if ros_msg.command_is_set:
        convert_bosdyn_msgs_robot_command_to_proto(ros_msg.command, proto.command)
    proto.clock_identifier = ros_msg.clock_identifier

def convert_proto_to_bosdyn_msgs_robot_command_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    ros_msg.message = proto.message
    ros_msg.robot_command_id = proto.robot_command_id

def convert_bosdyn_msgs_robot_command_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    proto.message = ros_msg.message
    proto.robot_command_id = ros_msg.robot_command_id

def convert_proto_to_bosdyn_msgs_robot_command_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.robot_command_id = proto.robot_command_id

def convert_bosdyn_msgs_robot_command_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.robot_command_id = ros_msg.robot_command_id

def convert_proto_to_bosdyn_msgs_robot_command_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    convert_proto_to_bosdyn_msgs_robot_command_feedback(proto.feedback, ros_msg.feedback)
    ros_msg.feedback_is_set = proto.HasField("feedback")

def convert_bosdyn_msgs_robot_command_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    if ros_msg.feedback_is_set:
        convert_bosdyn_msgs_robot_command_feedback_to_proto(ros_msg.feedback, proto.feedback)

def convert_proto_to_bosdyn_msgs_clear_behavior_fault_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    ros_msg.behavior_fault_id = proto.behavior_fault_id

def convert_bosdyn_msgs_clear_behavior_fault_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    proto.behavior_fault_id = ros_msg.behavior_fault_id

def convert_proto_to_bosdyn_msgs_clear_behavior_fault_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_clear_behavior_fault_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_gripper_camera_param_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_gripper_camera_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")

def convert_bosdyn_msgs_gripper_camera_param_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_gripper_camera_params_to_proto(ros_msg.params, proto.params)

def convert_proto_to_bosdyn_msgs_gripper_camera_param_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_gripper_camera_param_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_gripper_camera_get_param_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_gripper_camera_get_param_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_gripper_camera_get_param_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_gripper_camera_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")

def convert_bosdyn_msgs_gripper_camera_get_param_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_gripper_camera_params_to_proto(ros_msg.params, proto.params)

def convert_proto_to_bosdyn_msgs_gripper_camera_params(proto, ros_msg):
    ros_msg.camera_mode.value = proto.camera_mode
    ros_msg.brightness = proto.brightness.value
    ros_msg.brightness_is_set = proto.HasField("brightness")
    ros_msg.contrast = proto.contrast.value
    ros_msg.contrast_is_set = proto.HasField("contrast")
    ros_msg.saturation = proto.saturation.value
    ros_msg.saturation_is_set = proto.HasField("saturation")
    ros_msg.gain = proto.gain.value
    ros_msg.gain_is_set = proto.HasField("gain")
    ros_msg.exposure_auto = proto.exposure_auto.value
    ros_msg.exposure_auto_is_set = proto.HasField("exposure_auto")
    ros_msg.exposure_absolute = proto.exposure_absolute.value
    ros_msg.exposure_absolute_is_set = proto.HasField("exposure_absolute")
    convert_proto_to_bosdyn_msgs_roi_parameters(proto.exposure_roi, ros_msg.exposure_roi)
    ros_msg.exposure_roi_is_set = proto.HasField("exposure_roi")
    ros_msg.focus_auto = proto.focus_auto.value
    ros_msg.focus_auto_is_set = proto.HasField("focus_auto")
    ros_msg.focus_absolute = proto.focus_absolute.value
    ros_msg.focus_absolute_is_set = proto.HasField("focus_absolute")
    convert_proto_to_bosdyn_msgs_roi_parameters(proto.focus_roi, ros_msg.focus_roi)
    ros_msg.focus_roi_is_set = proto.HasField("focus_roi")
    ros_msg.draw_focus_roi_rectangle = proto.draw_focus_roi_rectangle.value
    ros_msg.draw_focus_roi_rectangle_is_set = proto.HasField("draw_focus_roi_rectangle")
    ros_msg.hdr.value = proto.hdr
    ros_msg.led_mode.value = proto.led_mode
    ros_msg.led_torch_brightness = proto.led_torch_brightness.value
    ros_msg.led_torch_brightness_is_set = proto.HasField("led_torch_brightness")

def convert_bosdyn_msgs_gripper_camera_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.camera_mode = ros_msg.camera_mode.value
    if ros_msg.brightness_is_set:
        convert_float32_to_proto(ros_msg.brightness, proto.brightness)
    if ros_msg.contrast_is_set:
        convert_float32_to_proto(ros_msg.contrast, proto.contrast)
    if ros_msg.saturation_is_set:
        convert_float32_to_proto(ros_msg.saturation, proto.saturation)
    if ros_msg.gain_is_set:
        convert_float32_to_proto(ros_msg.gain, proto.gain)
    if ros_msg.exposure_auto_is_set:
        convert_bool_to_proto(ros_msg.exposure_auto, proto.exposure_auto)
    if ros_msg.exposure_absolute_is_set:
        convert_float32_to_proto(ros_msg.exposure_absolute, proto.exposure_absolute)
    if ros_msg.exposure_roi_is_set:
        convert_bosdyn_msgs_roi_parameters_to_proto(ros_msg.exposure_roi, proto.exposure_roi)
    if ros_msg.focus_auto_is_set:
        convert_bool_to_proto(ros_msg.focus_auto, proto.focus_auto)
    if ros_msg.focus_absolute_is_set:
        convert_float32_to_proto(ros_msg.focus_absolute, proto.focus_absolute)
    if ros_msg.focus_roi_is_set:
        convert_bosdyn_msgs_roi_parameters_to_proto(ros_msg.focus_roi, proto.focus_roi)
    if ros_msg.draw_focus_roi_rectangle_is_set:
        convert_bool_to_proto(ros_msg.draw_focus_roi_rectangle, proto.draw_focus_roi_rectangle)
    proto.hdr = ros_msg.hdr.value
    proto.led_mode = ros_msg.led_mode.value
    if ros_msg.led_torch_brightness_is_set:
        convert_float32_to_proto(ros_msg.led_torch_brightness, proto.led_torch_brightness)

def convert_proto_to_bosdyn_msgs_roi_parameters(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.roi_percentage_in_image, ros_msg.roi_percentage_in_image)
    ros_msg.roi_percentage_in_image_is_set = proto.HasField("roi_percentage_in_image")
    ros_msg.window_size.value = proto.window_size

def convert_bosdyn_msgs_roi_parameters_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.roi_percentage_in_image_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.roi_percentage_in_image, proto.roi_percentage_in_image)
    proto.window_size = ros_msg.window_size.value

def convert_proto_to_bosdyn_msgs_add_log_annotation_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_log_annotations(proto.annotations, ros_msg.annotations)
    ros_msg.annotations_is_set = proto.HasField("annotations")

def convert_bosdyn_msgs_add_log_annotation_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.annotations_is_set:
        convert_bosdyn_msgs_log_annotations_to_proto(ros_msg.annotations, proto.annotations)

def convert_proto_to_bosdyn_msgs_log_annotations(proto, ros_msg):
    from bosdyn_msgs.msg import LogAnnotationTextMessage
    ros_msg.text_messages = []
    for _item in proto.text_messages:
        ros_msg.text_messages.append(LogAnnotationTextMessage())
        convert_proto_to_bosdyn_msgs_log_annotation_text_message(_item, ros_msg.text_messages[-1])
    from bosdyn_msgs.msg import LogAnnotationOperatorMessage
    ros_msg.operator_messages = []
    for _item in proto.operator_messages:
        ros_msg.operator_messages.append(LogAnnotationOperatorMessage())
        convert_proto_to_bosdyn_msgs_log_annotation_operator_message(_item, ros_msg.operator_messages[-1])
    from bosdyn_msgs.msg import LogAnnotationLogBlob
    ros_msg.blob_data = []
    for _item in proto.blob_data:
        ros_msg.blob_data.append(LogAnnotationLogBlob())
        convert_proto_to_bosdyn_msgs_log_annotation_log_blob(_item, ros_msg.blob_data[-1])

def convert_bosdyn_msgs_log_annotations_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.text_messages[:]
    for _item in ros_msg.text_messages:
        convert_bosdyn_msgs_log_annotation_text_message_to_proto(_item, proto.text_messages.add())
    del proto.operator_messages[:]
    for _item in ros_msg.operator_messages:
        convert_bosdyn_msgs_log_annotation_operator_message_to_proto(_item, proto.operator_messages.add())
    del proto.blob_data[:]
    for _item in ros_msg.blob_data:
        convert_bosdyn_msgs_log_annotation_log_blob_to_proto(_item, proto.blob_data.add())

def convert_proto_to_bosdyn_msgs_log_annotation_text_message(proto, ros_msg):
    ros_msg.message = proto.message
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.service = proto.service
    ros_msg.level.value = proto.level
    ros_msg.tag = proto.tag
    ros_msg.filename = proto.filename
    ros_msg.line_number = proto.line_number
    convert_proto_to_builtin_interfaces_time(proto.timestamp_client, ros_msg.timestamp_client)
    ros_msg.timestamp_client_is_set = proto.HasField("timestamp_client")

def convert_bosdyn_msgs_log_annotation_text_message_to_proto(ros_msg, proto):
    proto.Clear()
    proto.message = ros_msg.message
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.service = ros_msg.service
    proto.level = ros_msg.level.value
    proto.tag = ros_msg.tag
    proto.filename = ros_msg.filename
    proto.line_number = ros_msg.line_number
    if ros_msg.timestamp_client_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp_client, proto.timestamp_client)

def convert_proto_to_bosdyn_msgs_log_annotation_operator_message(proto, ros_msg):
    ros_msg.message = proto.message
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    convert_proto_to_builtin_interfaces_time(proto.timestamp_client, ros_msg.timestamp_client)
    ros_msg.timestamp_client_is_set = proto.HasField("timestamp_client")

def convert_bosdyn_msgs_log_annotation_operator_message_to_proto(ros_msg, proto):
    proto.Clear()
    proto.message = ros_msg.message
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    if ros_msg.timestamp_client_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp_client, proto.timestamp_client)

def convert_proto_to_bosdyn_msgs_log_annotation_log_blob(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.channel = proto.channel
    ros_msg.type_id = proto.type_id
    ros_msg.data = proto.data
    convert_proto_to_builtin_interfaces_time(proto.timestamp_client, ros_msg.timestamp_client)
    ros_msg.timestamp_client_is_set = proto.HasField("timestamp_client")

def convert_bosdyn_msgs_log_annotation_log_blob_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.channel = ros_msg.channel
    proto.type_id = ros_msg.type_id
    proto.data = ros_msg.data
    if ros_msg.timestamp_client_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp_client, proto.timestamp_client)

def convert_proto_to_bosdyn_msgs_add_log_annotation_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_add_log_annotation_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_alert_data(proto, ros_msg):
    ros_msg.severity.value = proto.severity
    ros_msg.title = proto.title
    ros_msg.source = proto.source

def convert_bosdyn_msgs_alert_data_to_proto(ros_msg, proto):
    proto.Clear()
    proto.severity = ros_msg.severity.value
    proto.title = ros_msg.title
    proto.source = ros_msg.source

def convert_proto_to_bosdyn_msgs_arm_command_request_one_of_command(proto, ros_msg):
    if proto.HasField("arm_cartesian_command"):
        ros_msg.command_choice = ros_msg.COMMAND_ARM_CARTESIAN_COMMAND_SET
        convert_proto_to_bosdyn_msgs_arm_cartesian_command_request(proto.arm_cartesian_command, ros_msg.arm_cartesian_command)
    if proto.HasField("arm_joint_move_command"):
        ros_msg.command_choice = ros_msg.COMMAND_ARM_JOINT_MOVE_COMMAND_SET
        convert_proto_to_bosdyn_msgs_arm_joint_move_command_request(proto.arm_joint_move_command, ros_msg.arm_joint_move_command)
    if proto.HasField("named_arm_position_command"):
        ros_msg.command_choice = ros_msg.COMMAND_NAMED_ARM_POSITION_COMMAND_SET
        convert_proto_to_bosdyn_msgs_named_arm_positions_command_request(proto.named_arm_position_command, ros_msg.named_arm_position_command)
    if proto.HasField("arm_velocity_command"):
        ros_msg.command_choice = ros_msg.COMMAND_ARM_VELOCITY_COMMAND_SET
        convert_proto_to_bosdyn_msgs_arm_velocity_command_request(proto.arm_velocity_command, ros_msg.arm_velocity_command)
    if proto.HasField("arm_gaze_command"):
        ros_msg.command_choice = ros_msg.COMMAND_ARM_GAZE_COMMAND_SET
        convert_proto_to_bosdyn_msgs_gaze_command_request(proto.arm_gaze_command, ros_msg.arm_gaze_command)
    if proto.HasField("arm_stop_command"):
        ros_msg.command_choice = ros_msg.COMMAND_ARM_STOP_COMMAND_SET
        convert_proto_to_bosdyn_msgs_arm_stop_command_request(proto.arm_stop_command, ros_msg.arm_stop_command)
    if proto.HasField("arm_drag_command"):
        ros_msg.command_choice = ros_msg.COMMAND_ARM_DRAG_COMMAND_SET
        convert_proto_to_bosdyn_msgs_arm_drag_command_request(proto.arm_drag_command, ros_msg.arm_drag_command)
    if proto.HasField("arm_impedance_command"):
        ros_msg.command_choice = ros_msg.COMMAND_ARM_IMPEDANCE_COMMAND_SET
        convert_proto_to_bosdyn_msgs_arm_impedance_command_request(proto.arm_impedance_command, ros_msg.arm_impedance_command)

def convert_bosdyn_msgs_arm_command_request_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_ARM_CARTESIAN_COMMAND_SET:
        convert_bosdyn_msgs_arm_cartesian_command_request_to_proto(ros_msg.arm_cartesian_command, proto.arm_cartesian_command)
    if ros_msg.command_choice == ros_msg.COMMAND_ARM_JOINT_MOVE_COMMAND_SET:
        convert_bosdyn_msgs_arm_joint_move_command_request_to_proto(ros_msg.arm_joint_move_command, proto.arm_joint_move_command)
    if ros_msg.command_choice == ros_msg.COMMAND_NAMED_ARM_POSITION_COMMAND_SET:
        convert_bosdyn_msgs_named_arm_positions_command_request_to_proto(ros_msg.named_arm_position_command, proto.named_arm_position_command)
    if ros_msg.command_choice == ros_msg.COMMAND_ARM_VELOCITY_COMMAND_SET:
        convert_bosdyn_msgs_arm_velocity_command_request_to_proto(ros_msg.arm_velocity_command, proto.arm_velocity_command)
    if ros_msg.command_choice == ros_msg.COMMAND_ARM_GAZE_COMMAND_SET:
        convert_bosdyn_msgs_gaze_command_request_to_proto(ros_msg.arm_gaze_command, proto.arm_gaze_command)
    if ros_msg.command_choice == ros_msg.COMMAND_ARM_STOP_COMMAND_SET:
        convert_bosdyn_msgs_arm_stop_command_request_to_proto(ros_msg.arm_stop_command, proto.arm_stop_command)
    if ros_msg.command_choice == ros_msg.COMMAND_ARM_DRAG_COMMAND_SET:
        convert_bosdyn_msgs_arm_drag_command_request_to_proto(ros_msg.arm_drag_command, proto.arm_drag_command)
    if ros_msg.command_choice == ros_msg.COMMAND_ARM_IMPEDANCE_COMMAND_SET:
        convert_bosdyn_msgs_arm_impedance_command_request_to_proto(ros_msg.arm_impedance_command, proto.arm_impedance_command)

def convert_proto_to_bosdyn_msgs_arm_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_arm_command_request_one_of_command(proto, ros_msg.command)
    convert_proto_to_bosdyn_msgs_arm_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")

def convert_bosdyn_msgs_arm_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_arm_command_request_one_of_command_to_proto(ros_msg.command, proto)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_arm_params_to_proto(ros_msg.params, proto.params)

def convert_proto_to_bosdyn_msgs_arm_command_feedback_one_of_feedback(proto, ros_msg):
    if proto.HasField("arm_cartesian_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_arm_cartesian_command_feedback(proto.arm_cartesian_feedback, ros_msg.arm_cartesian_feedback)
    if proto.HasField("arm_joint_move_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_arm_joint_move_command_feedback(proto.arm_joint_move_feedback, ros_msg.arm_joint_move_feedback)
    if proto.HasField("named_arm_position_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_named_arm_positions_command_feedback(proto.named_arm_position_feedback, ros_msg.named_arm_position_feedback)
    if proto.HasField("arm_velocity_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_ARM_VELOCITY_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_arm_velocity_command_feedback(proto.arm_velocity_feedback, ros_msg.arm_velocity_feedback)
    if proto.HasField("arm_gaze_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_ARM_GAZE_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_gaze_command_feedback(proto.arm_gaze_feedback, ros_msg.arm_gaze_feedback)
    if proto.HasField("arm_stop_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_ARM_STOP_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_arm_stop_command_feedback(proto.arm_stop_feedback, ros_msg.arm_stop_feedback)
    if proto.HasField("arm_drag_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_ARM_DRAG_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_arm_drag_command_feedback(proto.arm_drag_feedback, ros_msg.arm_drag_feedback)
    if proto.HasField("arm_impedance_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_arm_impedance_command_feedback(proto.arm_impedance_feedback, ros_msg.arm_impedance_feedback)

def convert_bosdyn_msgs_arm_command_feedback_one_of_feedback_to_proto(ros_msg, proto):
    proto.ClearField("feedback")
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_ARM_CARTESIAN_FEEDBACK_SET:
        convert_bosdyn_msgs_arm_cartesian_command_feedback_to_proto(ros_msg.arm_cartesian_feedback, proto.arm_cartesian_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_ARM_JOINT_MOVE_FEEDBACK_SET:
        convert_bosdyn_msgs_arm_joint_move_command_feedback_to_proto(ros_msg.arm_joint_move_feedback, proto.arm_joint_move_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_NAMED_ARM_POSITION_FEEDBACK_SET:
        convert_bosdyn_msgs_named_arm_positions_command_feedback_to_proto(ros_msg.named_arm_position_feedback, proto.named_arm_position_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_ARM_VELOCITY_FEEDBACK_SET:
        convert_bosdyn_msgs_arm_velocity_command_feedback_to_proto(ros_msg.arm_velocity_feedback, proto.arm_velocity_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_ARM_GAZE_FEEDBACK_SET:
        convert_bosdyn_msgs_gaze_command_feedback_to_proto(ros_msg.arm_gaze_feedback, proto.arm_gaze_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_ARM_STOP_FEEDBACK_SET:
        convert_bosdyn_msgs_arm_stop_command_feedback_to_proto(ros_msg.arm_stop_feedback, proto.arm_stop_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_ARM_DRAG_FEEDBACK_SET:
        convert_bosdyn_msgs_arm_drag_command_feedback_to_proto(ros_msg.arm_drag_feedback, proto.arm_drag_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_ARM_IMPEDANCE_FEEDBACK_SET:
        convert_bosdyn_msgs_arm_impedance_command_feedback_to_proto(ros_msg.arm_impedance_feedback, proto.arm_impedance_feedback)

def convert_proto_to_bosdyn_msgs_arm_command_feedback(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_arm_command_feedback_one_of_feedback(proto, ros_msg.feedback)
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_arm_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_arm_command_feedback_one_of_feedback_to_proto(ros_msg.feedback, proto)
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_arm_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_arm_params(proto, ros_msg):
    ros_msg.disable_body_force_limiter = proto.disable_body_force_limiter.value
    ros_msg.disable_body_force_limiter_is_set = proto.HasField("disable_body_force_limiter")

def convert_bosdyn_msgs_arm_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.disable_body_force_limiter_is_set:
        convert_bool_to_proto(ros_msg.disable_body_force_limiter, proto.disable_body_force_limiter)

def convert_proto_to_bosdyn_msgs_arm_velocity_command_cylindrical_velocity(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_cylindrical_coordinate(proto.linear_velocity, ros_msg.linear_velocity)
    ros_msg.linear_velocity_is_set = proto.HasField("linear_velocity")
    ros_msg.max_linear_velocity = proto.max_linear_velocity.value
    ros_msg.max_linear_velocity_is_set = proto.HasField("max_linear_velocity")

def convert_bosdyn_msgs_arm_velocity_command_cylindrical_velocity_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.linear_velocity_is_set:
        convert_bosdyn_msgs_cylindrical_coordinate_to_proto(ros_msg.linear_velocity, proto.linear_velocity)
    if ros_msg.max_linear_velocity_is_set:
        convert_float64_to_proto(ros_msg.max_linear_velocity, proto.max_linear_velocity)

def convert_proto_to_bosdyn_msgs_arm_velocity_command_cartesian_velocity(proto, ros_msg):
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_vector3(proto.velocity_in_frame_name, ros_msg.velocity_in_frame_name)
    ros_msg.velocity_in_frame_name_is_set = proto.HasField("velocity_in_frame_name")

def convert_bosdyn_msgs_arm_velocity_command_cartesian_velocity_to_proto(ros_msg, proto):
    proto.Clear()
    proto.frame_name = ros_msg.frame_name
    if ros_msg.velocity_in_frame_name_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.velocity_in_frame_name, proto.velocity_in_frame_name)

def convert_proto_to_bosdyn_msgs_arm_velocity_command_request_one_of_command(proto, ros_msg):
    if proto.HasField("cylindrical_velocity"):
        ros_msg.command_choice = ros_msg.COMMAND_CYLINDRICAL_VELOCITY_SET
        convert_proto_to_bosdyn_msgs_arm_velocity_command_cylindrical_velocity(proto.cylindrical_velocity, ros_msg.cylindrical_velocity)
    if proto.HasField("cartesian_velocity"):
        ros_msg.command_choice = ros_msg.COMMAND_CARTESIAN_VELOCITY_SET
        convert_proto_to_bosdyn_msgs_arm_velocity_command_cartesian_velocity(proto.cartesian_velocity, ros_msg.cartesian_velocity)

def convert_bosdyn_msgs_arm_velocity_command_request_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_CYLINDRICAL_VELOCITY_SET:
        convert_bosdyn_msgs_arm_velocity_command_cylindrical_velocity_to_proto(ros_msg.cylindrical_velocity, proto.cylindrical_velocity)
    if ros_msg.command_choice == ros_msg.COMMAND_CARTESIAN_VELOCITY_SET:
        convert_bosdyn_msgs_arm_velocity_command_cartesian_velocity_to_proto(ros_msg.cartesian_velocity, proto.cartesian_velocity)

def convert_proto_to_bosdyn_msgs_arm_velocity_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_arm_velocity_command_request_one_of_command(proto, ros_msg.command)
    convert_proto_to_geometry_msgs_vector3(proto.angular_velocity_of_hand_rt_odom_in_hand, ros_msg.angular_velocity_of_hand_rt_odom_in_hand)
    ros_msg.angular_velocity_of_hand_rt_odom_in_hand_is_set = proto.HasField("angular_velocity_of_hand_rt_odom_in_hand")
    ros_msg.maximum_acceleration = proto.maximum_acceleration.value
    ros_msg.maximum_acceleration_is_set = proto.HasField("maximum_acceleration")
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")

def convert_bosdyn_msgs_arm_velocity_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_arm_velocity_command_request_one_of_command_to_proto(ros_msg.command, proto)
    if ros_msg.angular_velocity_of_hand_rt_odom_in_hand_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.angular_velocity_of_hand_rt_odom_in_hand, proto.angular_velocity_of_hand_rt_odom_in_hand)
    if ros_msg.maximum_acceleration_is_set:
        convert_float64_to_proto(ros_msg.maximum_acceleration, proto.maximum_acceleration)
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)

def convert_bosdyn_msgs_arm_velocity_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_arm_velocity_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_named_arm_positions_command_request(proto, ros_msg):
    ros_msg.position.value = proto.position

def convert_bosdyn_msgs_named_arm_positions_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.position = ros_msg.position.value

def convert_proto_to_bosdyn_msgs_named_arm_positions_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_named_arm_positions_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_named_arm_positions_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_arm_cartesian_command_request_one_of_joint_configuration(proto, ros_msg):
    if proto.HasField("force_remain_near_current_joint_configuration"):
        ros_msg.joint_configuration_choice = ros_msg.JOINT_CONFIGURATION_FORCE_REMAIN_NEAR_CURRENT_JOINT_CONFIGURATION_SET
        ros_msg.force_remain_near_current_joint_configuration = proto.force_remain_near_current_joint_configuration
    if proto.HasField("preferred_joint_configuration"):
        ros_msg.joint_configuration_choice = ros_msg.JOINT_CONFIGURATION_PREFERRED_JOINT_CONFIGURATION_SET
        convert_proto_to_bosdyn_msgs_arm_joint_position(proto.preferred_joint_configuration, ros_msg.preferred_joint_configuration)

def convert_bosdyn_msgs_arm_cartesian_command_request_one_of_joint_configuration_to_proto(ros_msg, proto):
    proto.ClearField("joint_configuration")
    if ros_msg.joint_configuration_choice == ros_msg.JOINT_CONFIGURATION_FORCE_REMAIN_NEAR_CURRENT_JOINT_CONFIGURATION_SET:
        proto.force_remain_near_current_joint_configuration = ros_msg.force_remain_near_current_joint_configuration
    if ros_msg.joint_configuration_choice == ros_msg.JOINT_CONFIGURATION_PREFERRED_JOINT_CONFIGURATION_SET:
        convert_bosdyn_msgs_arm_joint_position_to_proto(ros_msg.preferred_joint_configuration, proto.preferred_joint_configuration)

def convert_proto_to_bosdyn_msgs_arm_cartesian_command_request(proto, ros_msg):
    ros_msg.root_frame_name = proto.root_frame_name
    convert_proto_to_geometry_msgs_pose(proto.wrist_tform_tool, ros_msg.wrist_tform_tool)
    ros_msg.wrist_tform_tool_is_set = proto.HasField("wrist_tform_tool")
    convert_proto_to_geometry_msgs_pose(proto.root_tform_task, ros_msg.root_tform_task)
    ros_msg.root_tform_task_is_set = proto.HasField("root_tform_task")
    convert_proto_to_bosdyn_msgs_se3_trajectory(proto.pose_trajectory_in_task, ros_msg.pose_trajectory_in_task)
    ros_msg.pose_trajectory_in_task_is_set = proto.HasField("pose_trajectory_in_task")
    ros_msg.maximum_acceleration = proto.maximum_acceleration.value
    ros_msg.maximum_acceleration_is_set = proto.HasField("maximum_acceleration")
    ros_msg.max_linear_velocity = proto.max_linear_velocity.value
    ros_msg.max_linear_velocity_is_set = proto.HasField("max_linear_velocity")
    ros_msg.max_angular_velocity = proto.max_angular_velocity.value
    ros_msg.max_angular_velocity_is_set = proto.HasField("max_angular_velocity")
    ros_msg.max_pos_tracking_error = proto.max_pos_tracking_error.value
    ros_msg.max_pos_tracking_error_is_set = proto.HasField("max_pos_tracking_error")
    ros_msg.max_rot_tracking_error = proto.max_rot_tracking_error.value
    ros_msg.max_rot_tracking_error_is_set = proto.HasField("max_rot_tracking_error")
    convert_proto_to_bosdyn_msgs_arm_cartesian_command_request_one_of_joint_configuration(proto, ros_msg.joint_configuration)
    ros_msg.x_axis.value = proto.x_axis
    ros_msg.y_axis.value = proto.y_axis
    ros_msg.z_axis.value = proto.z_axis
    ros_msg.rx_axis.value = proto.rx_axis
    ros_msg.ry_axis.value = proto.ry_axis
    ros_msg.rz_axis.value = proto.rz_axis
    convert_proto_to_bosdyn_msgs_wrench_trajectory(proto.wrench_trajectory_in_task, ros_msg.wrench_trajectory_in_task)
    ros_msg.wrench_trajectory_in_task_is_set = proto.HasField("wrench_trajectory_in_task")

def convert_bosdyn_msgs_arm_cartesian_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.root_frame_name = ros_msg.root_frame_name
    if ros_msg.wrist_tform_tool_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.wrist_tform_tool, proto.wrist_tform_tool)
    if ros_msg.root_tform_task_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.root_tform_task, proto.root_tform_task)
    if ros_msg.pose_trajectory_in_task_is_set:
        convert_bosdyn_msgs_se3_trajectory_to_proto(ros_msg.pose_trajectory_in_task, proto.pose_trajectory_in_task)
    if ros_msg.maximum_acceleration_is_set:
        convert_float64_to_proto(ros_msg.maximum_acceleration, proto.maximum_acceleration)
    if ros_msg.max_linear_velocity_is_set:
        convert_float64_to_proto(ros_msg.max_linear_velocity, proto.max_linear_velocity)
    if ros_msg.max_angular_velocity_is_set:
        convert_float64_to_proto(ros_msg.max_angular_velocity, proto.max_angular_velocity)
    if ros_msg.max_pos_tracking_error_is_set:
        convert_float64_to_proto(ros_msg.max_pos_tracking_error, proto.max_pos_tracking_error)
    if ros_msg.max_rot_tracking_error_is_set:
        convert_float64_to_proto(ros_msg.max_rot_tracking_error, proto.max_rot_tracking_error)
    convert_bosdyn_msgs_arm_cartesian_command_request_one_of_joint_configuration_to_proto(ros_msg.joint_configuration, proto)
    proto.x_axis = ros_msg.x_axis.value
    proto.y_axis = ros_msg.y_axis.value
    proto.z_axis = ros_msg.z_axis.value
    proto.rx_axis = ros_msg.rx_axis.value
    proto.ry_axis = ros_msg.ry_axis.value
    proto.rz_axis = ros_msg.rz_axis.value
    if ros_msg.wrench_trajectory_in_task_is_set:
        convert_bosdyn_msgs_wrench_trajectory_to_proto(ros_msg.wrench_trajectory_in_task, proto.wrench_trajectory_in_task)

def convert_proto_to_bosdyn_msgs_arm_cartesian_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.measured_pos_tracking_error = proto.measured_pos_tracking_error
    ros_msg.measured_rot_tracking_error = proto.measured_rot_tracking_error
    ros_msg.measured_pos_distance_to_goal = proto.measured_pos_distance_to_goal
    ros_msg.measured_rot_distance_to_goal = proto.measured_rot_distance_to_goal

def convert_bosdyn_msgs_arm_cartesian_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.measured_pos_tracking_error = ros_msg.measured_pos_tracking_error
    proto.measured_rot_tracking_error = ros_msg.measured_rot_tracking_error
    proto.measured_pos_distance_to_goal = ros_msg.measured_pos_distance_to_goal
    proto.measured_rot_distance_to_goal = ros_msg.measured_rot_distance_to_goal

def convert_bosdyn_msgs_arm_cartesian_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_arm_joint_move_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_arm_joint_trajectory(proto.trajectory, ros_msg.trajectory)
    ros_msg.trajectory_is_set = proto.HasField("trajectory")

def convert_bosdyn_msgs_arm_joint_move_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.trajectory_is_set:
        convert_bosdyn_msgs_arm_joint_trajectory_to_proto(ros_msg.trajectory, proto.trajectory)

def convert_proto_to_bosdyn_msgs_arm_joint_move_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.planner_status.value = proto.planner_status
    from bosdyn_msgs.msg import ArmJointTrajectoryPoint
    ros_msg.planned_points = []
    for _item in proto.planned_points:
        ros_msg.planned_points.append(ArmJointTrajectoryPoint())
        convert_proto_to_bosdyn_msgs_arm_joint_trajectory_point(_item, ros_msg.planned_points[-1])
    convert_proto_to_builtin_interfaces_duration(proto.time_to_goal, ros_msg.time_to_goal)
    ros_msg.time_to_goal_is_set = proto.HasField("time_to_goal")

def convert_bosdyn_msgs_arm_joint_move_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.planner_status = ros_msg.planner_status.value
    del proto.planned_points[:]
    for _item in ros_msg.planned_points:
        convert_bosdyn_msgs_arm_joint_trajectory_point_to_proto(_item, proto.planned_points.add())
    if ros_msg.time_to_goal_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.time_to_goal, proto.time_to_goal)

def convert_bosdyn_msgs_arm_joint_move_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_arm_joint_position(proto, ros_msg):
    ros_msg.sh0 = proto.sh0.value
    ros_msg.sh0_is_set = proto.HasField("sh0")
    ros_msg.sh1 = proto.sh1.value
    ros_msg.sh1_is_set = proto.HasField("sh1")
    ros_msg.el0 = proto.el0.value
    ros_msg.el0_is_set = proto.HasField("el0")
    ros_msg.el1 = proto.el1.value
    ros_msg.el1_is_set = proto.HasField("el1")
    ros_msg.wr0 = proto.wr0.value
    ros_msg.wr0_is_set = proto.HasField("wr0")
    ros_msg.wr1 = proto.wr1.value
    ros_msg.wr1_is_set = proto.HasField("wr1")

def convert_bosdyn_msgs_arm_joint_position_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.sh0_is_set:
        convert_float64_to_proto(ros_msg.sh0, proto.sh0)
    if ros_msg.sh1_is_set:
        convert_float64_to_proto(ros_msg.sh1, proto.sh1)
    if ros_msg.el0_is_set:
        convert_float64_to_proto(ros_msg.el0, proto.el0)
    if ros_msg.el1_is_set:
        convert_float64_to_proto(ros_msg.el1, proto.el1)
    if ros_msg.wr0_is_set:
        convert_float64_to_proto(ros_msg.wr0, proto.wr0)
    if ros_msg.wr1_is_set:
        convert_float64_to_proto(ros_msg.wr1, proto.wr1)

def convert_proto_to_bosdyn_msgs_arm_joint_velocity(proto, ros_msg):
    ros_msg.sh0 = proto.sh0.value
    ros_msg.sh0_is_set = proto.HasField("sh0")
    ros_msg.sh1 = proto.sh1.value
    ros_msg.sh1_is_set = proto.HasField("sh1")
    ros_msg.el0 = proto.el0.value
    ros_msg.el0_is_set = proto.HasField("el0")
    ros_msg.el1 = proto.el1.value
    ros_msg.el1_is_set = proto.HasField("el1")
    ros_msg.wr0 = proto.wr0.value
    ros_msg.wr0_is_set = proto.HasField("wr0")
    ros_msg.wr1 = proto.wr1.value
    ros_msg.wr1_is_set = proto.HasField("wr1")

def convert_bosdyn_msgs_arm_joint_velocity_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.sh0_is_set:
        convert_float64_to_proto(ros_msg.sh0, proto.sh0)
    if ros_msg.sh1_is_set:
        convert_float64_to_proto(ros_msg.sh1, proto.sh1)
    if ros_msg.el0_is_set:
        convert_float64_to_proto(ros_msg.el0, proto.el0)
    if ros_msg.el1_is_set:
        convert_float64_to_proto(ros_msg.el1, proto.el1)
    if ros_msg.wr0_is_set:
        convert_float64_to_proto(ros_msg.wr0, proto.wr0)
    if ros_msg.wr1_is_set:
        convert_float64_to_proto(ros_msg.wr1, proto.wr1)

def convert_proto_to_bosdyn_msgs_arm_joint_trajectory_point(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_arm_joint_position(proto.position, ros_msg.position)
    ros_msg.position_is_set = proto.HasField("position")
    convert_proto_to_bosdyn_msgs_arm_joint_velocity(proto.velocity, ros_msg.velocity)
    ros_msg.velocity_is_set = proto.HasField("velocity")
    convert_proto_to_builtin_interfaces_duration(proto.time_since_reference, ros_msg.time_since_reference)
    ros_msg.time_since_reference_is_set = proto.HasField("time_since_reference")

def convert_bosdyn_msgs_arm_joint_trajectory_point_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.position_is_set:
        convert_bosdyn_msgs_arm_joint_position_to_proto(ros_msg.position, proto.position)
    if ros_msg.velocity_is_set:
        convert_bosdyn_msgs_arm_joint_velocity_to_proto(ros_msg.velocity, proto.velocity)
    if ros_msg.time_since_reference_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.time_since_reference, proto.time_since_reference)

def convert_proto_to_bosdyn_msgs_arm_joint_trajectory(proto, ros_msg):
    from bosdyn_msgs.msg import ArmJointTrajectoryPoint
    ros_msg.points = []
    for _item in proto.points:
        ros_msg.points.append(ArmJointTrajectoryPoint())
        convert_proto_to_bosdyn_msgs_arm_joint_trajectory_point(_item, ros_msg.points[-1])
    convert_proto_to_builtin_interfaces_time(proto.reference_time, ros_msg.reference_time)
    ros_msg.reference_time_is_set = proto.HasField("reference_time")
    ros_msg.maximum_velocity = proto.maximum_velocity.value
    ros_msg.maximum_velocity_is_set = proto.HasField("maximum_velocity")
    ros_msg.maximum_acceleration = proto.maximum_acceleration.value
    ros_msg.maximum_acceleration_is_set = proto.HasField("maximum_acceleration")

def convert_bosdyn_msgs_arm_joint_trajectory_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.points[:]
    for _item in ros_msg.points:
        convert_bosdyn_msgs_arm_joint_trajectory_point_to_proto(_item, proto.points.add())
    if ros_msg.reference_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.reference_time, proto.reference_time)
    if ros_msg.maximum_velocity_is_set:
        convert_float64_to_proto(ros_msg.maximum_velocity, proto.maximum_velocity)
    if ros_msg.maximum_acceleration_is_set:
        convert_float64_to_proto(ros_msg.maximum_acceleration, proto.maximum_acceleration)

def convert_proto_to_bosdyn_msgs_gaze_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec3_trajectory(proto.target_trajectory_in_frame1, ros_msg.target_trajectory_in_frame1)
    ros_msg.target_trajectory_in_frame1_is_set = proto.HasField("target_trajectory_in_frame1")
    ros_msg.frame1_name = proto.frame1_name
    convert_proto_to_bosdyn_msgs_se3_trajectory(proto.tool_trajectory_in_frame2, ros_msg.tool_trajectory_in_frame2)
    ros_msg.tool_trajectory_in_frame2_is_set = proto.HasField("tool_trajectory_in_frame2")
    ros_msg.frame2_name = proto.frame2_name
    convert_proto_to_geometry_msgs_pose(proto.wrist_tform_tool, ros_msg.wrist_tform_tool)
    ros_msg.wrist_tform_tool_is_set = proto.HasField("wrist_tform_tool")
    ros_msg.target_trajectory_initial_velocity = proto.target_trajectory_initial_velocity.value
    ros_msg.target_trajectory_initial_velocity_is_set = proto.HasField("target_trajectory_initial_velocity")
    ros_msg.maximum_acceleration = proto.maximum_acceleration.value
    ros_msg.maximum_acceleration_is_set = proto.HasField("maximum_acceleration")
    ros_msg.max_linear_velocity = proto.max_linear_velocity.value
    ros_msg.max_linear_velocity_is_set = proto.HasField("max_linear_velocity")
    ros_msg.max_angular_velocity = proto.max_angular_velocity.value
    ros_msg.max_angular_velocity_is_set = proto.HasField("max_angular_velocity")

def convert_bosdyn_msgs_gaze_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.target_trajectory_in_frame1_is_set:
        convert_bosdyn_msgs_vec3_trajectory_to_proto(ros_msg.target_trajectory_in_frame1, proto.target_trajectory_in_frame1)
    proto.frame1_name = ros_msg.frame1_name
    if ros_msg.tool_trajectory_in_frame2_is_set:
        convert_bosdyn_msgs_se3_trajectory_to_proto(ros_msg.tool_trajectory_in_frame2, proto.tool_trajectory_in_frame2)
    proto.frame2_name = ros_msg.frame2_name
    if ros_msg.wrist_tform_tool_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.wrist_tform_tool, proto.wrist_tform_tool)
    if ros_msg.target_trajectory_initial_velocity_is_set:
        convert_float64_to_proto(ros_msg.target_trajectory_initial_velocity, proto.target_trajectory_initial_velocity)
    if ros_msg.maximum_acceleration_is_set:
        convert_float64_to_proto(ros_msg.maximum_acceleration, proto.maximum_acceleration)
    if ros_msg.max_linear_velocity_is_set:
        convert_float64_to_proto(ros_msg.max_linear_velocity, proto.max_linear_velocity)
    if ros_msg.max_angular_velocity_is_set:
        convert_float64_to_proto(ros_msg.max_angular_velocity, proto.max_angular_velocity)

def convert_proto_to_bosdyn_msgs_gaze_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.gazing_at_target = proto.gazing_at_target
    ros_msg.gaze_to_target_rotation_measured = proto.gaze_to_target_rotation_measured
    ros_msg.hand_position_at_goal = proto.hand_position_at_goal
    ros_msg.hand_distance_to_goal_measured = proto.hand_distance_to_goal_measured
    ros_msg.hand_roll_at_goal = proto.hand_roll_at_goal
    ros_msg.hand_roll_to_target_roll_measured = proto.hand_roll_to_target_roll_measured

def convert_bosdyn_msgs_gaze_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.gazing_at_target = ros_msg.gazing_at_target
    proto.gaze_to_target_rotation_measured = ros_msg.gaze_to_target_rotation_measured
    proto.hand_position_at_goal = ros_msg.hand_position_at_goal
    proto.hand_distance_to_goal_measured = ros_msg.hand_distance_to_goal_measured
    proto.hand_roll_at_goal = ros_msg.hand_roll_at_goal
    proto.hand_roll_to_target_roll_measured = ros_msg.hand_roll_to_target_roll_measured

def convert_bosdyn_msgs_gaze_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_arm_stop_command_request_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_arm_stop_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_arm_stop_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_arm_impedance_command_request(proto, ros_msg):
    ros_msg.root_frame_name = proto.root_frame_name
    convert_proto_to_geometry_msgs_pose(proto.root_tform_task, ros_msg.root_tform_task)
    ros_msg.root_tform_task_is_set = proto.HasField("root_tform_task")
    convert_proto_to_geometry_msgs_pose(proto.wrist_tform_tool, ros_msg.wrist_tform_tool)
    ros_msg.wrist_tform_tool_is_set = proto.HasField("wrist_tform_tool")
    convert_proto_to_bosdyn_msgs_se3_trajectory(proto.task_tform_desired_tool, ros_msg.task_tform_desired_tool)
    ros_msg.task_tform_desired_tool_is_set = proto.HasField("task_tform_desired_tool")
    convert_proto_to_geometry_msgs_wrench(proto.feed_forward_wrench_at_tool_in_desired_tool, ros_msg.feed_forward_wrench_at_tool_in_desired_tool)
    ros_msg.feed_forward_wrench_at_tool_in_desired_tool_is_set = proto.HasField("feed_forward_wrench_at_tool_in_desired_tool")
    convert_proto_to_bosdyn_msgs_vector(proto.diagonal_stiffness_matrix, ros_msg.diagonal_stiffness_matrix)
    ros_msg.diagonal_stiffness_matrix_is_set = proto.HasField("diagonal_stiffness_matrix")
    convert_proto_to_bosdyn_msgs_vector(proto.diagonal_damping_matrix, ros_msg.diagonal_damping_matrix)
    ros_msg.diagonal_damping_matrix_is_set = proto.HasField("diagonal_damping_matrix")
    ros_msg.max_force_mag = proto.max_force_mag.value
    ros_msg.max_force_mag_is_set = proto.HasField("max_force_mag")
    ros_msg.max_torque_mag = proto.max_torque_mag.value
    ros_msg.max_torque_mag_is_set = proto.HasField("max_torque_mag")

def convert_bosdyn_msgs_arm_impedance_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.root_frame_name = ros_msg.root_frame_name
    if ros_msg.root_tform_task_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.root_tform_task, proto.root_tform_task)
    if ros_msg.wrist_tform_tool_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.wrist_tform_tool, proto.wrist_tform_tool)
    if ros_msg.task_tform_desired_tool_is_set:
        convert_bosdyn_msgs_se3_trajectory_to_proto(ros_msg.task_tform_desired_tool, proto.task_tform_desired_tool)
    if ros_msg.feed_forward_wrench_at_tool_in_desired_tool_is_set:
        convert_geometry_msgs_wrench_to_proto(ros_msg.feed_forward_wrench_at_tool_in_desired_tool, proto.feed_forward_wrench_at_tool_in_desired_tool)
    if ros_msg.diagonal_stiffness_matrix_is_set:
        convert_bosdyn_msgs_vector_to_proto(ros_msg.diagonal_stiffness_matrix, proto.diagonal_stiffness_matrix)
    if ros_msg.diagonal_damping_matrix_is_set:
        convert_bosdyn_msgs_vector_to_proto(ros_msg.diagonal_damping_matrix, proto.diagonal_damping_matrix)
    if ros_msg.max_force_mag_is_set:
        convert_float64_to_proto(ros_msg.max_force_mag, proto.max_force_mag)
    if ros_msg.max_torque_mag_is_set:
        convert_float64_to_proto(ros_msg.max_torque_mag, proto.max_torque_mag)

def convert_bosdyn_msgs_arm_impedance_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_arm_impedance_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_arm_surface_contact_command(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    convert_proto_to_bosdyn_msgs_arm_surface_contact_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")

def convert_bosdyn_msgs_arm_surface_contact_command_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_arm_surface_contact_request_to_proto(ros_msg.request, proto.request)

def convert_proto_to_bosdyn_msgs_arm_surface_contact_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_arm_surface_contact_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_record_text_messages_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import TextMessage
    ros_msg.text_messages = []
    for _item in proto.text_messages:
        ros_msg.text_messages.append(TextMessage())
        convert_proto_to_bosdyn_msgs_text_message(_item, ros_msg.text_messages[-1])

def convert_bosdyn_msgs_record_text_messages_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.text_messages[:]
    for _item in ros_msg.text_messages:
        convert_bosdyn_msgs_text_message_to_proto(_item, proto.text_messages.add())

def convert_proto_to_bosdyn_msgs_record_operator_comments_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import OperatorComment
    ros_msg.operator_comments = []
    for _item in proto.operator_comments:
        ros_msg.operator_comments.append(OperatorComment())
        convert_proto_to_bosdyn_msgs_operator_comment(_item, ros_msg.operator_comments[-1])

def convert_bosdyn_msgs_record_operator_comments_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.operator_comments[:]
    for _item in ros_msg.operator_comments:
        convert_bosdyn_msgs_operator_comment_to_proto(_item, proto.operator_comments.add())

def convert_proto_to_bosdyn_msgs_record_data_blobs_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import DataBlob
    ros_msg.blob_data = []
    for _item in proto.blob_data:
        ros_msg.blob_data.append(DataBlob())
        convert_proto_to_bosdyn_msgs_data_blob(_item, ros_msg.blob_data[-1])
    ros_msg.sync = proto.sync

def convert_bosdyn_msgs_record_data_blobs_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.blob_data[:]
    for _item in ros_msg.blob_data:
        convert_bosdyn_msgs_data_blob_to_proto(_item, proto.blob_data.add())
    proto.sync = ros_msg.sync

def convert_proto_to_bosdyn_msgs_record_signal_ticks_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import SignalTick
    ros_msg.tick_data = []
    for _item in proto.tick_data:
        ros_msg.tick_data.append(SignalTick())
        convert_proto_to_bosdyn_msgs_signal_tick(_item, ros_msg.tick_data[-1])

def convert_bosdyn_msgs_record_signal_ticks_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.tick_data[:]
    for _item in ros_msg.tick_data:
        convert_bosdyn_msgs_signal_tick_to_proto(_item, proto.tick_data.add())

def convert_proto_to_bosdyn_msgs_record_events_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Event
    ros_msg.events = []
    for _item in proto.events:
        ros_msg.events.append(Event())
        convert_proto_to_bosdyn_msgs_event(_item, ros_msg.events[-1])

def convert_bosdyn_msgs_record_events_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.events[:]
    for _item in ros_msg.events:
        convert_bosdyn_msgs_event_to_proto(_item, proto.events.add())

def convert_proto_to_bosdyn_msgs_text_message(proto, ros_msg):
    ros_msg.message = proto.message
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.source = proto.source
    ros_msg.level.value = proto.level
    ros_msg.tag = proto.tag
    ros_msg.filename = proto.filename
    ros_msg.line_number = proto.line_number

def convert_bosdyn_msgs_text_message_to_proto(ros_msg, proto):
    proto.Clear()
    proto.message = ros_msg.message
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.source = ros_msg.source
    proto.level = ros_msg.level.value
    proto.tag = ros_msg.tag
    proto.filename = ros_msg.filename
    proto.line_number = ros_msg.line_number

def convert_proto_to_bosdyn_msgs_operator_comment(proto, ros_msg):
    ros_msg.message = proto.message
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")

def convert_bosdyn_msgs_operator_comment_to_proto(ros_msg, proto):
    proto.Clear()
    proto.message = ros_msg.message
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)

def convert_proto_to_bosdyn_msgs_data_blob(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.channel = proto.channel
    ros_msg.type_id = proto.type_id
    ros_msg.data = proto.data

def convert_bosdyn_msgs_data_blob_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.channel = ros_msg.channel
    proto.type_id = ros_msg.type_id
    proto.data = ros_msg.data

def convert_proto_to_bosdyn_msgs_signal_schema_variable(proto, ros_msg):
    ros_msg.name = proto.name
    ros_msg.type.value = proto.type
    ros_msg.is_time = proto.is_time

def convert_bosdyn_msgs_signal_schema_variable_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    proto.type = ros_msg.type.value
    proto.is_time = ros_msg.is_time

def convert_proto_to_bosdyn_msgs_signal_schema(proto, ros_msg):
    from bosdyn_msgs.msg import Variable
    ros_msg.vars = []
    for _item in proto.vars:
        ros_msg.vars.append(Variable())
        convert_proto_to_bosdyn_msgs_signal_schema_variable(_item, ros_msg.vars[-1])
    ros_msg.schema_name = proto.schema_name

def convert_bosdyn_msgs_signal_schema_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.vars[:]
    for _item in ros_msg.vars:
        convert_bosdyn_msgs_signal_schema_variable_to_proto(_item, proto.vars.add())
    proto.schema_name = ros_msg.schema_name

def convert_proto_to_bosdyn_msgs_signal_schema_id(proto, ros_msg):
    ros_msg.schema_id = proto.schema_id
    convert_proto_to_bosdyn_msgs_signal_schema(proto.schema, ros_msg.schema)
    ros_msg.schema_is_set = proto.HasField("schema")

def convert_bosdyn_msgs_signal_schema_id_to_proto(ros_msg, proto):
    proto.Clear()
    proto.schema_id = ros_msg.schema_id
    if ros_msg.schema_is_set:
        convert_bosdyn_msgs_signal_schema_to_proto(ros_msg.schema, proto.schema)

def convert_proto_to_bosdyn_msgs_signal_tick(proto, ros_msg):
    ros_msg.sequence_id = proto.sequence_id
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.source = proto.source
    ros_msg.schema_id = proto.schema_id
    ros_msg.encoding.value = proto.encoding
    ros_msg.data = proto.data

def convert_bosdyn_msgs_signal_tick_to_proto(ros_msg, proto):
    proto.Clear()
    proto.sequence_id = ros_msg.sequence_id
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.source = ros_msg.source
    proto.schema_id = ros_msg.schema_id
    proto.encoding = ros_msg.encoding.value
    proto.data = ros_msg.data

def convert_proto_to_bosdyn_msgs_event(proto, ros_msg):
    ros_msg.type = proto.type
    ros_msg.description = proto.description
    ros_msg.source = proto.source
    ros_msg.id = proto.id
    convert_proto_to_builtin_interfaces_time(proto.start_time, ros_msg.start_time)
    ros_msg.start_time_is_set = proto.HasField("start_time")
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.level.value = proto.level
    from bosdyn_msgs.msg import Parameter
    ros_msg.parameters = []
    for _item in proto.parameters:
        ros_msg.parameters.append(Parameter())
        convert_proto_to_bosdyn_msgs_parameter(_item, ros_msg.parameters[-1])
    ros_msg.log_preserve_hint.value = proto.log_preserve_hint

def convert_bosdyn_msgs_event_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type
    proto.description = ros_msg.description
    proto.source = ros_msg.source
    proto.id = ros_msg.id
    if ros_msg.start_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.start_time, proto.start_time)
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    proto.level = ros_msg.level.value
    del proto.parameters[:]
    for _item in ros_msg.parameters:
        convert_bosdyn_msgs_parameter_to_proto(_item, proto.parameters.add())
    proto.log_preserve_hint = ros_msg.log_preserve_hint.value

def convert_proto_to_bosdyn_msgs_record_text_messages_response_error(proto, ros_msg):
    ros_msg.type.value = proto.type
    ros_msg.message = proto.message
    ros_msg.index = proto.index

def convert_bosdyn_msgs_record_text_messages_response_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    proto.message = ros_msg.message
    proto.index = ros_msg.index

def convert_proto_to_bosdyn_msgs_record_text_messages_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Error
    ros_msg.errors = []
    for _item in proto.errors:
        ros_msg.errors.append(Error())
        convert_proto_to_bosdyn_msgs_record_text_messages_response_error(_item, ros_msg.errors[-1])

def convert_bosdyn_msgs_record_text_messages_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.errors[:]
    for _item in ros_msg.errors:
        convert_bosdyn_msgs_record_text_messages_response_error_to_proto(_item, proto.errors.add())

def convert_proto_to_bosdyn_msgs_record_operator_comments_response_error(proto, ros_msg):
    ros_msg.type.value = proto.type
    ros_msg.message = proto.message
    ros_msg.index = proto.index

def convert_bosdyn_msgs_record_operator_comments_response_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    proto.message = ros_msg.message
    proto.index = ros_msg.index

def convert_proto_to_bosdyn_msgs_record_operator_comments_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Error
    ros_msg.errors = []
    for _item in proto.errors:
        ros_msg.errors.append(Error())
        convert_proto_to_bosdyn_msgs_record_operator_comments_response_error(_item, ros_msg.errors[-1])

def convert_bosdyn_msgs_record_operator_comments_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.errors[:]
    for _item in ros_msg.errors:
        convert_bosdyn_msgs_record_operator_comments_response_error_to_proto(_item, proto.errors.add())

def convert_proto_to_bosdyn_msgs_record_data_blobs_response_error(proto, ros_msg):
    ros_msg.type.value = proto.type
    ros_msg.message = proto.message
    ros_msg.index = proto.index

def convert_bosdyn_msgs_record_data_blobs_response_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    proto.message = ros_msg.message
    proto.index = ros_msg.index

def convert_proto_to_bosdyn_msgs_record_data_blobs_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Error
    ros_msg.errors = []
    for _item in proto.errors:
        ros_msg.errors.append(Error())
        convert_proto_to_bosdyn_msgs_record_data_blobs_response_error(_item, ros_msg.errors[-1])

def convert_bosdyn_msgs_record_data_blobs_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.errors[:]
    for _item in ros_msg.errors:
        convert_bosdyn_msgs_record_data_blobs_response_error_to_proto(_item, proto.errors.add())

def convert_proto_to_bosdyn_msgs_record_signal_ticks_response_error(proto, ros_msg):
    ros_msg.type.value = proto.type
    ros_msg.message = proto.message
    ros_msg.index = proto.index

def convert_bosdyn_msgs_record_signal_ticks_response_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    proto.message = ros_msg.message
    proto.index = ros_msg.index

def convert_proto_to_bosdyn_msgs_record_signal_ticks_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Error
    ros_msg.errors = []
    for _item in proto.errors:
        ros_msg.errors.append(Error())
        convert_proto_to_bosdyn_msgs_record_signal_ticks_response_error(_item, ros_msg.errors[-1])

def convert_bosdyn_msgs_record_signal_ticks_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.errors[:]
    for _item in ros_msg.errors:
        convert_bosdyn_msgs_record_signal_ticks_response_error_to_proto(_item, proto.errors.add())

def convert_proto_to_bosdyn_msgs_record_events_response_error(proto, ros_msg):
    ros_msg.type.value = proto.type
    ros_msg.message = proto.message
    ros_msg.index = proto.index

def convert_bosdyn_msgs_record_events_response_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    proto.message = ros_msg.message
    proto.index = ros_msg.index

def convert_proto_to_bosdyn_msgs_record_events_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Error
    ros_msg.errors = []
    for _item in proto.errors:
        ros_msg.errors.append(Error())
        convert_proto_to_bosdyn_msgs_record_events_response_error(_item, ros_msg.errors[-1])

def convert_bosdyn_msgs_record_events_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.errors[:]
    for _item in ros_msg.errors:
        convert_bosdyn_msgs_record_events_response_error_to_proto(_item, proto.errors.add())

def convert_proto_to_bosdyn_msgs_register_signal_schema_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_signal_schema(proto.schema, ros_msg.schema)
    ros_msg.schema_is_set = proto.HasField("schema")

def convert_bosdyn_msgs_register_signal_schema_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.schema_is_set:
        convert_bosdyn_msgs_signal_schema_to_proto(ros_msg.schema, proto.schema)

def convert_proto_to_bosdyn_msgs_register_signal_schema_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.schema_id = proto.schema_id

def convert_bosdyn_msgs_register_signal_schema_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.schema_id = ros_msg.schema_id

def convert_proto_to_bosdyn_msgs_power_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    ros_msg.request.value = proto.request

def convert_bosdyn_msgs_power_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    proto.request = ros_msg.request.value

def convert_proto_to_bosdyn_msgs_power_command_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    ros_msg.power_command_id = proto.power_command_id
    ros_msg.license_status.value = proto.license_status
    from bosdyn_msgs.msg import SystemFault
    ros_msg.blocking_faults = []
    for _item in proto.blocking_faults:
        ros_msg.blocking_faults.append(SystemFault())
        convert_proto_to_bosdyn_msgs_system_fault(_item, ros_msg.blocking_faults[-1])

def convert_bosdyn_msgs_power_command_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    proto.power_command_id = ros_msg.power_command_id
    proto.license_status = ros_msg.license_status.value
    del proto.blocking_faults[:]
    for _item in ros_msg.blocking_faults:
        convert_bosdyn_msgs_system_fault_to_proto(_item, proto.blocking_faults.add())

def convert_proto_to_bosdyn_msgs_power_command_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.power_command_id = proto.power_command_id

def convert_bosdyn_msgs_power_command_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.power_command_id = ros_msg.power_command_id

def convert_proto_to_bosdyn_msgs_power_command_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    from bosdyn_msgs.msg import SystemFault
    ros_msg.blocking_faults = []
    for _item in proto.blocking_faults:
        ros_msg.blocking_faults.append(SystemFault())
        convert_proto_to_bosdyn_msgs_system_fault(_item, ros_msg.blocking_faults[-1])

def convert_bosdyn_msgs_power_command_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    del proto.blocking_faults[:]
    for _item in ros_msg.blocking_faults:
        convert_bosdyn_msgs_system_fault_to_proto(_item, proto.blocking_faults.add())

def convert_proto_to_bosdyn_msgs_fan_power_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    ros_msg.percent_power = proto.percent_power
    convert_proto_to_builtin_interfaces_duration(proto.duration, ros_msg.duration)
    ros_msg.duration_is_set = proto.HasField("duration")

def convert_bosdyn_msgs_fan_power_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    proto.percent_power = ros_msg.percent_power
    if ros_msg.duration_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.duration, proto.duration)

def convert_proto_to_bosdyn_msgs_fan_power_command_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    convert_proto_to_builtin_interfaces_time(proto.desired_end_time, ros_msg.desired_end_time)
    ros_msg.desired_end_time_is_set = proto.HasField("desired_end_time")
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_fan_power_command_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    if ros_msg.desired_end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.desired_end_time, proto.desired_end_time)
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_fan_power_command_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_fan_power_command_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_fan_power_command_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_builtin_interfaces_time(proto.desired_end_time, ros_msg.desired_end_time)
    ros_msg.desired_end_time_is_set = proto.HasField("desired_end_time")
    convert_proto_to_builtin_interfaces_time(proto.early_stop_time, ros_msg.early_stop_time)
    ros_msg.early_stop_time_is_set = proto.HasField("early_stop_time")

def convert_bosdyn_msgs_fan_power_command_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.desired_end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.desired_end_time, proto.desired_end_time)
    if ros_msg.early_stop_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.early_stop_time, proto.early_stop_time)

def convert_proto_to_bosdyn_msgs_ir_enable_disable_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.request.value = proto.request

def convert_bosdyn_msgs_ir_enable_disable_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.request = ros_msg.request.value

def convert_proto_to_bosdyn_msgs_ir_enable_disable_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_ir_enable_disable_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_time_range(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.start, ros_msg.start)
    ros_msg.start_is_set = proto.HasField("start")
    convert_proto_to_builtin_interfaces_time(proto.end, ros_msg.end)
    ros_msg.end_is_set = proto.HasField("end")

def convert_bosdyn_msgs_time_range_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.start_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.start, proto.start)
    if ros_msg.end_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end, proto.end)

def convert_proto_to_bosdyn_msgs_gripper_command_request_one_of_command(proto, ros_msg):
    if proto.HasField("claw_gripper_command"):
        ros_msg.command_choice = ros_msg.COMMAND_CLAW_GRIPPER_COMMAND_SET
        convert_proto_to_bosdyn_msgs_claw_gripper_command_request(proto.claw_gripper_command, ros_msg.claw_gripper_command)

def convert_bosdyn_msgs_gripper_command_request_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_CLAW_GRIPPER_COMMAND_SET:
        convert_bosdyn_msgs_claw_gripper_command_request_to_proto(ros_msg.claw_gripper_command, proto.claw_gripper_command)

def convert_proto_to_bosdyn_msgs_gripper_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_gripper_command_request_one_of_command(proto, ros_msg.command)

def convert_bosdyn_msgs_gripper_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_gripper_command_request_one_of_command_to_proto(ros_msg.command, proto)

def convert_proto_to_bosdyn_msgs_gripper_command_feedback_one_of_command(proto, ros_msg):
    if proto.HasField("claw_gripper_feedback"):
        ros_msg.command_choice = ros_msg.COMMAND_CLAW_GRIPPER_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_claw_gripper_command_feedback(proto.claw_gripper_feedback, ros_msg.claw_gripper_feedback)

def convert_bosdyn_msgs_gripper_command_feedback_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_CLAW_GRIPPER_FEEDBACK_SET:
        convert_bosdyn_msgs_claw_gripper_command_feedback_to_proto(ros_msg.claw_gripper_feedback, proto.claw_gripper_feedback)

def convert_proto_to_bosdyn_msgs_gripper_command_feedback(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_gripper_command_feedback_one_of_command(proto, ros_msg.command)
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_gripper_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_gripper_command_feedback_one_of_command_to_proto(ros_msg.command, proto)
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_gripper_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_claw_gripper_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_scalar_trajectory(proto.trajectory, ros_msg.trajectory)
    ros_msg.trajectory_is_set = proto.HasField("trajectory")
    ros_msg.maximum_open_close_velocity = proto.maximum_open_close_velocity.value
    ros_msg.maximum_open_close_velocity_is_set = proto.HasField("maximum_open_close_velocity")
    ros_msg.maximum_open_close_acceleration = proto.maximum_open_close_acceleration.value
    ros_msg.maximum_open_close_acceleration_is_set = proto.HasField("maximum_open_close_acceleration")
    ros_msg.maximum_torque = proto.maximum_torque.value
    ros_msg.maximum_torque_is_set = proto.HasField("maximum_torque")
    ros_msg.disable_force_on_contact = proto.disable_force_on_contact

def convert_bosdyn_msgs_claw_gripper_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.trajectory_is_set:
        convert_bosdyn_msgs_scalar_trajectory_to_proto(ros_msg.trajectory, proto.trajectory)
    if ros_msg.maximum_open_close_velocity_is_set:
        convert_float64_to_proto(ros_msg.maximum_open_close_velocity, proto.maximum_open_close_velocity)
    if ros_msg.maximum_open_close_acceleration_is_set:
        convert_float64_to_proto(ros_msg.maximum_open_close_acceleration, proto.maximum_open_close_acceleration)
    if ros_msg.maximum_torque_is_set:
        convert_float64_to_proto(ros_msg.maximum_torque, proto.maximum_torque)
    proto.disable_force_on_contact = ros_msg.disable_force_on_contact

def convert_proto_to_bosdyn_msgs_claw_gripper_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_claw_gripper_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_claw_gripper_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_service_fault_id(proto, ros_msg):
    ros_msg.fault_name = proto.fault_name
    ros_msg.service_name = proto.service_name
    ros_msg.payload_guid = proto.payload_guid

def convert_bosdyn_msgs_service_fault_id_to_proto(ros_msg, proto):
    proto.Clear()
    proto.fault_name = ros_msg.fault_name
    proto.service_name = ros_msg.service_name
    proto.payload_guid = ros_msg.payload_guid

def convert_proto_to_bosdyn_msgs_service_fault(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_service_fault_id(proto.fault_id, ros_msg.fault_id)
    ros_msg.fault_id_is_set = proto.HasField("fault_id")
    ros_msg.error_message = proto.error_message
    ros_msg.attributes = []
    for _item in proto.attributes:
        ros_msg.attributes.append(_item)
    ros_msg.severity.value = proto.severity
    convert_proto_to_builtin_interfaces_time(proto.onset_timestamp, ros_msg.onset_timestamp)
    ros_msg.onset_timestamp_is_set = proto.HasField("onset_timestamp")
    convert_proto_to_builtin_interfaces_duration(proto.duration, ros_msg.duration)
    ros_msg.duration_is_set = proto.HasField("duration")

def convert_bosdyn_msgs_service_fault_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.fault_id_is_set:
        convert_bosdyn_msgs_service_fault_id_to_proto(ros_msg.fault_id, proto.fault_id)
    proto.error_message = ros_msg.error_message
    del proto.attributes[:]
    for _item in ros_msg.attributes:
        proto.attributes.add(_item)
    proto.severity = ros_msg.severity.value
    if ros_msg.onset_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.onset_timestamp, proto.onset_timestamp)
    if ros_msg.duration_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.duration, proto.duration)

def convert_proto_to_bosdyn_msgs_trigger_service_fault_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_service_fault(proto.fault, ros_msg.fault)
    ros_msg.fault_is_set = proto.HasField("fault")

def convert_bosdyn_msgs_trigger_service_fault_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.fault_is_set:
        convert_bosdyn_msgs_service_fault_to_proto(ros_msg.fault, proto.fault)

def convert_proto_to_bosdyn_msgs_trigger_service_fault_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_trigger_service_fault_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_clear_service_fault_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_service_fault_id(proto.fault_id, ros_msg.fault_id)
    ros_msg.fault_id_is_set = proto.HasField("fault_id")
    ros_msg.clear_all_service_faults = proto.clear_all_service_faults
    ros_msg.clear_all_payload_faults = proto.clear_all_payload_faults

def convert_bosdyn_msgs_clear_service_fault_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.fault_id_is_set:
        convert_bosdyn_msgs_service_fault_id_to_proto(ros_msg.fault_id, proto.fault_id)
    proto.clear_all_service_faults = ros_msg.clear_all_service_faults
    proto.clear_all_payload_faults = ros_msg.clear_all_payload_faults

def convert_proto_to_bosdyn_msgs_clear_service_fault_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_clear_service_fault_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_lease(proto, ros_msg):
    ros_msg.resource = proto.resource
    ros_msg.epoch = proto.epoch
    ros_msg.sequence = []
    for _item in proto.sequence:
        ros_msg.sequence.append(_item)
    ros_msg.client_names = []
    for _item in proto.client_names:
        ros_msg.client_names.append(_item)

def convert_bosdyn_msgs_lease_to_proto(ros_msg, proto):
    proto.Clear()
    proto.resource = ros_msg.resource
    proto.epoch = ros_msg.epoch
    del proto.sequence[:]
    for _item in ros_msg.sequence:
        proto.sequence.add(_item)
    del proto.client_names[:]
    for _item in ros_msg.client_names:
        proto.client_names.add(_item)

def convert_proto_to_bosdyn_msgs_resource_tree(proto, ros_msg):
    ros_msg.resource = proto.resource
    from bosdyn_msgs.msg import SerializedMessage
    ros_msg.sub_resources = []
    for _item in proto.sub_resources:
        ros_msg.sub_resources.append(SerializedMessage())
        convert_proto_to_serialized_bosdyn_msgs_resource_tree(_item, ros_msg.sub_resources[-1])

def convert_bosdyn_msgs_resource_tree_to_proto(ros_msg, proto):
    proto.Clear()
    proto.resource = ros_msg.resource
    del proto.sub_resources[:]
    for _item in ros_msg.sub_resources:
        convert_serialized_bosdyn_msgs_resource_tree_to_proto(_item, proto.sub_resources.add())

def convert_proto_to_bosdyn_msgs_lease_owner(proto, ros_msg):
    ros_msg.client_name = proto.client_name
    ros_msg.user_name = proto.user_name

def convert_bosdyn_msgs_lease_owner_to_proto(ros_msg, proto):
    proto.Clear()
    proto.client_name = ros_msg.client_name
    proto.user_name = ros_msg.user_name

def convert_proto_to_bosdyn_msgs_lease_use_result(proto, ros_msg):
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_lease_owner(proto.owner, ros_msg.owner)
    ros_msg.owner_is_set = proto.HasField("owner")
    convert_proto_to_bosdyn_msgs_lease(proto.attempted_lease, ros_msg.attempted_lease)
    ros_msg.attempted_lease_is_set = proto.HasField("attempted_lease")
    convert_proto_to_bosdyn_msgs_lease(proto.previous_lease, ros_msg.previous_lease)
    ros_msg.previous_lease_is_set = proto.HasField("previous_lease")
    convert_proto_to_bosdyn_msgs_lease(proto.latest_known_lease, ros_msg.latest_known_lease)
    ros_msg.latest_known_lease_is_set = proto.HasField("latest_known_lease")
    from bosdyn_msgs.msg import Lease
    ros_msg.latest_resources = []
    for _item in proto.latest_resources:
        ros_msg.latest_resources.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.latest_resources[-1])

def convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    if ros_msg.owner_is_set:
        convert_bosdyn_msgs_lease_owner_to_proto(ros_msg.owner, proto.owner)
    if ros_msg.attempted_lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.attempted_lease, proto.attempted_lease)
    if ros_msg.previous_lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.previous_lease, proto.previous_lease)
    if ros_msg.latest_known_lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.latest_known_lease, proto.latest_known_lease)
    del proto.latest_resources[:]
    for _item in ros_msg.latest_resources:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.latest_resources.add())

def convert_proto_to_bosdyn_msgs_acquire_lease_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.resource = proto.resource

def convert_bosdyn_msgs_acquire_lease_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.resource = ros_msg.resource

def convert_proto_to_bosdyn_msgs_acquire_lease_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    convert_proto_to_bosdyn_msgs_lease_owner(proto.lease_owner, ros_msg.lease_owner)
    ros_msg.lease_owner_is_set = proto.HasField("lease_owner")

def convert_bosdyn_msgs_acquire_lease_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    if ros_msg.lease_owner_is_set:
        convert_bosdyn_msgs_lease_owner_to_proto(ros_msg.lease_owner, proto.lease_owner)

def convert_proto_to_bosdyn_msgs_take_lease_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.resource = proto.resource

def convert_bosdyn_msgs_take_lease_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.resource = ros_msg.resource

def convert_proto_to_bosdyn_msgs_take_lease_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    convert_proto_to_bosdyn_msgs_lease_owner(proto.lease_owner, ros_msg.lease_owner)
    ros_msg.lease_owner_is_set = proto.HasField("lease_owner")

def convert_bosdyn_msgs_take_lease_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    if ros_msg.lease_owner_is_set:
        convert_bosdyn_msgs_lease_owner_to_proto(ros_msg.lease_owner, proto.lease_owner)

def convert_proto_to_bosdyn_msgs_return_lease_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_return_lease_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_return_lease_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_return_lease_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_list_leases_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.include_full_lease_info = proto.include_full_lease_info

def convert_bosdyn_msgs_list_leases_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.include_full_lease_info = ros_msg.include_full_lease_info

def convert_proto_to_bosdyn_msgs_lease_resource(proto, ros_msg):
    ros_msg.resource = proto.resource
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    convert_proto_to_bosdyn_msgs_lease_owner(proto.lease_owner, ros_msg.lease_owner)
    ros_msg.lease_owner_is_set = proto.HasField("lease_owner")
    convert_proto_to_builtin_interfaces_time(proto.stale_time, ros_msg.stale_time)
    ros_msg.stale_time_is_set = proto.HasField("stale_time")

def convert_bosdyn_msgs_lease_resource_to_proto(ros_msg, proto):
    proto.Clear()
    proto.resource = ros_msg.resource
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    if ros_msg.lease_owner_is_set:
        convert_bosdyn_msgs_lease_owner_to_proto(ros_msg.lease_owner, proto.lease_owner)
    if ros_msg.stale_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.stale_time, proto.stale_time)

def convert_proto_to_bosdyn_msgs_list_leases_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import LeaseResource
    ros_msg.resources = []
    for _item in proto.resources:
        ros_msg.resources.append(LeaseResource())
        convert_proto_to_bosdyn_msgs_lease_resource(_item, ros_msg.resources[-1])
    convert_proto_to_bosdyn_msgs_resource_tree(proto.resource_tree, ros_msg.resource_tree)
    ros_msg.resource_tree_is_set = proto.HasField("resource_tree")

def convert_bosdyn_msgs_list_leases_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.resources[:]
    for _item in ros_msg.resources:
        convert_bosdyn_msgs_lease_resource_to_proto(_item, proto.resources.add())
    if ros_msg.resource_tree_is_set:
        convert_bosdyn_msgs_resource_tree_to_proto(ros_msg.resource_tree, proto.resource_tree)

def convert_proto_to_bosdyn_msgs_retain_lease_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_retain_lease_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_retain_lease_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_retain_lease_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_local_grid_type(proto, ros_msg):
    ros_msg.name = proto.name

def convert_bosdyn_msgs_local_grid_type_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name

def convert_proto_to_bosdyn_msgs_local_grid_request(proto, ros_msg):
    ros_msg.local_grid_type_name = proto.local_grid_type_name

def convert_bosdyn_msgs_local_grid_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.local_grid_type_name = ros_msg.local_grid_type_name

def convert_proto_to_bosdyn_msgs_local_grid_extent(proto, ros_msg):
    ros_msg.cell_size = proto.cell_size
    ros_msg.num_cells_x = proto.num_cells_x
    ros_msg.num_cells_y = proto.num_cells_y

def convert_bosdyn_msgs_local_grid_extent_to_proto(ros_msg, proto):
    proto.Clear()
    proto.cell_size = ros_msg.cell_size
    proto.num_cells_x = ros_msg.num_cells_x
    proto.num_cells_y = ros_msg.num_cells_y

def convert_proto_to_bosdyn_msgs_local_grid(proto, ros_msg):
    ros_msg.local_grid_type_name = proto.local_grid_type_name
    convert_proto_to_builtin_interfaces_time(proto.acquisition_time, ros_msg.acquisition_time)
    ros_msg.acquisition_time_is_set = proto.HasField("acquisition_time")
    ros_msg.frame_name_local_grid_data = proto.frame_name_local_grid_data
    convert_proto_to_bosdyn_msgs_local_grid_extent(proto.extent, ros_msg.extent)
    ros_msg.extent_is_set = proto.HasField("extent")
    ros_msg.cell_format.value = proto.cell_format
    ros_msg.encoding.value = proto.encoding
    ros_msg.data = proto.data
    ros_msg.rle_counts = []
    for _item in proto.rle_counts:
        ros_msg.rle_counts.append(_item)
    ros_msg.cell_value_scale = proto.cell_value_scale
    ros_msg.cell_value_offset = proto.cell_value_offset

def convert_bosdyn_msgs_local_grid_to_proto(ros_msg, proto):
    proto.Clear()
    proto.local_grid_type_name = ros_msg.local_grid_type_name
    if ros_msg.acquisition_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.acquisition_time, proto.acquisition_time)
    proto.frame_name_local_grid_data = ros_msg.frame_name_local_grid_data
    if ros_msg.extent_is_set:
        convert_bosdyn_msgs_local_grid_extent_to_proto(ros_msg.extent, proto.extent)
    proto.cell_format = ros_msg.cell_format.value
    proto.encoding = ros_msg.encoding.value
    proto.data = ros_msg.data
    del proto.rle_counts[:]
    for _item in ros_msg.rle_counts:
        proto.rle_counts.add(_item)
    proto.cell_value_scale = ros_msg.cell_value_scale
    proto.cell_value_offset = ros_msg.cell_value_offset

def convert_proto_to_bosdyn_msgs_local_grid_response(proto, ros_msg):
    ros_msg.local_grid_type_name = proto.local_grid_type_name
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_local_grid(proto.local_grid, ros_msg.local_grid)
    ros_msg.local_grid_is_set = proto.HasField("local_grid")

def convert_bosdyn_msgs_local_grid_response_to_proto(ros_msg, proto):
    proto.Clear()
    proto.local_grid_type_name = ros_msg.local_grid_type_name
    proto.status = ros_msg.status.value
    if ros_msg.local_grid_is_set:
        convert_bosdyn_msgs_local_grid_to_proto(ros_msg.local_grid, proto.local_grid)

def convert_proto_to_bosdyn_msgs_get_local_grid_types_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_local_grid_types_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_local_grid_types_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import LocalGridType
    ros_msg.local_grid_type = []
    for _item in proto.local_grid_type:
        ros_msg.local_grid_type.append(LocalGridType())
        convert_proto_to_bosdyn_msgs_local_grid_type(_item, ros_msg.local_grid_type[-1])

def convert_bosdyn_msgs_get_local_grid_types_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.local_grid_type[:]
    for _item in ros_msg.local_grid_type:
        convert_bosdyn_msgs_local_grid_type_to_proto(_item, proto.local_grid_type.add())

def convert_proto_to_bosdyn_msgs_get_local_grids_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import LocalGridRequest
    ros_msg.local_grid_requests = []
    for _item in proto.local_grid_requests:
        ros_msg.local_grid_requests.append(LocalGridRequest())
        convert_proto_to_bosdyn_msgs_local_grid_request(_item, ros_msg.local_grid_requests[-1])

def convert_bosdyn_msgs_get_local_grids_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.local_grid_requests[:]
    for _item in ros_msg.local_grid_requests:
        convert_bosdyn_msgs_local_grid_request_to_proto(_item, proto.local_grid_requests.add())

def convert_proto_to_bosdyn_msgs_get_local_grids_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import LocalGridResponse
    ros_msg.local_grid_responses = []
    for _item in proto.local_grid_responses:
        ros_msg.local_grid_responses.append(LocalGridResponse())
        convert_proto_to_bosdyn_msgs_local_grid_response(_item, ros_msg.local_grid_responses[-1])
    ros_msg.num_local_grid_errors = proto.num_local_grid_errors

def convert_bosdyn_msgs_get_local_grids_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.local_grid_responses[:]
    for _item in ros_msg.local_grid_responses:
        convert_bosdyn_msgs_local_grid_response_to_proto(_item, proto.local_grid_responses.add())
    proto.num_local_grid_errors = ros_msg.num_local_grid_errors

def convert_proto_to_bosdyn_msgs_parameter_one_of_values(proto, ros_msg):
    if proto.HasField("int_value"):
        ros_msg.values_choice = ros_msg.VALUES_INT_VALUE_SET
        ros_msg.int_value = proto.int_value
    if proto.HasField("float_value"):
        ros_msg.values_choice = ros_msg.VALUES_FLOAT_VALUE_SET
        ros_msg.float_value = proto.float_value
    if proto.HasField("timestamp"):
        ros_msg.values_choice = ros_msg.VALUES_TIMESTAMP_SET
        convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    if proto.HasField("duration"):
        ros_msg.values_choice = ros_msg.VALUES_DURATION_SET
        convert_proto_to_builtin_interfaces_duration(proto.duration, ros_msg.duration)
    if proto.HasField("string_value"):
        ros_msg.values_choice = ros_msg.VALUES_STRING_VALUE_SET
        ros_msg.string_value = proto.string_value
    if proto.HasField("bool_value"):
        ros_msg.values_choice = ros_msg.VALUES_BOOL_VALUE_SET
        ros_msg.bool_value = proto.bool_value
    if proto.HasField("uint_value"):
        ros_msg.values_choice = ros_msg.VALUES_UINT_VALUE_SET
        ros_msg.uint_value = proto.uint_value

def convert_bosdyn_msgs_parameter_one_of_values_to_proto(ros_msg, proto):
    proto.ClearField("values")
    if ros_msg.values_choice == ros_msg.VALUES_INT_VALUE_SET:
        proto.int_value = ros_msg.int_value
    if ros_msg.values_choice == ros_msg.VALUES_FLOAT_VALUE_SET:
        proto.float_value = ros_msg.float_value
    if ros_msg.values_choice == ros_msg.VALUES_TIMESTAMP_SET:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    if ros_msg.values_choice == ros_msg.VALUES_DURATION_SET:
        convert_builtin_interfaces_duration_to_proto(ros_msg.duration, proto.duration)
    if ros_msg.values_choice == ros_msg.VALUES_STRING_VALUE_SET:
        proto.string_value = ros_msg.string_value
    if ros_msg.values_choice == ros_msg.VALUES_BOOL_VALUE_SET:
        proto.bool_value = ros_msg.bool_value
    if ros_msg.values_choice == ros_msg.VALUES_UINT_VALUE_SET:
        proto.uint_value = ros_msg.uint_value

def convert_proto_to_bosdyn_msgs_parameter(proto, ros_msg):
    ros_msg.label = proto.label
    ros_msg.units = proto.units
    convert_proto_to_bosdyn_msgs_parameter_one_of_values(proto, ros_msg.values)
    ros_msg.notes = proto.notes

def convert_bosdyn_msgs_parameter_to_proto(ros_msg, proto):
    proto.Clear()
    proto.label = ros_msg.label
    proto.units = ros_msg.units
    convert_bosdyn_msgs_parameter_one_of_values_to_proto(ros_msg.values, proto)
    proto.notes = ros_msg.notes

def convert_proto_to_bosdyn_msgs_data_chunk(proto, ros_msg):
    ros_msg.total_size = proto.total_size
    ros_msg.data = proto.data

def convert_bosdyn_msgs_data_chunk_to_proto(ros_msg, proto):
    proto.Clear()
    proto.total_size = ros_msg.total_size
    proto.data = ros_msg.data

def convert_proto_to_bosdyn_msgs_synchronized_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_arm_command_request(proto.arm_command, ros_msg.arm_command)
    ros_msg.arm_command_is_set = proto.HasField("arm_command")
    convert_proto_to_bosdyn_msgs_mobility_command_request(proto.mobility_command, ros_msg.mobility_command)
    ros_msg.mobility_command_is_set = proto.HasField("mobility_command")
    convert_proto_to_bosdyn_msgs_gripper_command_request(proto.gripper_command, ros_msg.gripper_command)
    ros_msg.gripper_command_is_set = proto.HasField("gripper_command")

def convert_bosdyn_msgs_synchronized_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.arm_command_is_set:
        convert_bosdyn_msgs_arm_command_request_to_proto(ros_msg.arm_command, proto.arm_command)
    if ros_msg.mobility_command_is_set:
        convert_bosdyn_msgs_mobility_command_request_to_proto(ros_msg.mobility_command, proto.mobility_command)
    if ros_msg.gripper_command_is_set:
        convert_bosdyn_msgs_gripper_command_request_to_proto(ros_msg.gripper_command, proto.gripper_command)

def convert_proto_to_bosdyn_msgs_synchronized_command_feedback(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_arm_command_feedback(proto.arm_command_feedback, ros_msg.arm_command_feedback)
    ros_msg.arm_command_feedback_is_set = proto.HasField("arm_command_feedback")
    convert_proto_to_bosdyn_msgs_mobility_command_feedback(proto.mobility_command_feedback, ros_msg.mobility_command_feedback)
    ros_msg.mobility_command_feedback_is_set = proto.HasField("mobility_command_feedback")
    convert_proto_to_bosdyn_msgs_gripper_command_feedback(proto.gripper_command_feedback, ros_msg.gripper_command_feedback)
    ros_msg.gripper_command_feedback_is_set = proto.HasField("gripper_command_feedback")

def convert_bosdyn_msgs_synchronized_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.arm_command_feedback_is_set:
        convert_bosdyn_msgs_arm_command_feedback_to_proto(ros_msg.arm_command_feedback, proto.arm_command_feedback)
    if ros_msg.mobility_command_feedback_is_set:
        convert_bosdyn_msgs_mobility_command_feedback_to_proto(ros_msg.mobility_command_feedback, proto.mobility_command_feedback)
    if ros_msg.gripper_command_feedback_is_set:
        convert_bosdyn_msgs_gripper_command_feedback_to_proto(ros_msg.gripper_command_feedback, proto.gripper_command_feedback)

def convert_bosdyn_msgs_synchronized_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_license_info(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.id = proto.id
    ros_msg.robot_serial = proto.robot_serial
    convert_proto_to_builtin_interfaces_time(proto.not_valid_before, ros_msg.not_valid_before)
    ros_msg.not_valid_before_is_set = proto.HasField("not_valid_before")
    convert_proto_to_builtin_interfaces_time(proto.not_valid_after, ros_msg.not_valid_after)
    ros_msg.not_valid_after_is_set = proto.HasField("not_valid_after")
    ros_msg.licensed_features = []
    for _item in proto.licensed_features:
        ros_msg.licensed_features.append(_item)

def convert_bosdyn_msgs_license_info_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.id = ros_msg.id
    proto.robot_serial = ros_msg.robot_serial
    if ros_msg.not_valid_before_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.not_valid_before, proto.not_valid_before)
    if ros_msg.not_valid_after_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.not_valid_after, proto.not_valid_after)
    del proto.licensed_features[:]
    for _item in ros_msg.licensed_features:
        proto.licensed_features.add(_item)

def convert_proto_to_bosdyn_msgs_get_license_info_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_license_info_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_license_info_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_license_info(proto.license, ros_msg.license)
    ros_msg.license_is_set = proto.HasField("license")

def convert_bosdyn_msgs_get_license_info_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.license_is_set:
        convert_bosdyn_msgs_license_info_to_proto(ros_msg.license, proto.license)

def convert_proto_to_bosdyn_msgs_get_feature_enabled_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.feature_codes = []
    for _item in proto.feature_codes:
        ros_msg.feature_codes.append(_item)

def convert_bosdyn_msgs_get_feature_enabled_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.feature_codes[:]
    for _item in ros_msg.feature_codes:
        proto.feature_codes.add(_item)

def convert_proto_to_bosdyn_msgs_get_feature_enabled_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import KeyStringValueBool
    ros_msg.feature_enabled = []
    for _item in proto.feature_enabled:
        ros_msg.feature_enabled.append(KeyStringValueBool())
        ros_msg.feature_enabled[-1].key = _item
        ros_msg.feature_enabled[-1].value = proto.feature_enabled[_item]

def convert_bosdyn_msgs_get_feature_enabled_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    for _item in ros_msg.feature_enabled:
        proto.feature_enabled[_item.key] = _item.value

def convert_proto_to_bosdyn_msgs_get_auth_token_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.username = proto.username
    ros_msg.password = proto.password
    ros_msg.token = proto.token

def convert_bosdyn_msgs_get_auth_token_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.username = ros_msg.username
    proto.password = ros_msg.password
    proto.token = ros_msg.token

def convert_proto_to_bosdyn_msgs_get_auth_token_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.token = proto.token

def convert_bosdyn_msgs_get_auth_token_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.token = ros_msg.token

def convert_proto_to_bosdyn_msgs_stair_transform(proto, ros_msg):
    convert_proto_to_geometry_msgs_pose(proto.frame_tform_stairs, ros_msg.frame_tform_stairs)
    ros_msg.frame_tform_stairs_is_set = proto.HasField("frame_tform_stairs")
    ros_msg.frame_name = proto.frame_name

def convert_bosdyn_msgs_stair_transform_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.frame_tform_stairs_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.frame_tform_stairs, proto.frame_tform_stairs)
    proto.frame_name = ros_msg.frame_name

def convert_proto_to_bosdyn_msgs_straight_staircase_one_of_location(proto, ros_msg):
    if proto.HasField("from_ko_tform_stairs"):
        ros_msg.location_choice = ros_msg.LOCATION_FROM_KO_TFORM_STAIRS_SET
        convert_proto_to_geometry_msgs_pose(proto.from_ko_tform_stairs, ros_msg.from_ko_tform_stairs)
    if proto.HasField("tform"):
        ros_msg.location_choice = ros_msg.LOCATION_TFORM_SET
        convert_proto_to_bosdyn_msgs_stair_transform(proto.tform, ros_msg.tform)

def convert_bosdyn_msgs_straight_staircase_one_of_location_to_proto(ros_msg, proto):
    proto.ClearField("location")
    if ros_msg.location_choice == ros_msg.LOCATION_FROM_KO_TFORM_STAIRS_SET:
        convert_geometry_msgs_pose_to_proto(ros_msg.from_ko_tform_stairs, proto.from_ko_tform_stairs)
    if ros_msg.location_choice == ros_msg.LOCATION_TFORM_SET:
        convert_bosdyn_msgs_stair_transform_to_proto(ros_msg.tform, proto.tform)

def convert_proto_to_bosdyn_msgs_straight_staircase_stair(proto, ros_msg):
    ros_msg.rise = proto.rise
    ros_msg.run = proto.run

def convert_bosdyn_msgs_straight_staircase_stair_to_proto(ros_msg, proto):
    proto.Clear()
    proto.rise = ros_msg.rise
    proto.run = ros_msg.run

def convert_proto_to_bosdyn_msgs_straight_staircase_landing(proto, ros_msg):
    convert_proto_to_geometry_msgs_pose(proto.stairs_tform_landing_center, ros_msg.stairs_tform_landing_center)
    ros_msg.stairs_tform_landing_center_is_set = proto.HasField("stairs_tform_landing_center")
    ros_msg.landing_extent_x = proto.landing_extent_x
    ros_msg.landing_extent_y = proto.landing_extent_y

def convert_bosdyn_msgs_straight_staircase_landing_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.stairs_tform_landing_center_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.stairs_tform_landing_center, proto.stairs_tform_landing_center)
    proto.landing_extent_x = ros_msg.landing_extent_x
    proto.landing_extent_y = ros_msg.landing_extent_y

def convert_proto_to_bosdyn_msgs_straight_staircase(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_straight_staircase_one_of_location(proto, ros_msg.location)
    from bosdyn_msgs.msg import Stair
    ros_msg.stairs = []
    for _item in proto.stairs:
        ros_msg.stairs.append(Stair())
        convert_proto_to_bosdyn_msgs_straight_staircase_stair(_item, ros_msg.stairs[-1])
    convert_proto_to_bosdyn_msgs_straight_staircase_landing(proto.bottom_landing, ros_msg.bottom_landing)
    ros_msg.bottom_landing_is_set = proto.HasField("bottom_landing")
    convert_proto_to_bosdyn_msgs_straight_staircase_landing(proto.top_landing, ros_msg.top_landing)
    ros_msg.top_landing_is_set = proto.HasField("top_landing")

def convert_bosdyn_msgs_straight_staircase_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_straight_staircase_one_of_location_to_proto(ros_msg.location, proto)
    del proto.stairs[:]
    for _item in ros_msg.stairs:
        convert_bosdyn_msgs_straight_staircase_stair_to_proto(_item, proto.stairs.add())
    if ros_msg.bottom_landing_is_set:
        convert_bosdyn_msgs_straight_staircase_landing_to_proto(ros_msg.bottom_landing, proto.bottom_landing)
    if ros_msg.top_landing_is_set:
        convert_bosdyn_msgs_straight_staircase_landing_to_proto(ros_msg.top_landing, proto.top_landing)

def convert_proto_to_bosdyn_msgs_arm_surface_contact_request_one_of_joint_configuration(proto, ros_msg):
    if proto.HasField("force_remain_near_current_joint_configuration"):
        ros_msg.joint_configuration_choice = ros_msg.JOINT_CONFIGURATION_FORCE_REMAIN_NEAR_CURRENT_JOINT_CONFIGURATION_SET
        ros_msg.force_remain_near_current_joint_configuration = proto.force_remain_near_current_joint_configuration
    if proto.HasField("preferred_joint_configuration"):
        ros_msg.joint_configuration_choice = ros_msg.JOINT_CONFIGURATION_PREFERRED_JOINT_CONFIGURATION_SET
        convert_proto_to_bosdyn_msgs_arm_joint_position(proto.preferred_joint_configuration, ros_msg.preferred_joint_configuration)

def convert_bosdyn_msgs_arm_surface_contact_request_one_of_joint_configuration_to_proto(ros_msg, proto):
    proto.ClearField("joint_configuration")
    if ros_msg.joint_configuration_choice == ros_msg.JOINT_CONFIGURATION_FORCE_REMAIN_NEAR_CURRENT_JOINT_CONFIGURATION_SET:
        proto.force_remain_near_current_joint_configuration = ros_msg.force_remain_near_current_joint_configuration
    if ros_msg.joint_configuration_choice == ros_msg.JOINT_CONFIGURATION_PREFERRED_JOINT_CONFIGURATION_SET:
        convert_bosdyn_msgs_arm_joint_position_to_proto(ros_msg.preferred_joint_configuration, proto.preferred_joint_configuration)

def convert_proto_to_bosdyn_msgs_arm_surface_contact_request(proto, ros_msg):
    ros_msg.root_frame_name = proto.root_frame_name
    convert_proto_to_geometry_msgs_pose(proto.wrist_tform_tool, ros_msg.wrist_tform_tool)
    ros_msg.wrist_tform_tool_is_set = proto.HasField("wrist_tform_tool")
    convert_proto_to_geometry_msgs_pose(proto.root_tform_task, ros_msg.root_tform_task)
    ros_msg.root_tform_task_is_set = proto.HasField("root_tform_task")
    convert_proto_to_bosdyn_msgs_se3_trajectory(proto.pose_trajectory_in_task, ros_msg.pose_trajectory_in_task)
    ros_msg.pose_trajectory_in_task_is_set = proto.HasField("pose_trajectory_in_task")
    ros_msg.maximum_acceleration = proto.maximum_acceleration.value
    ros_msg.maximum_acceleration_is_set = proto.HasField("maximum_acceleration")
    ros_msg.max_linear_velocity = proto.max_linear_velocity.value
    ros_msg.max_linear_velocity_is_set = proto.HasField("max_linear_velocity")
    ros_msg.max_angular_velocity = proto.max_angular_velocity.value
    ros_msg.max_angular_velocity_is_set = proto.HasField("max_angular_velocity")
    ros_msg.max_pos_tracking_error = proto.max_pos_tracking_error.value
    ros_msg.max_pos_tracking_error_is_set = proto.HasField("max_pos_tracking_error")
    ros_msg.max_rot_tracking_error = proto.max_rot_tracking_error.value
    ros_msg.max_rot_tracking_error_is_set = proto.HasField("max_rot_tracking_error")
    convert_proto_to_bosdyn_msgs_arm_surface_contact_request_one_of_joint_configuration(proto, ros_msg.joint_configuration)
    ros_msg.x_axis.value = proto.x_axis
    ros_msg.y_axis.value = proto.y_axis
    ros_msg.z_axis.value = proto.z_axis
    convert_proto_to_geometry_msgs_vector3(proto.press_force_percentage, ros_msg.press_force_percentage)
    ros_msg.press_force_percentage_is_set = proto.HasField("press_force_percentage")
    ros_msg.xy_admittance.value = proto.xy_admittance
    ros_msg.z_admittance.value = proto.z_admittance
    ros_msg.xy_to_z_cross_term_admittance.value = proto.xy_to_z_cross_term_admittance
    convert_proto_to_geometry_msgs_vector3(proto.bias_force_ewrt_body, ros_msg.bias_force_ewrt_body)
    ros_msg.bias_force_ewrt_body_is_set = proto.HasField("bias_force_ewrt_body")
    convert_proto_to_bosdyn_msgs_claw_gripper_command_request(proto.gripper_command, ros_msg.gripper_command)
    ros_msg.gripper_command_is_set = proto.HasField("gripper_command")
    ros_msg.is_robot_following_hand = proto.is_robot_following_hand

def convert_bosdyn_msgs_arm_surface_contact_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.root_frame_name = ros_msg.root_frame_name
    if ros_msg.wrist_tform_tool_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.wrist_tform_tool, proto.wrist_tform_tool)
    if ros_msg.root_tform_task_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.root_tform_task, proto.root_tform_task)
    if ros_msg.pose_trajectory_in_task_is_set:
        convert_bosdyn_msgs_se3_trajectory_to_proto(ros_msg.pose_trajectory_in_task, proto.pose_trajectory_in_task)
    if ros_msg.maximum_acceleration_is_set:
        convert_float64_to_proto(ros_msg.maximum_acceleration, proto.maximum_acceleration)
    if ros_msg.max_linear_velocity_is_set:
        convert_float64_to_proto(ros_msg.max_linear_velocity, proto.max_linear_velocity)
    if ros_msg.max_angular_velocity_is_set:
        convert_float64_to_proto(ros_msg.max_angular_velocity, proto.max_angular_velocity)
    if ros_msg.max_pos_tracking_error_is_set:
        convert_float64_to_proto(ros_msg.max_pos_tracking_error, proto.max_pos_tracking_error)
    if ros_msg.max_rot_tracking_error_is_set:
        convert_float64_to_proto(ros_msg.max_rot_tracking_error, proto.max_rot_tracking_error)
    convert_bosdyn_msgs_arm_surface_contact_request_one_of_joint_configuration_to_proto(ros_msg.joint_configuration, proto)
    proto.x_axis = ros_msg.x_axis.value
    proto.y_axis = ros_msg.y_axis.value
    proto.z_axis = ros_msg.z_axis.value
    if ros_msg.press_force_percentage_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.press_force_percentage, proto.press_force_percentage)
    proto.xy_admittance = ros_msg.xy_admittance.value
    proto.z_admittance = ros_msg.z_admittance.value
    proto.xy_to_z_cross_term_admittance = ros_msg.xy_to_z_cross_term_admittance.value
    if ros_msg.bias_force_ewrt_body_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.bias_force_ewrt_body, proto.bias_force_ewrt_body)
    if ros_msg.gripper_command_is_set:
        convert_bosdyn_msgs_claw_gripper_command_request_to_proto(ros_msg.gripper_command, proto.gripper_command)
    proto.is_robot_following_hand = ros_msg.is_robot_following_hand

def convert_bosdyn_msgs_arm_surface_contact_feedback_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_arm_surface_contact_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_keypoint(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.coordinates, ros_msg.coordinates)
    ros_msg.coordinates_is_set = proto.HasField("coordinates")
    ros_msg.binary_descriptor = proto.binary_descriptor
    ros_msg.score = proto.score
    ros_msg.size = proto.size
    ros_msg.angle = proto.angle

def convert_bosdyn_msgs_keypoint_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.coordinates_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.coordinates, proto.coordinates)
    proto.binary_descriptor = ros_msg.binary_descriptor
    proto.score = ros_msg.score
    proto.size = ros_msg.size
    proto.angle = ros_msg.angle

def convert_proto_to_bosdyn_msgs_keypoint_set(proto, ros_msg):
    from bosdyn_msgs.msg import Keypoint
    ros_msg.keypoints = []
    for _item in proto.keypoints:
        ros_msg.keypoints.append(Keypoint())
        convert_proto_to_bosdyn_msgs_keypoint(_item, ros_msg.keypoints[-1])
    ros_msg.type.value = proto.type

def convert_bosdyn_msgs_keypoint_set_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.keypoints[:]
    for _item in ros_msg.keypoints:
        convert_bosdyn_msgs_keypoint_to_proto(_item, proto.keypoints.add())
    proto.type = ros_msg.type.value

def convert_proto_to_bosdyn_msgs_match(proto, ros_msg):
    ros_msg.reference_index = proto.reference_index
    ros_msg.live_index = proto.live_index
    ros_msg.distance = proto.distance

def convert_bosdyn_msgs_match_to_proto(ros_msg, proto):
    proto.Clear()
    proto.reference_index = ros_msg.reference_index
    proto.live_index = ros_msg.live_index
    proto.distance = ros_msg.distance

def convert_proto_to_bosdyn_msgs_keypoint_matches(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_keypoint_set(proto.reference_keypoints, ros_msg.reference_keypoints)
    ros_msg.reference_keypoints_is_set = proto.HasField("reference_keypoints")
    convert_proto_to_bosdyn_msgs_keypoint_set(proto.live_keypoints, ros_msg.live_keypoints)
    ros_msg.live_keypoints_is_set = proto.HasField("live_keypoints")
    from bosdyn_msgs.msg import Match
    ros_msg.matches = []
    for _item in proto.matches:
        ros_msg.matches.append(Match())
        convert_proto_to_bosdyn_msgs_match(_item, ros_msg.matches[-1])

def convert_bosdyn_msgs_keypoint_matches_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.reference_keypoints_is_set:
        convert_bosdyn_msgs_keypoint_set_to_proto(ros_msg.reference_keypoints, proto.reference_keypoints)
    if ros_msg.live_keypoints_is_set:
        convert_bosdyn_msgs_keypoint_set_to_proto(ros_msg.live_keypoints, proto.live_keypoints)
    del proto.matches[:]
    for _item in ros_msg.matches:
        convert_bosdyn_msgs_match_to_proto(_item, proto.matches.add())

def convert_proto_to_bosdyn_msgs_service_entry_one_of_service_type(proto, ros_msg):
    if proto.HasField("type"):
        ros_msg.service_type_choice = ros_msg.SERVICE_TYPE_TYPE_SET
        ros_msg.type = proto.type

def convert_bosdyn_msgs_service_entry_one_of_service_type_to_proto(ros_msg, proto):
    proto.ClearField("service_type")
    if ros_msg.service_type_choice == ros_msg.SERVICE_TYPE_TYPE_SET:
        proto.type = ros_msg.type

def convert_proto_to_bosdyn_msgs_service_entry(proto, ros_msg):
    ros_msg.name = proto.name
    convert_proto_to_bosdyn_msgs_service_entry_one_of_service_type(proto, ros_msg.service_type)
    ros_msg.authority = proto.authority
    convert_proto_to_builtin_interfaces_time(proto.last_update, ros_msg.last_update)
    ros_msg.last_update_is_set = proto.HasField("last_update")
    ros_msg.user_token_required = proto.user_token_required
    ros_msg.permission_required = proto.permission_required
    ros_msg.liveness_timeout_secs = proto.liveness_timeout_secs
    ros_msg.host_payload_guid = proto.host_payload_guid

def convert_bosdyn_msgs_service_entry_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    convert_bosdyn_msgs_service_entry_one_of_service_type_to_proto(ros_msg.service_type, proto)
    proto.authority = ros_msg.authority
    if ros_msg.last_update_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.last_update, proto.last_update)
    proto.user_token_required = ros_msg.user_token_required
    proto.permission_required = ros_msg.permission_required
    proto.liveness_timeout_secs = ros_msg.liveness_timeout_secs
    proto.host_payload_guid = ros_msg.host_payload_guid

def convert_proto_to_bosdyn_msgs_endpoint(proto, ros_msg):
    ros_msg.host_ip = proto.host_ip
    ros_msg.port = proto.port

def convert_bosdyn_msgs_endpoint_to_proto(ros_msg, proto):
    proto.Clear()
    proto.host_ip = ros_msg.host_ip
    proto.port = ros_msg.port

def convert_proto_to_bosdyn_msgs_get_service_entry_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.service_name = proto.service_name

def convert_bosdyn_msgs_get_service_entry_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.service_name = ros_msg.service_name

def convert_proto_to_bosdyn_msgs_get_service_entry_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_service_entry(proto.service_entry, ros_msg.service_entry)
    ros_msg.service_entry_is_set = proto.HasField("service_entry")

def convert_bosdyn_msgs_get_service_entry_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.service_entry_is_set:
        convert_bosdyn_msgs_service_entry_to_proto(ros_msg.service_entry, proto.service_entry)

def convert_proto_to_bosdyn_msgs_list_service_entries_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_service_entries_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_service_entries_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import ServiceEntry
    ros_msg.service_entries = []
    for _item in proto.service_entries:
        ros_msg.service_entries.append(ServiceEntry())
        convert_proto_to_bosdyn_msgs_service_entry(_item, ros_msg.service_entries[-1])

def convert_bosdyn_msgs_list_service_entries_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.service_entries[:]
    for _item in ros_msg.service_entries:
        convert_bosdyn_msgs_service_entry_to_proto(_item, proto.service_entries.add())

def convert_proto_to_bosdyn_msgs_full_body_command_request_one_of_command(proto, ros_msg):
    if proto.HasField("stop_request"):
        ros_msg.command_choice = ros_msg.COMMAND_STOP_REQUEST_SET
        convert_proto_to_bosdyn_msgs_stop_command_request(proto.stop_request, ros_msg.stop_request)
    if proto.HasField("freeze_request"):
        ros_msg.command_choice = ros_msg.COMMAND_FREEZE_REQUEST_SET
        convert_proto_to_bosdyn_msgs_freeze_command_request(proto.freeze_request, ros_msg.freeze_request)
    if proto.HasField("selfright_request"):
        ros_msg.command_choice = ros_msg.COMMAND_SELFRIGHT_REQUEST_SET
        convert_proto_to_bosdyn_msgs_self_right_command_request(proto.selfright_request, ros_msg.selfright_request)
    if proto.HasField("safe_power_off_request"):
        ros_msg.command_choice = ros_msg.COMMAND_SAFE_POWER_OFF_REQUEST_SET
        convert_proto_to_bosdyn_msgs_safe_power_off_command_request(proto.safe_power_off_request, ros_msg.safe_power_off_request)
    if proto.HasField("battery_change_pose_request"):
        ros_msg.command_choice = ros_msg.COMMAND_BATTERY_CHANGE_POSE_REQUEST_SET
        convert_proto_to_bosdyn_msgs_battery_change_pose_command_request(proto.battery_change_pose_request, ros_msg.battery_change_pose_request)
    if proto.HasField("payload_estimation_request"):
        ros_msg.command_choice = ros_msg.COMMAND_PAYLOAD_ESTIMATION_REQUEST_SET
        convert_proto_to_bosdyn_msgs_payload_estimation_command_request(proto.payload_estimation_request, ros_msg.payload_estimation_request)
    if proto.HasField("constrained_manipulation_request"):
        ros_msg.command_choice = ros_msg.COMMAND_CONSTRAINED_MANIPULATION_REQUEST_SET
        convert_proto_to_bosdyn_msgs_constrained_manipulation_command_request(proto.constrained_manipulation_request, ros_msg.constrained_manipulation_request)

def convert_bosdyn_msgs_full_body_command_request_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_STOP_REQUEST_SET:
        convert_bosdyn_msgs_stop_command_request_to_proto(ros_msg.stop_request, proto.stop_request)
    if ros_msg.command_choice == ros_msg.COMMAND_FREEZE_REQUEST_SET:
        convert_bosdyn_msgs_freeze_command_request_to_proto(ros_msg.freeze_request, proto.freeze_request)
    if ros_msg.command_choice == ros_msg.COMMAND_SELFRIGHT_REQUEST_SET:
        convert_bosdyn_msgs_self_right_command_request_to_proto(ros_msg.selfright_request, proto.selfright_request)
    if ros_msg.command_choice == ros_msg.COMMAND_SAFE_POWER_OFF_REQUEST_SET:
        convert_bosdyn_msgs_safe_power_off_command_request_to_proto(ros_msg.safe_power_off_request, proto.safe_power_off_request)
    if ros_msg.command_choice == ros_msg.COMMAND_BATTERY_CHANGE_POSE_REQUEST_SET:
        convert_bosdyn_msgs_battery_change_pose_command_request_to_proto(ros_msg.battery_change_pose_request, proto.battery_change_pose_request)
    if ros_msg.command_choice == ros_msg.COMMAND_PAYLOAD_ESTIMATION_REQUEST_SET:
        convert_bosdyn_msgs_payload_estimation_command_request_to_proto(ros_msg.payload_estimation_request, proto.payload_estimation_request)
    if ros_msg.command_choice == ros_msg.COMMAND_CONSTRAINED_MANIPULATION_REQUEST_SET:
        convert_bosdyn_msgs_constrained_manipulation_command_request_to_proto(ros_msg.constrained_manipulation_request, proto.constrained_manipulation_request)

def convert_proto_to_bosdyn_msgs_full_body_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_full_body_command_request_one_of_command(proto, ros_msg.command)

def convert_bosdyn_msgs_full_body_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_full_body_command_request_one_of_command_to_proto(ros_msg.command, proto)

def convert_proto_to_bosdyn_msgs_full_body_command_feedback_one_of_feedback(proto, ros_msg):
    if proto.HasField("stop_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_STOP_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_stop_command_feedback(proto.stop_feedback, ros_msg.stop_feedback)
    if proto.HasField("freeze_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_FREEZE_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_freeze_command_feedback(proto.freeze_feedback, ros_msg.freeze_feedback)
    if proto.HasField("selfright_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_SELFRIGHT_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_self_right_command_feedback(proto.selfright_feedback, ros_msg.selfright_feedback)
    if proto.HasField("safe_power_off_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_safe_power_off_command_feedback(proto.safe_power_off_feedback, ros_msg.safe_power_off_feedback)
    if proto.HasField("battery_change_pose_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_battery_change_pose_command_feedback(proto.battery_change_pose_feedback, ros_msg.battery_change_pose_feedback)
    if proto.HasField("payload_estimation_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_payload_estimation_command_feedback(proto.payload_estimation_feedback, ros_msg.payload_estimation_feedback)
    if proto.HasField("constrained_manipulation_feedback"):
        ros_msg.feedback_choice = ros_msg.FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET
        convert_proto_to_bosdyn_msgs_constrained_manipulation_command_feedback(proto.constrained_manipulation_feedback, ros_msg.constrained_manipulation_feedback)

def convert_bosdyn_msgs_full_body_command_feedback_one_of_feedback_to_proto(ros_msg, proto):
    proto.ClearField("feedback")
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_STOP_FEEDBACK_SET:
        convert_bosdyn_msgs_stop_command_feedback_to_proto(ros_msg.stop_feedback, proto.stop_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_FREEZE_FEEDBACK_SET:
        convert_bosdyn_msgs_freeze_command_feedback_to_proto(ros_msg.freeze_feedback, proto.freeze_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_SELFRIGHT_FEEDBACK_SET:
        convert_bosdyn_msgs_self_right_command_feedback_to_proto(ros_msg.selfright_feedback, proto.selfright_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_SAFE_POWER_OFF_FEEDBACK_SET:
        convert_bosdyn_msgs_safe_power_off_command_feedback_to_proto(ros_msg.safe_power_off_feedback, proto.safe_power_off_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_BATTERY_CHANGE_POSE_FEEDBACK_SET:
        convert_bosdyn_msgs_battery_change_pose_command_feedback_to_proto(ros_msg.battery_change_pose_feedback, proto.battery_change_pose_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_PAYLOAD_ESTIMATION_FEEDBACK_SET:
        convert_bosdyn_msgs_payload_estimation_command_feedback_to_proto(ros_msg.payload_estimation_feedback, proto.payload_estimation_feedback)
    if ros_msg.feedback_choice == ros_msg.FEEDBACK_CONSTRAINED_MANIPULATION_FEEDBACK_SET:
        convert_bosdyn_msgs_constrained_manipulation_command_feedback_to_proto(ros_msg.constrained_manipulation_feedback, proto.constrained_manipulation_feedback)

def convert_proto_to_bosdyn_msgs_full_body_command_feedback(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_full_body_command_feedback_one_of_feedback(proto, ros_msg.feedback)
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_full_body_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_full_body_command_feedback_one_of_feedback_to_proto(ros_msg.feedback, proto)
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_full_body_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_data_acquisition_capability(proto, ros_msg):
    ros_msg.name = proto.name
    ros_msg.description = proto.description
    ros_msg.channel_name = proto.channel_name
    ros_msg.service_name = proto.service_name

def convert_bosdyn_msgs_data_acquisition_capability_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    proto.description = ros_msg.description
    proto.channel_name = ros_msg.channel_name
    proto.service_name = ros_msg.service_name

def convert_proto_to_bosdyn_msgs_image_acquisition_capability(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.image_source_names = []
    for _item in proto.image_source_names:
        ros_msg.image_source_names.append(_item)
    from bosdyn_msgs.msg import ImageSource
    ros_msg.image_sources = []
    for _item in proto.image_sources:
        ros_msg.image_sources.append(ImageSource())
        convert_proto_to_bosdyn_msgs_image_source(_item, ros_msg.image_sources[-1])

def convert_bosdyn_msgs_image_acquisition_capability_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    del proto.image_source_names[:]
    for _item in ros_msg.image_source_names:
        proto.image_source_names.add(_item)
    del proto.image_sources[:]
    for _item in ros_msg.image_sources:
        convert_bosdyn_msgs_image_source_to_proto(_item, proto.image_sources.add())

def convert_proto_to_bosdyn_msgs_network_compute_capability(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_network_compute_server_configuration(proto.server_config, ros_msg.server_config)
    ros_msg.server_config_is_set = proto.HasField("server_config")
    ros_msg.available_models = []
    for _item in proto.available_models:
        ros_msg.available_models.append(_item)
    from bosdyn_msgs.msg import ModelLabels
    ros_msg.labels = []
    for _item in proto.labels:
        ros_msg.labels.append(ModelLabels())
        convert_proto_to_bosdyn_msgs_model_labels(_item, ros_msg.labels[-1])

def convert_bosdyn_msgs_network_compute_capability_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.server_config_is_set:
        convert_bosdyn_msgs_network_compute_server_configuration_to_proto(ros_msg.server_config, proto.server_config)
    del proto.available_models[:]
    for _item in ros_msg.available_models:
        proto.available_models.add(_item)
    del proto.labels[:]
    for _item in ros_msg.labels:
        convert_bosdyn_msgs_model_labels_to_proto(_item, proto.labels.add())

def convert_proto_to_bosdyn_msgs_acquisition_capability_list(proto, ros_msg):
    from bosdyn_msgs.msg import DataAcquisitionCapability
    ros_msg.data_sources = []
    for _item in proto.data_sources:
        ros_msg.data_sources.append(DataAcquisitionCapability())
        convert_proto_to_bosdyn_msgs_data_acquisition_capability(_item, ros_msg.data_sources[-1])
    from bosdyn_msgs.msg import ImageAcquisitionCapability
    ros_msg.image_sources = []
    for _item in proto.image_sources:
        ros_msg.image_sources.append(ImageAcquisitionCapability())
        convert_proto_to_bosdyn_msgs_image_acquisition_capability(_item, ros_msg.image_sources[-1])
    from bosdyn_msgs.msg import NetworkComputeCapability
    ros_msg.network_compute_sources = []
    for _item in proto.network_compute_sources:
        ros_msg.network_compute_sources.append(NetworkComputeCapability())
        convert_proto_to_bosdyn_msgs_network_compute_capability(_item, ros_msg.network_compute_sources[-1])

def convert_bosdyn_msgs_acquisition_capability_list_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.data_sources[:]
    for _item in ros_msg.data_sources:
        convert_bosdyn_msgs_data_acquisition_capability_to_proto(_item, proto.data_sources.add())
    del proto.image_sources[:]
    for _item in ros_msg.image_sources:
        convert_bosdyn_msgs_image_acquisition_capability_to_proto(_item, proto.image_sources.add())
    del proto.network_compute_sources[:]
    for _item in ros_msg.network_compute_sources:
        convert_bosdyn_msgs_network_compute_capability_to_proto(_item, proto.network_compute_sources.add())

def convert_proto_to_bosdyn_msgs_capture_action_id(proto, ros_msg):
    ros_msg.action_name = proto.action_name
    ros_msg.group_name = proto.group_name
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")

def convert_bosdyn_msgs_capture_action_id_to_proto(ros_msg, proto):
    proto.Clear()
    proto.action_name = ros_msg.action_name
    proto.group_name = ros_msg.group_name
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)

def convert_proto_to_bosdyn_msgs_data_identifier(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_capture_action_id(proto.action_id, ros_msg.action_id)
    ros_msg.action_id_is_set = proto.HasField("action_id")
    ros_msg.channel = proto.channel
    ros_msg.data_name = proto.data_name

def convert_bosdyn_msgs_data_identifier_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.action_id_is_set:
        convert_bosdyn_msgs_capture_action_id_to_proto(ros_msg.action_id, proto.action_id)
    proto.channel = ros_msg.channel
    proto.data_name = ros_msg.data_name

def convert_bosdyn_msgs_metadata_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_associated_metadata(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_data_identifier(proto.reference_id, ros_msg.reference_id)
    ros_msg.reference_id_is_set = proto.HasField("reference_id")
    convert_proto_to_bosdyn_msgs_metadata(proto.metadata, ros_msg.metadata)
    ros_msg.metadata_is_set = proto.HasField("metadata")

def convert_bosdyn_msgs_associated_metadata_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.reference_id_is_set:
        convert_bosdyn_msgs_data_identifier_to_proto(ros_msg.reference_id, proto.reference_id)
    if ros_msg.metadata_is_set:
        convert_bosdyn_msgs_metadata_to_proto(ros_msg.metadata, proto.metadata)

def convert_proto_to_bosdyn_msgs_associated_alert_data(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_data_identifier(proto.reference_id, ros_msg.reference_id)
    ros_msg.reference_id_is_set = proto.HasField("reference_id")
    convert_proto_to_bosdyn_msgs_alert_data(proto.alert_data, ros_msg.alert_data)
    ros_msg.alert_data_is_set = proto.HasField("alert_data")

def convert_bosdyn_msgs_associated_alert_data_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.reference_id_is_set:
        convert_bosdyn_msgs_data_identifier_to_proto(ros_msg.reference_id, proto.reference_id)
    if ros_msg.alert_data_is_set:
        convert_bosdyn_msgs_alert_data_to_proto(ros_msg.alert_data, proto.alert_data)

def convert_proto_to_bosdyn_msgs_image_source_capture(proto, ros_msg):
    ros_msg.image_service = proto.image_service
    convert_proto_to_bosdyn_msgs_image_request(proto.image_request, ros_msg.image_request)
    ros_msg.image_request_is_set = proto.HasField("image_request")

def convert_bosdyn_msgs_image_source_capture_to_proto(ros_msg, proto):
    proto.Clear()
    proto.image_service = ros_msg.image_service
    if ros_msg.image_request_is_set:
        convert_bosdyn_msgs_image_request_to_proto(ros_msg.image_request, proto.image_request)

def convert_proto_to_bosdyn_msgs_data_capture(proto, ros_msg):
    ros_msg.name = proto.name

def convert_bosdyn_msgs_data_capture_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name

def convert_proto_to_bosdyn_msgs_network_compute_capture(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_network_compute_input_data(proto.input_data, ros_msg.input_data)
    ros_msg.input_data_is_set = proto.HasField("input_data")
    convert_proto_to_bosdyn_msgs_network_compute_server_configuration(proto.server_config, ros_msg.server_config)
    ros_msg.server_config_is_set = proto.HasField("server_config")

def convert_bosdyn_msgs_network_compute_capture_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.input_data_is_set:
        convert_bosdyn_msgs_network_compute_input_data_to_proto(ros_msg.input_data, proto.input_data)
    if ros_msg.server_config_is_set:
        convert_bosdyn_msgs_network_compute_server_configuration_to_proto(ros_msg.server_config, proto.server_config)

def convert_proto_to_bosdyn_msgs_acquisition_request_list(proto, ros_msg):
    from bosdyn_msgs.msg import ImageSourceCapture
    ros_msg.image_captures = []
    for _item in proto.image_captures:
        ros_msg.image_captures.append(ImageSourceCapture())
        convert_proto_to_bosdyn_msgs_image_source_capture(_item, ros_msg.image_captures[-1])
    from bosdyn_msgs.msg import DataCapture
    ros_msg.data_captures = []
    for _item in proto.data_captures:
        ros_msg.data_captures.append(DataCapture())
        convert_proto_to_bosdyn_msgs_data_capture(_item, ros_msg.data_captures[-1])
    from bosdyn_msgs.msg import NetworkComputeCapture
    ros_msg.network_compute_captures = []
    for _item in proto.network_compute_captures:
        ros_msg.network_compute_captures.append(NetworkComputeCapture())
        convert_proto_to_bosdyn_msgs_network_compute_capture(_item, ros_msg.network_compute_captures[-1])

def convert_bosdyn_msgs_acquisition_request_list_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.image_captures[:]
    for _item in ros_msg.image_captures:
        convert_bosdyn_msgs_image_source_capture_to_proto(_item, proto.image_captures.add())
    del proto.data_captures[:]
    for _item in ros_msg.data_captures:
        convert_bosdyn_msgs_data_capture_to_proto(_item, proto.data_captures.add())
    del proto.network_compute_captures[:]
    for _item in ros_msg.network_compute_captures:
        convert_bosdyn_msgs_network_compute_capture_to_proto(_item, proto.network_compute_captures.add())

def convert_proto_to_bosdyn_msgs_data_error(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_data_identifier(proto.data_id, ros_msg.data_id)
    ros_msg.data_id_is_set = proto.HasField("data_id")
    ros_msg.error_message = proto.error_message

def convert_bosdyn_msgs_data_error_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.data_id_is_set:
        convert_bosdyn_msgs_data_identifier_to_proto(ros_msg.data_id, proto.data_id)
    proto.error_message = ros_msg.error_message

def convert_proto_to_bosdyn_msgs_plugin_service_error(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.error.value = proto.error
    ros_msg.message = proto.message

def convert_bosdyn_msgs_plugin_service_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.error = ros_msg.error.value
    proto.message = ros_msg.message

def convert_proto_to_bosdyn_msgs_network_compute_error(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.error.value = proto.error
    ros_msg.network_compute_status.value = proto.network_compute_status
    ros_msg.message = proto.message

def convert_bosdyn_msgs_network_compute_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.error = ros_msg.error.value
    proto.network_compute_status = ros_msg.network_compute_status.value
    proto.message = ros_msg.message

def convert_proto_to_bosdyn_msgs_acquire_data_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_capture_action_id(proto.action_id, ros_msg.action_id)
    ros_msg.action_id_is_set = proto.HasField("action_id")
    convert_proto_to_bosdyn_msgs_metadata(proto.metadata, ros_msg.metadata)
    ros_msg.metadata_is_set = proto.HasField("metadata")
    convert_proto_to_bosdyn_msgs_acquisition_request_list(proto.acquisition_requests, ros_msg.acquisition_requests)
    ros_msg.acquisition_requests_is_set = proto.HasField("acquisition_requests")
    convert_proto_to_builtin_interfaces_duration(proto.min_timeout, ros_msg.min_timeout)
    ros_msg.min_timeout_is_set = proto.HasField("min_timeout")

def convert_bosdyn_msgs_acquire_data_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.action_id_is_set:
        convert_bosdyn_msgs_capture_action_id_to_proto(ros_msg.action_id, proto.action_id)
    if ros_msg.metadata_is_set:
        convert_bosdyn_msgs_metadata_to_proto(ros_msg.metadata, proto.metadata)
    if ros_msg.acquisition_requests_is_set:
        convert_bosdyn_msgs_acquisition_request_list_to_proto(ros_msg.acquisition_requests, proto.acquisition_requests)
    if ros_msg.min_timeout_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.min_timeout, proto.min_timeout)

def convert_proto_to_bosdyn_msgs_acquire_data_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.request_id = proto.request_id

def convert_bosdyn_msgs_acquire_data_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.request_id = ros_msg.request_id

def convert_proto_to_bosdyn_msgs_acquire_plugin_data_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import DataIdentifier
    ros_msg.data_id = []
    for _item in proto.data_id:
        ros_msg.data_id.append(DataIdentifier())
        convert_proto_to_bosdyn_msgs_data_identifier(_item, ros_msg.data_id[-1])
    convert_proto_to_bosdyn_msgs_metadata(proto.metadata, ros_msg.metadata)
    ros_msg.metadata_is_set = proto.HasField("metadata")
    convert_proto_to_bosdyn_msgs_capture_action_id(proto.action_id, ros_msg.action_id)
    ros_msg.action_id_is_set = proto.HasField("action_id")
    convert_proto_to_bosdyn_msgs_acquisition_request_list(proto.acquisition_requests, ros_msg.acquisition_requests)
    ros_msg.acquisition_requests_is_set = proto.HasField("acquisition_requests")

def convert_bosdyn_msgs_acquire_plugin_data_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.data_id[:]
    for _item in ros_msg.data_id:
        convert_bosdyn_msgs_data_identifier_to_proto(_item, proto.data_id.add())
    if ros_msg.metadata_is_set:
        convert_bosdyn_msgs_metadata_to_proto(ros_msg.metadata, proto.metadata)
    if ros_msg.action_id_is_set:
        convert_bosdyn_msgs_capture_action_id_to_proto(ros_msg.action_id, proto.action_id)
    if ros_msg.acquisition_requests_is_set:
        convert_bosdyn_msgs_acquisition_request_list_to_proto(ros_msg.acquisition_requests, proto.acquisition_requests)

def convert_proto_to_bosdyn_msgs_acquire_plugin_data_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.request_id = proto.request_id
    convert_proto_to_builtin_interfaces_time(proto.timeout_deadline, ros_msg.timeout_deadline)
    ros_msg.timeout_deadline_is_set = proto.HasField("timeout_deadline")

def convert_bosdyn_msgs_acquire_plugin_data_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.request_id = ros_msg.request_id
    if ros_msg.timeout_deadline_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timeout_deadline, proto.timeout_deadline)

def convert_proto_to_bosdyn_msgs_get_status_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.request_id = proto.request_id

def convert_bosdyn_msgs_get_status_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.request_id = ros_msg.request_id

def convert_proto_to_bosdyn_msgs_get_status_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    from bosdyn_msgs.msg import DataIdentifier
    ros_msg.data_saved = []
    for _item in proto.data_saved:
        ros_msg.data_saved.append(DataIdentifier())
        convert_proto_to_bosdyn_msgs_data_identifier(_item, ros_msg.data_saved[-1])
    from bosdyn_msgs.msg import DataError
    ros_msg.data_errors = []
    for _item in proto.data_errors:
        ros_msg.data_errors.append(DataError())
        convert_proto_to_bosdyn_msgs_data_error(_item, ros_msg.data_errors[-1])
    from bosdyn_msgs.msg import PluginServiceError
    ros_msg.service_errors = []
    for _item in proto.service_errors:
        ros_msg.service_errors.append(PluginServiceError())
        convert_proto_to_bosdyn_msgs_plugin_service_error(_item, ros_msg.service_errors[-1])
    from bosdyn_msgs.msg import NetworkComputeError
    ros_msg.network_compute_errors = []
    for _item in proto.network_compute_errors:
        ros_msg.network_compute_errors.append(NetworkComputeError())
        convert_proto_to_bosdyn_msgs_network_compute_error(_item, ros_msg.network_compute_errors[-1])

def convert_bosdyn_msgs_get_status_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    del proto.data_saved[:]
    for _item in ros_msg.data_saved:
        convert_bosdyn_msgs_data_identifier_to_proto(_item, proto.data_saved.add())
    del proto.data_errors[:]
    for _item in ros_msg.data_errors:
        convert_bosdyn_msgs_data_error_to_proto(_item, proto.data_errors.add())
    del proto.service_errors[:]
    for _item in ros_msg.service_errors:
        convert_bosdyn_msgs_plugin_service_error_to_proto(_item, proto.service_errors.add())
    del proto.network_compute_errors[:]
    for _item in ros_msg.network_compute_errors:
        convert_bosdyn_msgs_network_compute_error_to_proto(_item, proto.network_compute_errors.add())

def convert_proto_to_bosdyn_msgs_get_service_info_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_service_info_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_service_info_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_acquisition_capability_list(proto.capabilities, ros_msg.capabilities)
    ros_msg.capabilities_is_set = proto.HasField("capabilities")

def convert_bosdyn_msgs_get_service_info_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.capabilities_is_set:
        convert_bosdyn_msgs_acquisition_capability_list_to_proto(ros_msg.capabilities, proto.capabilities)

def convert_proto_to_bosdyn_msgs_cancel_acquisition_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.request_id = proto.request_id

def convert_bosdyn_msgs_cancel_acquisition_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.request_id = ros_msg.request_id

def convert_proto_to_bosdyn_msgs_cancel_acquisition_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_cancel_acquisition_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_descriptor_block_one_of_descriptor_type(proto, ros_msg):
    if proto.HasField("file_descriptor"):
        ros_msg.descriptor_type_choice = ros_msg.DESCRIPTORTYPE_FILE_DESCRIPTOR_SET
        convert_proto_to_bosdyn_msgs_file_format_descriptor(proto.file_descriptor, ros_msg.file_descriptor)
    if proto.HasField("series_descriptor"):
        ros_msg.descriptor_type_choice = ros_msg.DESCRIPTORTYPE_SERIES_DESCRIPTOR_SET
        convert_proto_to_bosdyn_msgs_series_descriptor(proto.series_descriptor, ros_msg.series_descriptor)
    if proto.HasField("series_block_index"):
        ros_msg.descriptor_type_choice = ros_msg.DESCRIPTORTYPE_SERIES_BLOCK_INDEX_SET
        convert_proto_to_bosdyn_msgs_series_block_index(proto.series_block_index, ros_msg.series_block_index)
    if proto.HasField("file_index"):
        ros_msg.descriptor_type_choice = ros_msg.DESCRIPTORTYPE_FILE_INDEX_SET
        convert_proto_to_bosdyn_msgs_file_index(proto.file_index, ros_msg.file_index)

def convert_bosdyn_msgs_descriptor_block_one_of_descriptor_type_to_proto(ros_msg, proto):
    proto.ClearField("DescriptorType")
    if ros_msg.descriptor_type_choice == ros_msg.DESCRIPTORTYPE_FILE_DESCRIPTOR_SET:
        convert_bosdyn_msgs_file_format_descriptor_to_proto(ros_msg.file_descriptor, proto.file_descriptor)
    if ros_msg.descriptor_type_choice == ros_msg.DESCRIPTORTYPE_SERIES_DESCRIPTOR_SET:
        convert_bosdyn_msgs_series_descriptor_to_proto(ros_msg.series_descriptor, proto.series_descriptor)
    if ros_msg.descriptor_type_choice == ros_msg.DESCRIPTORTYPE_SERIES_BLOCK_INDEX_SET:
        convert_bosdyn_msgs_series_block_index_to_proto(ros_msg.series_block_index, proto.series_block_index)
    if ros_msg.descriptor_type_choice == ros_msg.DESCRIPTORTYPE_FILE_INDEX_SET:
        convert_bosdyn_msgs_file_index_to_proto(ros_msg.file_index, proto.file_index)

def convert_proto_to_bosdyn_msgs_descriptor_block(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_descriptor_block_one_of_descriptor_type(proto, ros_msg.descriptor_type)

def convert_bosdyn_msgs_descriptor_block_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_descriptor_block_one_of_descriptor_type_to_proto(ros_msg.descriptor_type, proto)

def convert_proto_to_bosdyn_msgs_data_descriptor(proto, ros_msg):
    ros_msg.series_index = proto.series_index
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.additional_indexes = []
    for _item in proto.additional_indexes:
        ros_msg.additional_indexes.append(_item)

def convert_bosdyn_msgs_data_descriptor_to_proto(ros_msg, proto):
    proto.Clear()
    proto.series_index = ros_msg.series_index
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    del proto.additional_indexes[:]
    for _item in ros_msg.additional_indexes:
        proto.additional_indexes.add(_item)

def convert_proto_to_bosdyn_msgs_file_format_descriptor(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_file_format_version(proto.version, ros_msg.version)
    ros_msg.version_is_set = proto.HasField("version")
    from bosdyn_msgs.msg import KeyStringValueString
    ros_msg.annotations = []
    for _item in proto.annotations:
        ros_msg.annotations.append(KeyStringValueString())
        ros_msg.annotations[-1].key = _item
        ros_msg.annotations[-1].value = proto.annotations[_item]
    ros_msg.checksum_type.value = proto.checksum_type
    ros_msg.checksum_num_bytes = proto.checksum_num_bytes

def convert_bosdyn_msgs_file_format_descriptor_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.version_is_set:
        convert_bosdyn_msgs_file_format_version_to_proto(ros_msg.version, proto.version)
    for _item in ros_msg.annotations:
        proto.annotations[_item.key] = _item.value
    proto.checksum_type = ros_msg.checksum_type.value
    proto.checksum_num_bytes = ros_msg.checksum_num_bytes

def convert_proto_to_bosdyn_msgs_file_format_version(proto, ros_msg):
    ros_msg.major_version = proto.major_version
    ros_msg.minor_version = proto.minor_version
    ros_msg.patch_level = proto.patch_level

def convert_bosdyn_msgs_file_format_version_to_proto(ros_msg, proto):
    proto.Clear()
    proto.major_version = ros_msg.major_version
    proto.minor_version = ros_msg.minor_version
    proto.patch_level = ros_msg.patch_level

def convert_proto_to_bosdyn_msgs_series_descriptor_one_of_data_type(proto, ros_msg):
    if proto.HasField("message_type"):
        ros_msg.data_type_choice = ros_msg.DATATYPE_MESSAGE_TYPE_SET
        convert_proto_to_bosdyn_msgs_message_type_descriptor(proto.message_type, ros_msg.message_type)
    if proto.HasField("pod_type"):
        ros_msg.data_type_choice = ros_msg.DATATYPE_POD_TYPE_SET
        convert_proto_to_bosdyn_msgs_pod_type_descriptor(proto.pod_type, ros_msg.pod_type)
    if proto.HasField("struct_type"):
        ros_msg.data_type_choice = ros_msg.DATATYPE_STRUCT_TYPE_SET
        convert_proto_to_bosdyn_msgs_struct_type_descriptor(proto.struct_type, ros_msg.struct_type)

def convert_bosdyn_msgs_series_descriptor_one_of_data_type_to_proto(ros_msg, proto):
    proto.ClearField("DataType")
    if ros_msg.data_type_choice == ros_msg.DATATYPE_MESSAGE_TYPE_SET:
        convert_bosdyn_msgs_message_type_descriptor_to_proto(ros_msg.message_type, proto.message_type)
    if ros_msg.data_type_choice == ros_msg.DATATYPE_POD_TYPE_SET:
        convert_bosdyn_msgs_pod_type_descriptor_to_proto(ros_msg.pod_type, proto.pod_type)
    if ros_msg.data_type_choice == ros_msg.DATATYPE_STRUCT_TYPE_SET:
        convert_bosdyn_msgs_struct_type_descriptor_to_proto(ros_msg.struct_type, proto.struct_type)

def convert_proto_to_bosdyn_msgs_series_descriptor(proto, ros_msg):
    ros_msg.series_index = proto.series_index
    convert_proto_to_bosdyn_msgs_series_identifier(proto.series_identifier, ros_msg.series_identifier)
    ros_msg.series_identifier_is_set = proto.HasField("series_identifier")
    ros_msg.identifier_hash = proto.identifier_hash
    convert_proto_to_bosdyn_msgs_series_descriptor_one_of_data_type(proto, ros_msg.data_type)
    from bosdyn_msgs.msg import KeyStringValueString
    ros_msg.annotations = []
    for _item in proto.annotations:
        ros_msg.annotations.append(KeyStringValueString())
        ros_msg.annotations[-1].key = _item
        ros_msg.annotations[-1].value = proto.annotations[_item]
    ros_msg.additional_index_names = []
    for _item in proto.additional_index_names:
        ros_msg.additional_index_names.append(_item)
    ros_msg.description = proto.description

def convert_bosdyn_msgs_series_descriptor_to_proto(ros_msg, proto):
    proto.Clear()
    proto.series_index = ros_msg.series_index
    if ros_msg.series_identifier_is_set:
        convert_bosdyn_msgs_series_identifier_to_proto(ros_msg.series_identifier, proto.series_identifier)
    proto.identifier_hash = ros_msg.identifier_hash
    convert_bosdyn_msgs_series_descriptor_one_of_data_type_to_proto(ros_msg.data_type, proto)
    for _item in ros_msg.annotations:
        proto.annotations[_item.key] = _item.value
    del proto.additional_index_names[:]
    for _item in ros_msg.additional_index_names:
        proto.additional_index_names.add(_item)
    proto.description = ros_msg.description

def convert_proto_to_bosdyn_msgs_message_type_descriptor(proto, ros_msg):
    ros_msg.content_type = proto.content_type
    ros_msg.type_name = proto.type_name
    ros_msg.is_metadata = proto.is_metadata

def convert_bosdyn_msgs_message_type_descriptor_to_proto(ros_msg, proto):
    proto.Clear()
    proto.content_type = ros_msg.content_type
    proto.type_name = ros_msg.type_name
    proto.is_metadata = ros_msg.is_metadata

def convert_proto_to_bosdyn_msgs_pod_type_descriptor(proto, ros_msg):
    ros_msg.pod_type.value = proto.pod_type
    ros_msg.dimension = []
    for _item in proto.dimension:
        ros_msg.dimension.append(_item)

def convert_bosdyn_msgs_pod_type_descriptor_to_proto(ros_msg, proto):
    proto.Clear()
    proto.pod_type = ros_msg.pod_type.value
    del proto.dimension[:]
    for _item in ros_msg.dimension:
        proto.dimension.add(_item)

def convert_proto_to_bosdyn_msgs_struct_type_descriptor(proto, ros_msg):
    from bosdyn_msgs.msg import KeyStringValueUint64
    ros_msg.key_to_series_identifier_hash = []
    for _item in proto.key_to_series_identifier_hash:
        ros_msg.key_to_series_identifier_hash.append(KeyStringValueUint64())
        ros_msg.key_to_series_identifier_hash[-1].key = _item
        ros_msg.key_to_series_identifier_hash[-1].value = proto.key_to_series_identifier_hash[_item]

def convert_bosdyn_msgs_struct_type_descriptor_to_proto(ros_msg, proto):
    proto.Clear()
    for _item in ros_msg.key_to_series_identifier_hash:
        proto.key_to_series_identifier_hash[_item.key] = _item.value

def convert_proto_to_bosdyn_msgs_file_index(proto, ros_msg):
    from bosdyn_msgs.msg import SeriesIdentifier
    ros_msg.series_identifiers = []
    for _item in proto.series_identifiers:
        ros_msg.series_identifiers.append(SeriesIdentifier())
        convert_proto_to_bosdyn_msgs_series_identifier(_item, ros_msg.series_identifiers[-1])
    ros_msg.series_block_index_offsets = []
    for _item in proto.series_block_index_offsets:
        ros_msg.series_block_index_offsets.append(_item)
    ros_msg.series_identifier_hashes = []
    for _item in proto.series_identifier_hashes:
        ros_msg.series_identifier_hashes.append(_item)

def convert_bosdyn_msgs_file_index_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.series_identifiers[:]
    for _item in ros_msg.series_identifiers:
        convert_bosdyn_msgs_series_identifier_to_proto(_item, proto.series_identifiers.add())
    del proto.series_block_index_offsets[:]
    for _item in ros_msg.series_block_index_offsets:
        proto.series_block_index_offsets.add(_item)
    del proto.series_identifier_hashes[:]
    for _item in ros_msg.series_identifier_hashes:
        proto.series_identifier_hashes.add(_item)

def convert_proto_to_bosdyn_msgs_series_block_index_block_entry(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    ros_msg.file_offset = proto.file_offset
    ros_msg.additional_indexes = []
    for _item in proto.additional_indexes:
        ros_msg.additional_indexes.append(_item)

def convert_bosdyn_msgs_series_block_index_block_entry_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    proto.file_offset = ros_msg.file_offset
    del proto.additional_indexes[:]
    for _item in ros_msg.additional_indexes:
        proto.additional_indexes.add(_item)

def convert_proto_to_bosdyn_msgs_series_block_index(proto, ros_msg):
    ros_msg.series_index = proto.series_index
    ros_msg.descriptor_file_offset = proto.descriptor_file_offset
    from bosdyn_msgs.msg import BlockEntry
    ros_msg.block_entries = []
    for _item in proto.block_entries:
        ros_msg.block_entries.append(BlockEntry())
        convert_proto_to_bosdyn_msgs_series_block_index_block_entry(_item, ros_msg.block_entries[-1])
    ros_msg.total_bytes = proto.total_bytes

def convert_bosdyn_msgs_series_block_index_to_proto(ros_msg, proto):
    proto.Clear()
    proto.series_index = ros_msg.series_index
    proto.descriptor_file_offset = ros_msg.descriptor_file_offset
    del proto.block_entries[:]
    for _item in ros_msg.block_entries:
        convert_bosdyn_msgs_series_block_index_block_entry_to_proto(_item, proto.block_entries.add())
    proto.total_bytes = ros_msg.total_bytes

def convert_proto_to_bosdyn_msgs_series_identifier(proto, ros_msg):
    ros_msg.series_type = proto.series_type
    from bosdyn_msgs.msg import KeyStringValueString
    ros_msg.spec = []
    for _item in proto.spec:
        ros_msg.spec.append(KeyStringValueString())
        ros_msg.spec[-1].key = _item
        ros_msg.spec[-1].value = proto.spec[_item]

def convert_bosdyn_msgs_series_identifier_to_proto(ros_msg, proto):
    proto.Clear()
    proto.series_type = ros_msg.series_type
    for _item in ros_msg.spec:
        proto.spec[_item.key] = _item.value

def convert_proto_to_bosdyn_msgs_params(proto, ros_msg):
    ros_msg.max_displacement = proto.max_displacement
    convert_proto_to_builtin_interfaces_duration(proto.max_duration, ros_msg.max_duration)
    ros_msg.max_duration_is_set = proto.HasField("max_duration")

def convert_bosdyn_msgs_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.max_displacement = ros_msg.max_displacement
    if ros_msg.max_duration_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.max_duration, proto.max_duration)

def convert_proto_to_bosdyn_msgs_configure_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    convert_proto_to_bosdyn_msgs_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")
    ros_msg.clear_buffer = proto.clear_buffer

def convert_bosdyn_msgs_configure_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_params_to_proto(ros_msg.params, proto.params)
    proto.clear_buffer = ros_msg.clear_buffer

def convert_proto_to_bosdyn_msgs_configure_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_params(proto.invalid_params, ros_msg.invalid_params)
    ros_msg.invalid_params_is_set = proto.HasField("invalid_params")

def convert_bosdyn_msgs_configure_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.invalid_params_is_set:
        convert_bosdyn_msgs_params_to_proto(ros_msg.invalid_params, proto.invalid_params)

def convert_proto_to_bosdyn_msgs_get_configuration_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_configuration_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_configuration_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.enabled = proto.enabled
    convert_proto_to_bosdyn_msgs_configure_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")

def convert_bosdyn_msgs_get_configuration_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.enabled = ros_msg.enabled
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_configure_request_to_proto(ros_msg.request, proto.request)

def convert_proto_to_bosdyn_msgs_start_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_start_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_start_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_start_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_area_callback_error_one_of_response_error(proto, ros_msg):
    if proto.HasField("begin_callback"):
        ros_msg.response_error_choice = ros_msg.RESPONSE_ERROR_BEGIN_CALLBACK_SET
        convert_proto_to_bosdyn_msgs_begin_callback_response(proto.begin_callback, ros_msg.begin_callback)
    if proto.HasField("begin_control"):
        ros_msg.response_error_choice = ros_msg.RESPONSE_ERROR_BEGIN_CONTROL_SET
        convert_proto_to_bosdyn_msgs_begin_control_response(proto.begin_control, ros_msg.begin_control)
    if proto.HasField("update_callback"):
        ros_msg.response_error_choice = ros_msg.RESPONSE_ERROR_UPDATE_CALLBACK_SET
        convert_proto_to_bosdyn_msgs_update_callback_response(proto.update_callback, ros_msg.update_callback)
    if proto.HasField("end_callback"):
        ros_msg.response_error_choice = ros_msg.RESPONSE_ERROR_END_CALLBACK_SET
        convert_proto_to_bosdyn_msgs_end_callback_response(proto.end_callback, ros_msg.end_callback)

def convert_bosdyn_msgs_area_callback_error_one_of_response_error_to_proto(ros_msg, proto):
    proto.ClearField("response_error")
    if ros_msg.response_error_choice == ros_msg.RESPONSE_ERROR_BEGIN_CALLBACK_SET:
        convert_bosdyn_msgs_begin_callback_response_to_proto(ros_msg.begin_callback, proto.begin_callback)
    if ros_msg.response_error_choice == ros_msg.RESPONSE_ERROR_BEGIN_CONTROL_SET:
        convert_bosdyn_msgs_begin_control_response_to_proto(ros_msg.begin_control, proto.begin_control)
    if ros_msg.response_error_choice == ros_msg.RESPONSE_ERROR_UPDATE_CALLBACK_SET:
        convert_bosdyn_msgs_update_callback_response_to_proto(ros_msg.update_callback, proto.update_callback)
    if ros_msg.response_error_choice == ros_msg.RESPONSE_ERROR_END_CALLBACK_SET:
        convert_bosdyn_msgs_end_callback_response_to_proto(ros_msg.end_callback, proto.end_callback)

def convert_proto_to_bosdyn_msgs_area_callback_error(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.error.value = proto.error
    convert_proto_to_bosdyn_msgs_area_callback_error_one_of_response_error(proto, ros_msg.response_error)

def convert_bosdyn_msgs_area_callback_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.error = ros_msg.error.value
    convert_bosdyn_msgs_area_callback_error_one_of_response_error_to_proto(ros_msg.response_error, proto)

def convert_proto_to_bosdyn_msgs_area_callback_information_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_area_callback_information_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_area_callback_information(proto, ros_msg):
    ros_msg.required_lease_resources = []
    for _item in proto.required_lease_resources:
        ros_msg.required_lease_resources.append(_item)

def convert_bosdyn_msgs_area_callback_information_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.required_lease_resources[:]
    for _item in ros_msg.required_lease_resources:
        proto.required_lease_resources.add(_item)

def convert_proto_to_bosdyn_msgs_area_callback_information_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_area_callback_information(proto.info, ros_msg.info)
    ros_msg.info_is_set = proto.HasField("info")

def convert_bosdyn_msgs_area_callback_information_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.info_is_set:
        convert_bosdyn_msgs_area_callback_information_to_proto(ros_msg.info, proto.info)

def convert_proto_to_bosdyn_msgs_region_information(proto, ros_msg):
    ros_msg.region_id = proto.region_id
    ros_msg.description = proto.description
    convert_proto_to_bosdyn_msgs_route(proto.route, ros_msg.route)
    ros_msg.route_is_set = proto.HasField("route")

def convert_bosdyn_msgs_region_information_to_proto(ros_msg, proto):
    proto.Clear()
    proto.region_id = ros_msg.region_id
    proto.description = ros_msg.description
    if ros_msg.route_is_set:
        convert_bosdyn_msgs_route_to_proto(ros_msg.route, proto.route)

def convert_proto_to_bosdyn_msgs_begin_callback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_region_information(proto.region_info, ros_msg.region_info)
    ros_msg.region_info_is_set = proto.HasField("region_info")
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")

def convert_bosdyn_msgs_begin_callback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.region_info_is_set:
        convert_bosdyn_msgs_region_information_to_proto(ros_msg.region_info, proto.region_info)
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)

def convert_proto_to_bosdyn_msgs_begin_callback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_begin_callback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_begin_control_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_begin_control_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_begin_control_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_begin_control_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_update_callback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.command_id = proto.command_id
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.stage.value = proto.stage

def convert_bosdyn_msgs_update_callback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.command_id = ros_msg.command_id
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    proto.stage = ros_msg.stage.value

def convert_proto_to_bosdyn_msgs_update_callback_response_one_of_response(proto, ros_msg):
    if proto.HasField("policy"):
        ros_msg.response_choice = ros_msg.RESPONSE_POLICY_SET
        convert_proto_to_bosdyn_msgs_update_callback_response_nav_policy(proto.policy, ros_msg.policy)
    if proto.HasField("error"):
        ros_msg.response_choice = ros_msg.RESPONSE_ERROR_SET
        convert_proto_to_bosdyn_msgs_update_callback_response_error(proto.error, ros_msg.error)
    if proto.HasField("complete"):
        ros_msg.response_choice = ros_msg.RESPONSE_COMPLETE_SET
        convert_proto_to_bosdyn_msgs_update_callback_response_complete(proto.complete, ros_msg.complete)

def convert_bosdyn_msgs_update_callback_response_one_of_response_to_proto(ros_msg, proto):
    proto.ClearField("response")
    if ros_msg.response_choice == ros_msg.RESPONSE_POLICY_SET:
        convert_bosdyn_msgs_update_callback_response_nav_policy_to_proto(ros_msg.policy, proto.policy)
    if ros_msg.response_choice == ros_msg.RESPONSE_ERROR_SET:
        convert_bosdyn_msgs_update_callback_response_error_to_proto(ros_msg.error, proto.error)
    if ros_msg.response_choice == ros_msg.RESPONSE_COMPLETE_SET:
        convert_bosdyn_msgs_update_callback_response_complete_to_proto(ros_msg.complete, proto.complete)

def convert_proto_to_bosdyn_msgs_update_callback_response_nav_policy(proto, ros_msg):
    ros_msg.at_start.value = proto.at_start
    ros_msg.at_end.value = proto.at_end

def convert_bosdyn_msgs_update_callback_response_nav_policy_to_proto(ros_msg, proto):
    proto.Clear()
    proto.at_start = ros_msg.at_start.value
    proto.at_end = ros_msg.at_end.value

def convert_proto_to_bosdyn_msgs_update_callback_response_error(proto, ros_msg):
    ros_msg.error.value = proto.error
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])

def convert_bosdyn_msgs_update_callback_response_error_to_proto(ros_msg, proto):
    proto.Clear()
    proto.error = ros_msg.error.value
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())

def convert_bosdyn_msgs_update_callback_response_complete_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_update_callback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_update_callback_response_one_of_response(proto, ros_msg.response)

def convert_bosdyn_msgs_update_callback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    convert_bosdyn_msgs_update_callback_response_one_of_response_to_proto(ros_msg.response, proto)

def convert_proto_to_bosdyn_msgs_end_callback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_end_callback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_end_callback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_end_callback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_waypoint_annotations_localize_region_default_to_proto(ros_msg, proto):
    proto.Clear()

def convert_bosdyn_msgs_waypoint_annotations_localize_region_empty_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_waypoint_annotations_localize_region_circle2_d(proto, ros_msg):
    ros_msg.dist_2d = proto.dist_2d

def convert_bosdyn_msgs_waypoint_annotations_localize_region_circle2_d_to_proto(ros_msg, proto):
    proto.Clear()
    proto.dist_2d = ros_msg.dist_2d

def convert_proto_to_bosdyn_msgs_waypoint_annotations_localize_region_one_of_region(proto, ros_msg):
    if proto.HasField("default_region"):
        ros_msg.region_choice = ros_msg.REGION_DEFAULT_REGION_SET
        convert_proto_to_bosdyn_msgs_waypoint_annotations_localize_region_default(proto.default_region, ros_msg.default_region)
    if proto.HasField("empty"):
        ros_msg.region_choice = ros_msg.REGION_EMPTY_SET
        convert_proto_to_bosdyn_msgs_waypoint_annotations_localize_region_empty(proto.empty, ros_msg.empty)
    if proto.HasField("circle"):
        ros_msg.region_choice = ros_msg.REGION_CIRCLE_SET
        convert_proto_to_bosdyn_msgs_waypoint_annotations_localize_region_circle2_d(proto.circle, ros_msg.circle)

def convert_bosdyn_msgs_waypoint_annotations_localize_region_one_of_region_to_proto(ros_msg, proto):
    proto.ClearField("region")
    if ros_msg.region_choice == ros_msg.REGION_DEFAULT_REGION_SET:
        convert_bosdyn_msgs_waypoint_annotations_localize_region_default_to_proto(ros_msg.default_region, proto.default_region)
    if ros_msg.region_choice == ros_msg.REGION_EMPTY_SET:
        convert_bosdyn_msgs_waypoint_annotations_localize_region_empty_to_proto(ros_msg.empty, proto.empty)
    if ros_msg.region_choice == ros_msg.REGION_CIRCLE_SET:
        convert_bosdyn_msgs_waypoint_annotations_localize_region_circle2_d_to_proto(ros_msg.circle, proto.circle)

def convert_proto_to_bosdyn_msgs_waypoint_annotations_localize_region(proto, ros_msg):
    ros_msg.state.value = proto.state
    convert_proto_to_bosdyn_msgs_waypoint_annotations_localize_region_one_of_region(proto, ros_msg.region)

def convert_bosdyn_msgs_waypoint_annotations_localize_region_to_proto(ros_msg, proto):
    proto.Clear()
    proto.state = ros_msg.state.value
    convert_bosdyn_msgs_waypoint_annotations_localize_region_one_of_region_to_proto(ros_msg.region, proto)

def convert_proto_to_bosdyn_msgs_waypoint_annotations(proto, ros_msg):
    ros_msg.name = proto.name
    convert_proto_to_builtin_interfaces_time(proto.creation_time, ros_msg.creation_time)
    ros_msg.creation_time_is_set = proto.HasField("creation_time")
    convert_proto_to_bosdyn_msgs_se3_covariance(proto.icp_variance, ros_msg.icp_variance)
    ros_msg.icp_variance_is_set = proto.HasField("icp_variance")
    convert_proto_to_bosdyn_msgs_waypoint_annotations_localize_region(proto.scan_match_region, ros_msg.scan_match_region)
    ros_msg.scan_match_region_is_set = proto.HasField("scan_match_region")
    ros_msg.waypoint_source.value = proto.waypoint_source
    convert_proto_to_bosdyn_msgs_client_metadata(proto.client_metadata, ros_msg.client_metadata)
    ros_msg.client_metadata_is_set = proto.HasField("client_metadata")

def convert_bosdyn_msgs_waypoint_annotations_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    if ros_msg.creation_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.creation_time, proto.creation_time)
    if ros_msg.icp_variance_is_set:
        convert_bosdyn_msgs_se3_covariance_to_proto(ros_msg.icp_variance, proto.icp_variance)
    if ros_msg.scan_match_region_is_set:
        convert_bosdyn_msgs_waypoint_annotations_localize_region_to_proto(ros_msg.scan_match_region, proto.scan_match_region)
    proto.waypoint_source = ros_msg.waypoint_source.value
    if ros_msg.client_metadata_is_set:
        convert_bosdyn_msgs_client_metadata_to_proto(ros_msg.client_metadata, proto.client_metadata)

def convert_proto_to_bosdyn_msgs_waypoint(proto, ros_msg):
    ros_msg.id = proto.id
    ros_msg.snapshot_id = proto.snapshot_id
    convert_proto_to_geometry_msgs_pose(proto.waypoint_tform_ko, ros_msg.waypoint_tform_ko)
    ros_msg.waypoint_tform_ko_is_set = proto.HasField("waypoint_tform_ko")
    convert_proto_to_bosdyn_msgs_waypoint_annotations(proto.annotations, ros_msg.annotations)
    ros_msg.annotations_is_set = proto.HasField("annotations")

def convert_bosdyn_msgs_waypoint_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    proto.snapshot_id = ros_msg.snapshot_id
    if ros_msg.waypoint_tform_ko_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.waypoint_tform_ko, proto.waypoint_tform_ko)
    if ros_msg.annotations_is_set:
        convert_bosdyn_msgs_waypoint_annotations_to_proto(ros_msg.annotations, proto.annotations)

def convert_proto_to_bosdyn_msgs_client_metadata(proto, ros_msg):
    ros_msg.session_name = proto.session_name
    ros_msg.client_username = proto.client_username
    ros_msg.client_software_version = proto.client_software_version
    ros_msg.client_id = proto.client_id
    ros_msg.client_type = proto.client_type

def convert_bosdyn_msgs_client_metadata_to_proto(ros_msg, proto):
    proto.Clear()
    proto.session_name = ros_msg.session_name
    proto.client_username = ros_msg.client_username
    proto.client_software_version = ros_msg.client_software_version
    proto.client_id = ros_msg.client_id
    proto.client_type = ros_msg.client_type

def convert_proto_to_bosdyn_msgs_waypoint_snapshot(proto, ros_msg):
    ros_msg.id = proto.id
    from bosdyn_msgs.msg import ImageResponse
    ros_msg.images = []
    for _item in proto.images:
        ros_msg.images.append(ImageResponse())
        convert_proto_to_bosdyn_msgs_image_response(_item, ros_msg.images[-1])
    convert_proto_to_bosdyn_msgs_point_cloud(proto.point_cloud, ros_msg.point_cloud)
    ros_msg.point_cloud_is_set = proto.HasField("point_cloud")
    from bosdyn_msgs.msg import WorldObject
    ros_msg.objects = []
    for _item in proto.objects:
        ros_msg.objects.append(WorldObject())
        convert_proto_to_bosdyn_msgs_world_object(_item, ros_msg.objects[-1])
    convert_proto_to_bosdyn_msgs_robot_state(proto.robot_state, ros_msg.robot_state)
    ros_msg.robot_state_is_set = proto.HasField("robot_state")
    from bosdyn_msgs.msg import LocalGrid
    ros_msg.robot_local_grids = []
    for _item in proto.robot_local_grids:
        ros_msg.robot_local_grids.append(LocalGrid())
        convert_proto_to_bosdyn_msgs_local_grid(_item, ros_msg.robot_local_grids[-1])
    ros_msg.is_point_cloud_processed = proto.is_point_cloud_processed
    ros_msg.version_id = proto.version_id
    ros_msg.has_remote_point_cloud_sensor = proto.has_remote_point_cloud_sensor
    convert_proto_to_geometry_msgs_pose(proto.body_tform_remote_point_cloud_sensor, ros_msg.body_tform_remote_point_cloud_sensor)
    ros_msg.body_tform_remote_point_cloud_sensor_is_set = proto.HasField("body_tform_remote_point_cloud_sensor")
    from bosdyn_msgs.msg import Payload
    ros_msg.payloads = []
    for _item in proto.payloads:
        ros_msg.payloads.append(Payload())
        convert_proto_to_bosdyn_msgs_payload(_item, ros_msg.payloads[-1])
    convert_proto_to_bosdyn_msgs_robot_id(proto.robot_id, ros_msg.robot_id)
    ros_msg.robot_id_is_set = proto.HasField("robot_id")
    convert_proto_to_builtin_interfaces_time(proto.recording_started_on, ros_msg.recording_started_on)
    ros_msg.recording_started_on_is_set = proto.HasField("recording_started_on")

def convert_bosdyn_msgs_waypoint_snapshot_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    del proto.images[:]
    for _item in ros_msg.images:
        convert_bosdyn_msgs_image_response_to_proto(_item, proto.images.add())
    if ros_msg.point_cloud_is_set:
        convert_bosdyn_msgs_point_cloud_to_proto(ros_msg.point_cloud, proto.point_cloud)
    del proto.objects[:]
    for _item in ros_msg.objects:
        convert_bosdyn_msgs_world_object_to_proto(_item, proto.objects.add())
    if ros_msg.robot_state_is_set:
        convert_bosdyn_msgs_robot_state_to_proto(ros_msg.robot_state, proto.robot_state)
    del proto.robot_local_grids[:]
    for _item in ros_msg.robot_local_grids:
        convert_bosdyn_msgs_local_grid_to_proto(_item, proto.robot_local_grids.add())
    proto.is_point_cloud_processed = ros_msg.is_point_cloud_processed
    proto.version_id = ros_msg.version_id
    proto.has_remote_point_cloud_sensor = ros_msg.has_remote_point_cloud_sensor
    if ros_msg.body_tform_remote_point_cloud_sensor_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.body_tform_remote_point_cloud_sensor, proto.body_tform_remote_point_cloud_sensor)
    del proto.payloads[:]
    for _item in ros_msg.payloads:
        convert_bosdyn_msgs_payload_to_proto(_item, proto.payloads.add())
    if ros_msg.robot_id_is_set:
        convert_bosdyn_msgs_robot_id_to_proto(ros_msg.robot_id, proto.robot_id)
    if ros_msg.recording_started_on_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.recording_started_on, proto.recording_started_on)

def convert_proto_to_bosdyn_msgs_edge_id(proto, ros_msg):
    ros_msg.from_waypoint = proto.from_waypoint
    ros_msg.to_waypoint = proto.to_waypoint

def convert_bosdyn_msgs_edge_id_to_proto(ros_msg, proto):
    proto.Clear()
    proto.from_waypoint = ros_msg.from_waypoint
    proto.to_waypoint = ros_msg.to_waypoint

def convert_proto_to_bosdyn_msgs_edge_annotations_stair_data(proto, ros_msg):
    ros_msg.state.value = proto.state
    convert_proto_to_bosdyn_msgs_straight_staircase(proto.straight_staircase, ros_msg.straight_staircase)
    ros_msg.straight_staircase_is_set = proto.HasField("straight_staircase")

def convert_bosdyn_msgs_edge_annotations_stair_data_to_proto(ros_msg, proto):
    proto.Clear()
    proto.state = ros_msg.state.value
    if ros_msg.straight_staircase_is_set:
        convert_bosdyn_msgs_straight_staircase_to_proto(ros_msg.straight_staircase, proto.straight_staircase)

def convert_proto_to_bosdyn_msgs_edge_annotations(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_edge_annotations_stair_data(proto.stairs, ros_msg.stairs)
    ros_msg.stairs_is_set = proto.HasField("stairs")
    ros_msg.direction_constraint.value = proto.direction_constraint
    ros_msg.require_alignment = proto.require_alignment.value
    ros_msg.require_alignment_is_set = proto.HasField("require_alignment")
    ros_msg.flat_ground = proto.flat_ground.value
    ros_msg.flat_ground_is_set = proto.HasField("flat_ground")
    convert_proto_to_bosdyn_msgs_mobility_params(proto.mobility_params, ros_msg.mobility_params)
    ros_msg.mobility_params_is_set = proto.HasField("mobility_params")
    ros_msg.cost = proto.cost.value
    ros_msg.cost_is_set = proto.HasField("cost")
    ros_msg.edge_source.value = proto.edge_source
    ros_msg.disable_alternate_route_finding = proto.disable_alternate_route_finding
    ros_msg.path_following_mode.value = proto.path_following_mode
    ros_msg.disable_directed_exploration = proto.disable_directed_exploration
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsAreaCallbackRegion
    ros_msg.area_callbacks = []
    for _item in proto.area_callbacks:
        ros_msg.area_callbacks.append(KeyStringValueBosdynMsgsAreaCallbackRegion())
        ros_msg.area_callbacks[-1].key = _item
        convert_proto_to_bosdyn_msgs_area_callback_region(proto.area_callbacks[_item], ros_msg.area_callbacks[-1].value)
    ros_msg.ground_clutter_mode.value = proto.ground_clutter_mode

def convert_bosdyn_msgs_edge_annotations_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.stairs_is_set:
        convert_bosdyn_msgs_edge_annotations_stair_data_to_proto(ros_msg.stairs, proto.stairs)
    proto.direction_constraint = ros_msg.direction_constraint.value
    if ros_msg.require_alignment_is_set:
        convert_bool_to_proto(ros_msg.require_alignment, proto.require_alignment)
    if ros_msg.flat_ground_is_set:
        convert_bool_to_proto(ros_msg.flat_ground, proto.flat_ground)
    if ros_msg.mobility_params_is_set:
        convert_bosdyn_msgs_mobility_params_to_proto(ros_msg.mobility_params, proto.mobility_params)
    if ros_msg.cost_is_set:
        convert_float64_to_proto(ros_msg.cost, proto.cost)
    proto.edge_source = ros_msg.edge_source.value
    proto.disable_alternate_route_finding = ros_msg.disable_alternate_route_finding
    proto.path_following_mode = ros_msg.path_following_mode.value
    proto.disable_directed_exploration = ros_msg.disable_directed_exploration
    for _item in ros_msg.area_callbacks:
        convert_bosdyn_msgs_area_callback_region_to_proto(_item.value, proto.area_callbacks[_item.key])
    proto.ground_clutter_mode = ros_msg.ground_clutter_mode.value

def convert_proto_to_bosdyn_msgs_edge(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_edge_id(proto.id, ros_msg.id)
    ros_msg.id_is_set = proto.HasField("id")
    ros_msg.snapshot_id = proto.snapshot_id
    convert_proto_to_geometry_msgs_pose(proto.from_tform_to, ros_msg.from_tform_to)
    ros_msg.from_tform_to_is_set = proto.HasField("from_tform_to")
    convert_proto_to_bosdyn_msgs_edge_annotations(proto.annotations, ros_msg.annotations)
    ros_msg.annotations_is_set = proto.HasField("annotations")

def convert_bosdyn_msgs_edge_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.id_is_set:
        convert_bosdyn_msgs_edge_id_to_proto(ros_msg.id, proto.id)
    proto.snapshot_id = ros_msg.snapshot_id
    if ros_msg.from_tform_to_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.from_tform_to, proto.from_tform_to)
    if ros_msg.annotations_is_set:
        convert_bosdyn_msgs_edge_annotations_to_proto(ros_msg.annotations, proto.annotations)

def convert_proto_to_bosdyn_msgs_edge_snapshot_stance(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    from bosdyn_msgs.msg import FootState
    ros_msg.foot_states = []
    for _item in proto.foot_states:
        ros_msg.foot_states.append(FootState())
        convert_proto_to_bosdyn_msgs_foot_state(_item, ros_msg.foot_states[-1])
    convert_proto_to_geometry_msgs_pose(proto.ko_tform_body, ros_msg.ko_tform_body)
    ros_msg.ko_tform_body_is_set = proto.HasField("ko_tform_body")
    convert_proto_to_geometry_msgs_pose(proto.vision_tform_body, ros_msg.vision_tform_body)
    ros_msg.vision_tform_body_is_set = proto.HasField("vision_tform_body")
    ros_msg.planar_ground = proto.planar_ground.value
    ros_msg.planar_ground_is_set = proto.HasField("planar_ground")

def convert_bosdyn_msgs_edge_snapshot_stance_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    del proto.foot_states[:]
    for _item in ros_msg.foot_states:
        convert_bosdyn_msgs_foot_state_to_proto(_item, proto.foot_states.add())
    if ros_msg.ko_tform_body_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.ko_tform_body, proto.ko_tform_body)
    if ros_msg.vision_tform_body_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.vision_tform_body, proto.vision_tform_body)
    if ros_msg.planar_ground_is_set:
        convert_bool_to_proto(ros_msg.planar_ground, proto.planar_ground)

def convert_proto_to_bosdyn_msgs_edge_snapshot(proto, ros_msg):
    ros_msg.id = proto.id
    from bosdyn_msgs.msg import Stance
    ros_msg.stances = []
    for _item in proto.stances:
        ros_msg.stances.append(Stance())
        convert_proto_to_bosdyn_msgs_edge_snapshot_stance(_item, ros_msg.stances[-1])
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsAreaCallbackData
    ros_msg.area_callbacks = []
    for _item in proto.area_callbacks:
        ros_msg.area_callbacks.append(KeyStringValueBosdynMsgsAreaCallbackData())
        ros_msg.area_callbacks[-1].key = _item
        convert_proto_to_bosdyn_msgs_area_callback_data(proto.area_callbacks[_item], ros_msg.area_callbacks[-1].value)

def convert_bosdyn_msgs_edge_snapshot_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    del proto.stances[:]
    for _item in ros_msg.stances:
        convert_bosdyn_msgs_edge_snapshot_stance_to_proto(_item, proto.stances.add())
    for _item in ros_msg.area_callbacks:
        convert_bosdyn_msgs_area_callback_data_to_proto(_item.value, proto.area_callbacks[_item.key])

def convert_proto_to_bosdyn_msgs_anchor(proto, ros_msg):
    ros_msg.id = proto.id
    convert_proto_to_geometry_msgs_pose(proto.seed_tform_waypoint, ros_msg.seed_tform_waypoint)
    ros_msg.seed_tform_waypoint_is_set = proto.HasField("seed_tform_waypoint")

def convert_bosdyn_msgs_anchor_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    if ros_msg.seed_tform_waypoint_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.seed_tform_waypoint, proto.seed_tform_waypoint)

def convert_proto_to_bosdyn_msgs_anchored_world_object(proto, ros_msg):
    ros_msg.id = proto.id
    convert_proto_to_geometry_msgs_pose(proto.seed_tform_object, ros_msg.seed_tform_object)
    ros_msg.seed_tform_object_is_set = proto.HasField("seed_tform_object")

def convert_bosdyn_msgs_anchored_world_object_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    if ros_msg.seed_tform_object_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.seed_tform_object, proto.seed_tform_object)

def convert_proto_to_bosdyn_msgs_anchoring(proto, ros_msg):
    from bosdyn_msgs.msg import Anchor
    ros_msg.anchors = []
    for _item in proto.anchors:
        ros_msg.anchors.append(Anchor())
        convert_proto_to_bosdyn_msgs_anchor(_item, ros_msg.anchors[-1])
    from bosdyn_msgs.msg import AnchoredWorldObject
    ros_msg.objects = []
    for _item in proto.objects:
        ros_msg.objects.append(AnchoredWorldObject())
        convert_proto_to_bosdyn_msgs_anchored_world_object(_item, ros_msg.objects[-1])

def convert_bosdyn_msgs_anchoring_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.anchors[:]
    for _item in ros_msg.anchors:
        convert_bosdyn_msgs_anchor_to_proto(_item, proto.anchors.add())
    del proto.objects[:]
    for _item in ros_msg.objects:
        convert_bosdyn_msgs_anchored_world_object_to_proto(_item, proto.objects.add())

def convert_proto_to_bosdyn_msgs_area_callback_region(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.description = proto.description

def convert_bosdyn_msgs_area_callback_region_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.description = ros_msg.description

def convert_bosdyn_msgs_area_callback_data_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_graph(proto, ros_msg):
    from bosdyn_msgs.msg import Waypoint
    ros_msg.waypoints = []
    for _item in proto.waypoints:
        ros_msg.waypoints.append(Waypoint())
        convert_proto_to_bosdyn_msgs_waypoint(_item, ros_msg.waypoints[-1])
    from bosdyn_msgs.msg import Edge
    ros_msg.edges = []
    for _item in proto.edges:
        ros_msg.edges.append(Edge())
        convert_proto_to_bosdyn_msgs_edge(_item, ros_msg.edges[-1])
    convert_proto_to_bosdyn_msgs_anchoring(proto.anchoring, ros_msg.anchoring)
    ros_msg.anchoring_is_set = proto.HasField("anchoring")

def convert_bosdyn_msgs_graph_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.waypoints[:]
    for _item in ros_msg.waypoints:
        convert_bosdyn_msgs_waypoint_to_proto(_item, proto.waypoints.add())
    del proto.edges[:]
    for _item in ros_msg.edges:
        convert_bosdyn_msgs_edge_to_proto(_item, proto.edges.add())
    if ros_msg.anchoring_is_set:
        convert_bosdyn_msgs_anchoring_to_proto(ros_msg.anchoring, proto.anchoring)

def convert_proto_to_bosdyn_msgs_process_topology_request_icp_params(proto, ros_msg):
    ros_msg.icp_iters = proto.icp_iters.value
    ros_msg.icp_iters_is_set = proto.HasField("icp_iters")
    ros_msg.max_point_match_distance = proto.max_point_match_distance.value
    ros_msg.max_point_match_distance_is_set = proto.HasField("max_point_match_distance")

def convert_bosdyn_msgs_process_topology_request_icp_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.icp_iters_is_set:
        convert_int32_to_proto(ros_msg.icp_iters, proto.icp_iters)
    if ros_msg.max_point_match_distance_is_set:
        convert_float64_to_proto(ros_msg.max_point_match_distance, proto.max_point_match_distance)

def convert_proto_to_bosdyn_msgs_process_topology_request_odometry_loop_closure_params(proto, ros_msg):
    ros_msg.max_loop_closure_path_length = proto.max_loop_closure_path_length.value
    ros_msg.max_loop_closure_path_length_is_set = proto.HasField("max_loop_closure_path_length")
    ros_msg.min_loop_closure_path_length = proto.min_loop_closure_path_length.value
    ros_msg.min_loop_closure_path_length_is_set = proto.HasField("min_loop_closure_path_length")
    ros_msg.max_loop_closure_height_change = proto.max_loop_closure_height_change.value
    ros_msg.max_loop_closure_height_change_is_set = proto.HasField("max_loop_closure_height_change")
    ros_msg.max_loop_closure_edge_length = proto.max_loop_closure_edge_length.value
    ros_msg.max_loop_closure_edge_length_is_set = proto.HasField("max_loop_closure_edge_length")
    ros_msg.num_extra_loop_closure_iterations = proto.num_extra_loop_closure_iterations.value
    ros_msg.num_extra_loop_closure_iterations_is_set = proto.HasField("num_extra_loop_closure_iterations")

def convert_bosdyn_msgs_process_topology_request_odometry_loop_closure_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.max_loop_closure_path_length_is_set:
        convert_float64_to_proto(ros_msg.max_loop_closure_path_length, proto.max_loop_closure_path_length)
    if ros_msg.min_loop_closure_path_length_is_set:
        convert_float64_to_proto(ros_msg.min_loop_closure_path_length, proto.min_loop_closure_path_length)
    if ros_msg.max_loop_closure_height_change_is_set:
        convert_float64_to_proto(ros_msg.max_loop_closure_height_change, proto.max_loop_closure_height_change)
    if ros_msg.max_loop_closure_edge_length_is_set:
        convert_float64_to_proto(ros_msg.max_loop_closure_edge_length, proto.max_loop_closure_edge_length)
    if ros_msg.num_extra_loop_closure_iterations_is_set:
        convert_int32_to_proto(ros_msg.num_extra_loop_closure_iterations, proto.num_extra_loop_closure_iterations)

def convert_proto_to_bosdyn_msgs_process_topology_request_fiducial_loop_closure_params(proto, ros_msg):
    ros_msg.min_loop_closure_path_length = proto.min_loop_closure_path_length.value
    ros_msg.min_loop_closure_path_length_is_set = proto.HasField("min_loop_closure_path_length")
    ros_msg.max_loop_closure_edge_length = proto.max_loop_closure_edge_length.value
    ros_msg.max_loop_closure_edge_length_is_set = proto.HasField("max_loop_closure_edge_length")
    ros_msg.max_fiducial_distance = proto.max_fiducial_distance.value
    ros_msg.max_fiducial_distance_is_set = proto.HasField("max_fiducial_distance")
    ros_msg.max_loop_closure_height_change = proto.max_loop_closure_height_change.value
    ros_msg.max_loop_closure_height_change_is_set = proto.HasField("max_loop_closure_height_change")

def convert_bosdyn_msgs_process_topology_request_fiducial_loop_closure_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.min_loop_closure_path_length_is_set:
        convert_float64_to_proto(ros_msg.min_loop_closure_path_length, proto.min_loop_closure_path_length)
    if ros_msg.max_loop_closure_edge_length_is_set:
        convert_float64_to_proto(ros_msg.max_loop_closure_edge_length, proto.max_loop_closure_edge_length)
    if ros_msg.max_fiducial_distance_is_set:
        convert_float64_to_proto(ros_msg.max_fiducial_distance, proto.max_fiducial_distance)
    if ros_msg.max_loop_closure_height_change_is_set:
        convert_float64_to_proto(ros_msg.max_loop_closure_height_change, proto.max_loop_closure_height_change)

def convert_proto_to_bosdyn_msgs_process_topology_request_collision_checking_params(proto, ros_msg):
    ros_msg.check_edges_for_collision = proto.check_edges_for_collision.value
    ros_msg.check_edges_for_collision_is_set = proto.HasField("check_edges_for_collision")
    ros_msg.collision_check_robot_radius = proto.collision_check_robot_radius.value
    ros_msg.collision_check_robot_radius_is_set = proto.HasField("collision_check_robot_radius")
    ros_msg.collision_check_height_variation = proto.collision_check_height_variation.value
    ros_msg.collision_check_height_variation_is_set = proto.HasField("collision_check_height_variation")

def convert_bosdyn_msgs_process_topology_request_collision_checking_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.check_edges_for_collision_is_set:
        convert_bool_to_proto(ros_msg.check_edges_for_collision, proto.check_edges_for_collision)
    if ros_msg.collision_check_robot_radius_is_set:
        convert_float64_to_proto(ros_msg.collision_check_robot_radius, proto.collision_check_robot_radius)
    if ros_msg.collision_check_height_variation_is_set:
        convert_float64_to_proto(ros_msg.collision_check_height_variation, proto.collision_check_height_variation)

def convert_proto_to_bosdyn_msgs_process_topology_request_params(proto, ros_msg):
    ros_msg.do_odometry_loop_closure = proto.do_odometry_loop_closure.value
    ros_msg.do_odometry_loop_closure_is_set = proto.HasField("do_odometry_loop_closure")
    convert_proto_to_bosdyn_msgs_process_topology_request_odometry_loop_closure_params(proto.odometry_loop_closure_params, ros_msg.odometry_loop_closure_params)
    ros_msg.odometry_loop_closure_params_is_set = proto.HasField("odometry_loop_closure_params")
    convert_proto_to_bosdyn_msgs_process_topology_request_icp_params(proto.icp_params, ros_msg.icp_params)
    ros_msg.icp_params_is_set = proto.HasField("icp_params")
    ros_msg.do_fiducial_loop_closure = proto.do_fiducial_loop_closure.value
    ros_msg.do_fiducial_loop_closure_is_set = proto.HasField("do_fiducial_loop_closure")
    convert_proto_to_bosdyn_msgs_process_topology_request_fiducial_loop_closure_params(proto.fiducial_loop_closure_params, ros_msg.fiducial_loop_closure_params)
    ros_msg.fiducial_loop_closure_params_is_set = proto.HasField("fiducial_loop_closure_params")
    convert_proto_to_bosdyn_msgs_process_topology_request_collision_checking_params(proto.collision_check_params, ros_msg.collision_check_params)
    ros_msg.collision_check_params_is_set = proto.HasField("collision_check_params")
    ros_msg.timeout_seconds = proto.timeout_seconds

def convert_bosdyn_msgs_process_topology_request_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.do_odometry_loop_closure_is_set:
        convert_bool_to_proto(ros_msg.do_odometry_loop_closure, proto.do_odometry_loop_closure)
    if ros_msg.odometry_loop_closure_params_is_set:
        convert_bosdyn_msgs_process_topology_request_odometry_loop_closure_params_to_proto(ros_msg.odometry_loop_closure_params, proto.odometry_loop_closure_params)
    if ros_msg.icp_params_is_set:
        convert_bosdyn_msgs_process_topology_request_icp_params_to_proto(ros_msg.icp_params, proto.icp_params)
    if ros_msg.do_fiducial_loop_closure_is_set:
        convert_bool_to_proto(ros_msg.do_fiducial_loop_closure, proto.do_fiducial_loop_closure)
    if ros_msg.fiducial_loop_closure_params_is_set:
        convert_bosdyn_msgs_process_topology_request_fiducial_loop_closure_params_to_proto(ros_msg.fiducial_loop_closure_params, proto.fiducial_loop_closure_params)
    if ros_msg.collision_check_params_is_set:
        convert_bosdyn_msgs_process_topology_request_collision_checking_params_to_proto(ros_msg.collision_check_params, proto.collision_check_params)
    proto.timeout_seconds = ros_msg.timeout_seconds

def convert_proto_to_bosdyn_msgs_process_topology_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_process_topology_request_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")
    ros_msg.modify_map_on_server = proto.modify_map_on_server

def convert_bosdyn_msgs_process_topology_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_process_topology_request_params_to_proto(ros_msg.params, proto.params)
    proto.modify_map_on_server = ros_msg.modify_map_on_server

def convert_proto_to_bosdyn_msgs_process_topology_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_graph(proto.new_subgraph, ros_msg.new_subgraph)
    ros_msg.new_subgraph_is_set = proto.HasField("new_subgraph")
    ros_msg.map_on_server_was_modified = proto.map_on_server_was_modified
    ros_msg.missing_snapshot_ids = []
    for _item in proto.missing_snapshot_ids:
        ros_msg.missing_snapshot_ids.append(_item)
    ros_msg.missing_waypoint_ids = []
    for _item in proto.missing_waypoint_ids:
        ros_msg.missing_waypoint_ids.append(_item)
    ros_msg.timed_out = proto.timed_out

def convert_bosdyn_msgs_process_topology_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.new_subgraph_is_set:
        convert_bosdyn_msgs_graph_to_proto(ros_msg.new_subgraph, proto.new_subgraph)
    proto.map_on_server_was_modified = ros_msg.map_on_server_was_modified
    del proto.missing_snapshot_ids[:]
    for _item in ros_msg.missing_snapshot_ids:
        proto.missing_snapshot_ids.add(_item)
    del proto.missing_waypoint_ids[:]
    for _item in ros_msg.missing_waypoint_ids:
        proto.missing_waypoint_ids.add(_item)
    proto.timed_out = ros_msg.timed_out

def convert_proto_to_bosdyn_msgs_pose_bounds(proto, ros_msg):
    ros_msg.x_bounds = proto.x_bounds
    ros_msg.y_bounds = proto.y_bounds
    ros_msg.z_bounds = proto.z_bounds
    ros_msg.yaw_bounds = proto.yaw_bounds

def convert_bosdyn_msgs_pose_bounds_to_proto(ros_msg, proto):
    proto.Clear()
    proto.x_bounds = ros_msg.x_bounds
    proto.y_bounds = ros_msg.y_bounds
    proto.z_bounds = ros_msg.z_bounds
    proto.yaw_bounds = ros_msg.yaw_bounds

def convert_proto_to_bosdyn_msgs_anchor_hint_uncertainty_one_of_uncertainty(proto, ros_msg):
    if proto.HasField("se3_covariance"):
        ros_msg.uncertainty_choice = ros_msg.UNCERTAINTY_SE3_COVARIANCE_SET
        convert_proto_to_bosdyn_msgs_se3_covariance(proto.se3_covariance, ros_msg.se3_covariance)
    if proto.HasField("confidence_bounds"):
        ros_msg.uncertainty_choice = ros_msg.UNCERTAINTY_CONFIDENCE_BOUNDS_SET
        convert_proto_to_bosdyn_msgs_pose_bounds(proto.confidence_bounds, ros_msg.confidence_bounds)

def convert_bosdyn_msgs_anchor_hint_uncertainty_one_of_uncertainty_to_proto(ros_msg, proto):
    proto.ClearField("uncertainty")
    if ros_msg.uncertainty_choice == ros_msg.UNCERTAINTY_SE3_COVARIANCE_SET:
        convert_bosdyn_msgs_se3_covariance_to_proto(ros_msg.se3_covariance, proto.se3_covariance)
    if ros_msg.uncertainty_choice == ros_msg.UNCERTAINTY_CONFIDENCE_BOUNDS_SET:
        convert_bosdyn_msgs_pose_bounds_to_proto(ros_msg.confidence_bounds, proto.confidence_bounds)

def convert_proto_to_bosdyn_msgs_anchor_hint_uncertainty(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_anchor_hint_uncertainty_one_of_uncertainty(proto, ros_msg.uncertainty)

def convert_bosdyn_msgs_anchor_hint_uncertainty_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_anchor_hint_uncertainty_one_of_uncertainty_to_proto(ros_msg.uncertainty, proto)

def convert_proto_to_bosdyn_msgs_waypoint_anchor_hint(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_anchor(proto.waypoint_anchor, ros_msg.waypoint_anchor)
    ros_msg.waypoint_anchor_is_set = proto.HasField("waypoint_anchor")
    convert_proto_to_bosdyn_msgs_anchor_hint_uncertainty(proto.seed_tform_waypoint_uncertainty, ros_msg.seed_tform_waypoint_uncertainty)
    ros_msg.seed_tform_waypoint_uncertainty_is_set = proto.HasField("seed_tform_waypoint_uncertainty")
    convert_proto_to_bosdyn_msgs_pose_bounds(proto.seed_tform_waypoint_constraint, ros_msg.seed_tform_waypoint_constraint)
    ros_msg.seed_tform_waypoint_constraint_is_set = proto.HasField("seed_tform_waypoint_constraint")

def convert_bosdyn_msgs_waypoint_anchor_hint_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.waypoint_anchor_is_set:
        convert_bosdyn_msgs_anchor_to_proto(ros_msg.waypoint_anchor, proto.waypoint_anchor)
    if ros_msg.seed_tform_waypoint_uncertainty_is_set:
        convert_bosdyn_msgs_anchor_hint_uncertainty_to_proto(ros_msg.seed_tform_waypoint_uncertainty, proto.seed_tform_waypoint_uncertainty)
    if ros_msg.seed_tform_waypoint_constraint_is_set:
        convert_bosdyn_msgs_pose_bounds_to_proto(ros_msg.seed_tform_waypoint_constraint, proto.seed_tform_waypoint_constraint)

def convert_proto_to_bosdyn_msgs_world_object_anchor_hint(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_anchored_world_object(proto.object_anchor, ros_msg.object_anchor)
    ros_msg.object_anchor_is_set = proto.HasField("object_anchor")
    convert_proto_to_bosdyn_msgs_anchor_hint_uncertainty(proto.seed_tform_object_uncertainty, ros_msg.seed_tform_object_uncertainty)
    ros_msg.seed_tform_object_uncertainty_is_set = proto.HasField("seed_tform_object_uncertainty")
    convert_proto_to_bosdyn_msgs_pose_bounds(proto.seed_tform_object_constraint, ros_msg.seed_tform_object_constraint)
    ros_msg.seed_tform_object_constraint_is_set = proto.HasField("seed_tform_object_constraint")

def convert_bosdyn_msgs_world_object_anchor_hint_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.object_anchor_is_set:
        convert_bosdyn_msgs_anchored_world_object_to_proto(ros_msg.object_anchor, proto.object_anchor)
    if ros_msg.seed_tform_object_uncertainty_is_set:
        convert_bosdyn_msgs_anchor_hint_uncertainty_to_proto(ros_msg.seed_tform_object_uncertainty, proto.seed_tform_object_uncertainty)
    if ros_msg.seed_tform_object_constraint_is_set:
        convert_bosdyn_msgs_pose_bounds_to_proto(ros_msg.seed_tform_object_constraint, proto.seed_tform_object_constraint)

def convert_proto_to_bosdyn_msgs_anchoring_hint(proto, ros_msg):
    from bosdyn_msgs.msg import WaypointAnchorHint
    ros_msg.waypoint_anchors = []
    for _item in proto.waypoint_anchors:
        ros_msg.waypoint_anchors.append(WaypointAnchorHint())
        convert_proto_to_bosdyn_msgs_waypoint_anchor_hint(_item, ros_msg.waypoint_anchors[-1])
    from bosdyn_msgs.msg import WorldObjectAnchorHint
    ros_msg.world_objects = []
    for _item in proto.world_objects:
        ros_msg.world_objects.append(WorldObjectAnchorHint())
        convert_proto_to_bosdyn_msgs_world_object_anchor_hint(_item, ros_msg.world_objects[-1])

def convert_bosdyn_msgs_anchoring_hint_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.waypoint_anchors[:]
    for _item in ros_msg.waypoint_anchors:
        convert_bosdyn_msgs_waypoint_anchor_hint_to_proto(_item, proto.waypoint_anchors.add())
    del proto.world_objects[:]
    for _item in ros_msg.world_objects:
        convert_bosdyn_msgs_world_object_anchor_hint_to_proto(_item, proto.world_objects.add())

def convert_proto_to_bosdyn_msgs_process_anchoring_request_params_optimizer_params(proto, ros_msg):
    ros_msg.max_iters = proto.max_iters.value
    ros_msg.max_iters_is_set = proto.HasField("max_iters")
    ros_msg.max_time_seconds = proto.max_time_seconds.value
    ros_msg.max_time_seconds_is_set = proto.HasField("max_time_seconds")

def convert_bosdyn_msgs_process_anchoring_request_params_optimizer_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.max_iters_is_set:
        convert_int32_to_proto(ros_msg.max_iters, proto.max_iters)
    if ros_msg.max_time_seconds_is_set:
        convert_float64_to_proto(ros_msg.max_time_seconds, proto.max_time_seconds)

def convert_proto_to_bosdyn_msgs_process_anchoring_request_params_measurement_params(proto, ros_msg):
    ros_msg.use_kinematic_odometry = proto.use_kinematic_odometry.value
    ros_msg.use_kinematic_odometry_is_set = proto.HasField("use_kinematic_odometry")
    ros_msg.use_visual_odometry = proto.use_visual_odometry.value
    ros_msg.use_visual_odometry_is_set = proto.HasField("use_visual_odometry")
    ros_msg.use_gyroscope_measurements = proto.use_gyroscope_measurements.value
    ros_msg.use_gyroscope_measurements_is_set = proto.HasField("use_gyroscope_measurements")
    ros_msg.use_loop_closures = proto.use_loop_closures.value
    ros_msg.use_loop_closures_is_set = proto.HasField("use_loop_closures")
    ros_msg.use_world_objects = proto.use_world_objects.value
    ros_msg.use_world_objects_is_set = proto.HasField("use_world_objects")

def convert_bosdyn_msgs_process_anchoring_request_params_measurement_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.use_kinematic_odometry_is_set:
        convert_bool_to_proto(ros_msg.use_kinematic_odometry, proto.use_kinematic_odometry)
    if ros_msg.use_visual_odometry_is_set:
        convert_bool_to_proto(ros_msg.use_visual_odometry, proto.use_visual_odometry)
    if ros_msg.use_gyroscope_measurements_is_set:
        convert_bool_to_proto(ros_msg.use_gyroscope_measurements, proto.use_gyroscope_measurements)
    if ros_msg.use_loop_closures_is_set:
        convert_bool_to_proto(ros_msg.use_loop_closures, proto.use_loop_closures)
    if ros_msg.use_world_objects_is_set:
        convert_bool_to_proto(ros_msg.use_world_objects, proto.use_world_objects)

def convert_proto_to_bosdyn_msgs_process_anchoring_request_params_weights(proto, ros_msg):
    ros_msg.kinematic_odometry_weight = proto.kinematic_odometry_weight
    ros_msg.visual_odometry_weight = proto.visual_odometry_weight
    ros_msg.world_object_weight = proto.world_object_weight
    ros_msg.hint_weight = proto.hint_weight
    ros_msg.gyroscope_weight = proto.gyroscope_weight
    ros_msg.loop_closure_weight = proto.loop_closure_weight

def convert_bosdyn_msgs_process_anchoring_request_params_weights_to_proto(ros_msg, proto):
    proto.Clear()
    proto.kinematic_odometry_weight = ros_msg.kinematic_odometry_weight
    proto.visual_odometry_weight = ros_msg.visual_odometry_weight
    proto.world_object_weight = ros_msg.world_object_weight
    proto.hint_weight = ros_msg.hint_weight
    proto.gyroscope_weight = ros_msg.gyroscope_weight
    proto.loop_closure_weight = ros_msg.loop_closure_weight

def convert_proto_to_bosdyn_msgs_process_anchoring_request_params(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_process_anchoring_request_params_optimizer_params(proto.optimizer_params, ros_msg.optimizer_params)
    ros_msg.optimizer_params_is_set = proto.HasField("optimizer_params")
    convert_proto_to_bosdyn_msgs_process_anchoring_request_params_measurement_params(proto.measurement_params, ros_msg.measurement_params)
    ros_msg.measurement_params_is_set = proto.HasField("measurement_params")
    convert_proto_to_bosdyn_msgs_process_anchoring_request_params_weights(proto.weights, ros_msg.weights)
    ros_msg.weights_is_set = proto.HasField("weights")
    ros_msg.optimize_existing_anchoring = proto.optimize_existing_anchoring.value
    ros_msg.optimize_existing_anchoring_is_set = proto.HasField("optimize_existing_anchoring")
    convert_proto_to_geometry_msgs_vector3(proto.gravity_ewrt_seed, ros_msg.gravity_ewrt_seed)
    ros_msg.gravity_ewrt_seed_is_set = proto.HasField("gravity_ewrt_seed")

def convert_bosdyn_msgs_process_anchoring_request_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.optimizer_params_is_set:
        convert_bosdyn_msgs_process_anchoring_request_params_optimizer_params_to_proto(ros_msg.optimizer_params, proto.optimizer_params)
    if ros_msg.measurement_params_is_set:
        convert_bosdyn_msgs_process_anchoring_request_params_measurement_params_to_proto(ros_msg.measurement_params, proto.measurement_params)
    if ros_msg.weights_is_set:
        convert_bosdyn_msgs_process_anchoring_request_params_weights_to_proto(ros_msg.weights, proto.weights)
    if ros_msg.optimize_existing_anchoring_is_set:
        convert_bool_to_proto(ros_msg.optimize_existing_anchoring, proto.optimize_existing_anchoring)
    if ros_msg.gravity_ewrt_seed_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.gravity_ewrt_seed, proto.gravity_ewrt_seed)

def convert_proto_to_bosdyn_msgs_process_anchoring_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_process_anchoring_request_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")
    convert_proto_to_bosdyn_msgs_anchoring_hint(proto.initial_hint, ros_msg.initial_hint)
    ros_msg.initial_hint_is_set = proto.HasField("initial_hint")
    ros_msg.modify_anchoring_on_server = proto.modify_anchoring_on_server
    ros_msg.stream_intermediate_results = proto.stream_intermediate_results

def convert_bosdyn_msgs_process_anchoring_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_process_anchoring_request_params_to_proto(ros_msg.params, proto.params)
    if ros_msg.initial_hint_is_set:
        convert_bosdyn_msgs_anchoring_hint_to_proto(ros_msg.initial_hint, proto.initial_hint)
    proto.modify_anchoring_on_server = ros_msg.modify_anchoring_on_server
    proto.stream_intermediate_results = ros_msg.stream_intermediate_results

def convert_proto_to_bosdyn_msgs_process_anchoring_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    from bosdyn_msgs.msg import Anchor
    ros_msg.waypoint_results = []
    for _item in proto.waypoint_results:
        ros_msg.waypoint_results.append(Anchor())
        convert_proto_to_bosdyn_msgs_anchor(_item, ros_msg.waypoint_results[-1])
    from bosdyn_msgs.msg import AnchoredWorldObject
    ros_msg.world_object_results = []
    for _item in proto.world_object_results:
        ros_msg.world_object_results.append(AnchoredWorldObject())
        convert_proto_to_bosdyn_msgs_anchored_world_object(_item, ros_msg.world_object_results[-1])
    ros_msg.anchoring_on_server_was_modified = proto.anchoring_on_server_was_modified
    ros_msg.iteration = proto.iteration
    ros_msg.cost = proto.cost
    ros_msg.final_iteration = proto.final_iteration
    from bosdyn_msgs.msg import WaypointAnchorHint
    ros_msg.violated_waypoint_constraints = []
    for _item in proto.violated_waypoint_constraints:
        ros_msg.violated_waypoint_constraints.append(WaypointAnchorHint())
        convert_proto_to_bosdyn_msgs_waypoint_anchor_hint(_item, ros_msg.violated_waypoint_constraints[-1])
    from bosdyn_msgs.msg import WorldObjectAnchorHint
    ros_msg.violated_object_constraints = []
    for _item in proto.violated_object_constraints:
        ros_msg.violated_object_constraints.append(WorldObjectAnchorHint())
        convert_proto_to_bosdyn_msgs_world_object_anchor_hint(_item, ros_msg.violated_object_constraints[-1])
    ros_msg.missing_snapshot_ids = []
    for _item in proto.missing_snapshot_ids:
        ros_msg.missing_snapshot_ids.append(_item)
    ros_msg.missing_waypoint_ids = []
    for _item in proto.missing_waypoint_ids:
        ros_msg.missing_waypoint_ids.append(_item)
    ros_msg.invalid_hints = []
    for _item in proto.invalid_hints:
        ros_msg.invalid_hints.append(_item)

def convert_bosdyn_msgs_process_anchoring_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    del proto.waypoint_results[:]
    for _item in ros_msg.waypoint_results:
        convert_bosdyn_msgs_anchor_to_proto(_item, proto.waypoint_results.add())
    del proto.world_object_results[:]
    for _item in ros_msg.world_object_results:
        convert_bosdyn_msgs_anchored_world_object_to_proto(_item, proto.world_object_results.add())
    proto.anchoring_on_server_was_modified = ros_msg.anchoring_on_server_was_modified
    proto.iteration = ros_msg.iteration
    proto.cost = ros_msg.cost
    proto.final_iteration = ros_msg.final_iteration
    del proto.violated_waypoint_constraints[:]
    for _item in ros_msg.violated_waypoint_constraints:
        convert_bosdyn_msgs_waypoint_anchor_hint_to_proto(_item, proto.violated_waypoint_constraints.add())
    del proto.violated_object_constraints[:]
    for _item in ros_msg.violated_object_constraints:
        convert_bosdyn_msgs_world_object_anchor_hint_to_proto(_item, proto.violated_object_constraints.add())
    del proto.missing_snapshot_ids[:]
    for _item in ros_msg.missing_snapshot_ids:
        proto.missing_snapshot_ids.add(_item)
    del proto.missing_waypoint_ids[:]
    for _item in ros_msg.missing_waypoint_ids:
        proto.missing_waypoint_ids.add(_item)
    del proto.invalid_hints[:]
    for _item in ros_msg.invalid_hints:
        proto.invalid_hints.add(_item)

def convert_proto_to_bosdyn_msgs_recording_environment(proto, ros_msg):
    ros_msg.name_prefix = proto.name_prefix
    convert_proto_to_bosdyn_msgs_waypoint_annotations(proto.waypoint_environment, ros_msg.waypoint_environment)
    ros_msg.waypoint_environment_is_set = proto.HasField("waypoint_environment")
    convert_proto_to_bosdyn_msgs_edge_annotations(proto.edge_environment, ros_msg.edge_environment)
    ros_msg.edge_environment_is_set = proto.HasField("edge_environment")

def convert_bosdyn_msgs_recording_environment_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name_prefix = ros_msg.name_prefix
    if ros_msg.waypoint_environment_is_set:
        convert_bosdyn_msgs_waypoint_annotations_to_proto(ros_msg.waypoint_environment, proto.waypoint_environment)
    if ros_msg.edge_environment_is_set:
        convert_bosdyn_msgs_edge_annotations_to_proto(ros_msg.edge_environment, proto.edge_environment)

def convert_proto_to_bosdyn_msgs_set_recording_environment_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_recording_environment(proto.environment, ros_msg.environment)
    ros_msg.environment_is_set = proto.HasField("environment")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_set_recording_environment_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.environment_is_set:
        convert_bosdyn_msgs_recording_environment_to_proto(ros_msg.environment, proto.environment)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_set_recording_environment_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_set_recording_environment_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_start_recording_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    convert_proto_to_bosdyn_msgs_recording_environment(proto.recording_environment, ros_msg.recording_environment)
    ros_msg.recording_environment_is_set = proto.HasField("recording_environment")
    ros_msg.require_fiducials = []
    for _item in proto.require_fiducials:
        ros_msg.require_fiducials.append(_item)

def convert_bosdyn_msgs_start_recording_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    if ros_msg.recording_environment_is_set:
        convert_bosdyn_msgs_recording_environment_to_proto(ros_msg.recording_environment, proto.recording_environment)
    del proto.require_fiducials[:]
    for _item in ros_msg.require_fiducials:
        proto.require_fiducials.add(_item)

def convert_proto_to_bosdyn_msgs_start_recording_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_waypoint(proto.created_waypoint, ros_msg.created_waypoint)
    ros_msg.created_waypoint_is_set = proto.HasField("created_waypoint")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    ros_msg.missing_fiducials = []
    for _item in proto.missing_fiducials:
        ros_msg.missing_fiducials.append(_item)
    ros_msg.bad_pose_fiducials = []
    for _item in proto.bad_pose_fiducials:
        ros_msg.bad_pose_fiducials.append(_item)
    ros_msg.license_status.value = proto.license_status
    convert_proto_to_bosdyn_msgs_robot_impaired_state(proto.impaired_state, ros_msg.impaired_state)
    ros_msg.impaired_state_is_set = proto.HasField("impaired_state")

def convert_bosdyn_msgs_start_recording_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.created_waypoint_is_set:
        convert_bosdyn_msgs_waypoint_to_proto(ros_msg.created_waypoint, proto.created_waypoint)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    del proto.missing_fiducials[:]
    for _item in ros_msg.missing_fiducials:
        proto.missing_fiducials.add(_item)
    del proto.bad_pose_fiducials[:]
    for _item in ros_msg.bad_pose_fiducials:
        proto.bad_pose_fiducials.add(_item)
    proto.license_status = ros_msg.license_status.value
    if ros_msg.impaired_state_is_set:
        convert_bosdyn_msgs_robot_impaired_state_to_proto(ros_msg.impaired_state, proto.impaired_state)

def convert_proto_to_bosdyn_msgs_stop_recording_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_stop_recording_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_stop_recording_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.error_waypoint_localized_id = proto.error_waypoint_localized_id
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_stop_recording_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.error_waypoint_localized_id = ros_msg.error_waypoint_localized_id
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_create_waypoint_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.waypoint_name = proto.waypoint_name
    convert_proto_to_bosdyn_msgs_recording_environment(proto.recording_environment, ros_msg.recording_environment)
    ros_msg.recording_environment_is_set = proto.HasField("recording_environment")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    ros_msg.require_fiducials = []
    for _item in proto.require_fiducials:
        ros_msg.require_fiducials.append(_item)
    from bosdyn_msgs.msg import WorldObject
    ros_msg.world_objects = []
    for _item in proto.world_objects:
        ros_msg.world_objects.append(WorldObject())
        convert_proto_to_bosdyn_msgs_world_object(_item, ros_msg.world_objects[-1])

def convert_bosdyn_msgs_create_waypoint_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.waypoint_name = ros_msg.waypoint_name
    if ros_msg.recording_environment_is_set:
        convert_bosdyn_msgs_recording_environment_to_proto(ros_msg.recording_environment, proto.recording_environment)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    del proto.require_fiducials[:]
    for _item in ros_msg.require_fiducials:
        proto.require_fiducials.add(_item)
    del proto.world_objects[:]
    for _item in ros_msg.world_objects:
        convert_bosdyn_msgs_world_object_to_proto(_item, proto.world_objects.add())

def convert_proto_to_bosdyn_msgs_create_waypoint_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_waypoint(proto.created_waypoint, ros_msg.created_waypoint)
    ros_msg.created_waypoint_is_set = proto.HasField("created_waypoint")
    convert_proto_to_bosdyn_msgs_edge(proto.created_edge, ros_msg.created_edge)
    ros_msg.created_edge_is_set = proto.HasField("created_edge")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.missing_fiducials = []
    for _item in proto.missing_fiducials:
        ros_msg.missing_fiducials.append(_item)
    ros_msg.bad_pose_fiducials = []
    for _item in proto.bad_pose_fiducials:
        ros_msg.bad_pose_fiducials.append(_item)
    ros_msg.license_status.value = proto.license_status

def convert_bosdyn_msgs_create_waypoint_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.created_waypoint_is_set:
        convert_bosdyn_msgs_waypoint_to_proto(ros_msg.created_waypoint, proto.created_waypoint)
    if ros_msg.created_edge_is_set:
        convert_bosdyn_msgs_edge_to_proto(ros_msg.created_edge, proto.created_edge)
    proto.status = ros_msg.status.value
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    del proto.missing_fiducials[:]
    for _item in ros_msg.missing_fiducials:
        proto.missing_fiducials.add(_item)
    del proto.bad_pose_fiducials[:]
    for _item in ros_msg.bad_pose_fiducials:
        proto.bad_pose_fiducials.add(_item)
    proto.license_status = ros_msg.license_status.value

def convert_proto_to_bosdyn_msgs_create_edge_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_edge(proto.edge, ros_msg.edge)
    ros_msg.edge_is_set = proto.HasField("edge")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_create_edge_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.edge_is_set:
        convert_bosdyn_msgs_edge_to_proto(ros_msg.edge, proto.edge)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_create_edge_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_edge(proto.error_existing_edge, ros_msg.error_existing_edge)
    ros_msg.error_existing_edge_is_set = proto.HasField("error_existing_edge")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_create_edge_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.error_existing_edge_is_set:
        convert_bosdyn_msgs_edge_to_proto(ros_msg.error_existing_edge, proto.error_existing_edge)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_get_record_status_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_record_status_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_record_status_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.is_recording = proto.is_recording
    convert_proto_to_bosdyn_msgs_recording_environment(proto.recording_environment, ros_msg.recording_environment)
    ros_msg.recording_environment_is_set = proto.HasField("recording_environment")
    ros_msg.map_state.value = proto.map_state
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_robot_impaired_state(proto.impaired_state, ros_msg.impaired_state)
    ros_msg.impaired_state_is_set = proto.HasField("impaired_state")

def convert_bosdyn_msgs_get_record_status_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.is_recording = ros_msg.is_recording
    if ros_msg.recording_environment_is_set:
        convert_bosdyn_msgs_recording_environment_to_proto(ros_msg.recording_environment, proto.recording_environment)
    proto.map_state = ros_msg.map_state.value
    proto.status = ros_msg.status.value
    if ros_msg.impaired_state_is_set:
        convert_bosdyn_msgs_robot_impaired_state_to_proto(ros_msg.impaired_state, proto.impaired_state)

def convert_proto_to_bosdyn_msgs_route(proto, ros_msg):
    ros_msg.waypoint_id = []
    for _item in proto.waypoint_id:
        ros_msg.waypoint_id.append(_item)
    from bosdyn_msgs.msg import EdgeId
    ros_msg.edge_id = []
    for _item in proto.edge_id:
        ros_msg.edge_id.append(EdgeId())
        convert_proto_to_bosdyn_msgs_edge_id(_item, ros_msg.edge_id[-1])

def convert_bosdyn_msgs_route_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.waypoint_id[:]
    for _item in ros_msg.waypoint_id:
        proto.waypoint_id.add(_item)
    del proto.edge_id[:]
    for _item in ros_msg.edge_id:
        convert_bosdyn_msgs_edge_id_to_proto(_item, proto.edge_id.add())

def convert_proto_to_bosdyn_msgs_localization(proto, ros_msg):
    ros_msg.waypoint_id = proto.waypoint_id
    convert_proto_to_geometry_msgs_pose(proto.waypoint_tform_body, ros_msg.waypoint_tform_body)
    ros_msg.waypoint_tform_body_is_set = proto.HasField("waypoint_tform_body")
    convert_proto_to_geometry_msgs_pose(proto.seed_tform_body, ros_msg.seed_tform_body)
    ros_msg.seed_tform_body_is_set = proto.HasField("seed_tform_body")
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")

def convert_bosdyn_msgs_localization_to_proto(ros_msg, proto):
    proto.Clear()
    proto.waypoint_id = ros_msg.waypoint_id
    if ros_msg.waypoint_tform_body_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.waypoint_tform_body, proto.waypoint_tform_body)
    if ros_msg.seed_tform_body_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.seed_tform_body, proto.seed_tform_body)
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)

def convert_proto_to_bosdyn_msgs_visual_refinement_options(proto, ros_msg):
    ros_msg.verify_refinement_quality = proto.verify_refinement_quality

def convert_bosdyn_msgs_visual_refinement_options_to_proto(ros_msg, proto):
    proto.Clear()
    proto.verify_refinement_quality = ros_msg.verify_refinement_quality

def convert_proto_to_bosdyn_msgs_set_localization_request_one_of_refinement(proto, ros_msg):
    if proto.HasField("refine_fiducial_result_with_icp"):
        ros_msg.refinement_choice = ros_msg.REFINEMENT_REFINE_FIDUCIAL_RESULT_WITH_ICP_SET
        ros_msg.refine_fiducial_result_with_icp = proto.refine_fiducial_result_with_icp
    if proto.HasField("refine_with_visual_features"):
        ros_msg.refinement_choice = ros_msg.REFINEMENT_REFINE_WITH_VISUAL_FEATURES_SET
        convert_proto_to_bosdyn_msgs_visual_refinement_options(proto.refine_with_visual_features, ros_msg.refine_with_visual_features)

def convert_bosdyn_msgs_set_localization_request_one_of_refinement_to_proto(ros_msg, proto):
    proto.ClearField("refinement")
    if ros_msg.refinement_choice == ros_msg.REFINEMENT_REFINE_FIDUCIAL_RESULT_WITH_ICP_SET:
        proto.refine_fiducial_result_with_icp = ros_msg.refine_fiducial_result_with_icp
    if ros_msg.refinement_choice == ros_msg.REFINEMENT_REFINE_WITH_VISUAL_FEATURES_SET:
        convert_bosdyn_msgs_visual_refinement_options_to_proto(ros_msg.refine_with_visual_features, proto.refine_with_visual_features)

def convert_proto_to_bosdyn_msgs_set_localization_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_localization(proto.initial_guess, ros_msg.initial_guess)
    ros_msg.initial_guess_is_set = proto.HasField("initial_guess")
    convert_proto_to_geometry_msgs_pose(proto.ko_tform_body, ros_msg.ko_tform_body)
    ros_msg.ko_tform_body_is_set = proto.HasField("ko_tform_body")
    ros_msg.max_distance = proto.max_distance
    ros_msg.max_yaw = proto.max_yaw
    ros_msg.fiducial_init.value = proto.fiducial_init
    ros_msg.use_fiducial_id = proto.use_fiducial_id
    ros_msg.do_ambiguity_check = proto.do_ambiguity_check
    ros_msg.restrict_fiducial_detections_to_target_waypoint = proto.restrict_fiducial_detections_to_target_waypoint
    convert_proto_to_bosdyn_msgs_set_localization_request_one_of_refinement(proto, ros_msg.refinement)

def convert_bosdyn_msgs_set_localization_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.initial_guess_is_set:
        convert_bosdyn_msgs_localization_to_proto(ros_msg.initial_guess, proto.initial_guess)
    if ros_msg.ko_tform_body_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.ko_tform_body, proto.ko_tform_body)
    proto.max_distance = ros_msg.max_distance
    proto.max_yaw = ros_msg.max_yaw
    proto.fiducial_init = ros_msg.fiducial_init.value
    proto.use_fiducial_id = ros_msg.use_fiducial_id
    proto.do_ambiguity_check = ros_msg.do_ambiguity_check
    proto.restrict_fiducial_detections_to_target_waypoint = ros_msg.restrict_fiducial_detections_to_target_waypoint
    convert_bosdyn_msgs_set_localization_request_one_of_refinement_to_proto(ros_msg.refinement, proto)

def convert_proto_to_bosdyn_msgs_sensor_compatibility_status(proto, ros_msg):
    ros_msg.map_has_lidar_data = proto.map_has_lidar_data
    ros_msg.robot_configured_for_lidar = proto.robot_configured_for_lidar

def convert_bosdyn_msgs_sensor_compatibility_status_to_proto(ros_msg, proto):
    proto.Clear()
    proto.map_has_lidar_data = ros_msg.map_has_lidar_data
    proto.robot_configured_for_lidar = ros_msg.robot_configured_for_lidar

def convert_proto_to_bosdyn_msgs_set_localization_response_suspected_ambiguity(proto, ros_msg):
    convert_proto_to_geometry_msgs_pose(proto.alternate_robot_tform_waypoint, ros_msg.alternate_robot_tform_waypoint)
    ros_msg.alternate_robot_tform_waypoint_is_set = proto.HasField("alternate_robot_tform_waypoint")

def convert_bosdyn_msgs_set_localization_response_suspected_ambiguity_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.alternate_robot_tform_waypoint_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.alternate_robot_tform_waypoint, proto.alternate_robot_tform_waypoint)

def convert_proto_to_bosdyn_msgs_set_localization_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    ros_msg.error_report = proto.error_report
    convert_proto_to_bosdyn_msgs_localization(proto.localization, ros_msg.localization)
    ros_msg.localization_is_set = proto.HasField("localization")
    convert_proto_to_bosdyn_msgs_set_localization_response_suspected_ambiguity(proto.suspected_ambiguity, ros_msg.suspected_ambiguity)
    ros_msg.suspected_ambiguity_is_set = proto.HasField("suspected_ambiguity")
    convert_proto_to_bosdyn_msgs_robot_impaired_state(proto.impaired_state, ros_msg.impaired_state)
    ros_msg.impaired_state_is_set = proto.HasField("impaired_state")
    convert_proto_to_bosdyn_msgs_sensor_compatibility_status(proto.sensor_status, ros_msg.sensor_status)
    ros_msg.sensor_status_is_set = proto.HasField("sensor_status")

def convert_bosdyn_msgs_set_localization_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    proto.error_report = ros_msg.error_report
    if ros_msg.localization_is_set:
        convert_bosdyn_msgs_localization_to_proto(ros_msg.localization, proto.localization)
    if ros_msg.suspected_ambiguity_is_set:
        convert_bosdyn_msgs_set_localization_response_suspected_ambiguity_to_proto(ros_msg.suspected_ambiguity, proto.suspected_ambiguity)
    if ros_msg.impaired_state_is_set:
        convert_bosdyn_msgs_robot_impaired_state_to_proto(ros_msg.impaired_state, proto.impaired_state)
    if ros_msg.sensor_status_is_set:
        convert_bosdyn_msgs_sensor_compatibility_status_to_proto(ros_msg.sensor_status, proto.sensor_status)

def convert_bosdyn_msgs_route_gen_params_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_travel_params(proto, ros_msg):
    ros_msg.max_distance = proto.max_distance
    ros_msg.max_yaw = proto.max_yaw
    convert_proto_to_bosdyn_msgs_se2_velocity_limit(proto.velocity_limit, ros_msg.velocity_limit)
    ros_msg.velocity_limit_is_set = proto.HasField("velocity_limit")
    ros_msg.ignore_final_yaw = proto.ignore_final_yaw
    ros_msg.feature_quality_tolerance.value = proto.feature_quality_tolerance
    ros_msg.disable_directed_exploration = proto.disable_directed_exploration
    ros_msg.disable_alternate_route_finding = proto.disable_alternate_route_finding
    ros_msg.path_following_mode.value = proto.path_following_mode
    convert_proto_to_builtin_interfaces_duration(proto.blocked_path_wait_time, ros_msg.blocked_path_wait_time)
    ros_msg.blocked_path_wait_time_is_set = proto.HasField("blocked_path_wait_time")
    ros_msg.ground_clutter_mode.value = proto.ground_clutter_mode

def convert_bosdyn_msgs_travel_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.max_distance = ros_msg.max_distance
    proto.max_yaw = ros_msg.max_yaw
    if ros_msg.velocity_limit_is_set:
        convert_bosdyn_msgs_se2_velocity_limit_to_proto(ros_msg.velocity_limit, proto.velocity_limit)
    proto.ignore_final_yaw = ros_msg.ignore_final_yaw
    proto.feature_quality_tolerance = ros_msg.feature_quality_tolerance.value
    proto.disable_directed_exploration = ros_msg.disable_directed_exploration
    proto.disable_alternate_route_finding = ros_msg.disable_alternate_route_finding
    proto.path_following_mode = ros_msg.path_following_mode.value
    if ros_msg.blocked_path_wait_time_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.blocked_path_wait_time, proto.blocked_path_wait_time)
    proto.ground_clutter_mode = ros_msg.ground_clutter_mode.value

def convert_proto_to_bosdyn_msgs_navigate_to_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    ros_msg.destination_waypoint_id = proto.destination_waypoint_id
    convert_proto_to_bosdyn_msgs_route_gen_params(proto.route_params, ros_msg.route_params)
    ros_msg.route_params_is_set = proto.HasField("route_params")
    convert_proto_to_bosdyn_msgs_travel_params(proto.travel_params, ros_msg.travel_params)
    ros_msg.travel_params_is_set = proto.HasField("travel_params")
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.clock_identifier = proto.clock_identifier
    convert_proto_to_bosdyn_msgs_se2_pose(proto.destination_waypoint_tform_body_goal, ros_msg.destination_waypoint_tform_body_goal)
    ros_msg.destination_waypoint_tform_body_goal_is_set = proto.HasField("destination_waypoint_tform_body_goal")
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_navigate_to_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    proto.destination_waypoint_id = ros_msg.destination_waypoint_id
    if ros_msg.route_params_is_set:
        convert_bosdyn_msgs_route_gen_params_to_proto(ros_msg.route_params, proto.route_params)
    if ros_msg.travel_params_is_set:
        convert_bosdyn_msgs_travel_params_to_proto(ros_msg.travel_params, proto.travel_params)
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    proto.clock_identifier = ros_msg.clock_identifier
    if ros_msg.destination_waypoint_tform_body_goal_is_set:
        convert_bosdyn_msgs_se2_pose_to_proto(ros_msg.destination_waypoint_tform_body_goal, proto.destination_waypoint_tform_body_goal)
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_navigate_to_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_robot_impaired_state(proto.impaired_state, ros_msg.impaired_state)
    ros_msg.impaired_state_is_set = proto.HasField("impaired_state")
    ros_msg.command_id = proto.command_id
    ros_msg.error_waypoint_ids = []
    for _item in proto.error_waypoint_ids:
        ros_msg.error_waypoint_ids.append(_item)
    convert_proto_to_bosdyn_msgs_area_callback_service_error(proto.area_callback_error, ros_msg.area_callback_error)
    ros_msg.area_callback_error_is_set = proto.HasField("area_callback_error")

def convert_bosdyn_msgs_navigate_to_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    proto.status = ros_msg.status.value
    if ros_msg.impaired_state_is_set:
        convert_bosdyn_msgs_robot_impaired_state_to_proto(ros_msg.impaired_state, proto.impaired_state)
    proto.command_id = ros_msg.command_id
    del proto.error_waypoint_ids[:]
    for _item in ros_msg.error_waypoint_ids:
        proto.error_waypoint_ids.add(_item)
    if ros_msg.area_callback_error_is_set:
        convert_bosdyn_msgs_area_callback_service_error_to_proto(ros_msg.area_callback_error, proto.area_callback_error)

def convert_proto_to_bosdyn_msgs_route_following_params(proto, ros_msg):
    ros_msg.new_cmd_behavior.value = proto.new_cmd_behavior
    ros_msg.existing_cmd_behavior.value = proto.existing_cmd_behavior
    ros_msg.route_blocked_behavior.value = proto.route_blocked_behavior

def convert_bosdyn_msgs_route_following_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.new_cmd_behavior = ros_msg.new_cmd_behavior.value
    proto.existing_cmd_behavior = ros_msg.existing_cmd_behavior.value
    proto.route_blocked_behavior = ros_msg.route_blocked_behavior.value

def convert_proto_to_bosdyn_msgs_navigate_route_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    convert_proto_to_bosdyn_msgs_route(proto.route, ros_msg.route)
    ros_msg.route_is_set = proto.HasField("route")
    convert_proto_to_bosdyn_msgs_route_following_params(proto.route_follow_params, ros_msg.route_follow_params)
    ros_msg.route_follow_params_is_set = proto.HasField("route_follow_params")
    convert_proto_to_bosdyn_msgs_travel_params(proto.travel_params, ros_msg.travel_params)
    ros_msg.travel_params_is_set = proto.HasField("travel_params")
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.clock_identifier = proto.clock_identifier
    convert_proto_to_bosdyn_msgs_se2_pose(proto.destination_waypoint_tform_body_goal, ros_msg.destination_waypoint_tform_body_goal)
    ros_msg.destination_waypoint_tform_body_goal_is_set = proto.HasField("destination_waypoint_tform_body_goal")
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_navigate_route_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    if ros_msg.route_is_set:
        convert_bosdyn_msgs_route_to_proto(ros_msg.route, proto.route)
    if ros_msg.route_follow_params_is_set:
        convert_bosdyn_msgs_route_following_params_to_proto(ros_msg.route_follow_params, proto.route_follow_params)
    if ros_msg.travel_params_is_set:
        convert_bosdyn_msgs_travel_params_to_proto(ros_msg.travel_params, proto.travel_params)
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    proto.clock_identifier = ros_msg.clock_identifier
    if ros_msg.destination_waypoint_tform_body_goal_is_set:
        convert_bosdyn_msgs_se2_pose_to_proto(ros_msg.destination_waypoint_tform_body_goal, proto.destination_waypoint_tform_body_goal)
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_navigate_route_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_robot_impaired_state(proto.impaired_state, ros_msg.impaired_state)
    ros_msg.impaired_state_is_set = proto.HasField("impaired_state")
    ros_msg.command_id = proto.command_id
    ros_msg.error_waypoint_ids = []
    for _item in proto.error_waypoint_ids:
        ros_msg.error_waypoint_ids.append(_item)
    from bosdyn_msgs.msg import EdgeId
    ros_msg.error_edge_ids = []
    for _item in proto.error_edge_ids:
        ros_msg.error_edge_ids.append(EdgeId())
        convert_proto_to_bosdyn_msgs_edge_id(_item, ros_msg.error_edge_ids[-1])
    convert_proto_to_bosdyn_msgs_area_callback_service_error(proto.area_callback_error, ros_msg.area_callback_error)
    ros_msg.area_callback_error_is_set = proto.HasField("area_callback_error")

def convert_bosdyn_msgs_navigate_route_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    proto.status = ros_msg.status.value
    if ros_msg.impaired_state_is_set:
        convert_bosdyn_msgs_robot_impaired_state_to_proto(ros_msg.impaired_state, proto.impaired_state)
    proto.command_id = ros_msg.command_id
    del proto.error_waypoint_ids[:]
    for _item in ros_msg.error_waypoint_ids:
        proto.error_waypoint_ids.add(_item)
    del proto.error_edge_ids[:]
    for _item in ros_msg.error_edge_ids:
        convert_bosdyn_msgs_edge_id_to_proto(_item, proto.error_edge_ids.add())
    if ros_msg.area_callback_error_is_set:
        convert_bosdyn_msgs_area_callback_service_error_to_proto(ros_msg.area_callback_error, proto.area_callback_error)

def convert_proto_to_bosdyn_msgs_navigate_to_anchor_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    convert_proto_to_geometry_msgs_pose(proto.seed_tform_goal, ros_msg.seed_tform_goal)
    ros_msg.seed_tform_goal_is_set = proto.HasField("seed_tform_goal")
    convert_proto_to_geometry_msgs_vector3(proto.goal_waypoint_rt_seed_ewrt_seed_tolerance, ros_msg.goal_waypoint_rt_seed_ewrt_seed_tolerance)
    ros_msg.goal_waypoint_rt_seed_ewrt_seed_tolerance_is_set = proto.HasField("goal_waypoint_rt_seed_ewrt_seed_tolerance")
    convert_proto_to_bosdyn_msgs_route_gen_params(proto.route_params, ros_msg.route_params)
    ros_msg.route_params_is_set = proto.HasField("route_params")
    convert_proto_to_bosdyn_msgs_travel_params(proto.travel_params, ros_msg.travel_params)
    ros_msg.travel_params_is_set = proto.HasField("travel_params")
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.clock_identifier = proto.clock_identifier
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_navigate_to_anchor_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    if ros_msg.seed_tform_goal_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.seed_tform_goal, proto.seed_tform_goal)
    if ros_msg.goal_waypoint_rt_seed_ewrt_seed_tolerance_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.goal_waypoint_rt_seed_ewrt_seed_tolerance, proto.goal_waypoint_rt_seed_ewrt_seed_tolerance)
    if ros_msg.route_params_is_set:
        convert_bosdyn_msgs_route_gen_params_to_proto(ros_msg.route_params, proto.route_params)
    if ros_msg.travel_params_is_set:
        convert_bosdyn_msgs_travel_params_to_proto(ros_msg.travel_params, proto.travel_params)
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    proto.clock_identifier = ros_msg.clock_identifier
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_navigate_to_anchor_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_robot_impaired_state(proto.impaired_state, ros_msg.impaired_state)
    ros_msg.impaired_state_is_set = proto.HasField("impaired_state")
    ros_msg.command_id = proto.command_id
    ros_msg.error_waypoint_ids = []
    for _item in proto.error_waypoint_ids:
        ros_msg.error_waypoint_ids.append(_item)
    convert_proto_to_bosdyn_msgs_area_callback_service_error(proto.area_callback_error, ros_msg.area_callback_error)
    ros_msg.area_callback_error_is_set = proto.HasField("area_callback_error")

def convert_bosdyn_msgs_navigate_to_anchor_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    proto.status = ros_msg.status.value
    if ros_msg.impaired_state_is_set:
        convert_bosdyn_msgs_robot_impaired_state_to_proto(ros_msg.impaired_state, proto.impaired_state)
    proto.command_id = ros_msg.command_id
    del proto.error_waypoint_ids[:]
    for _item in ros_msg.error_waypoint_ids:
        proto.error_waypoint_ids.add(_item)
    if ros_msg.area_callback_error_is_set:
        convert_bosdyn_msgs_area_callback_service_error_to_proto(ros_msg.area_callback_error, proto.area_callback_error)

def convert_proto_to_bosdyn_msgs_navigation_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.command_id = proto.command_id

def convert_bosdyn_msgs_navigation_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.command_id = ros_msg.command_id

def convert_proto_to_bosdyn_msgs_navigation_feedback_response_active_region_information(proto, ros_msg):
    ros_msg.description = proto.description
    ros_msg.service_name = proto.service_name
    ros_msg.region_status.value = proto.region_status

def convert_bosdyn_msgs_navigation_feedback_response_active_region_information_to_proto(ros_msg, proto):
    proto.Clear()
    proto.description = ros_msg.description
    proto.service_name = ros_msg.service_name
    proto.region_status = ros_msg.region_status.value

def convert_proto_to_bosdyn_msgs_navigation_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_robot_impaired_state(proto.impaired_state, ros_msg.impaired_state)
    ros_msg.impaired_state_is_set = proto.HasField("impaired_state")
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsAreaCallbackError
    ros_msg.area_callback_errors = []
    for _item in proto.area_callback_errors:
        ros_msg.area_callback_errors.append(KeyStringValueBosdynMsgsAreaCallbackError())
        ros_msg.area_callback_errors[-1].key = _item
        convert_proto_to_bosdyn_msgs_area_callback_error(proto.area_callback_errors[_item], ros_msg.area_callback_errors[-1].value)
    convert_proto_to_bosdyn_msgs_route(proto.remaining_route, ros_msg.remaining_route)
    ros_msg.remaining_route_is_set = proto.HasField("remaining_route")
    ros_msg.command_id = proto.command_id
    convert_proto_to_geometry_msgs_pose(proto.last_ko_tform_goal, ros_msg.last_ko_tform_goal)
    ros_msg.last_ko_tform_goal_is_set = proto.HasField("last_ko_tform_goal")
    ros_msg.body_movement_status.value = proto.body_movement_status
    ros_msg.path_following_mode.value = proto.path_following_mode
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsActiveRegionInformation
    ros_msg.active_region_information = []
    for _item in proto.active_region_information:
        ros_msg.active_region_information.append(KeyStringValueBosdynMsgsActiveRegionInformation())
        ros_msg.active_region_information[-1].key = _item
        convert_proto_to_bosdyn_msgs_navigation_feedback_response_active_region_information(proto.active_region_information[_item], ros_msg.active_region_information[-1].value)
    ros_msg.route_following_status.value = proto.route_following_status
    ros_msg.blockage_status.value = proto.blockage_status

def convert_bosdyn_msgs_navigation_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.impaired_state_is_set:
        convert_bosdyn_msgs_robot_impaired_state_to_proto(ros_msg.impaired_state, proto.impaired_state)
    for _item in ros_msg.area_callback_errors:
        convert_bosdyn_msgs_area_callback_error_to_proto(_item.value, proto.area_callback_errors[_item.key])
    if ros_msg.remaining_route_is_set:
        convert_bosdyn_msgs_route_to_proto(ros_msg.remaining_route, proto.remaining_route)
    proto.command_id = ros_msg.command_id
    if ros_msg.last_ko_tform_goal_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.last_ko_tform_goal, proto.last_ko_tform_goal)
    proto.body_movement_status = ros_msg.body_movement_status.value
    proto.path_following_mode = ros_msg.path_following_mode.value
    for _item in ros_msg.active_region_information:
        convert_bosdyn_msgs_navigation_feedback_response_active_region_information_to_proto(_item.value, proto.active_region_information[_item.key])
    proto.route_following_status = ros_msg.route_following_status.value
    proto.blockage_status = ros_msg.blockage_status.value

def convert_proto_to_bosdyn_msgs_get_localization_state_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.waypoint_id = proto.waypoint_id
    ros_msg.request_live_point_cloud = proto.request_live_point_cloud
    ros_msg.request_live_images = proto.request_live_images
    ros_msg.request_live_terrain_maps = proto.request_live_terrain_maps
    ros_msg.request_live_world_objects = proto.request_live_world_objects
    ros_msg.request_live_robot_state = proto.request_live_robot_state
    ros_msg.compress_live_point_cloud = proto.compress_live_point_cloud

def convert_bosdyn_msgs_get_localization_state_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.waypoint_id = ros_msg.waypoint_id
    proto.request_live_point_cloud = ros_msg.request_live_point_cloud
    proto.request_live_images = ros_msg.request_live_images
    proto.request_live_terrain_maps = ros_msg.request_live_terrain_maps
    proto.request_live_world_objects = ros_msg.request_live_world_objects
    proto.request_live_robot_state = ros_msg.request_live_robot_state
    proto.compress_live_point_cloud = ros_msg.compress_live_point_cloud

def convert_proto_to_bosdyn_msgs_remote_point_cloud_status(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.exists_in_directory = proto.exists_in_directory
    ros_msg.has_data = proto.has_data

def convert_bosdyn_msgs_remote_point_cloud_status_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.exists_in_directory = ros_msg.exists_in_directory
    proto.has_data = ros_msg.has_data

def convert_proto_to_bosdyn_msgs_lost_detector_state(proto, ros_msg):
    ros_msg.is_lost = proto.is_lost

def convert_bosdyn_msgs_lost_detector_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.is_lost = ros_msg.is_lost

def convert_proto_to_bosdyn_msgs_get_localization_state_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_localization(proto.localization, ros_msg.localization)
    ros_msg.localization_is_set = proto.HasField("localization")
    convert_proto_to_bosdyn_msgs_kinematic_state(proto.robot_kinematics, ros_msg.robot_kinematics)
    ros_msg.robot_kinematics_is_set = proto.HasField("robot_kinematics")
    from bosdyn_msgs.msg import RemotePointCloudStatus
    ros_msg.remote_cloud_status = []
    for _item in proto.remote_cloud_status:
        ros_msg.remote_cloud_status.append(RemotePointCloudStatus())
        convert_proto_to_bosdyn_msgs_remote_point_cloud_status(_item, ros_msg.remote_cloud_status[-1])
    convert_proto_to_bosdyn_msgs_waypoint_snapshot(proto.live_data, ros_msg.live_data)
    ros_msg.live_data_is_set = proto.HasField("live_data")
    convert_proto_to_bosdyn_msgs_lost_detector_state(proto.lost_detector_state, ros_msg.lost_detector_state)
    ros_msg.lost_detector_state_is_set = proto.HasField("lost_detector_state")

def convert_bosdyn_msgs_get_localization_state_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.localization_is_set:
        convert_bosdyn_msgs_localization_to_proto(ros_msg.localization, proto.localization)
    if ros_msg.robot_kinematics_is_set:
        convert_bosdyn_msgs_kinematic_state_to_proto(ros_msg.robot_kinematics, proto.robot_kinematics)
    del proto.remote_cloud_status[:]
    for _item in ros_msg.remote_cloud_status:
        convert_bosdyn_msgs_remote_point_cloud_status_to_proto(_item, proto.remote_cloud_status.add())
    if ros_msg.live_data_is_set:
        convert_bosdyn_msgs_waypoint_snapshot_to_proto(ros_msg.live_data, proto.live_data)
    if ros_msg.lost_detector_state_is_set:
        convert_bosdyn_msgs_lost_detector_state_to_proto(ros_msg.lost_detector_state, proto.lost_detector_state)

def convert_proto_to_bosdyn_msgs_clear_graph_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_clear_graph_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_clear_graph_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_clear_graph_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_upload_graph_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_graph(proto.graph, ros_msg.graph)
    ros_msg.graph_is_set = proto.HasField("graph")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    ros_msg.generate_new_anchoring = proto.generate_new_anchoring

def convert_bosdyn_msgs_upload_graph_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.graph_is_set:
        convert_bosdyn_msgs_graph_to_proto(ros_msg.graph, proto.graph)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    proto.generate_new_anchoring = ros_msg.generate_new_anchoring

def convert_proto_to_bosdyn_msgs_upload_graph_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.loaded_waypoint_snapshot_ids = []
    for _item in proto.loaded_waypoint_snapshot_ids:
        ros_msg.loaded_waypoint_snapshot_ids.append(_item)
    ros_msg.unknown_waypoint_snapshot_ids = []
    for _item in proto.unknown_waypoint_snapshot_ids:
        ros_msg.unknown_waypoint_snapshot_ids.append(_item)
    ros_msg.loaded_edge_snapshot_ids = []
    for _item in proto.loaded_edge_snapshot_ids:
        ros_msg.loaded_edge_snapshot_ids.append(_item)
    ros_msg.unknown_edge_snapshot_ids = []
    for _item in proto.unknown_edge_snapshot_ids:
        ros_msg.unknown_edge_snapshot_ids.append(_item)
    ros_msg.license_status.value = proto.license_status
    convert_proto_to_bosdyn_msgs_sensor_compatibility_status(proto.sensor_status, ros_msg.sensor_status)
    ros_msg.sensor_status_is_set = proto.HasField("sensor_status")
    convert_proto_to_bosdyn_msgs_area_callback_service_error(proto.area_callback_error, ros_msg.area_callback_error)
    ros_msg.area_callback_error_is_set = proto.HasField("area_callback_error")

def convert_bosdyn_msgs_upload_graph_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    del proto.loaded_waypoint_snapshot_ids[:]
    for _item in ros_msg.loaded_waypoint_snapshot_ids:
        proto.loaded_waypoint_snapshot_ids.add(_item)
    del proto.unknown_waypoint_snapshot_ids[:]
    for _item in ros_msg.unknown_waypoint_snapshot_ids:
        proto.unknown_waypoint_snapshot_ids.add(_item)
    del proto.loaded_edge_snapshot_ids[:]
    for _item in ros_msg.loaded_edge_snapshot_ids:
        proto.loaded_edge_snapshot_ids.add(_item)
    del proto.unknown_edge_snapshot_ids[:]
    for _item in ros_msg.unknown_edge_snapshot_ids:
        proto.unknown_edge_snapshot_ids.add(_item)
    proto.license_status = ros_msg.license_status.value
    if ros_msg.sensor_status_is_set:
        convert_bosdyn_msgs_sensor_compatibility_status_to_proto(ros_msg.sensor_status, proto.sensor_status)
    if ros_msg.area_callback_error_is_set:
        convert_bosdyn_msgs_area_callback_service_error_to_proto(ros_msg.area_callback_error, proto.area_callback_error)

def convert_proto_to_bosdyn_msgs_download_graph_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_download_graph_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_download_graph_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_graph(proto.graph, ros_msg.graph)
    ros_msg.graph_is_set = proto.HasField("graph")

def convert_bosdyn_msgs_download_graph_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.graph_is_set:
        convert_bosdyn_msgs_graph_to_proto(ros_msg.graph, proto.graph)

def convert_proto_to_bosdyn_msgs_upload_waypoint_snapshot_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_chunk(proto.chunk, ros_msg.chunk)
    ros_msg.chunk_is_set = proto.HasField("chunk")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_upload_waypoint_snapshot_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.chunk_is_set:
        convert_bosdyn_msgs_data_chunk_to_proto(ros_msg.chunk, proto.chunk)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_upload_waypoint_snapshot_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_sensor_compatibility_status(proto.sensor_status, ros_msg.sensor_status)
    ros_msg.sensor_status_is_set = proto.HasField("sensor_status")

def convert_bosdyn_msgs_upload_waypoint_snapshot_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    if ros_msg.sensor_status_is_set:
        convert_bosdyn_msgs_sensor_compatibility_status_to_proto(ros_msg.sensor_status, proto.sensor_status)

def convert_proto_to_bosdyn_msgs_upload_edge_snapshot_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_chunk(proto.chunk, ros_msg.chunk)
    ros_msg.chunk_is_set = proto.HasField("chunk")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_upload_edge_snapshot_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.chunk_is_set:
        convert_bosdyn_msgs_data_chunk_to_proto(ros_msg.chunk, proto.chunk)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_upload_edge_snapshot_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_upload_edge_snapshot_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_download_waypoint_snapshot_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.waypoint_snapshot_id = proto.waypoint_snapshot_id
    ros_msg.download_images = proto.download_images
    ros_msg.compress_point_cloud = proto.compress_point_cloud
    ros_msg.do_not_download_point_cloud = proto.do_not_download_point_cloud

def convert_bosdyn_msgs_download_waypoint_snapshot_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.waypoint_snapshot_id = ros_msg.waypoint_snapshot_id
    proto.download_images = ros_msg.download_images
    proto.compress_point_cloud = ros_msg.compress_point_cloud
    proto.do_not_download_point_cloud = ros_msg.do_not_download_point_cloud

def convert_proto_to_bosdyn_msgs_download_waypoint_snapshot_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.waypoint_snapshot_id = proto.waypoint_snapshot_id
    convert_proto_to_bosdyn_msgs_data_chunk(proto.chunk, ros_msg.chunk)
    ros_msg.chunk_is_set = proto.HasField("chunk")

def convert_bosdyn_msgs_download_waypoint_snapshot_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.waypoint_snapshot_id = ros_msg.waypoint_snapshot_id
    if ros_msg.chunk_is_set:
        convert_bosdyn_msgs_data_chunk_to_proto(ros_msg.chunk, proto.chunk)

def convert_proto_to_bosdyn_msgs_download_edge_snapshot_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.edge_snapshot_id = proto.edge_snapshot_id

def convert_bosdyn_msgs_download_edge_snapshot_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.edge_snapshot_id = ros_msg.edge_snapshot_id

def convert_proto_to_bosdyn_msgs_download_edge_snapshot_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.edge_snapshot_id = proto.edge_snapshot_id
    convert_proto_to_bosdyn_msgs_data_chunk(proto.chunk, ros_msg.chunk)
    ros_msg.chunk_is_set = proto.HasField("chunk")

def convert_bosdyn_msgs_download_edge_snapshot_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.edge_snapshot_id = ros_msg.edge_snapshot_id
    if ros_msg.chunk_is_set:
        convert_bosdyn_msgs_data_chunk_to_proto(ros_msg.chunk, proto.chunk)

def convert_proto_to_bosdyn_msgs_area_callback_service_error(proto, ros_msg):
    ros_msg.missing_services = []
    for _item in proto.missing_services:
        ros_msg.missing_services.append(_item)

def convert_bosdyn_msgs_area_callback_service_error_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.missing_services[:]
    for _item in ros_msg.missing_services:
        proto.missing_services.add(_item)

def convert_proto_to_bosdyn_msgs_validate_graph_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_validate_graph_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_validate_graph_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_sensor_compatibility_status(proto.sensor_status, ros_msg.sensor_status)
    ros_msg.sensor_status_is_set = proto.HasField("sensor_status")
    convert_proto_to_bosdyn_msgs_area_callback_service_error(proto.area_callback_error, ros_msg.area_callback_error)
    ros_msg.area_callback_error_is_set = proto.HasField("area_callback_error")

def convert_bosdyn_msgs_validate_graph_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.sensor_status_is_set:
        convert_bosdyn_msgs_sensor_compatibility_status_to_proto(ros_msg.sensor_status, proto.sensor_status)
    if ros_msg.area_callback_error_is_set:
        convert_bosdyn_msgs_area_callback_service_error_to_proto(ros_msg.area_callback_error, proto.area_callback_error)

def convert_proto_to_bosdyn_msgs_docking_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    ros_msg.docking_station_id = proto.docking_station_id
    ros_msg.clock_identifier = proto.clock_identifier
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")
    ros_msg.prep_pose_behavior.value = proto.prep_pose_behavior

def convert_bosdyn_msgs_docking_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    proto.docking_station_id = ros_msg.docking_station_id
    proto.clock_identifier = ros_msg.clock_identifier
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)
    proto.prep_pose_behavior = ros_msg.prep_pose_behavior.value

def convert_proto_to_bosdyn_msgs_docking_command_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    ros_msg.docking_command_id = proto.docking_command_id

def convert_bosdyn_msgs_docking_command_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    proto.docking_command_id = ros_msg.docking_command_id

def convert_proto_to_bosdyn_msgs_update_docking_params(proto, ros_msg):
    convert_proto_to_builtin_interfaces_time(proto.end_time, ros_msg.end_time)
    ros_msg.end_time_is_set = proto.HasField("end_time")

def convert_bosdyn_msgs_update_docking_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.end_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.end_time, proto.end_time)

def convert_proto_to_bosdyn_msgs_docking_command_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.docking_command_id = proto.docking_command_id
    convert_proto_to_bosdyn_msgs_update_docking_params(proto.update_docking_params, ros_msg.update_docking_params)
    ros_msg.update_docking_params_is_set = proto.HasField("update_docking_params")

def convert_bosdyn_msgs_docking_command_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.docking_command_id = ros_msg.docking_command_id
    if ros_msg.update_docking_params_is_set:
        convert_bosdyn_msgs_update_docking_params_to_proto(ros_msg.update_docking_params, proto.update_docking_params)

def convert_proto_to_bosdyn_msgs_docking_command_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_docking_command_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_config_range(proto, ros_msg):
    ros_msg.id_start = proto.id_start
    ros_msg.id_end = proto.id_end
    ros_msg.type.value = proto.type

def convert_bosdyn_msgs_config_range_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id_start = ros_msg.id_start
    proto.id_end = ros_msg.id_end
    proto.type = ros_msg.type.value

def convert_proto_to_bosdyn_msgs_get_docking_config_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_docking_config_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_docking_config_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import ConfigRange
    ros_msg.dock_configs = []
    for _item in proto.dock_configs:
        ros_msg.dock_configs.append(ConfigRange())
        convert_proto_to_bosdyn_msgs_config_range(_item, ros_msg.dock_configs[-1])

def convert_bosdyn_msgs_get_docking_config_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.dock_configs[:]
    for _item in ros_msg.dock_configs:
        convert_bosdyn_msgs_config_range_to_proto(_item, proto.dock_configs.add())

def convert_proto_to_bosdyn_msgs_dock_state(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.dock_type.value = proto.dock_type
    ros_msg.dock_id = proto.dock_id
    ros_msg.power_status.value = proto.power_status

def convert_bosdyn_msgs_dock_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.dock_type = ros_msg.dock_type.value
    proto.dock_id = ros_msg.dock_id
    proto.power_status = ros_msg.power_status.value

def convert_proto_to_bosdyn_msgs_get_docking_state_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_docking_state_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_docking_state_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_dock_state(proto.dock_state, ros_msg.dock_state)
    ros_msg.dock_state_is_set = proto.HasField("dock_state")

def convert_bosdyn_msgs_get_docking_state_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.dock_state_is_set:
        convert_bosdyn_msgs_dock_state_to_proto(ros_msg.dock_state, proto.dock_state)

def convert_proto_to_bosdyn_msgs_establish_session_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    from bosdyn_msgs.msg import VariableDeclaration
    ros_msg.inputs = []
    for _item in proto.inputs:
        ros_msg.inputs.append(VariableDeclaration())
        convert_proto_to_bosdyn_msgs_variable_declaration(_item, ros_msg.inputs[-1])

def convert_bosdyn_msgs_establish_session_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    del proto.inputs[:]
    for _item in ros_msg.inputs:
        convert_bosdyn_msgs_variable_declaration_to_proto(_item, proto.inputs.add())

def convert_proto_to_bosdyn_msgs_establish_session_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.session_id = proto.session_id
    ros_msg.missing_lease_resources = []
    for _item in proto.missing_lease_resources:
        ros_msg.missing_lease_resources.append(_item)
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    from bosdyn_msgs.msg import VariableDeclaration
    ros_msg.missing_inputs = []
    for _item in proto.missing_inputs:
        ros_msg.missing_inputs.append(VariableDeclaration())
        convert_proto_to_bosdyn_msgs_variable_declaration(_item, ros_msg.missing_inputs[-1])

def convert_bosdyn_msgs_establish_session_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.session_id = ros_msg.session_id
    del proto.missing_lease_resources[:]
    for _item in ros_msg.missing_lease_resources:
        proto.missing_lease_resources.add(_item)
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    del proto.missing_inputs[:]
    for _item in ros_msg.missing_inputs:
        convert_bosdyn_msgs_variable_declaration_to_proto(_item, proto.missing_inputs.add())

def convert_proto_to_bosdyn_msgs_tick_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.session_id = proto.session_id
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    from bosdyn_msgs.msg import KeyValue
    ros_msg.inputs = []
    for _item in proto.inputs:
        ros_msg.inputs.append(KeyValue())
        convert_proto_to_bosdyn_msgs_key_value(_item, ros_msg.inputs[-1])

def convert_bosdyn_msgs_tick_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.session_id = ros_msg.session_id
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    del proto.inputs[:]
    for _item in ros_msg.inputs:
        convert_bosdyn_msgs_key_value_to_proto(_item, proto.inputs.add())

def convert_proto_to_bosdyn_msgs_tick_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.missing_lease_resources = []
    for _item in proto.missing_lease_resources:
        ros_msg.missing_lease_resources.append(_item)
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    from bosdyn_msgs.msg import VariableDeclaration
    ros_msg.missing_inputs = []
    for _item in proto.missing_inputs:
        ros_msg.missing_inputs.append(VariableDeclaration())
        convert_proto_to_bosdyn_msgs_variable_declaration(_item, ros_msg.missing_inputs[-1])
    ros_msg.error_message = proto.error_message

def convert_bosdyn_msgs_tick_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    del proto.missing_lease_resources[:]
    for _item in ros_msg.missing_lease_resources:
        proto.missing_lease_resources.add(_item)
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    del proto.missing_inputs[:]
    for _item in ros_msg.missing_inputs:
        convert_bosdyn_msgs_variable_declaration_to_proto(_item, proto.missing_inputs.add())
    proto.error_message = ros_msg.error_message

def convert_proto_to_bosdyn_msgs_stop_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.session_id = proto.session_id

def convert_bosdyn_msgs_stop_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.session_id = ros_msg.session_id

def convert_proto_to_bosdyn_msgs_stop_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_stop_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_teardown_session_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.session_id = proto.session_id

def convert_bosdyn_msgs_teardown_session_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.session_id = ros_msg.session_id

def convert_proto_to_bosdyn_msgs_teardown_session_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_teardown_session_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_node_one_of_type(proto, ros_msg):
    if proto.HasField("node_reference"):
        ros_msg.type_choice = ros_msg.TYPE_NODE_REFERENCE_SET
        ros_msg.node_reference = proto.node_reference

def convert_bosdyn_msgs_node_one_of_type_to_proto(ros_msg, proto):
    proto.ClearField("type")
    if ros_msg.type_choice == ros_msg.TYPE_NODE_REFERENCE_SET:
        proto.node_reference = ros_msg.node_reference

def convert_proto_to_bosdyn_msgs_node(proto, ros_msg):
    ros_msg.name = proto.name
    convert_proto_to_bosdyn_msgs_user_data(proto.user_data, ros_msg.user_data)
    ros_msg.user_data_is_set = proto.HasField("user_data")
    ros_msg.reference_id = proto.reference_id
    convert_proto_to_bosdyn_msgs_node_one_of_type(proto, ros_msg.type)
    from bosdyn_msgs.msg import KeyValue
    ros_msg.parameter_values = []
    for _item in proto.parameter_values:
        ros_msg.parameter_values.append(KeyValue())
        convert_proto_to_bosdyn_msgs_key_value(_item, ros_msg.parameter_values[-1])
    from bosdyn_msgs.msg import KeyValue
    ros_msg.overrides = []
    for _item in proto.overrides:
        ros_msg.overrides.append(KeyValue())
        convert_proto_to_bosdyn_msgs_key_value(_item, ros_msg.overrides[-1])
    from bosdyn_msgs.msg import VariableDeclaration
    ros_msg.parameters = []
    for _item in proto.parameters:
        ros_msg.parameters.append(VariableDeclaration())
        convert_proto_to_bosdyn_msgs_variable_declaration(_item, ros_msg.parameters[-1])

def convert_bosdyn_msgs_node_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    if ros_msg.user_data_is_set:
        convert_bosdyn_msgs_user_data_to_proto(ros_msg.user_data, proto.user_data)
    proto.reference_id = ros_msg.reference_id
    convert_bosdyn_msgs_node_one_of_type_to_proto(ros_msg.type, proto)
    del proto.parameter_values[:]
    for _item in ros_msg.parameter_values:
        convert_bosdyn_msgs_key_value_to_proto(_item, proto.parameter_values.add())
    del proto.overrides[:]
    for _item in ros_msg.overrides:
        convert_bosdyn_msgs_key_value_to_proto(_item, proto.overrides.add())
    del proto.parameters[:]
    for _item in ros_msg.parameters:
        convert_bosdyn_msgs_variable_declaration_to_proto(_item, proto.parameters.add())

def convert_proto_to_bosdyn_msgs_sequence(proto, ros_msg):
    ros_msg.always_restart = proto.always_restart
    from bosdyn_msgs.msg import Node
    ros_msg.children = []
    for _item in proto.children:
        ros_msg.children.append(Node())
        convert_proto_to_bosdyn_msgs_node(_item, ros_msg.children[-1])

def convert_bosdyn_msgs_sequence_to_proto(ros_msg, proto):
    proto.Clear()
    proto.always_restart = ros_msg.always_restart
    del proto.children[:]
    for _item in ros_msg.children:
        convert_bosdyn_msgs_node_to_proto(_item, proto.children.add())

def convert_proto_to_bosdyn_msgs_selector(proto, ros_msg):
    ros_msg.always_restart = proto.always_restart
    from bosdyn_msgs.msg import Node
    ros_msg.children = []
    for _item in proto.children:
        ros_msg.children.append(Node())
        convert_proto_to_bosdyn_msgs_node(_item, ros_msg.children[-1])

def convert_bosdyn_msgs_selector_to_proto(ros_msg, proto):
    proto.Clear()
    proto.always_restart = ros_msg.always_restart
    del proto.children[:]
    for _item in ros_msg.children:
        convert_bosdyn_msgs_node_to_proto(_item, proto.children.add())

def convert_proto_to_bosdyn_msgs_switch(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_value(proto.pivot_value, ros_msg.pivot_value)
    ros_msg.pivot_value_is_set = proto.HasField("pivot_value")
    ros_msg.always_restart = proto.always_restart
    from bosdyn_msgs.msg import KeyInt32ValueBosdynMsgsNode
    ros_msg.int_children = []
    for _item in proto.int_children:
        ros_msg.int_children.append(KeyInt32ValueBosdynMsgsNode())
        ros_msg.int_children[-1].key = _item
        convert_proto_to_bosdyn_msgs_node(proto.int_children[_item], ros_msg.int_children[-1].value)
    convert_proto_to_bosdyn_msgs_node(proto.default_child, ros_msg.default_child)
    ros_msg.default_child_is_set = proto.HasField("default_child")

def convert_bosdyn_msgs_switch_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.pivot_value_is_set:
        convert_bosdyn_msgs_value_to_proto(ros_msg.pivot_value, proto.pivot_value)
    proto.always_restart = ros_msg.always_restart
    for _item in ros_msg.int_children:
        convert_bosdyn_msgs_node_to_proto(_item.value, proto.int_children[_item.key])
    if ros_msg.default_child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.default_child, proto.default_child)

def convert_proto_to_bosdyn_msgs_repeat(proto, ros_msg):
    ros_msg.max_starts = proto.max_starts
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")
    ros_msg.start_counter_state_name = proto.start_counter_state_name
    ros_msg.respect_child_failure = proto.respect_child_failure

def convert_bosdyn_msgs_repeat_to_proto(ros_msg, proto):
    proto.Clear()
    proto.max_starts = ros_msg.max_starts
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)
    proto.start_counter_state_name = ros_msg.start_counter_state_name
    proto.respect_child_failure = ros_msg.respect_child_failure

def convert_proto_to_bosdyn_msgs_retry(proto, ros_msg):
    ros_msg.max_attempts = proto.max_attempts
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")
    ros_msg.attempt_counter_state_name = proto.attempt_counter_state_name

def convert_bosdyn_msgs_retry_to_proto(ros_msg, proto):
    proto.Clear()
    proto.max_attempts = ros_msg.max_attempts
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)
    proto.attempt_counter_state_name = ros_msg.attempt_counter_state_name

def convert_proto_to_bosdyn_msgs_for_duration(proto, ros_msg):
    convert_proto_to_builtin_interfaces_duration(proto.duration, ros_msg.duration)
    ros_msg.duration_is_set = proto.HasField("duration")
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")
    ros_msg.time_remaining_name = proto.time_remaining_name
    convert_proto_to_bosdyn_msgs_node(proto.timeout_child, ros_msg.timeout_child)
    ros_msg.timeout_child_is_set = proto.HasField("timeout_child")

def convert_bosdyn_msgs_for_duration_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.duration_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.duration, proto.duration)
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)
    proto.time_remaining_name = ros_msg.time_remaining_name
    if ros_msg.timeout_child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.timeout_child, proto.timeout_child)

def convert_proto_to_bosdyn_msgs_simple_parallel(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_node(proto.primary, ros_msg.primary)
    ros_msg.primary_is_set = proto.HasField("primary")
    convert_proto_to_bosdyn_msgs_node(proto.secondary, ros_msg.secondary)
    ros_msg.secondary_is_set = proto.HasField("secondary")

def convert_bosdyn_msgs_simple_parallel_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.primary_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.primary, proto.primary)
    if ros_msg.secondary_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.secondary, proto.secondary)

def convert_proto_to_bosdyn_msgs_condition_operand_one_of_type(proto, ros_msg):
    if proto.HasField("var"):
        ros_msg.type_choice = ros_msg.TYPE_VAR_SET
        convert_proto_to_bosdyn_msgs_variable_declaration(proto.var, ros_msg.var)
    if proto.HasField("const"):
        ros_msg.type_choice = ros_msg.TYPE_CONST_SET
        convert_proto_to_bosdyn_msgs_constant_value(proto.const, ros_msg.constant)

def convert_bosdyn_msgs_condition_operand_one_of_type_to_proto(ros_msg, proto):
    proto.ClearField("type")
    if ros_msg.type_choice == ros_msg.TYPE_VAR_SET:
        convert_bosdyn_msgs_variable_declaration_to_proto(ros_msg.var, proto.var)
    if ros_msg.type_choice == ros_msg.TYPE_CONST_SET:
        convert_bosdyn_msgs_constant_value_to_proto(ros_msg.constant, proto.const)

def convert_proto_to_bosdyn_msgs_condition_operand(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_condition_operand_one_of_type(proto, ros_msg.type)

def convert_bosdyn_msgs_condition_operand_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_condition_operand_one_of_type_to_proto(ros_msg.type, proto)

def convert_proto_to_bosdyn_msgs_condition(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_condition_operand(proto.lhs, ros_msg.lhs)
    ros_msg.lhs_is_set = proto.HasField("lhs")
    convert_proto_to_bosdyn_msgs_condition_operand(proto.rhs, ros_msg.rhs)
    ros_msg.rhs_is_set = proto.HasField("rhs")
    ros_msg.operation.value = proto.operation
    ros_msg.handle_staleness.value = proto.handle_staleness

def convert_bosdyn_msgs_condition_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.lhs_is_set:
        convert_bosdyn_msgs_condition_operand_to_proto(ros_msg.lhs, proto.lhs)
    if ros_msg.rhs_is_set:
        convert_bosdyn_msgs_condition_operand_to_proto(ros_msg.rhs, proto.rhs)
    proto.operation = ros_msg.operation.value
    proto.handle_staleness = ros_msg.handle_staleness.value

def convert_proto_to_bosdyn_msgs_bosdyn_robot_state(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")
    ros_msg.state_name = proto.state_name

def convert_bosdyn_msgs_bosdyn_robot_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)
    proto.state_name = ros_msg.state_name

def convert_proto_to_bosdyn_msgs_bosdyn_dock_state(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")
    ros_msg.state_name = proto.state_name

def convert_bosdyn_msgs_bosdyn_dock_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)
    proto.state_name = ros_msg.state_name

def convert_proto_to_bosdyn_msgs_bosdyn_robot_command(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_robot_command(proto.command, ros_msg.command)
    ros_msg.command_is_set = proto.HasField("command")

def convert_bosdyn_msgs_bosdyn_robot_command_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.command_is_set:
        convert_bosdyn_msgs_robot_command_to_proto(ros_msg.command, proto.command)

def convert_proto_to_bosdyn_msgs_bosdyn_power_request(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    ros_msg.request.value = proto.request

def convert_bosdyn_msgs_bosdyn_power_request_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    proto.request = ros_msg.request.value

def convert_proto_to_bosdyn_msgs_bosdyn_navigate_to(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    ros_msg.destination_waypoint_id = proto.destination_waypoint_id
    convert_proto_to_bosdyn_msgs_route_gen_params(proto.route_gen_params, ros_msg.route_gen_params)
    ros_msg.route_gen_params_is_set = proto.HasField("route_gen_params")
    convert_proto_to_bosdyn_msgs_travel_params(proto.travel_params, ros_msg.travel_params)
    ros_msg.travel_params_is_set = proto.HasField("travel_params")
    ros_msg.navigation_feedback_response_blackboard_key = proto.navigation_feedback_response_blackboard_key
    ros_msg.navigate_to_response_blackboard_key = proto.navigate_to_response_blackboard_key

def convert_bosdyn_msgs_bosdyn_navigate_to_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    proto.destination_waypoint_id = ros_msg.destination_waypoint_id
    if ros_msg.route_gen_params_is_set:
        convert_bosdyn_msgs_route_gen_params_to_proto(ros_msg.route_gen_params, proto.route_gen_params)
    if ros_msg.travel_params_is_set:
        convert_bosdyn_msgs_travel_params_to_proto(ros_msg.travel_params, proto.travel_params)
    proto.navigation_feedback_response_blackboard_key = ros_msg.navigation_feedback_response_blackboard_key
    proto.navigate_to_response_blackboard_key = ros_msg.navigate_to_response_blackboard_key

def convert_proto_to_bosdyn_msgs_bosdyn_navigate_route(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_route(proto.route, ros_msg.route)
    ros_msg.route_is_set = proto.HasField("route")
    convert_proto_to_bosdyn_msgs_route_following_params(proto.route_follow_params, ros_msg.route_follow_params)
    ros_msg.route_follow_params_is_set = proto.HasField("route_follow_params")
    convert_proto_to_bosdyn_msgs_travel_params(proto.travel_params, ros_msg.travel_params)
    ros_msg.travel_params_is_set = proto.HasField("travel_params")
    ros_msg.navigation_feedback_response_blackboard_key = proto.navigation_feedback_response_blackboard_key
    ros_msg.navigate_route_response_blackboard_key = proto.navigate_route_response_blackboard_key

def convert_bosdyn_msgs_bosdyn_navigate_route_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.route_is_set:
        convert_bosdyn_msgs_route_to_proto(ros_msg.route, proto.route)
    if ros_msg.route_follow_params_is_set:
        convert_bosdyn_msgs_route_following_params_to_proto(ros_msg.route_follow_params, proto.route_follow_params)
    if ros_msg.travel_params_is_set:
        convert_bosdyn_msgs_travel_params_to_proto(ros_msg.travel_params, proto.travel_params)
    proto.navigation_feedback_response_blackboard_key = ros_msg.navigation_feedback_response_blackboard_key
    proto.navigate_route_response_blackboard_key = ros_msg.navigate_route_response_blackboard_key

def convert_proto_to_bosdyn_msgs_bosdyn_graph_nav_state(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")
    ros_msg.state_name = proto.state_name
    ros_msg.waypoint_id = proto.waypoint_id

def convert_bosdyn_msgs_bosdyn_graph_nav_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)
    proto.state_name = ros_msg.state_name
    proto.waypoint_id = ros_msg.waypoint_id

def convert_proto_to_bosdyn_msgs_bosdyn_graph_nav_localize(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_set_localization_request(proto.localization_request, ros_msg.localization_request)
    ros_msg.localization_request_is_set = proto.HasField("localization_request")

def convert_bosdyn_msgs_bosdyn_graph_nav_localize_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.localization_request_is_set:
        convert_bosdyn_msgs_set_localization_request_to_proto(ros_msg.localization_request, proto.localization_request)

def convert_proto_to_bosdyn_msgs_bosdyn_record_event(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_event(proto.event, ros_msg.event)
    ros_msg.event_is_set = proto.HasField("event")
    ros_msg.succeed_early = proto.succeed_early
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsValue
    ros_msg.additional_parameters = []
    for _item in proto.additional_parameters:
        ros_msg.additional_parameters.append(KeyStringValueBosdynMsgsValue())
        ros_msg.additional_parameters[-1].key = _item
        convert_proto_to_bosdyn_msgs_value(proto.additional_parameters[_item], ros_msg.additional_parameters[-1].value)

def convert_bosdyn_msgs_bosdyn_record_event_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.event_is_set:
        convert_bosdyn_msgs_event_to_proto(ros_msg.event, proto.event)
    proto.succeed_early = ros_msg.succeed_early
    for _item in ros_msg.additional_parameters:
        convert_bosdyn_msgs_value_to_proto(_item.value, proto.additional_parameters[_item.key])

def convert_proto_to_bosdyn_msgs_remote_grpc(proto, ros_msg):
    ros_msg.host = proto.host
    ros_msg.service_name = proto.service_name
    ros_msg.timeout = proto.timeout
    ros_msg.lease_resources = []
    for _item in proto.lease_resources:
        ros_msg.lease_resources.append(_item)
    from bosdyn_msgs.msg import KeyValue
    ros_msg.inputs = []
    for _item in proto.inputs:
        ros_msg.inputs.append(KeyValue())
        convert_proto_to_bosdyn_msgs_key_value(_item, ros_msg.inputs[-1])

def convert_bosdyn_msgs_remote_grpc_to_proto(ros_msg, proto):
    proto.Clear()
    proto.host = ros_msg.host
    proto.service_name = ros_msg.service_name
    proto.timeout = ros_msg.timeout
    del proto.lease_resources[:]
    for _item in ros_msg.lease_resources:
        proto.lease_resources.add(_item)
    del proto.inputs[:]
    for _item in ros_msg.inputs:
        convert_bosdyn_msgs_key_value_to_proto(_item, proto.inputs.add())

def convert_proto_to_bosdyn_msgs_sleep(proto, ros_msg):
    ros_msg.seconds = proto.seconds
    ros_msg.restart_after_stop = proto.restart_after_stop

def convert_bosdyn_msgs_sleep_to_proto(ros_msg, proto):
    proto.Clear()
    proto.seconds = ros_msg.seconds
    proto.restart_after_stop = ros_msg.restart_after_stop

def convert_proto_to_bosdyn_msgs_prompt_option(proto, ros_msg):
    ros_msg.text = proto.text
    ros_msg.answer_code = proto.answer_code

def convert_bosdyn_msgs_prompt_option_to_proto(ros_msg, proto):
    proto.Clear()
    proto.text = ros_msg.text
    proto.answer_code = ros_msg.answer_code

def convert_proto_to_bosdyn_msgs_prompt(proto, ros_msg):
    ros_msg.always_reprompt = proto.always_reprompt
    ros_msg.text = proto.text
    ros_msg.source = proto.source
    from bosdyn_msgs.msg import Option
    ros_msg.options = []
    for _item in proto.options:
        ros_msg.options.append(Option())
        convert_proto_to_bosdyn_msgs_prompt_option(_item, ros_msg.options[-1])
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")
    ros_msg.for_autonomous_processing = proto.for_autonomous_processing
    ros_msg.severity.value = proto.severity

def convert_bosdyn_msgs_prompt_to_proto(ros_msg, proto):
    proto.Clear()
    proto.always_reprompt = ros_msg.always_reprompt
    proto.text = ros_msg.text
    proto.source = ros_msg.source
    del proto.options[:]
    for _item in ros_msg.options:
        convert_bosdyn_msgs_prompt_option_to_proto(_item, proto.options.add())
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)
    proto.for_autonomous_processing = ros_msg.for_autonomous_processing
    proto.severity = ros_msg.severity.value

def convert_proto_to_bosdyn_msgs_bosdyn_gripper_camera_params_state(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")
    ros_msg.state_name = proto.state_name

def convert_bosdyn_msgs_bosdyn_gripper_camera_params_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)
    proto.state_name = ros_msg.state_name

def convert_proto_to_bosdyn_msgs_set_gripper_camera_params_one_of_params(proto, ros_msg):
    if proto.HasField("params_in_blackboard_key"):
        ros_msg.params_choice = ros_msg.PARAMS_PARAMS_IN_BLACKBOARD_KEY_SET
        ros_msg.params_in_blackboard_key = proto.params_in_blackboard_key
    if proto.HasField("new_params"):
        ros_msg.params_choice = ros_msg.PARAMS_NEW_PARAMS_SET
        convert_proto_to_bosdyn_msgs_gripper_camera_params(proto.new_params, ros_msg.new_params)

def convert_bosdyn_msgs_set_gripper_camera_params_one_of_params_to_proto(ros_msg, proto):
    proto.ClearField("params")
    if ros_msg.params_choice == ros_msg.PARAMS_PARAMS_IN_BLACKBOARD_KEY_SET:
        proto.params_in_blackboard_key = ros_msg.params_in_blackboard_key
    if ros_msg.params_choice == ros_msg.PARAMS_NEW_PARAMS_SET:
        convert_bosdyn_msgs_gripper_camera_params_to_proto(ros_msg.new_params, proto.new_params)

def convert_proto_to_bosdyn_msgs_set_gripper_camera_params(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_set_gripper_camera_params_one_of_params(proto, ros_msg.params)

def convert_bosdyn_msgs_set_gripper_camera_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    convert_bosdyn_msgs_set_gripper_camera_params_one_of_params_to_proto(ros_msg.params, proto)

def convert_proto_to_bosdyn_msgs_spot_cam_ptz_adjust_parameters(proto, ros_msg):
    ros_msg.localization_varname = proto.localization_varname
    ros_msg.waypoint_id = proto.waypoint_id
    convert_proto_to_geometry_msgs_pose(proto.waypoint_tform_body, ros_msg.waypoint_tform_body)
    ros_msg.waypoint_tform_body_is_set = proto.HasField("waypoint_tform_body")

def convert_bosdyn_msgs_spot_cam_ptz_adjust_parameters_to_proto(ros_msg, proto):
    proto.Clear()
    proto.localization_varname = ros_msg.localization_varname
    proto.waypoint_id = ros_msg.waypoint_id
    if ros_msg.waypoint_tform_body_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.waypoint_tform_body, proto.waypoint_tform_body)

def convert_proto_to_bosdyn_msgs_spot_cam_ptz(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_ptz_position(proto.ptz_position, ros_msg.ptz_position)
    ros_msg.ptz_position_is_set = proto.HasField("ptz_position")
    convert_proto_to_bosdyn_msgs_spot_cam_ptz_adjust_parameters(proto.adjust_parameters, ros_msg.adjust_parameters)
    ros_msg.adjust_parameters_is_set = proto.HasField("adjust_parameters")

def convert_bosdyn_msgs_spot_cam_ptz_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.ptz_position_is_set:
        convert_bosdyn_msgs_ptz_position_to_proto(ros_msg.ptz_position, proto.ptz_position)
    if ros_msg.adjust_parameters_is_set:
        convert_bosdyn_msgs_spot_cam_ptz_adjust_parameters_to_proto(ros_msg.adjust_parameters, proto.adjust_parameters)

def convert_proto_to_bosdyn_msgs_spot_cam_store_media(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_camera(proto.camera, ros_msg.camera)
    ros_msg.camera_is_set = proto.HasField("camera")
    ros_msg.type.value = proto.type
    ros_msg.tag = proto.tag

def convert_bosdyn_msgs_spot_cam_store_media_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.camera_is_set:
        convert_bosdyn_msgs_camera_to_proto(ros_msg.camera, proto.camera)
    proto.type = ros_msg.type.value
    proto.tag = ros_msg.tag

def convert_proto_to_bosdyn_msgs_spot_cam_led(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    from bosdyn_msgs.msg import KeyInt32ValueFloat32
    ros_msg.brightnesses = []
    for _item in proto.brightnesses:
        ros_msg.brightnesses.append(KeyInt32ValueFloat32())
        ros_msg.brightnesses[-1].key = _item
        ros_msg.brightnesses[-1].value = proto.brightnesses[_item]

def convert_bosdyn_msgs_spot_cam_led_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    for _item in ros_msg.brightnesses:
        proto.brightnesses[_item.key] = _item.value

def convert_proto_to_bosdyn_msgs_spot_cam_reset_autofocus(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host

def convert_bosdyn_msgs_spot_cam_reset_autofocus_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host

def convert_proto_to_bosdyn_msgs_dock(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    ros_msg.docking_station_id = proto.docking_station_id
    ros_msg.prep_pose_behavior.value = proto.prep_pose_behavior
    ros_msg.docking_command_feedback_response_blackboard_key = proto.docking_command_feedback_response_blackboard_key
    ros_msg.docking_command_response_blackboard_key = proto.docking_command_response_blackboard_key

def convert_bosdyn_msgs_dock_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    proto.docking_station_id = ros_msg.docking_station_id
    proto.prep_pose_behavior = ros_msg.prep_pose_behavior.value
    proto.docking_command_feedback_response_blackboard_key = ros_msg.docking_command_feedback_response_blackboard_key
    proto.docking_command_response_blackboard_key = ros_msg.docking_command_response_blackboard_key

def convert_proto_to_bosdyn_msgs_store_metadata(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    ros_msg.acquire_data_request_name = proto.acquire_data_request_name
    ros_msg.metadata_name = proto.metadata_name
    ros_msg.metadata_channel = proto.metadata_channel

def convert_bosdyn_msgs_store_metadata_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    proto.acquire_data_request_name = ros_msg.acquire_data_request_name
    proto.metadata_name = ros_msg.metadata_name
    proto.metadata_channel = ros_msg.metadata_channel

def convert_proto_to_bosdyn_msgs_data_acquisition(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    convert_proto_to_bosdyn_msgs_acquire_data_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")
    ros_msg.completion_behavior.value = proto.completion_behavior
    ros_msg.group_name_format = proto.group_name_format
    ros_msg.request_name_in_blackboard = proto.request_name_in_blackboard

def convert_bosdyn_msgs_data_acquisition_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_acquire_data_request_to_proto(ros_msg.request, proto.request)
    proto.completion_behavior = ros_msg.completion_behavior.value
    proto.group_name_format = ros_msg.group_name_format
    proto.request_name_in_blackboard = ros_msg.request_name_in_blackboard

def convert_proto_to_bosdyn_msgs_retain_lease(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host

def convert_bosdyn_msgs_retain_lease_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host

def convert_proto_to_bosdyn_msgs_define_blackboard(proto, ros_msg):
    from bosdyn_msgs.msg import KeyValue
    ros_msg.blackboard_variables = []
    for _item in proto.blackboard_variables:
        ros_msg.blackboard_variables.append(KeyValue())
        convert_proto_to_bosdyn_msgs_key_value(_item, ros_msg.blackboard_variables[-1])
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")

def convert_bosdyn_msgs_define_blackboard_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.blackboard_variables[:]
    for _item in ros_msg.blackboard_variables:
        convert_bosdyn_msgs_key_value_to_proto(_item, proto.blackboard_variables.add())
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)

def convert_proto_to_bosdyn_msgs_set_blackboard(proto, ros_msg):
    from bosdyn_msgs.msg import KeyValue
    ros_msg.blackboard_variables = []
    for _item in proto.blackboard_variables:
        ros_msg.blackboard_variables.append(KeyValue())
        convert_proto_to_bosdyn_msgs_key_value(_item, ros_msg.blackboard_variables[-1])

def convert_bosdyn_msgs_set_blackboard_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.blackboard_variables[:]
    for _item in ros_msg.blackboard_variables:
        convert_bosdyn_msgs_key_value_to_proto(_item, proto.blackboard_variables.add())

def convert_proto_to_bosdyn_msgs_format_blackboard(proto, ros_msg):
    ros_msg.key = proto.key
    ros_msg.format = proto.format

def convert_bosdyn_msgs_format_blackboard_to_proto(ros_msg, proto):
    proto.Clear()
    proto.key = ros_msg.key
    proto.format = ros_msg.format

def convert_proto_to_bosdyn_msgs_date_to_blackboard(proto, ros_msg):
    ros_msg.key = proto.key

def convert_bosdyn_msgs_date_to_blackboard_to_proto(ros_msg, proto):
    proto.Clear()
    proto.key = ros_msg.key

def convert_proto_to_bosdyn_msgs_constant_result(proto, ros_msg):
    ros_msg.result.value = proto.result

def convert_bosdyn_msgs_constant_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.result = ros_msg.result.value

def convert_proto_to_bosdyn_msgs_restart_when_paused(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_node(proto.child, ros_msg.child)
    ros_msg.child_is_set = proto.HasField("child")

def convert_bosdyn_msgs_restart_when_paused_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.child_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.child, proto.child)

def convert_proto_to_bosdyn_msgs_clear_behavior_faults(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    ros_msg.host = proto.host
    ros_msg.robot_state_blackboard_name = proto.robot_state_blackboard_name
    ros_msg.cleared_cause_fall_blackboard_name = proto.cleared_cause_fall_blackboard_name
    ros_msg.cleared_cause_hardware_blackboard_name = proto.cleared_cause_hardware_blackboard_name
    ros_msg.cleared_cause_lease_timeout_blackboard_name = proto.cleared_cause_lease_timeout_blackboard_name

def convert_bosdyn_msgs_clear_behavior_faults_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    proto.host = ros_msg.host
    proto.robot_state_blackboard_name = ros_msg.robot_state_blackboard_name
    proto.cleared_cause_fall_blackboard_name = ros_msg.cleared_cause_fall_blackboard_name
    proto.cleared_cause_hardware_blackboard_name = ros_msg.cleared_cause_hardware_blackboard_name
    proto.cleared_cause_lease_timeout_blackboard_name = ros_msg.cleared_cause_lease_timeout_blackboard_name

def convert_proto_to_bosdyn_msgs_get_state_request_one_of_lower_bound(proto, ros_msg):
    if proto.HasField("history_lower_tick_bound"):
        ros_msg.lower_bound_choice = ros_msg.LOWER_BOUND_HISTORY_LOWER_TICK_BOUND_SET
        ros_msg.history_lower_tick_bound = proto.history_lower_tick_bound
    if proto.HasField("history_past_ticks"):
        ros_msg.lower_bound_choice = ros_msg.LOWER_BOUND_HISTORY_PAST_TICKS_SET
        ros_msg.history_past_ticks = proto.history_past_ticks

def convert_bosdyn_msgs_get_state_request_one_of_lower_bound_to_proto(ros_msg, proto):
    proto.ClearField("lower_bound")
    if ros_msg.lower_bound_choice == ros_msg.LOWER_BOUND_HISTORY_LOWER_TICK_BOUND_SET:
        proto.history_lower_tick_bound = ros_msg.history_lower_tick_bound
    if ros_msg.lower_bound_choice == ros_msg.LOWER_BOUND_HISTORY_PAST_TICKS_SET:
        proto.history_past_ticks = ros_msg.history_past_ticks

def convert_proto_to_bosdyn_msgs_get_state_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.history_upper_tick_bound = proto.history_upper_tick_bound.value
    ros_msg.history_upper_tick_bound_is_set = proto.HasField("history_upper_tick_bound")
    convert_proto_to_bosdyn_msgs_get_state_request_one_of_lower_bound(proto, ros_msg.lower_bound)

def convert_bosdyn_msgs_get_state_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.history_upper_tick_bound_is_set:
        convert_int64_to_proto(ros_msg.history_upper_tick_bound, proto.history_upper_tick_bound)
    convert_bosdyn_msgs_get_state_request_one_of_lower_bound_to_proto(ros_msg.lower_bound, proto)

def convert_proto_to_bosdyn_msgs_get_state_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_state(proto.state, ros_msg.state)
    ros_msg.state_is_set = proto.HasField("state")

def convert_bosdyn_msgs_get_state_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.state_is_set:
        convert_bosdyn_msgs_state_to_proto(ros_msg.state, proto.state)

def convert_proto_to_bosdyn_msgs_state_answered_question(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_question(proto.question, ros_msg.question)
    ros_msg.question_is_set = proto.HasField("question")
    ros_msg.accepted_answer_code = proto.accepted_answer_code

def convert_bosdyn_msgs_state_answered_question_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.question_is_set:
        convert_bosdyn_msgs_question_to_proto(ros_msg.question, proto.question)
    proto.accepted_answer_code = ros_msg.accepted_answer_code

def convert_proto_to_bosdyn_msgs_state_node_states_at_tick_node_state(proto, ros_msg):
    ros_msg.result.value = proto.result
    ros_msg.error = proto.error
    ros_msg.id = proto.id

def convert_bosdyn_msgs_state_node_states_at_tick_node_state_to_proto(ros_msg, proto):
    proto.Clear()
    proto.result = ros_msg.result.value
    proto.error = ros_msg.error
    proto.id = ros_msg.id

def convert_proto_to_bosdyn_msgs_state_node_states_at_tick(proto, ros_msg):
    ros_msg.tick_counter = proto.tick_counter
    convert_proto_to_builtin_interfaces_time(proto.tick_start_timestamp, ros_msg.tick_start_timestamp)
    ros_msg.tick_start_timestamp_is_set = proto.HasField("tick_start_timestamp")
    from bosdyn_msgs.msg import NodeState
    ros_msg.node_states = []
    for _item in proto.node_states:
        ros_msg.node_states.append(NodeState())
        convert_proto_to_bosdyn_msgs_state_node_states_at_tick_node_state(_item, ros_msg.node_states[-1])

def convert_bosdyn_msgs_state_node_states_at_tick_to_proto(ros_msg, proto):
    proto.Clear()
    proto.tick_counter = ros_msg.tick_counter
    if ros_msg.tick_start_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.tick_start_timestamp, proto.tick_start_timestamp)
    del proto.node_states[:]
    for _item in ros_msg.node_states:
        convert_bosdyn_msgs_state_node_states_at_tick_node_state_to_proto(_item, proto.node_states.add())

def convert_proto_to_bosdyn_msgs_state(proto, ros_msg):
    from bosdyn_msgs.msg import Question
    ros_msg.questions = []
    for _item in proto.questions:
        ros_msg.questions.append(Question())
        convert_proto_to_bosdyn_msgs_question(_item, ros_msg.questions[-1])
    from bosdyn_msgs.msg import AnsweredQuestion
    ros_msg.answered_questions = []
    for _item in proto.answered_questions:
        ros_msg.answered_questions.append(AnsweredQuestion())
        convert_proto_to_bosdyn_msgs_state_answered_question(_item, ros_msg.answered_questions[-1])
    from bosdyn_msgs.msg import NodeStatesAtTick
    ros_msg.history = []
    for _item in proto.history:
        ros_msg.history.append(NodeStatesAtTick())
        convert_proto_to_bosdyn_msgs_state_node_states_at_tick(_item, ros_msg.history[-1])
    ros_msg.status.value = proto.status
    ros_msg.error = proto.error
    ros_msg.tick_counter = proto.tick_counter
    ros_msg.mission_id = proto.mission_id

def convert_bosdyn_msgs_state_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.questions[:]
    for _item in ros_msg.questions:
        convert_bosdyn_msgs_question_to_proto(_item, proto.questions.add())
    del proto.answered_questions[:]
    for _item in ros_msg.answered_questions:
        convert_bosdyn_msgs_state_answered_question_to_proto(_item, proto.answered_questions.add())
    del proto.history[:]
    for _item in ros_msg.history:
        convert_bosdyn_msgs_state_node_states_at_tick_to_proto(_item, proto.history.add())
    proto.status = ros_msg.status.value
    proto.error = ros_msg.error
    proto.tick_counter = ros_msg.tick_counter
    proto.mission_id = ros_msg.mission_id

def convert_proto_to_bosdyn_msgs_question(proto, ros_msg):
    ros_msg.id = proto.id
    ros_msg.source = proto.source
    ros_msg.text = proto.text
    from bosdyn_msgs.msg import PromptOption
    ros_msg.options = []
    for _item in proto.options:
        ros_msg.options.append(PromptOption())
        convert_proto_to_bosdyn_msgs_prompt_option(_item, ros_msg.options[-1])
    ros_msg.for_autonomous_processing = proto.for_autonomous_processing
    ros_msg.severity.value = proto.severity

def convert_bosdyn_msgs_question_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    proto.source = ros_msg.source
    proto.text = ros_msg.text
    del proto.options[:]
    for _item in ros_msg.options:
        convert_bosdyn_msgs_prompt_option_to_proto(_item, proto.options.add())
    proto.for_autonomous_processing = ros_msg.for_autonomous_processing
    proto.severity = ros_msg.severity.value

def convert_proto_to_bosdyn_msgs_answer_question_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.question_id = proto.question_id
    ros_msg.code = proto.code

def convert_bosdyn_msgs_answer_question_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.question_id = ros_msg.question_id
    proto.code = ros_msg.code

def convert_proto_to_bosdyn_msgs_answer_question_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_answer_question_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value

def convert_proto_to_bosdyn_msgs_mission_info(proto, ros_msg):
    ros_msg.id = proto.id
    convert_proto_to_bosdyn_msgs_node_info(proto.root, ros_msg.root)
    ros_msg.root_is_set = proto.HasField("root")

def convert_bosdyn_msgs_mission_info_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    if ros_msg.root_is_set:
        convert_bosdyn_msgs_node_info_to_proto(ros_msg.root, proto.root)

def convert_proto_to_bosdyn_msgs_node_info(proto, ros_msg):
    ros_msg.id = proto.id
    ros_msg.name = proto.name
    convert_proto_to_bosdyn_msgs_user_data(proto.user_data, ros_msg.user_data)
    ros_msg.user_data_is_set = proto.HasField("user_data")
    from bosdyn_msgs.msg import SerializedMessage
    ros_msg.children = []
    for _item in proto.children:
        ros_msg.children.append(SerializedMessage())
        convert_proto_to_serialized_bosdyn_msgs_node_info(_item, ros_msg.children[-1])

def convert_bosdyn_msgs_node_info_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    proto.name = ros_msg.name
    if ros_msg.user_data_is_set:
        convert_bosdyn_msgs_user_data_to_proto(ros_msg.user_data, proto.user_data)
    del proto.children[:]
    for _item in ros_msg.children:
        convert_serialized_bosdyn_msgs_node_info_to_proto(_item, proto.children.add())

def convert_proto_to_bosdyn_msgs_failed_node(proto, ros_msg):
    ros_msg.name = proto.name
    ros_msg.error = proto.error
    ros_msg.impl_typename = proto.impl_typename

def convert_bosdyn_msgs_failed_node_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    proto.error = ros_msg.error
    proto.impl_typename = ros_msg.impl_typename

def convert_proto_to_bosdyn_msgs_play_mission_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_builtin_interfaces_time(proto.pause_time, ros_msg.pause_time)
    ros_msg.pause_time_is_set = proto.HasField("pause_time")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    convert_proto_to_bosdyn_msgs_play_settings(proto.settings, ros_msg.settings)
    ros_msg.settings_is_set = proto.HasField("settings")

def convert_bosdyn_msgs_play_mission_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.pause_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.pause_time, proto.pause_time)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    if ros_msg.settings_is_set:
        convert_bosdyn_msgs_play_settings_to_proto(ros_msg.settings, proto.settings)

def convert_proto_to_bosdyn_msgs_play_settings(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_se2_velocity_limit(proto.velocity_limit, ros_msg.velocity_limit)
    ros_msg.velocity_limit_is_set = proto.HasField("velocity_limit")
    ros_msg.disable_directed_exploration = proto.disable_directed_exploration
    ros_msg.disable_alternate_route_finding = proto.disable_alternate_route_finding
    ros_msg.path_following_mode.value = proto.path_following_mode
    ros_msg.ground_clutter_mode.value = proto.ground_clutter_mode

def convert_bosdyn_msgs_play_settings_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.velocity_limit_is_set:
        convert_bosdyn_msgs_se2_velocity_limit_to_proto(ros_msg.velocity_limit, proto.velocity_limit)
    proto.disable_directed_exploration = ros_msg.disable_directed_exploration
    proto.disable_alternate_route_finding = ros_msg.disable_alternate_route_finding
    proto.path_following_mode = ros_msg.path_following_mode.value
    proto.ground_clutter_mode = ros_msg.ground_clutter_mode.value

def convert_proto_to_bosdyn_msgs_play_mission_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])

def convert_bosdyn_msgs_play_mission_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())

def convert_proto_to_bosdyn_msgs_restart_mission_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_builtin_interfaces_time(proto.pause_time, ros_msg.pause_time)
    ros_msg.pause_time_is_set = proto.HasField("pause_time")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    convert_proto_to_bosdyn_msgs_play_settings(proto.settings, ros_msg.settings)
    ros_msg.settings_is_set = proto.HasField("settings")

def convert_bosdyn_msgs_restart_mission_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.pause_time_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.pause_time, proto.pause_time)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    if ros_msg.settings_is_set:
        convert_bosdyn_msgs_play_settings_to_proto(ros_msg.settings, proto.settings)

def convert_proto_to_bosdyn_msgs_restart_mission_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    from bosdyn_msgs.msg import FailedNode
    ros_msg.failed_nodes = []
    for _item in proto.failed_nodes:
        ros_msg.failed_nodes.append(FailedNode())
        convert_proto_to_bosdyn_msgs_failed_node(_item, ros_msg.failed_nodes[-1])

def convert_bosdyn_msgs_restart_mission_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    del proto.failed_nodes[:]
    for _item in ros_msg.failed_nodes:
        convert_bosdyn_msgs_failed_node_to_proto(_item, proto.failed_nodes.add())

def convert_proto_to_bosdyn_msgs_load_mission_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_node(proto.root, ros_msg.root)
    ros_msg.root_is_set = proto.HasField("root")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])

def convert_bosdyn_msgs_load_mission_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.root_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.root, proto.root)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())

def convert_proto_to_bosdyn_msgs_load_mission_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    convert_proto_to_bosdyn_msgs_mission_info(proto.mission_info, ros_msg.mission_info)
    ros_msg.mission_info_is_set = proto.HasField("mission_info")
    from bosdyn_msgs.msg import FailedNode
    ros_msg.failed_nodes = []
    for _item in proto.failed_nodes:
        ros_msg.failed_nodes.append(FailedNode())
        convert_proto_to_bosdyn_msgs_failed_node(_item, ros_msg.failed_nodes[-1])

def convert_bosdyn_msgs_load_mission_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    if ros_msg.mission_info_is_set:
        convert_bosdyn_msgs_mission_info_to_proto(ros_msg.mission_info, proto.mission_info)
    del proto.failed_nodes[:]
    for _item in ros_msg.failed_nodes:
        convert_bosdyn_msgs_failed_node_to_proto(_item, proto.failed_nodes.add())

def convert_proto_to_bosdyn_msgs_get_info_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_info_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_info_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_mission_info(proto.mission_info, ros_msg.mission_info)
    ros_msg.mission_info_is_set = proto.HasField("mission_info")

def convert_bosdyn_msgs_get_info_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.mission_info_is_set:
        convert_bosdyn_msgs_mission_info_to_proto(ros_msg.mission_info, proto.mission_info)

def convert_proto_to_bosdyn_msgs_pause_mission_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_pause_mission_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_pause_mission_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_pause_mission_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_stop_mission_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")

def convert_bosdyn_msgs_stop_mission_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)

def convert_proto_to_bosdyn_msgs_stop_mission_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_stop_mission_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_get_mission_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_mission_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_mission_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_node(proto.root, ros_msg.root)
    ros_msg.root_is_set = proto.HasField("root")
    ros_msg.id = proto.id

def convert_bosdyn_msgs_get_mission_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.root_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.root, proto.root)
    proto.id = ros_msg.id

def convert_proto_to_bosdyn_msgs_key_value(proto, ros_msg):
    ros_msg.key = proto.key
    convert_proto_to_bosdyn_msgs_value(proto.value, ros_msg.value)
    ros_msg.value_is_set = proto.HasField("value")

def convert_bosdyn_msgs_key_value_to_proto(ros_msg, proto):
    proto.Clear()
    proto.key = ros_msg.key
    if ros_msg.value_is_set:
        convert_bosdyn_msgs_value_to_proto(ros_msg.value, proto.value)

def convert_proto_to_bosdyn_msgs_value_one_of_source(proto, ros_msg):
    if proto.HasField("constant"):
        ros_msg.source_choice = ros_msg.SOURCE_CONSTANT_SET
        convert_proto_to_bosdyn_msgs_constant_value(proto.constant, ros_msg.constant)
    if proto.HasField("runtime_var"):
        ros_msg.source_choice = ros_msg.SOURCE_RUNTIME_VAR_SET
        convert_proto_to_bosdyn_msgs_variable_declaration(proto.runtime_var, ros_msg.runtime_var)
    if proto.HasField("parameter"):
        ros_msg.source_choice = ros_msg.SOURCE_PARAMETER_SET
        convert_proto_to_bosdyn_msgs_variable_declaration(proto.parameter, ros_msg.parameter)

def convert_bosdyn_msgs_value_one_of_source_to_proto(ros_msg, proto):
    proto.ClearField("source")
    if ros_msg.source_choice == ros_msg.SOURCE_CONSTANT_SET:
        convert_bosdyn_msgs_constant_value_to_proto(ros_msg.constant, proto.constant)
    if ros_msg.source_choice == ros_msg.SOURCE_RUNTIME_VAR_SET:
        convert_bosdyn_msgs_variable_declaration_to_proto(ros_msg.runtime_var, proto.runtime_var)
    if ros_msg.source_choice == ros_msg.SOURCE_PARAMETER_SET:
        convert_bosdyn_msgs_variable_declaration_to_proto(ros_msg.parameter, proto.parameter)

def convert_proto_to_bosdyn_msgs_value(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_value_one_of_source(proto, ros_msg.source)

def convert_bosdyn_msgs_value_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_value_one_of_source_to_proto(ros_msg.source, proto)

def convert_proto_to_bosdyn_msgs_variable_declaration(proto, ros_msg):
    ros_msg.name = proto.name
    ros_msg.type.value = proto.type

def convert_bosdyn_msgs_variable_declaration_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    proto.type = ros_msg.type.value

def convert_proto_to_bosdyn_msgs_constant_value_one_of_value(proto, ros_msg):
    if proto.HasField("float_value"):
        ros_msg.value_choice = ros_msg.VALUE_FLOAT_VALUE_SET
        ros_msg.float_value = proto.float_value
    if proto.HasField("string_value"):
        ros_msg.value_choice = ros_msg.VALUE_STRING_VALUE_SET
        ros_msg.string_value = proto.string_value
    if proto.HasField("int_value"):
        ros_msg.value_choice = ros_msg.VALUE_INT_VALUE_SET
        ros_msg.int_value = proto.int_value
    if proto.HasField("bool_value"):
        ros_msg.value_choice = ros_msg.VALUE_BOOL_VALUE_SET
        ros_msg.bool_value = proto.bool_value

def convert_bosdyn_msgs_constant_value_one_of_value_to_proto(ros_msg, proto):
    proto.ClearField("value")
    if ros_msg.value_choice == ros_msg.VALUE_FLOAT_VALUE_SET:
        proto.float_value = ros_msg.float_value
    if ros_msg.value_choice == ros_msg.VALUE_STRING_VALUE_SET:
        proto.string_value = ros_msg.string_value
    if ros_msg.value_choice == ros_msg.VALUE_INT_VALUE_SET:
        proto.int_value = ros_msg.int_value
    if ros_msg.value_choice == ros_msg.VALUE_BOOL_VALUE_SET:
        proto.bool_value = ros_msg.bool_value

def convert_proto_to_bosdyn_msgs_constant_value(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_constant_value_one_of_value(proto, ros_msg.value)

def convert_bosdyn_msgs_constant_value_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_constant_value_one_of_value_to_proto(ros_msg.value, proto)

def convert_proto_to_bosdyn_msgs_user_data(proto, ros_msg):
    ros_msg.id = proto.id
    ros_msg.bytestring = proto.bytestring

def convert_bosdyn_msgs_user_data_to_proto(ros_msg, proto):
    proto.Clear()
    proto.id = ros_msg.id
    proto.bytestring = ros_msg.bytestring

def convert_proto_to_bosdyn_msgs_spot_check_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    ros_msg.command.value = proto.command

def convert_bosdyn_msgs_spot_check_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    proto.command = ros_msg.command.value

def convert_proto_to_bosdyn_msgs_spot_check_command_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    ros_msg.message = proto.message

def convert_bosdyn_msgs_spot_check_command_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    proto.message = ros_msg.message

def convert_proto_to_bosdyn_msgs_spot_check_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_spot_check_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_spot_check_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.state.value = proto.state
    ros_msg.last_command.value = proto.last_command
    ros_msg.error.value = proto.error
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsDepthPlaneSpotCheckResult
    ros_msg.camera_results = []
    for _item in proto.camera_results:
        ros_msg.camera_results.append(KeyStringValueBosdynMsgsDepthPlaneSpotCheckResult())
        ros_msg.camera_results[-1].key = _item
        convert_proto_to_bosdyn_msgs_depth_plane_spot_check_result(proto.camera_results[_item], ros_msg.camera_results[-1].value)
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsLoadCellSpotCheckResult
    ros_msg.load_cell_results = []
    for _item in proto.load_cell_results:
        ros_msg.load_cell_results.append(KeyStringValueBosdynMsgsLoadCellSpotCheckResult())
        ros_msg.load_cell_results[-1].key = _item
        convert_proto_to_bosdyn_msgs_load_cell_spot_check_result(proto.load_cell_results[_item], ros_msg.load_cell_results[-1].value)
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsJointKinematicCheckResult
    ros_msg.kinematic_cal_results = []
    for _item in proto.kinematic_cal_results:
        ros_msg.kinematic_cal_results.append(KeyStringValueBosdynMsgsJointKinematicCheckResult())
        ros_msg.kinematic_cal_results[-1].key = _item
        convert_proto_to_bosdyn_msgs_joint_kinematic_check_result(proto.kinematic_cal_results[_item], ros_msg.kinematic_cal_results[-1].value)
    convert_proto_to_bosdyn_msgs_payload_check_result(proto.payload_result, ros_msg.payload_result)
    ros_msg.payload_result_is_set = proto.HasField("payload_result")
    from bosdyn_msgs.msg import KeyStringValueBosdynMsgsHipRangeOfMotionResult
    ros_msg.hip_range_of_motion_results = []
    for _item in proto.hip_range_of_motion_results:
        ros_msg.hip_range_of_motion_results.append(KeyStringValueBosdynMsgsHipRangeOfMotionResult())
        ros_msg.hip_range_of_motion_results[-1].key = _item
        convert_proto_to_bosdyn_msgs_hip_range_of_motion_result(proto.hip_range_of_motion_results[_item], ros_msg.hip_range_of_motion_results[-1].value)
    ros_msg.progress = proto.progress
    convert_proto_to_builtin_interfaces_time(proto.last_cal_timestamp, ros_msg.last_cal_timestamp)
    ros_msg.last_cal_timestamp_is_set = proto.HasField("last_cal_timestamp")

def convert_bosdyn_msgs_spot_check_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.state = ros_msg.state.value
    proto.last_command = ros_msg.last_command.value
    proto.error = ros_msg.error.value
    for _item in ros_msg.camera_results:
        convert_bosdyn_msgs_depth_plane_spot_check_result_to_proto(_item.value, proto.camera_results[_item.key])
    for _item in ros_msg.load_cell_results:
        convert_bosdyn_msgs_load_cell_spot_check_result_to_proto(_item.value, proto.load_cell_results[_item.key])
    for _item in ros_msg.kinematic_cal_results:
        convert_bosdyn_msgs_joint_kinematic_check_result_to_proto(_item.value, proto.kinematic_cal_results[_item.key])
    if ros_msg.payload_result_is_set:
        convert_bosdyn_msgs_payload_check_result_to_proto(ros_msg.payload_result, proto.payload_result)
    for _item in ros_msg.hip_range_of_motion_results:
        convert_bosdyn_msgs_hip_range_of_motion_result_to_proto(_item.value, proto.hip_range_of_motion_results[_item.key])
    proto.progress = ros_msg.progress
    if ros_msg.last_cal_timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.last_cal_timestamp, proto.last_cal_timestamp)

def convert_proto_to_bosdyn_msgs_depth_plane_spot_check_result(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.severity_score = proto.severity_score

def convert_bosdyn_msgs_depth_plane_spot_check_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.severity_score = ros_msg.severity_score

def convert_proto_to_bosdyn_msgs_payload_check_result(proto, ros_msg):
    ros_msg.error.value = proto.error
    ros_msg.extra_payload = proto.extra_payload

def convert_bosdyn_msgs_payload_check_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.error = ros_msg.error.value
    proto.extra_payload = ros_msg.extra_payload

def convert_proto_to_bosdyn_msgs_load_cell_spot_check_result(proto, ros_msg):
    ros_msg.error.value = proto.error
    ros_msg.zero = proto.zero
    ros_msg.old_zero = proto.old_zero

def convert_bosdyn_msgs_load_cell_spot_check_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.error = ros_msg.error.value
    proto.zero = ros_msg.zero
    proto.old_zero = ros_msg.old_zero

def convert_proto_to_bosdyn_msgs_joint_kinematic_check_result(proto, ros_msg):
    ros_msg.error.value = proto.error
    ros_msg.offset = proto.offset
    ros_msg.old_offset = proto.old_offset
    ros_msg.health_score = proto.health_score

def convert_bosdyn_msgs_joint_kinematic_check_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.error = ros_msg.error.value
    proto.offset = ros_msg.offset
    proto.old_offset = ros_msg.old_offset
    proto.health_score = ros_msg.health_score

def convert_proto_to_bosdyn_msgs_foot_height_check_result(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.foot_height_error_from_mean = proto.foot_height_error_from_mean

def convert_bosdyn_msgs_foot_height_check_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.foot_height_error_from_mean = ros_msg.foot_height_error_from_mean

def convert_proto_to_bosdyn_msgs_leg_pair_check_result(proto, ros_msg):
    ros_msg.status.value = proto.status
    ros_msg.leg_pair_distance_change = proto.leg_pair_distance_change

def convert_bosdyn_msgs_leg_pair_check_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value
    proto.leg_pair_distance_change = ros_msg.leg_pair_distance_change

def convert_proto_to_bosdyn_msgs_hip_range_of_motion_result(proto, ros_msg):
    ros_msg.error.value = proto.error
    ros_msg.hx = []
    for _item in proto.hx:
        ros_msg.hx.append(_item)
    ros_msg.hy = []
    for _item in proto.hy:
        ros_msg.hy.append(_item)

def convert_bosdyn_msgs_hip_range_of_motion_result_to_proto(ros_msg, proto):
    proto.Clear()
    proto.error = ros_msg.error.value
    del proto.hx[:]
    for _item in ros_msg.hx:
        proto.hx.add(_item)
    del proto.hy[:]
    for _item in ros_msg.hy:
        proto.hy.add(_item)

def convert_proto_to_bosdyn_msgs_camera_calibration_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    ros_msg.command.value = proto.command

def convert_bosdyn_msgs_camera_calibration_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    proto.command = ros_msg.command.value

def convert_proto_to_bosdyn_msgs_camera_calibration_command_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")

def convert_bosdyn_msgs_camera_calibration_command_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)

def convert_proto_to_bosdyn_msgs_camera_calibration_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_camera_calibration_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_camera_calibration_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    ros_msg.progress = proto.progress

def convert_bosdyn_msgs_camera_calibration_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    proto.progress = ros_msg.progress

def convert_proto_to_bosdyn_msgs_mobility_params(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_se2_velocity_limit(proto.vel_limit, ros_msg.vel_limit)
    ros_msg.vel_limit_is_set = proto.HasField("vel_limit")
    convert_proto_to_bosdyn_msgs_body_control_params(proto.body_control, ros_msg.body_control)
    ros_msg.body_control_is_set = proto.HasField("body_control")
    ros_msg.locomotion_hint.value = proto.locomotion_hint
    ros_msg.stairs_mode.value = proto.stairs_mode
    ros_msg.allow_degraded_perception = proto.allow_degraded_perception
    convert_proto_to_bosdyn_msgs_obstacle_params(proto.obstacle_params, ros_msg.obstacle_params)
    ros_msg.obstacle_params_is_set = proto.HasField("obstacle_params")
    ros_msg.swing_height.value = proto.swing_height
    convert_proto_to_bosdyn_msgs_terrain_params(proto.terrain_params, ros_msg.terrain_params)
    ros_msg.terrain_params_is_set = proto.HasField("terrain_params")
    ros_msg.disallow_stair_tracker = proto.disallow_stair_tracker
    ros_msg.disable_stair_error_auto_descent = proto.disable_stair_error_auto_descent
    convert_proto_to_bosdyn_msgs_body_external_force_params(proto.external_force_params, ros_msg.external_force_params)
    ros_msg.external_force_params_is_set = proto.HasField("external_force_params")
    ros_msg.disallow_non_stairs_pitch_limiting = proto.disallow_non_stairs_pitch_limiting
    ros_msg.disable_nearmap_cliff_avoidance = proto.disable_nearmap_cliff_avoidance

def convert_bosdyn_msgs_mobility_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.vel_limit_is_set:
        convert_bosdyn_msgs_se2_velocity_limit_to_proto(ros_msg.vel_limit, proto.vel_limit)
    if ros_msg.body_control_is_set:
        convert_bosdyn_msgs_body_control_params_to_proto(ros_msg.body_control, proto.body_control)
    proto.locomotion_hint = ros_msg.locomotion_hint.value
    proto.stairs_mode = ros_msg.stairs_mode.value
    proto.allow_degraded_perception = ros_msg.allow_degraded_perception
    if ros_msg.obstacle_params_is_set:
        convert_bosdyn_msgs_obstacle_params_to_proto(ros_msg.obstacle_params, proto.obstacle_params)
    proto.swing_height = ros_msg.swing_height.value
    if ros_msg.terrain_params_is_set:
        convert_bosdyn_msgs_terrain_params_to_proto(ros_msg.terrain_params, proto.terrain_params)
    proto.disallow_stair_tracker = ros_msg.disallow_stair_tracker
    proto.disable_stair_error_auto_descent = ros_msg.disable_stair_error_auto_descent
    if ros_msg.external_force_params_is_set:
        convert_bosdyn_msgs_body_external_force_params_to_proto(ros_msg.external_force_params, proto.external_force_params)
    proto.disallow_non_stairs_pitch_limiting = ros_msg.disallow_non_stairs_pitch_limiting
    proto.disable_nearmap_cliff_avoidance = ros_msg.disable_nearmap_cliff_avoidance

def convert_proto_to_bosdyn_msgs_body_control_params_one_of_param(proto, ros_msg):
    if proto.HasField("base_offset_rt_footprint"):
        ros_msg.param_choice = ros_msg.PARAM_BASE_OFFSET_RT_FOOTPRINT_SET
        convert_proto_to_bosdyn_msgs_se3_trajectory(proto.base_offset_rt_footprint, ros_msg.base_offset_rt_footprint)
    if proto.HasField("body_assist_for_manipulation"):
        ros_msg.param_choice = ros_msg.PARAM_BODY_ASSIST_FOR_MANIPULATION_SET
        convert_proto_to_bosdyn_msgs_body_control_params_body_assist_for_manipulation(proto.body_assist_for_manipulation, ros_msg.body_assist_for_manipulation)

def convert_bosdyn_msgs_body_control_params_one_of_param_to_proto(ros_msg, proto):
    proto.ClearField("param")
    if ros_msg.param_choice == ros_msg.PARAM_BASE_OFFSET_RT_FOOTPRINT_SET:
        convert_bosdyn_msgs_se3_trajectory_to_proto(ros_msg.base_offset_rt_footprint, proto.base_offset_rt_footprint)
    if ros_msg.param_choice == ros_msg.PARAM_BODY_ASSIST_FOR_MANIPULATION_SET:
        convert_bosdyn_msgs_body_control_params_body_assist_for_manipulation_to_proto(ros_msg.body_assist_for_manipulation, proto.body_assist_for_manipulation)

def convert_proto_to_bosdyn_msgs_body_control_params_body_assist_for_manipulation(proto, ros_msg):
    ros_msg.enable_body_yaw_assist = proto.enable_body_yaw_assist
    ros_msg.enable_hip_height_assist = proto.enable_hip_height_assist

def convert_bosdyn_msgs_body_control_params_body_assist_for_manipulation_to_proto(ros_msg, proto):
    proto.Clear()
    proto.enable_body_yaw_assist = ros_msg.enable_body_yaw_assist
    proto.enable_hip_height_assist = ros_msg.enable_hip_height_assist

def convert_proto_to_bosdyn_msgs_body_control_params(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_body_control_params_one_of_param(proto, ros_msg.param)
    ros_msg.rotation_setting.value = proto.rotation_setting

def convert_bosdyn_msgs_body_control_params_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_body_control_params_one_of_param_to_proto(ros_msg.param, proto)
    proto.rotation_setting = ros_msg.rotation_setting.value

def convert_proto_to_bosdyn_msgs_obstacle_params(proto, ros_msg):
    ros_msg.disable_vision_foot_obstacle_avoidance = proto.disable_vision_foot_obstacle_avoidance
    ros_msg.disable_vision_foot_constraint_avoidance = proto.disable_vision_foot_constraint_avoidance
    ros_msg.disable_vision_body_obstacle_avoidance = proto.disable_vision_body_obstacle_avoidance
    ros_msg.obstacle_avoidance_padding = proto.obstacle_avoidance_padding
    ros_msg.disable_vision_foot_obstacle_body_assist = proto.disable_vision_foot_obstacle_body_assist
    ros_msg.disable_vision_negative_obstacles = proto.disable_vision_negative_obstacles

def convert_bosdyn_msgs_obstacle_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.disable_vision_foot_obstacle_avoidance = ros_msg.disable_vision_foot_obstacle_avoidance
    proto.disable_vision_foot_constraint_avoidance = ros_msg.disable_vision_foot_constraint_avoidance
    proto.disable_vision_body_obstacle_avoidance = ros_msg.disable_vision_body_obstacle_avoidance
    proto.obstacle_avoidance_padding = ros_msg.obstacle_avoidance_padding
    proto.disable_vision_foot_obstacle_body_assist = ros_msg.disable_vision_foot_obstacle_body_assist
    proto.disable_vision_negative_obstacles = ros_msg.disable_vision_negative_obstacles

def convert_proto_to_bosdyn_msgs_terrain_params(proto, ros_msg):
    ros_msg.ground_mu_hint = proto.ground_mu_hint.value
    ros_msg.ground_mu_hint_is_set = proto.HasField("ground_mu_hint")
    ros_msg.grated_surfaces_mode.value = proto.grated_surfaces_mode

def convert_bosdyn_msgs_terrain_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.ground_mu_hint_is_set:
        convert_float64_to_proto(ros_msg.ground_mu_hint, proto.ground_mu_hint)
    proto.grated_surfaces_mode = ros_msg.grated_surfaces_mode.value

def convert_proto_to_bosdyn_msgs_body_external_force_params(proto, ros_msg):
    ros_msg.external_force_indicator.value = proto.external_force_indicator
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_vector3(proto.external_force_override, ros_msg.external_force_override)
    ros_msg.external_force_override_is_set = proto.HasField("external_force_override")

def convert_bosdyn_msgs_body_external_force_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.external_force_indicator = ros_msg.external_force_indicator.value
    proto.frame_name = ros_msg.frame_name
    if ros_msg.external_force_override_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.external_force_override, proto.external_force_override)

def convert_proto_to_bosdyn_msgs_open_door_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease(proto.lease, ros_msg.lease)
    ros_msg.lease_is_set = proto.HasField("lease")
    convert_proto_to_bosdyn_msgs_door_command_request(proto.door_command, ros_msg.door_command)
    ros_msg.door_command_is_set = proto.HasField("door_command")

def convert_bosdyn_msgs_open_door_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_is_set:
        convert_bosdyn_msgs_lease_to_proto(ros_msg.lease, proto.lease)
    if ros_msg.door_command_is_set:
        convert_bosdyn_msgs_door_command_request_to_proto(ros_msg.door_command, proto.door_command)

def convert_proto_to_bosdyn_msgs_open_door_command_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_lease_use_result(proto.lease_use_result, ros_msg.lease_use_result)
    ros_msg.lease_use_result_is_set = proto.HasField("lease_use_result")
    ros_msg.status.value = proto.status
    ros_msg.message = proto.message
    ros_msg.door_command_id = proto.door_command_id

def convert_bosdyn_msgs_open_door_command_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.lease_use_result_is_set:
        convert_bosdyn_msgs_lease_use_result_to_proto(ros_msg.lease_use_result, proto.lease_use_result)
    proto.status = ros_msg.status.value
    proto.message = ros_msg.message
    proto.door_command_id = ros_msg.door_command_id

def convert_proto_to_bosdyn_msgs_open_door_feedback_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.door_command_id = proto.door_command_id

def convert_bosdyn_msgs_open_door_feedback_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.door_command_id = ros_msg.door_command_id

def convert_proto_to_bosdyn_msgs_open_door_feedback_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_door_command_feedback(proto.feedback, ros_msg.feedback)
    ros_msg.feedback_is_set = proto.HasField("feedback")

def convert_bosdyn_msgs_open_door_feedback_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.feedback_is_set:
        convert_bosdyn_msgs_door_command_feedback_to_proto(ros_msg.feedback, proto.feedback)

def convert_proto_to_bosdyn_msgs_door_command_auto_grasp_command(proto, ros_msg):
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_vector3(proto.search_ray_start_in_frame, ros_msg.search_ray_start_in_frame)
    ros_msg.search_ray_start_in_frame_is_set = proto.HasField("search_ray_start_in_frame")
    convert_proto_to_geometry_msgs_vector3(proto.search_ray_end_in_frame, ros_msg.search_ray_end_in_frame)
    ros_msg.search_ray_end_in_frame_is_set = proto.HasField("search_ray_end_in_frame")
    ros_msg.hinge_side.value = proto.hinge_side
    ros_msg.swing_direction.value = proto.swing_direction

def convert_bosdyn_msgs_door_command_auto_grasp_command_to_proto(ros_msg, proto):
    proto.Clear()
    proto.frame_name = ros_msg.frame_name
    if ros_msg.search_ray_start_in_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.search_ray_start_in_frame, proto.search_ray_start_in_frame)
    if ros_msg.search_ray_end_in_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.search_ray_end_in_frame, proto.search_ray_end_in_frame)
    proto.hinge_side = ros_msg.hinge_side.value
    proto.swing_direction = ros_msg.swing_direction.value

def convert_proto_to_bosdyn_msgs_door_command_warmstart_command(proto, ros_msg):
    ros_msg.hinge_side.value = proto.hinge_side
    ros_msg.swing_direction.value = proto.swing_direction
    ros_msg.handle_type.value = proto.handle_type

def convert_bosdyn_msgs_door_command_warmstart_command_to_proto(ros_msg, proto):
    proto.Clear()
    proto.hinge_side = ros_msg.hinge_side.value
    proto.swing_direction = ros_msg.swing_direction.value
    proto.handle_type = ros_msg.handle_type.value

def convert_proto_to_bosdyn_msgs_door_command_auto_push_command(proto, ros_msg):
    ros_msg.frame_name = proto.frame_name
    convert_proto_to_geometry_msgs_vector3(proto.push_point_in_frame, ros_msg.push_point_in_frame)
    ros_msg.push_point_in_frame_is_set = proto.HasField("push_point_in_frame")
    ros_msg.hinge_side.value = proto.hinge_side

def convert_bosdyn_msgs_door_command_auto_push_command_to_proto(ros_msg, proto):
    proto.Clear()
    proto.frame_name = ros_msg.frame_name
    if ros_msg.push_point_in_frame_is_set:
        convert_geometry_msgs_vector3_to_proto(ros_msg.push_point_in_frame, proto.push_point_in_frame)
    proto.hinge_side = ros_msg.hinge_side.value

def convert_proto_to_bosdyn_msgs_door_command_request_one_of_command(proto, ros_msg):
    if proto.HasField("auto_grasp_command"):
        ros_msg.command_choice = ros_msg.COMMAND_AUTO_GRASP_COMMAND_SET
        convert_proto_to_bosdyn_msgs_door_command_auto_grasp_command(proto.auto_grasp_command, ros_msg.auto_grasp_command)
    if proto.HasField("warmstart_command"):
        ros_msg.command_choice = ros_msg.COMMAND_WARMSTART_COMMAND_SET
        convert_proto_to_bosdyn_msgs_door_command_warmstart_command(proto.warmstart_command, ros_msg.warmstart_command)
    if proto.HasField("auto_push_command"):
        ros_msg.command_choice = ros_msg.COMMAND_AUTO_PUSH_COMMAND_SET
        convert_proto_to_bosdyn_msgs_door_command_auto_push_command(proto.auto_push_command, ros_msg.auto_push_command)

def convert_bosdyn_msgs_door_command_request_one_of_command_to_proto(ros_msg, proto):
    proto.ClearField("command")
    if ros_msg.command_choice == ros_msg.COMMAND_AUTO_GRASP_COMMAND_SET:
        convert_bosdyn_msgs_door_command_auto_grasp_command_to_proto(ros_msg.auto_grasp_command, proto.auto_grasp_command)
    if ros_msg.command_choice == ros_msg.COMMAND_WARMSTART_COMMAND_SET:
        convert_bosdyn_msgs_door_command_warmstart_command_to_proto(ros_msg.warmstart_command, proto.warmstart_command)
    if ros_msg.command_choice == ros_msg.COMMAND_AUTO_PUSH_COMMAND_SET:
        convert_bosdyn_msgs_door_command_auto_push_command_to_proto(ros_msg.auto_push_command, proto.auto_push_command)

def convert_proto_to_bosdyn_msgs_door_command_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_door_command_request_one_of_command(proto, ros_msg.command)

def convert_bosdyn_msgs_door_command_request_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_door_command_request_one_of_command_to_proto(ros_msg.command, proto)

def convert_proto_to_bosdyn_msgs_door_command_feedback(proto, ros_msg):
    ros_msg.status.value = proto.status

def convert_bosdyn_msgs_door_command_feedback_to_proto(ros_msg, proto):
    proto.Clear()
    proto.status = ros_msg.status.value

def convert_bosdyn_msgs_door_command_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_get_software_version_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_software_version_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_software_version_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_software_version(proto.version, ros_msg.version)
    ros_msg.version_is_set = proto.HasField("version")
    ros_msg.detail = proto.detail

def convert_bosdyn_msgs_get_software_version_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.version_is_set:
        convert_bosdyn_msgs_software_version_to_proto(ros_msg.version, proto.version)
    proto.detail = ros_msg.detail

def convert_proto_to_bosdyn_msgs_temperature(proto, ros_msg):
    ros_msg.channel_name = proto.channel_name
    ros_msg.temperature = proto.temperature

def convert_bosdyn_msgs_temperature_to_proto(ros_msg, proto):
    proto.Clear()
    proto.channel_name = ros_msg.channel_name
    proto.temperature = ros_msg.temperature

def convert_proto_to_bosdyn_msgs_clear_bit_events_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_clear_bit_events_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_clear_bit_events_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_clear_bit_events_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_bit_status_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_bit_status_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_bit_status_response_degradation(proto, ros_msg):
    ros_msg.type.value = proto.type
    ros_msg.description = proto.description

def convert_bosdyn_msgs_get_bit_status_response_degradation_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    proto.description = ros_msg.description

def convert_proto_to_bosdyn_msgs_get_bit_status_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import SystemFault
    ros_msg.events = []
    for _item in proto.events:
        ros_msg.events.append(SystemFault())
        convert_proto_to_bosdyn_msgs_system_fault(_item, ros_msg.events[-1])
    from bosdyn_msgs.msg import Degradation
    ros_msg.degradations = []
    for _item in proto.degradations:
        ros_msg.degradations.append(Degradation())
        convert_proto_to_bosdyn_msgs_get_bit_status_response_degradation(_item, ros_msg.degradations[-1])

def convert_bosdyn_msgs_get_bit_status_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.events[:]
    for _item in ros_msg.events:
        convert_bosdyn_msgs_system_fault_to_proto(_item, proto.events.add())
    del proto.degradations[:]
    for _item in ros_msg.degradations:
        convert_bosdyn_msgs_get_bit_status_response_degradation_to_proto(_item, proto.degradations.add())

def convert_proto_to_bosdyn_msgs_get_temperature_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_temperature_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_temperature_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Temperature
    ros_msg.temps = []
    for _item in proto.temps:
        ros_msg.temps.append(Temperature())
        convert_proto_to_bosdyn_msgs_temperature(_item, ros_msg.temps[-1])

def convert_bosdyn_msgs_get_temperature_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.temps[:]
    for _item in ros_msg.temps:
        convert_bosdyn_msgs_temperature_to_proto(_item, proto.temps.add())

def convert_proto_to_bosdyn_msgs_get_system_log_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_system_log_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_system_log_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_data_chunk(proto.data, ros_msg.data)
    ros_msg.data_is_set = proto.HasField("data")

def convert_bosdyn_msgs_get_system_log_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.data_is_set:
        convert_bosdyn_msgs_data_chunk_to_proto(ros_msg.data, proto.data)

def convert_proto_to_bosdyn_msgs_logpoint_image_params(proto, ros_msg):
    ros_msg.width = proto.width
    ros_msg.height = proto.height
    ros_msg.format.value = proto.format

def convert_bosdyn_msgs_logpoint_image_params_to_proto(ros_msg, proto):
    proto.Clear()
    proto.width = ros_msg.width
    proto.height = ros_msg.height
    proto.format = ros_msg.format.value

def convert_proto_to_bosdyn_msgs_logpoint_calibration(proto, ros_msg):
    ros_msg.xoffset = proto.xoffset
    ros_msg.yoffset = proto.yoffset
    ros_msg.width = proto.width
    ros_msg.height = proto.height
    ros_msg.base_frame_name = proto.base_frame_name
    convert_proto_to_geometry_msgs_pose(proto.base_tform_sensor, ros_msg.base_tform_sensor)
    ros_msg.base_tform_sensor_is_set = proto.HasField("base_tform_sensor")
    convert_proto_to_bosdyn_msgs_camera_pinhole_intrinsics(proto.intrinsics, ros_msg.intrinsics)
    ros_msg.intrinsics_is_set = proto.HasField("intrinsics")

def convert_bosdyn_msgs_logpoint_calibration_to_proto(ros_msg, proto):
    proto.Clear()
    proto.xoffset = ros_msg.xoffset
    proto.yoffset = ros_msg.yoffset
    proto.width = ros_msg.width
    proto.height = ros_msg.height
    proto.base_frame_name = ros_msg.base_frame_name
    if ros_msg.base_tform_sensor_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.base_tform_sensor, proto.base_tform_sensor)
    if ros_msg.intrinsics_is_set:
        convert_bosdyn_msgs_camera_pinhole_intrinsics_to_proto(ros_msg.intrinsics, proto.intrinsics)

def convert_proto_to_bosdyn_msgs_logpoint(proto, ros_msg):
    ros_msg.name = proto.name
    ros_msg.type.value = proto.type
    ros_msg.status.value = proto.status
    ros_msg.queue_status.value = proto.queue_status
    ros_msg.tag = proto.tag
    convert_proto_to_builtin_interfaces_time(proto.timestamp, ros_msg.timestamp)
    ros_msg.timestamp_is_set = proto.HasField("timestamp")
    convert_proto_to_bosdyn_msgs_logpoint_image_params(proto.image_params, ros_msg.image_params)
    ros_msg.image_params_is_set = proto.HasField("image_params")
    from bosdyn_msgs.msg import Calibration
    ros_msg.calibration = []
    for _item in proto.calibration:
        ros_msg.calibration.append(Calibration())
        convert_proto_to_bosdyn_msgs_logpoint_calibration(_item, ros_msg.calibration[-1])

def convert_bosdyn_msgs_logpoint_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    proto.type = ros_msg.type.value
    proto.status = ros_msg.status.value
    proto.queue_status = ros_msg.queue_status.value
    proto.tag = ros_msg.tag
    if ros_msg.timestamp_is_set:
        convert_builtin_interfaces_time_to_proto(ros_msg.timestamp, proto.timestamp)
    if ros_msg.image_params_is_set:
        convert_bosdyn_msgs_logpoint_image_params_to_proto(ros_msg.image_params, proto.image_params)
    del proto.calibration[:]
    for _item in ros_msg.calibration:
        convert_bosdyn_msgs_logpoint_calibration_to_proto(_item, proto.calibration.add())

def convert_proto_to_bosdyn_msgs_delete_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_logpoint(proto.point, ros_msg.point)
    ros_msg.point_is_set = proto.HasField("point")

def convert_bosdyn_msgs_delete_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.point_is_set:
        convert_bosdyn_msgs_logpoint_to_proto(ros_msg.point, proto.point)

def convert_proto_to_bosdyn_msgs_delete_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_delete_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_retrieve_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_logpoint(proto.point, ros_msg.point)
    ros_msg.point_is_set = proto.HasField("point")

def convert_bosdyn_msgs_retrieve_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.point_is_set:
        convert_bosdyn_msgs_logpoint_to_proto(ros_msg.point, proto.point)

def convert_proto_to_bosdyn_msgs_retrieve_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_logpoint(proto.logpoint, ros_msg.logpoint)
    ros_msg.logpoint_is_set = proto.HasField("logpoint")
    convert_proto_to_bosdyn_msgs_data_chunk(proto.data, ros_msg.data)
    ros_msg.data_is_set = proto.HasField("data")

def convert_bosdyn_msgs_retrieve_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.logpoint_is_set:
        convert_bosdyn_msgs_logpoint_to_proto(ros_msg.logpoint, proto.logpoint)
    if ros_msg.data_is_set:
        convert_bosdyn_msgs_data_chunk_to_proto(ros_msg.data, proto.data)

def convert_proto_to_bosdyn_msgs_retrieve_raw_data_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_logpoint(proto.point, ros_msg.point)
    ros_msg.point_is_set = proto.HasField("point")

def convert_bosdyn_msgs_retrieve_raw_data_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.point_is_set:
        convert_bosdyn_msgs_logpoint_to_proto(ros_msg.point, proto.point)

def convert_proto_to_bosdyn_msgs_retrieve_raw_data_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_logpoint(proto.logpoint, ros_msg.logpoint)
    ros_msg.logpoint_is_set = proto.HasField("logpoint")
    convert_proto_to_bosdyn_msgs_data_chunk(proto.data, ros_msg.data)
    ros_msg.data_is_set = proto.HasField("data")

def convert_bosdyn_msgs_retrieve_raw_data_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.logpoint_is_set:
        convert_bosdyn_msgs_logpoint_to_proto(ros_msg.logpoint, proto.logpoint)
    if ros_msg.data_is_set:
        convert_bosdyn_msgs_data_chunk_to_proto(ros_msg.data, proto.data)

def convert_proto_to_bosdyn_msgs_store_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_camera(proto.camera, ros_msg.camera)
    ros_msg.camera_is_set = proto.HasField("camera")
    ros_msg.type.value = proto.type
    ros_msg.tag = proto.tag

def convert_bosdyn_msgs_store_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.camera_is_set:
        convert_bosdyn_msgs_camera_to_proto(ros_msg.camera, proto.camera)
    proto.type = ros_msg.type.value
    proto.tag = ros_msg.tag

def convert_proto_to_bosdyn_msgs_store_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_logpoint(proto.point, ros_msg.point)
    ros_msg.point_is_set = proto.HasField("point")

def convert_bosdyn_msgs_store_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.point_is_set:
        convert_bosdyn_msgs_logpoint_to_proto(ros_msg.point, proto.point)

def convert_proto_to_bosdyn_msgs_tag_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_logpoint(proto.point, ros_msg.point)
    ros_msg.point_is_set = proto.HasField("point")

def convert_bosdyn_msgs_tag_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.point_is_set:
        convert_bosdyn_msgs_logpoint_to_proto(ros_msg.point, proto.point)

def convert_proto_to_bosdyn_msgs_tag_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_tag_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_cameras_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_cameras_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_cameras_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Camera
    ros_msg.cameras = []
    for _item in proto.cameras:
        ros_msg.cameras.append(Camera())
        convert_proto_to_bosdyn_msgs_camera(_item, ros_msg.cameras[-1])

def convert_bosdyn_msgs_list_cameras_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.cameras[:]
    for _item in ros_msg.cameras:
        convert_bosdyn_msgs_camera_to_proto(_item, proto.cameras.add())

def convert_proto_to_bosdyn_msgs_list_logpoints_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_logpoints_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_logpoints_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Logpoint
    ros_msg.logpoints = []
    for _item in proto.logpoints:
        ros_msg.logpoints.append(Logpoint())
        convert_proto_to_bosdyn_msgs_logpoint(_item, ros_msg.logpoints[-1])

def convert_bosdyn_msgs_list_logpoints_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.logpoints[:]
    for _item in ros_msg.logpoints:
        convert_bosdyn_msgs_logpoint_to_proto(_item, proto.logpoints.add())

def convert_proto_to_bosdyn_msgs_set_passphrase_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.passphrase = proto.passphrase

def convert_bosdyn_msgs_set_passphrase_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.passphrase = ros_msg.passphrase

def convert_proto_to_bosdyn_msgs_set_passphrase_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_set_passphrase_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_debug_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.enable_temperature = proto.enable_temperature
    ros_msg.enable_humidity = proto.enable_humidity
    ros_msg.enable_bit = proto.enable_BIT
    ros_msg.enable_shock = proto.enable_shock
    ros_msg.enable_system_stat = proto.enable_system_stat

def convert_bosdyn_msgs_debug_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.enable_temperature = ros_msg.enable_temperature
    proto.enable_humidity = ros_msg.enable_humidity
    proto.enable_BIT = ros_msg.enable_bit
    proto.enable_shock = ros_msg.enable_shock
    proto.enable_system_stat = ros_msg.enable_system_stat

def convert_proto_to_bosdyn_msgs_debug_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_debug_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_camera_pinhole_intrinsics(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.focal_length, ros_msg.focal_length)
    ros_msg.focal_length_is_set = proto.HasField("focal_length")
    convert_proto_to_bosdyn_msgs_vec2(proto.center_point, ros_msg.center_point)
    ros_msg.center_point_is_set = proto.HasField("center_point")
    ros_msg.k1 = proto.k1
    ros_msg.k2 = proto.k2
    ros_msg.k3 = proto.k3
    ros_msg.k4 = proto.k4

def convert_bosdyn_msgs_camera_pinhole_intrinsics_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.focal_length_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.focal_length, proto.focal_length)
    if ros_msg.center_point_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.center_point, proto.center_point)
    proto.k1 = ros_msg.k1
    proto.k2 = ros_msg.k2
    proto.k3 = ros_msg.k3
    proto.k4 = ros_msg.k4

def convert_proto_to_bosdyn_msgs_camera_spherical_limits(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_vec2(proto.min_angle, ros_msg.min_angle)
    ros_msg.min_angle_is_set = proto.HasField("min_angle")
    convert_proto_to_bosdyn_msgs_vec2(proto.max_angle, ros_msg.max_angle)
    ros_msg.max_angle_is_set = proto.HasField("max_angle")

def convert_bosdyn_msgs_camera_spherical_limits_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.min_angle_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.min_angle, proto.min_angle)
    if ros_msg.max_angle_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.max_angle, proto.max_angle)

def convert_proto_to_bosdyn_msgs_camera_one_of_intrinsics(proto, ros_msg):
    if proto.HasField("pinhole"):
        ros_msg.intrinsics_choice = ros_msg.INTRINSICS_PINHOLE_SET
        convert_proto_to_bosdyn_msgs_camera_pinhole_intrinsics(proto.pinhole, ros_msg.pinhole)
    if proto.HasField("spherical"):
        ros_msg.intrinsics_choice = ros_msg.INTRINSICS_SPHERICAL_SET
        convert_proto_to_bosdyn_msgs_camera_spherical_limits(proto.spherical, ros_msg.spherical)

def convert_bosdyn_msgs_camera_one_of_intrinsics_to_proto(ros_msg, proto):
    proto.ClearField("intrinsics")
    if ros_msg.intrinsics_choice == ros_msg.INTRINSICS_PINHOLE_SET:
        convert_bosdyn_msgs_camera_pinhole_intrinsics_to_proto(ros_msg.pinhole, proto.pinhole)
    if ros_msg.intrinsics_choice == ros_msg.INTRINSICS_SPHERICAL_SET:
        convert_bosdyn_msgs_camera_spherical_limits_to_proto(ros_msg.spherical, proto.spherical)

def convert_proto_to_bosdyn_msgs_camera(proto, ros_msg):
    ros_msg.name = proto.name
    convert_proto_to_bosdyn_msgs_vec2(proto.resolution, ros_msg.resolution)
    ros_msg.resolution_is_set = proto.HasField("resolution")
    ros_msg.base_frame_name = proto.base_frame_name
    convert_proto_to_geometry_msgs_pose(proto.base_tform_sensor, ros_msg.base_tform_sensor)
    ros_msg.base_tform_sensor_is_set = proto.HasField("base_tform_sensor")
    convert_proto_to_bosdyn_msgs_camera_one_of_intrinsics(proto, ros_msg.intrinsics)

def convert_bosdyn_msgs_camera_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    if ros_msg.resolution_is_set:
        convert_bosdyn_msgs_vec2_to_proto(ros_msg.resolution, proto.resolution)
    proto.base_frame_name = ros_msg.base_frame_name
    if ros_msg.base_tform_sensor_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.base_tform_sensor, proto.base_tform_sensor)
    convert_bosdyn_msgs_camera_one_of_intrinsics_to_proto(ros_msg.intrinsics, proto)

def convert_proto_to_bosdyn_msgs_get_led_brightness_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_led_brightness_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_led_brightness_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.brightnesses = []
    for _item in proto.brightnesses:
        ros_msg.brightnesses.append(_item)

def convert_bosdyn_msgs_get_led_brightness_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.brightnesses[:]
    for _item in ros_msg.brightnesses:
        proto.brightnesses.add(_item)

def convert_proto_to_bosdyn_msgs_set_led_brightness_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import KeyInt32ValueFloat32
    ros_msg.brightnesses = []
    for _item in proto.brightnesses:
        ros_msg.brightnesses.append(KeyInt32ValueFloat32())
        ros_msg.brightnesses[-1].key = _item
        ros_msg.brightnesses[-1].value = proto.brightnesses[_item]

def convert_bosdyn_msgs_set_led_brightness_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    for _item in ros_msg.brightnesses:
        proto.brightnesses[_item.key] = _item.value

def convert_proto_to_bosdyn_msgs_set_led_brightness_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_set_led_brightness_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_sound(proto, ros_msg):
    ros_msg.name = proto.name

def convert_bosdyn_msgs_sound_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name

def convert_proto_to_bosdyn_msgs_list_sounds_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_sounds_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_sounds_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Sound
    ros_msg.sounds = []
    for _item in proto.sounds:
        ros_msg.sounds.append(Sound())
        convert_proto_to_bosdyn_msgs_sound(_item, ros_msg.sounds[-1])

def convert_bosdyn_msgs_list_sounds_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.sounds[:]
    for _item in ros_msg.sounds:
        convert_bosdyn_msgs_sound_to_proto(_item, proto.sounds.add())

def convert_proto_to_bosdyn_msgs_set_volume_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.volume = proto.volume

def convert_bosdyn_msgs_set_volume_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.volume = ros_msg.volume

def convert_proto_to_bosdyn_msgs_set_volume_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_set_volume_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_volume_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_volume_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_volume_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.volume = proto.volume

def convert_bosdyn_msgs_get_volume_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.volume = ros_msg.volume

def convert_proto_to_bosdyn_msgs_play_sound_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_sound(proto.sound, ros_msg.sound)
    ros_msg.sound_is_set = proto.HasField("sound")
    ros_msg.gain = proto.gain.value
    ros_msg.gain_is_set = proto.HasField("gain")

def convert_bosdyn_msgs_play_sound_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.sound_is_set:
        convert_bosdyn_msgs_sound_to_proto(ros_msg.sound, proto.sound)
    if ros_msg.gain_is_set:
        convert_float32_to_proto(ros_msg.gain, proto.gain)

def convert_proto_to_bosdyn_msgs_play_sound_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_play_sound_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_delete_sound_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_sound(proto.sound, ros_msg.sound)
    ros_msg.sound_is_set = proto.HasField("sound")

def convert_bosdyn_msgs_delete_sound_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.sound_is_set:
        convert_bosdyn_msgs_sound_to_proto(ros_msg.sound, proto.sound)

def convert_proto_to_bosdyn_msgs_delete_sound_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_delete_sound_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_load_sound_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_sound(proto.sound, ros_msg.sound)
    ros_msg.sound_is_set = proto.HasField("sound")
    convert_proto_to_bosdyn_msgs_data_chunk(proto.data, ros_msg.data)
    ros_msg.data_is_set = proto.HasField("data")

def convert_bosdyn_msgs_load_sound_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.sound_is_set:
        convert_bosdyn_msgs_sound_to_proto(ros_msg.sound, proto.sound)
    if ros_msg.data_is_set:
        convert_bosdyn_msgs_data_chunk_to_proto(ros_msg.data, proto.data)

def convert_proto_to_bosdyn_msgs_load_sound_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_load_sound_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_set_audio_capture_channel_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.channel.value = proto.channel

def convert_bosdyn_msgs_set_audio_capture_channel_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.channel = ros_msg.channel.value

def convert_proto_to_bosdyn_msgs_set_audio_capture_channel_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_set_audio_capture_channel_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_audio_capture_channel_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_audio_capture_channel_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_audio_capture_channel_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.channel.value = proto.channel

def convert_bosdyn_msgs_get_audio_capture_channel_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.channel = ros_msg.channel.value

def convert_proto_to_bosdyn_msgs_set_audio_capture_gain_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.channel.value = proto.channel
    ros_msg.gain = proto.gain

def convert_bosdyn_msgs_set_audio_capture_gain_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.channel = ros_msg.channel.value
    proto.gain = ros_msg.gain

def convert_proto_to_bosdyn_msgs_set_audio_capture_gain_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_set_audio_capture_gain_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_audio_capture_gain_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.channel.value = proto.channel

def convert_bosdyn_msgs_get_audio_capture_gain_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.channel = ros_msg.channel.value

def convert_proto_to_bosdyn_msgs_get_audio_capture_gain_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.gain = proto.gain

def convert_bosdyn_msgs_get_audio_capture_gain_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.gain = ros_msg.gain

def convert_proto_to_bosdyn_msgs_network_tuple(proto, ros_msg):
    ros_msg.address = proto.address.value
    ros_msg.address_is_set = proto.HasField("address")
    ros_msg.netmask = proto.netmask.value
    ros_msg.netmask_is_set = proto.HasField("netmask")
    ros_msg.gateway = proto.gateway.value
    ros_msg.gateway_is_set = proto.HasField("gateway")
    ros_msg.mtu = proto.mtu.value
    ros_msg.mtu_is_set = proto.HasField("mtu")

def convert_bosdyn_msgs_network_tuple_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.address_is_set:
        convert_uint32_to_proto(ros_msg.address, proto.address)
    if ros_msg.netmask_is_set:
        convert_uint32_to_proto(ros_msg.netmask, proto.netmask)
    if ros_msg.gateway_is_set:
        convert_uint32_to_proto(ros_msg.gateway, proto.gateway)
    if ros_msg.mtu_is_set:
        convert_uint32_to_proto(ros_msg.mtu, proto.mtu)

def convert_proto_to_bosdyn_msgs_get_network_settings_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_network_settings_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_network_settings_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_network_tuple(proto.settings, ros_msg.settings)
    ros_msg.settings_is_set = proto.HasField("settings")

def convert_bosdyn_msgs_get_network_settings_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.settings_is_set:
        convert_bosdyn_msgs_network_tuple_to_proto(ros_msg.settings, proto.settings)

def convert_proto_to_bosdyn_msgs_get_ssl_cert_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_ssl_cert_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_ssl_cert_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.certificate = proto.certificate

def convert_bosdyn_msgs_get_ssl_cert_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.certificate = ros_msg.certificate

def convert_proto_to_bosdyn_msgs_ice_server(proto, ros_msg):
    ros_msg.type.value = proto.type
    ros_msg.address = proto.address
    ros_msg.port = proto.port

def convert_bosdyn_msgs_ice_server_to_proto(ros_msg, proto):
    proto.Clear()
    proto.type = ros_msg.type.value
    proto.address = ros_msg.address
    proto.port = ros_msg.port

def convert_proto_to_bosdyn_msgs_get_ice_configuration_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_ice_configuration_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_ice_configuration_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import ICEServer
    ros_msg.servers = []
    for _item in proto.servers:
        ros_msg.servers.append(ICEServer())
        convert_proto_to_bosdyn_msgs_ice_server(_item, ros_msg.servers[-1])

def convert_bosdyn_msgs_get_ice_configuration_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.servers[:]
    for _item in ros_msg.servers:
        convert_bosdyn_msgs_ice_server_to_proto(_item, proto.servers.add())

def convert_proto_to_bosdyn_msgs_set_ice_configuration_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import ICEServer
    ros_msg.servers = []
    for _item in proto.servers:
        ros_msg.servers.append(ICEServer())
        convert_proto_to_bosdyn_msgs_ice_server(_item, ros_msg.servers[-1])

def convert_bosdyn_msgs_set_ice_configuration_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    del proto.servers[:]
    for _item in ros_msg.servers:
        convert_bosdyn_msgs_ice_server_to_proto(_item, proto.servers.add())

def convert_proto_to_bosdyn_msgs_set_ice_configuration_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_set_ice_configuration_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_power_status(proto, ros_msg):
    ros_msg.ptz = proto.ptz.value
    ros_msg.ptz_is_set = proto.HasField("ptz")
    ros_msg.aux1 = proto.aux1.value
    ros_msg.aux1_is_set = proto.HasField("aux1")
    ros_msg.aux2 = proto.aux2.value
    ros_msg.aux2_is_set = proto.HasField("aux2")
    ros_msg.external_mic = proto.external_mic.value
    ros_msg.external_mic_is_set = proto.HasField("external_mic")

def convert_bosdyn_msgs_power_status_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.ptz_is_set:
        convert_bool_to_proto(ros_msg.ptz, proto.ptz)
    if ros_msg.aux1_is_set:
        convert_bool_to_proto(ros_msg.aux1, proto.aux1)
    if ros_msg.aux2_is_set:
        convert_bool_to_proto(ros_msg.aux2, proto.aux2)
    if ros_msg.external_mic_is_set:
        convert_bool_to_proto(ros_msg.external_mic, proto.external_mic)

def convert_proto_to_bosdyn_msgs_get_power_status_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_power_status_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_power_status_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_power_status(proto.status, ros_msg.status)
    ros_msg.status_is_set = proto.HasField("status")

def convert_bosdyn_msgs_get_power_status_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.status_is_set:
        convert_bosdyn_msgs_power_status_to_proto(ros_msg.status, proto.status)

def convert_proto_to_bosdyn_msgs_set_power_status_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_power_status(proto.status, ros_msg.status)
    ros_msg.status_is_set = proto.HasField("status")

def convert_bosdyn_msgs_set_power_status_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.status_is_set:
        convert_bosdyn_msgs_power_status_to_proto(ros_msg.status, proto.status)

def convert_proto_to_bosdyn_msgs_set_power_status_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_power_status(proto.status, ros_msg.status)
    ros_msg.status_is_set = proto.HasField("status")

def convert_bosdyn_msgs_set_power_status_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.status_is_set:
        convert_bosdyn_msgs_power_status_to_proto(ros_msg.status, proto.status)

def convert_proto_to_bosdyn_msgs_cycle_power_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_power_status(proto.status, ros_msg.status)
    ros_msg.status_is_set = proto.HasField("status")

def convert_bosdyn_msgs_cycle_power_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.status_is_set:
        convert_bosdyn_msgs_power_status_to_proto(ros_msg.status, proto.status)

def convert_proto_to_bosdyn_msgs_cycle_power_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_power_status(proto.status, ros_msg.status)
    ros_msg.status_is_set = proto.HasField("status")

def convert_bosdyn_msgs_cycle_power_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.status_is_set:
        convert_bosdyn_msgs_power_status_to_proto(ros_msg.status, proto.status)

def convert_proto_to_bosdyn_msgs_screen_description(proto, ros_msg):
    ros_msg.name = proto.name

def convert_bosdyn_msgs_screen_description_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name

def convert_proto_to_bosdyn_msgs_get_screen_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_screen_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_screen_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.name = proto.name

def convert_bosdyn_msgs_get_screen_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.name = ros_msg.name

def convert_proto_to_bosdyn_msgs_get_visible_cameras_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_visible_cameras_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_visible_cameras_response_stream_window(proto, ros_msg):
    ros_msg.xoffset = proto.xoffset
    ros_msg.yoffset = proto.yoffset
    ros_msg.width = proto.width
    ros_msg.height = proto.height

def convert_bosdyn_msgs_get_visible_cameras_response_stream_window_to_proto(ros_msg, proto):
    proto.Clear()
    proto.xoffset = ros_msg.xoffset
    proto.yoffset = ros_msg.yoffset
    proto.width = ros_msg.width
    proto.height = ros_msg.height

def convert_proto_to_bosdyn_msgs_get_visible_cameras_response_stream(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_get_visible_cameras_response_stream_window(proto.window, ros_msg.window)
    ros_msg.window_is_set = proto.HasField("window")
    convert_proto_to_bosdyn_msgs_camera(proto.camera, ros_msg.camera)
    ros_msg.camera_is_set = proto.HasField("camera")

def convert_bosdyn_msgs_get_visible_cameras_response_stream_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.window_is_set:
        convert_bosdyn_msgs_get_visible_cameras_response_stream_window_to_proto(ros_msg.window, proto.window)
    if ros_msg.camera_is_set:
        convert_bosdyn_msgs_camera_to_proto(ros_msg.camera, proto.camera)

def convert_proto_to_bosdyn_msgs_get_visible_cameras_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import Stream
    ros_msg.streams = []
    for _item in proto.streams:
        ros_msg.streams.append(Stream())
        convert_proto_to_bosdyn_msgs_get_visible_cameras_response_stream(_item, ros_msg.streams[-1])

def convert_bosdyn_msgs_get_visible_cameras_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.streams[:]
    for _item in ros_msg.streams:
        convert_bosdyn_msgs_get_visible_cameras_response_stream_to_proto(_item, proto.streams.add())

def convert_proto_to_bosdyn_msgs_list_screens_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_screens_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_screens_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import ScreenDescription
    ros_msg.screens = []
    for _item in proto.screens:
        ros_msg.screens.append(ScreenDescription())
        convert_proto_to_bosdyn_msgs_screen_description(_item, ros_msg.screens[-1])

def convert_bosdyn_msgs_list_screens_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.screens[:]
    for _item in ros_msg.screens:
        convert_bosdyn_msgs_screen_description_to_proto(_item, proto.screens.add())

def convert_proto_to_bosdyn_msgs_set_screen_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.name = proto.name

def convert_bosdyn_msgs_set_screen_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.name = ros_msg.name

def convert_proto_to_bosdyn_msgs_set_screen_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.name = proto.name

def convert_bosdyn_msgs_set_screen_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.name = ros_msg.name

def convert_proto_to_bosdyn_msgs_ir_color_map_scaling_pair(proto, ros_msg):
    ros_msg.min = proto.min
    ros_msg.max = proto.max

def convert_bosdyn_msgs_ir_color_map_scaling_pair_to_proto(ros_msg, proto):
    proto.Clear()
    proto.min = ros_msg.min
    proto.max = ros_msg.max

def convert_proto_to_bosdyn_msgs_ir_color_map(proto, ros_msg):
    ros_msg.colormap.value = proto.colormap
    convert_proto_to_bosdyn_msgs_ir_color_map_scaling_pair(proto.scale, ros_msg.scale)
    ros_msg.scale_is_set = proto.HasField("scale")
    ros_msg.auto_scale = proto.auto_scale.value
    ros_msg.auto_scale_is_set = proto.HasField("auto_scale")

def convert_bosdyn_msgs_ir_color_map_to_proto(ros_msg, proto):
    proto.Clear()
    proto.colormap = ros_msg.colormap.value
    if ros_msg.scale_is_set:
        convert_bosdyn_msgs_ir_color_map_scaling_pair_to_proto(ros_msg.scale, proto.scale)
    if ros_msg.auto_scale_is_set:
        convert_bool_to_proto(ros_msg.auto_scale, proto.auto_scale)

def convert_proto_to_bosdyn_msgs_set_ir_colormap_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ir_color_map(proto.map, ros_msg.map)
    ros_msg.map_is_set = proto.HasField("map")

def convert_bosdyn_msgs_set_ir_colormap_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.map_is_set:
        convert_bosdyn_msgs_ir_color_map_to_proto(ros_msg.map, proto.map)

def convert_proto_to_bosdyn_msgs_set_ir_colormap_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_set_ir_colormap_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_ir_colormap_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_ir_colormap_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_ir_colormap_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ir_color_map(proto.map, ros_msg.map)
    ros_msg.map_is_set = proto.HasField("map")

def convert_bosdyn_msgs_get_ir_colormap_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.map_is_set:
        convert_bosdyn_msgs_ir_color_map_to_proto(ros_msg.map, proto.map)

def convert_proto_to_bosdyn_msgs_ir_meter_overlay_normalized_coordinates(proto, ros_msg):
    ros_msg.x = proto.x
    ros_msg.y = proto.y

def convert_bosdyn_msgs_ir_meter_overlay_normalized_coordinates_to_proto(ros_msg, proto):
    proto.Clear()
    proto.x = ros_msg.x
    proto.y = ros_msg.y

def convert_proto_to_bosdyn_msgs_ir_meter_overlay(proto, ros_msg):
    ros_msg.enable = proto.enable
    convert_proto_to_bosdyn_msgs_ir_meter_overlay_normalized_coordinates(proto.coords, ros_msg.coords)
    ros_msg.coords_is_set = proto.HasField("coords")

def convert_bosdyn_msgs_ir_meter_overlay_to_proto(ros_msg, proto):
    proto.Clear()
    proto.enable = ros_msg.enable
    if ros_msg.coords_is_set:
        convert_bosdyn_msgs_ir_meter_overlay_normalized_coordinates_to_proto(ros_msg.coords, proto.coords)

def convert_proto_to_bosdyn_msgs_set_ir_meter_overlay_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ir_meter_overlay(proto.overlay, ros_msg.overlay)
    ros_msg.overlay_is_set = proto.HasField("overlay")

def convert_bosdyn_msgs_set_ir_meter_overlay_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.overlay_is_set:
        convert_bosdyn_msgs_ir_meter_overlay_to_proto(ros_msg.overlay, proto.overlay)

def convert_proto_to_bosdyn_msgs_set_ir_meter_overlay_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_set_ir_meter_overlay_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_ptz_description_limits(proto, ros_msg):
    ros_msg.min = proto.min.value
    ros_msg.min_is_set = proto.HasField("min")
    ros_msg.max = proto.max.value
    ros_msg.max_is_set = proto.HasField("max")

def convert_bosdyn_msgs_ptz_description_limits_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.min_is_set:
        convert_float32_to_proto(ros_msg.min, proto.min)
    if ros_msg.max_is_set:
        convert_float32_to_proto(ros_msg.max, proto.max)

def convert_proto_to_bosdyn_msgs_ptz_description(proto, ros_msg):
    ros_msg.name = proto.name
    convert_proto_to_bosdyn_msgs_ptz_description_limits(proto.pan_limit, ros_msg.pan_limit)
    ros_msg.pan_limit_is_set = proto.HasField("pan_limit")
    convert_proto_to_bosdyn_msgs_ptz_description_limits(proto.tilt_limit, ros_msg.tilt_limit)
    ros_msg.tilt_limit_is_set = proto.HasField("tilt_limit")
    convert_proto_to_bosdyn_msgs_ptz_description_limits(proto.zoom_limit, ros_msg.zoom_limit)
    ros_msg.zoom_limit_is_set = proto.HasField("zoom_limit")

def convert_bosdyn_msgs_ptz_description_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    if ros_msg.pan_limit_is_set:
        convert_bosdyn_msgs_ptz_description_limits_to_proto(ros_msg.pan_limit, proto.pan_limit)
    if ros_msg.tilt_limit_is_set:
        convert_bosdyn_msgs_ptz_description_limits_to_proto(ros_msg.tilt_limit, proto.tilt_limit)
    if ros_msg.zoom_limit_is_set:
        convert_bosdyn_msgs_ptz_description_limits_to_proto(ros_msg.zoom_limit, proto.zoom_limit)

def convert_proto_to_bosdyn_msgs_ptz_position(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_ptz_description(proto.ptz, ros_msg.ptz)
    ros_msg.ptz_is_set = proto.HasField("ptz")
    ros_msg.pan = proto.pan.value
    ros_msg.pan_is_set = proto.HasField("pan")
    ros_msg.tilt = proto.tilt.value
    ros_msg.tilt_is_set = proto.HasField("tilt")
    ros_msg.zoom = proto.zoom.value
    ros_msg.zoom_is_set = proto.HasField("zoom")

def convert_bosdyn_msgs_ptz_position_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.ptz_is_set:
        convert_bosdyn_msgs_ptz_description_to_proto(ros_msg.ptz, proto.ptz)
    if ros_msg.pan_is_set:
        convert_float32_to_proto(ros_msg.pan, proto.pan)
    if ros_msg.tilt_is_set:
        convert_float32_to_proto(ros_msg.tilt, proto.tilt)
    if ros_msg.zoom_is_set:
        convert_float32_to_proto(ros_msg.zoom, proto.zoom)

def convert_proto_to_bosdyn_msgs_ptz_velocity(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_ptz_description(proto.ptz, ros_msg.ptz)
    ros_msg.ptz_is_set = proto.HasField("ptz")
    ros_msg.pan = proto.pan.value
    ros_msg.pan_is_set = proto.HasField("pan")
    ros_msg.tilt = proto.tilt.value
    ros_msg.tilt_is_set = proto.HasField("tilt")
    ros_msg.zoom = proto.zoom.value
    ros_msg.zoom_is_set = proto.HasField("zoom")

def convert_bosdyn_msgs_ptz_velocity_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.ptz_is_set:
        convert_bosdyn_msgs_ptz_description_to_proto(ros_msg.ptz, proto.ptz)
    if ros_msg.pan_is_set:
        convert_float32_to_proto(ros_msg.pan, proto.pan)
    if ros_msg.tilt_is_set:
        convert_float32_to_proto(ros_msg.tilt, proto.tilt)
    if ros_msg.zoom_is_set:
        convert_float32_to_proto(ros_msg.zoom, proto.zoom)

def convert_proto_to_bosdyn_msgs_get_ptz_position_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ptz_description(proto.ptz, ros_msg.ptz)
    ros_msg.ptz_is_set = proto.HasField("ptz")

def convert_bosdyn_msgs_get_ptz_position_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.ptz_is_set:
        convert_bosdyn_msgs_ptz_description_to_proto(ros_msg.ptz, proto.ptz)

def convert_proto_to_bosdyn_msgs_get_ptz_position_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ptz_position(proto.position, ros_msg.position)
    ros_msg.position_is_set = proto.HasField("position")

def convert_bosdyn_msgs_get_ptz_position_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.position_is_set:
        convert_bosdyn_msgs_ptz_position_to_proto(ros_msg.position, proto.position)

def convert_proto_to_bosdyn_msgs_get_ptz_velocity_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ptz_description(proto.ptz, ros_msg.ptz)
    ros_msg.ptz_is_set = proto.HasField("ptz")

def convert_bosdyn_msgs_get_ptz_velocity_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.ptz_is_set:
        convert_bosdyn_msgs_ptz_description_to_proto(ros_msg.ptz, proto.ptz)

def convert_proto_to_bosdyn_msgs_get_ptz_velocity_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ptz_velocity(proto.velocity, ros_msg.velocity)
    ros_msg.velocity_is_set = proto.HasField("velocity")

def convert_bosdyn_msgs_get_ptz_velocity_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.velocity_is_set:
        convert_bosdyn_msgs_ptz_velocity_to_proto(ros_msg.velocity, proto.velocity)

def convert_proto_to_bosdyn_msgs_list_ptz_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_list_ptz_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_list_ptz_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    from bosdyn_msgs.msg import PtzDescription
    ros_msg.ptzs = []
    for _item in proto.ptzs:
        ros_msg.ptzs.append(PtzDescription())
        convert_proto_to_bosdyn_msgs_ptz_description(_item, ros_msg.ptzs[-1])

def convert_bosdyn_msgs_list_ptz_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    del proto.ptzs[:]
    for _item in ros_msg.ptzs:
        convert_bosdyn_msgs_ptz_description_to_proto(_item, proto.ptzs.add())

def convert_proto_to_bosdyn_msgs_set_ptz_position_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ptz_position(proto.position, ros_msg.position)
    ros_msg.position_is_set = proto.HasField("position")

def convert_bosdyn_msgs_set_ptz_position_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.position_is_set:
        convert_bosdyn_msgs_ptz_position_to_proto(ros_msg.position, proto.position)

def convert_proto_to_bosdyn_msgs_set_ptz_position_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ptz_position(proto.position, ros_msg.position)
    ros_msg.position_is_set = proto.HasField("position")

def convert_bosdyn_msgs_set_ptz_position_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.position_is_set:
        convert_bosdyn_msgs_ptz_position_to_proto(ros_msg.position, proto.position)

def convert_proto_to_bosdyn_msgs_set_ptz_velocity_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ptz_velocity(proto.velocity, ros_msg.velocity)
    ros_msg.velocity_is_set = proto.HasField("velocity")

def convert_bosdyn_msgs_set_ptz_velocity_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.velocity_is_set:
        convert_bosdyn_msgs_ptz_velocity_to_proto(ros_msg.velocity, proto.velocity)

def convert_proto_to_bosdyn_msgs_set_ptz_velocity_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_ptz_velocity(proto.velocity, ros_msg.velocity)
    ros_msg.velocity_is_set = proto.HasField("velocity")

def convert_bosdyn_msgs_set_ptz_velocity_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.velocity_is_set:
        convert_bosdyn_msgs_ptz_velocity_to_proto(ros_msg.velocity, proto.velocity)

def convert_proto_to_bosdyn_msgs_initialize_lens_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_initialize_lens_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_initialize_lens_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_initialize_lens_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_stream_params_awb_mode(proto, ros_msg):
    ros_msg.awb.value = proto.awb

def convert_bosdyn_msgs_stream_params_awb_mode_to_proto(ros_msg, proto):
    proto.Clear()
    proto.awb = ros_msg.awb.value

def convert_proto_to_bosdyn_msgs_stream_params(proto, ros_msg):
    ros_msg.targetbitrate = proto.targetbitrate.value
    ros_msg.targetbitrate_is_set = proto.HasField("targetbitrate")
    ros_msg.refreshinterval = proto.refreshinterval.value
    ros_msg.refreshinterval_is_set = proto.HasField("refreshinterval")
    ros_msg.idrinterval = proto.idrinterval.value
    ros_msg.idrinterval_is_set = proto.HasField("idrinterval")
    convert_proto_to_bosdyn_msgs_stream_params_awb_mode(proto.awb, ros_msg.awb)
    ros_msg.awb_is_set = proto.HasField("awb")

def convert_bosdyn_msgs_stream_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.targetbitrate_is_set:
        convert_int64_to_proto(ros_msg.targetbitrate, proto.targetbitrate)
    if ros_msg.refreshinterval_is_set:
        convert_int64_to_proto(ros_msg.refreshinterval, proto.refreshinterval)
    if ros_msg.idrinterval_is_set:
        convert_int64_to_proto(ros_msg.idrinterval, proto.idrinterval)
    if ros_msg.awb_is_set:
        convert_bosdyn_msgs_stream_params_awb_mode_to_proto(ros_msg.awb, proto.awb)

def convert_proto_to_bosdyn_msgs_get_stream_params_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_get_stream_params_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_get_stream_params_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_stream_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")

def convert_bosdyn_msgs_get_stream_params_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_stream_params_to_proto(ros_msg.params, proto.params)

def convert_proto_to_bosdyn_msgs_set_stream_params_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_stream_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")

def convert_bosdyn_msgs_set_stream_params_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_stream_params_to_proto(ros_msg.params, proto.params)

def convert_proto_to_bosdyn_msgs_set_stream_params_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_stream_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")

def convert_bosdyn_msgs_set_stream_params_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_stream_params_to_proto(ros_msg.params, proto.params)

def convert_proto_to_bosdyn_msgs_enable_congestion_control_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.enable_congestion_control = proto.enable_congestion_control

def convert_bosdyn_msgs_enable_congestion_control_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    proto.enable_congestion_control = ros_msg.enable_congestion_control

def convert_proto_to_bosdyn_msgs_enable_congestion_control_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")

def convert_bosdyn_msgs_enable_congestion_control_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)

def convert_proto_to_bosdyn_msgs_failed_element(proto, ros_msg):
    ros_msg.errors = []
    for _item in proto.errors:
        ros_msg.errors.append(_item)
    ros_msg.warnings = []
    for _item in proto.warnings:
        ros_msg.warnings.append(_item)

def convert_bosdyn_msgs_failed_element_to_proto(ros_msg, proto):
    proto.Clear()
    del proto.errors[:]
    for _item in ros_msg.errors:
        proto.errors.add(_item)
    del proto.warnings[:]
    for _item in ros_msg.warnings:
        proto.warnings.add(_item)

def convert_proto_to_bosdyn_msgs_node_identifier(proto, ros_msg):
    ros_msg.node_id = proto.node_id
    ros_msg.user_data_id = proto.user_data_id

def convert_bosdyn_msgs_node_identifier_to_proto(ros_msg, proto):
    proto.Clear()
    proto.node_id = ros_msg.node_id
    proto.user_data_id = ros_msg.user_data_id

def convert_proto_to_bosdyn_msgs_element_identifiers(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_node_identifier(proto.root_id, ros_msg.root_id)
    ros_msg.root_id_is_set = proto.HasField("root_id")
    convert_proto_to_bosdyn_msgs_node_identifier(proto.action_id, ros_msg.action_id)
    ros_msg.action_id_is_set = proto.HasField("action_id")

def convert_bosdyn_msgs_element_identifiers_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.root_id_is_set:
        convert_bosdyn_msgs_node_identifier_to_proto(ros_msg.root_id, proto.root_id)
    if ros_msg.action_id_is_set:
        convert_bosdyn_msgs_node_identifier_to_proto(ros_msg.action_id, proto.action_id)

def convert_proto_to_bosdyn_msgs_compile_autowalk_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_walk(proto.walk, ros_msg.walk)
    ros_msg.walk_is_set = proto.HasField("walk")
    ros_msg.treat_warnings_as_errors = proto.treat_warnings_as_errors

def convert_bosdyn_msgs_compile_autowalk_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.walk_is_set:
        convert_bosdyn_msgs_walk_to_proto(ros_msg.walk, proto.walk)
    proto.treat_warnings_as_errors = ros_msg.treat_warnings_as_errors

def convert_proto_to_bosdyn_msgs_compile_autowalk_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    convert_proto_to_bosdyn_msgs_node(proto.root, ros_msg.root)
    ros_msg.root_is_set = proto.HasField("root")
    from bosdyn_msgs.msg import ElementIdentifiers
    ros_msg.element_identifiers = []
    for _item in proto.element_identifiers:
        ros_msg.element_identifiers.append(ElementIdentifiers())
        convert_proto_to_bosdyn_msgs_element_identifiers(_item, ros_msg.element_identifiers[-1])
    from bosdyn_msgs.msg import KeyInt32ValueBosdynMsgsFailedElement
    ros_msg.failed_elements = []
    for _item in proto.failed_elements:
        ros_msg.failed_elements.append(KeyInt32ValueBosdynMsgsFailedElement())
        ros_msg.failed_elements[-1].key = _item
        convert_proto_to_bosdyn_msgs_failed_element(proto.failed_elements[_item], ros_msg.failed_elements[-1].value)
    convert_proto_to_bosdyn_msgs_node_identifier(proto.docking_node, ros_msg.docking_node)
    ros_msg.docking_node_is_set = proto.HasField("docking_node")
    convert_proto_to_bosdyn_msgs_node_identifier(proto.loop_node, ros_msg.loop_node)
    ros_msg.loop_node_is_set = proto.HasField("loop_node")

def convert_bosdyn_msgs_compile_autowalk_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    if ros_msg.root_is_set:
        convert_bosdyn_msgs_node_to_proto(ros_msg.root, proto.root)
    del proto.element_identifiers[:]
    for _item in ros_msg.element_identifiers:
        convert_bosdyn_msgs_element_identifiers_to_proto(_item, proto.element_identifiers.add())
    for _item in ros_msg.failed_elements:
        convert_bosdyn_msgs_failed_element_to_proto(_item.value, proto.failed_elements[_item.key])
    if ros_msg.docking_node_is_set:
        convert_bosdyn_msgs_node_identifier_to_proto(ros_msg.docking_node, proto.docking_node)
    if ros_msg.loop_node_is_set:
        convert_bosdyn_msgs_node_identifier_to_proto(ros_msg.loop_node, proto.loop_node)

def convert_proto_to_bosdyn_msgs_load_autowalk_request(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_request_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    convert_proto_to_bosdyn_msgs_walk(proto.walk, ros_msg.walk)
    ros_msg.walk_is_set = proto.HasField("walk")
    from bosdyn_msgs.msg import Lease
    ros_msg.leases = []
    for _item in proto.leases:
        ros_msg.leases.append(Lease())
        convert_proto_to_bosdyn_msgs_lease(_item, ros_msg.leases[-1])
    ros_msg.treat_warnings_as_errors = proto.treat_warnings_as_errors

def convert_bosdyn_msgs_load_autowalk_request_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_request_header_to_proto(ros_msg.header, proto.header)
    if ros_msg.walk_is_set:
        convert_bosdyn_msgs_walk_to_proto(ros_msg.walk, proto.walk)
    del proto.leases[:]
    for _item in ros_msg.leases:
        convert_bosdyn_msgs_lease_to_proto(_item, proto.leases.add())
    proto.treat_warnings_as_errors = ros_msg.treat_warnings_as_errors

def convert_proto_to_bosdyn_msgs_load_autowalk_response(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_response_header(proto.header, ros_msg.header)
    ros_msg.header_is_set = proto.HasField("header")
    ros_msg.status.value = proto.status
    from bosdyn_msgs.msg import LeaseUseResult
    ros_msg.lease_use_results = []
    for _item in proto.lease_use_results:
        ros_msg.lease_use_results.append(LeaseUseResult())
        convert_proto_to_bosdyn_msgs_lease_use_result(_item, ros_msg.lease_use_results[-1])
    from bosdyn_msgs.msg import FailedNode
    ros_msg.failed_nodes = []
    for _item in proto.failed_nodes:
        ros_msg.failed_nodes.append(FailedNode())
        convert_proto_to_bosdyn_msgs_failed_node(_item, ros_msg.failed_nodes[-1])
    from bosdyn_msgs.msg import ElementIdentifiers
    ros_msg.element_identifiers = []
    for _item in proto.element_identifiers:
        ros_msg.element_identifiers.append(ElementIdentifiers())
        convert_proto_to_bosdyn_msgs_element_identifiers(_item, ros_msg.element_identifiers[-1])
    from bosdyn_msgs.msg import KeyInt32ValueBosdynMsgsFailedElement
    ros_msg.failed_elements = []
    for _item in proto.failed_elements:
        ros_msg.failed_elements.append(KeyInt32ValueBosdynMsgsFailedElement())
        ros_msg.failed_elements[-1].key = _item
        convert_proto_to_bosdyn_msgs_failed_element(proto.failed_elements[_item], ros_msg.failed_elements[-1].value)
    ros_msg.mission_id = proto.mission_id
    convert_proto_to_bosdyn_msgs_node_identifier(proto.docking_node, ros_msg.docking_node)
    ros_msg.docking_node_is_set = proto.HasField("docking_node")
    convert_proto_to_bosdyn_msgs_node_identifier(proto.loop_node, ros_msg.loop_node)
    ros_msg.loop_node_is_set = proto.HasField("loop_node")

def convert_bosdyn_msgs_load_autowalk_response_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.header_is_set:
        convert_bosdyn_msgs_response_header_to_proto(ros_msg.header, proto.header)
    proto.status = ros_msg.status.value
    del proto.lease_use_results[:]
    for _item in ros_msg.lease_use_results:
        convert_bosdyn_msgs_lease_use_result_to_proto(_item, proto.lease_use_results.add())
    del proto.failed_nodes[:]
    for _item in ros_msg.failed_nodes:
        convert_bosdyn_msgs_failed_node_to_proto(_item, proto.failed_nodes.add())
    del proto.element_identifiers[:]
    for _item in ros_msg.element_identifiers:
        convert_bosdyn_msgs_element_identifiers_to_proto(_item, proto.element_identifiers.add())
    for _item in ros_msg.failed_elements:
        convert_bosdyn_msgs_failed_element_to_proto(_item.value, proto.failed_elements[_item.key])
    proto.mission_id = ros_msg.mission_id
    if ros_msg.docking_node_is_set:
        convert_bosdyn_msgs_node_identifier_to_proto(ros_msg.docking_node, proto.docking_node)
    if ros_msg.loop_node_is_set:
        convert_bosdyn_msgs_node_identifier_to_proto(ros_msg.loop_node, proto.loop_node)

def convert_proto_to_bosdyn_msgs_walk(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_global_parameters(proto.global_parameters, ros_msg.global_parameters)
    ros_msg.global_parameters_is_set = proto.HasField("global_parameters")
    convert_proto_to_bosdyn_msgs_playback_mode(proto.playback_mode, ros_msg.playback_mode)
    ros_msg.playback_mode_is_set = proto.HasField("playback_mode")
    ros_msg.map_name = proto.map_name
    ros_msg.mission_name = proto.mission_name
    from bosdyn_msgs.msg import Element
    ros_msg.elements = []
    for _item in proto.elements:
        ros_msg.elements.append(Element())
        convert_proto_to_bosdyn_msgs_element(_item, ros_msg.elements[-1])
    from bosdyn_msgs.msg import Dock
    ros_msg.docks = []
    for _item in proto.docks:
        ros_msg.docks.append(Dock())
        convert_proto_to_bosdyn_msgs_dock(_item, ros_msg.docks[-1])

def convert_bosdyn_msgs_walk_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.global_parameters_is_set:
        convert_bosdyn_msgs_global_parameters_to_proto(ros_msg.global_parameters, proto.global_parameters)
    if ros_msg.playback_mode_is_set:
        convert_bosdyn_msgs_playback_mode_to_proto(ros_msg.playback_mode, proto.playback_mode)
    proto.map_name = ros_msg.map_name
    proto.mission_name = ros_msg.mission_name
    del proto.elements[:]
    for _item in ros_msg.elements:
        convert_bosdyn_msgs_element_to_proto(_item, proto.elements.add())
    del proto.docks[:]
    for _item in ros_msg.docks:
        convert_bosdyn_msgs_dock_to_proto(_item, proto.docks.add())

def convert_proto_to_bosdyn_msgs_global_parameters(proto, ros_msg):
    ros_msg.group_name = proto.group_name
    ros_msg.should_autofocus_ptz = proto.should_autofocus_ptz
    ros_msg.self_right_attempts = proto.self_right_attempts
    from bosdyn_msgs.msg import ActionRemoteGrpc
    ros_msg.post_mission_callbacks = []
    for _item in proto.post_mission_callbacks:
        ros_msg.post_mission_callbacks.append(ActionRemoteGrpc())
        convert_proto_to_bosdyn_msgs_action_remote_grpc(_item, ros_msg.post_mission_callbacks[-1])

def convert_bosdyn_msgs_global_parameters_to_proto(ros_msg, proto):
    proto.Clear()
    proto.group_name = ros_msg.group_name
    proto.should_autofocus_ptz = ros_msg.should_autofocus_ptz
    proto.self_right_attempts = ros_msg.self_right_attempts
    del proto.post_mission_callbacks[:]
    for _item in ros_msg.post_mission_callbacks:
        convert_bosdyn_msgs_action_remote_grpc_to_proto(_item, proto.post_mission_callbacks.add())

def convert_proto_to_bosdyn_msgs_playback_mode_once(proto, ros_msg):
    ros_msg.skip_docking_after_completion = proto.skip_docking_after_completion

def convert_bosdyn_msgs_playback_mode_once_to_proto(ros_msg, proto):
    proto.Clear()
    proto.skip_docking_after_completion = ros_msg.skip_docking_after_completion

def convert_proto_to_bosdyn_msgs_playback_mode_periodic(proto, ros_msg):
    convert_proto_to_builtin_interfaces_duration(proto.interval, ros_msg.interval)
    ros_msg.interval_is_set = proto.HasField("interval")
    ros_msg.repetitions = proto.repetitions

def convert_bosdyn_msgs_playback_mode_periodic_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.interval_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.interval, proto.interval)
    proto.repetitions = ros_msg.repetitions

def convert_bosdyn_msgs_playback_mode_continuous_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_playback_mode_one_of_mode(proto, ros_msg):
    if proto.HasField("once"):
        ros_msg.mode_choice = ros_msg.MODE_ONCE_SET
        convert_proto_to_bosdyn_msgs_playback_mode_once(proto.once, ros_msg.once)
    if proto.HasField("periodic"):
        ros_msg.mode_choice = ros_msg.MODE_PERIODIC_SET
        convert_proto_to_bosdyn_msgs_playback_mode_periodic(proto.periodic, ros_msg.periodic)
    if proto.HasField("continuous"):
        ros_msg.mode_choice = ros_msg.MODE_CONTINUOUS_SET
        convert_proto_to_bosdyn_msgs_playback_mode_continuous(proto.continuous, ros_msg.continuous)

def convert_bosdyn_msgs_playback_mode_one_of_mode_to_proto(ros_msg, proto):
    proto.ClearField("mode")
    if ros_msg.mode_choice == ros_msg.MODE_ONCE_SET:
        convert_bosdyn_msgs_playback_mode_once_to_proto(ros_msg.once, proto.once)
    if ros_msg.mode_choice == ros_msg.MODE_PERIODIC_SET:
        convert_bosdyn_msgs_playback_mode_periodic_to_proto(ros_msg.periodic, proto.periodic)
    if ros_msg.mode_choice == ros_msg.MODE_CONTINUOUS_SET:
        convert_bosdyn_msgs_playback_mode_continuous_to_proto(ros_msg.continuous, proto.continuous)

def convert_proto_to_bosdyn_msgs_playback_mode(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_playback_mode_one_of_mode(proto, ros_msg.mode)

def convert_bosdyn_msgs_playback_mode_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_playback_mode_one_of_mode_to_proto(ros_msg.mode, proto)

def convert_proto_to_bosdyn_msgs_element(proto, ros_msg):
    ros_msg.name = proto.name
    convert_proto_to_bosdyn_msgs_target(proto.target, ros_msg.target)
    ros_msg.target_is_set = proto.HasField("target")
    convert_proto_to_bosdyn_msgs_failure_behavior(proto.target_failure_behavior, ros_msg.target_failure_behavior)
    ros_msg.target_failure_behavior_is_set = proto.HasField("target_failure_behavior")
    convert_proto_to_bosdyn_msgs_action(proto.action, ros_msg.action)
    ros_msg.action_is_set = proto.HasField("action")
    convert_proto_to_bosdyn_msgs_action_wrapper(proto.action_wrapper, ros_msg.action_wrapper)
    ros_msg.action_wrapper_is_set = proto.HasField("action_wrapper")
    convert_proto_to_bosdyn_msgs_failure_behavior(proto.action_failure_behavior, ros_msg.action_failure_behavior)
    ros_msg.action_failure_behavior_is_set = proto.HasField("action_failure_behavior")
    ros_msg.is_skipped = proto.is_skipped
    convert_proto_to_bosdyn_msgs_battery_monitor(proto.battery_monitor, ros_msg.battery_monitor)
    ros_msg.battery_monitor_is_set = proto.HasField("battery_monitor")

def convert_bosdyn_msgs_element_to_proto(ros_msg, proto):
    proto.Clear()
    proto.name = ros_msg.name
    if ros_msg.target_is_set:
        convert_bosdyn_msgs_target_to_proto(ros_msg.target, proto.target)
    if ros_msg.target_failure_behavior_is_set:
        convert_bosdyn_msgs_failure_behavior_to_proto(ros_msg.target_failure_behavior, proto.target_failure_behavior)
    if ros_msg.action_is_set:
        convert_bosdyn_msgs_action_to_proto(ros_msg.action, proto.action)
    if ros_msg.action_wrapper_is_set:
        convert_bosdyn_msgs_action_wrapper_to_proto(ros_msg.action_wrapper, proto.action_wrapper)
    if ros_msg.action_failure_behavior_is_set:
        convert_bosdyn_msgs_failure_behavior_to_proto(ros_msg.action_failure_behavior, proto.action_failure_behavior)
    proto.is_skipped = ros_msg.is_skipped
    if ros_msg.battery_monitor_is_set:
        convert_bosdyn_msgs_battery_monitor_to_proto(ros_msg.battery_monitor, proto.battery_monitor)

def convert_proto_to_bosdyn_msgs_target_relocalize(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_set_localization_request(proto.set_localization_request, ros_msg.set_localization_request)
    ros_msg.set_localization_request_is_set = proto.HasField("set_localization_request")

def convert_bosdyn_msgs_target_relocalize_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.set_localization_request_is_set:
        convert_bosdyn_msgs_set_localization_request_to_proto(ros_msg.set_localization_request, proto.set_localization_request)

def convert_proto_to_bosdyn_msgs_target_navigate_to(proto, ros_msg):
    ros_msg.destination_waypoint_id = proto.destination_waypoint_id
    convert_proto_to_bosdyn_msgs_travel_params(proto.travel_params, ros_msg.travel_params)
    ros_msg.travel_params_is_set = proto.HasField("travel_params")

def convert_bosdyn_msgs_target_navigate_to_to_proto(ros_msg, proto):
    proto.Clear()
    proto.destination_waypoint_id = ros_msg.destination_waypoint_id
    if ros_msg.travel_params_is_set:
        convert_bosdyn_msgs_travel_params_to_proto(ros_msg.travel_params, proto.travel_params)

def convert_proto_to_bosdyn_msgs_target_navigate_route(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_route(proto.route, ros_msg.route)
    ros_msg.route_is_set = proto.HasField("route")
    convert_proto_to_bosdyn_msgs_travel_params(proto.travel_params, ros_msg.travel_params)
    ros_msg.travel_params_is_set = proto.HasField("travel_params")

def convert_bosdyn_msgs_target_navigate_route_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.route_is_set:
        convert_bosdyn_msgs_route_to_proto(ros_msg.route, proto.route)
    if ros_msg.travel_params_is_set:
        convert_bosdyn_msgs_travel_params_to_proto(ros_msg.travel_params, proto.travel_params)

def convert_proto_to_bosdyn_msgs_target_one_of_target(proto, ros_msg):
    if proto.HasField("navigate_to"):
        ros_msg.target_choice = ros_msg.TARGET_NAVIGATE_TO_SET
        convert_proto_to_bosdyn_msgs_target_navigate_to(proto.navigate_to, ros_msg.navigate_to)
    if proto.HasField("navigate_route"):
        ros_msg.target_choice = ros_msg.TARGET_NAVIGATE_ROUTE_SET
        convert_proto_to_bosdyn_msgs_target_navigate_route(proto.navigate_route, ros_msg.navigate_route)

def convert_bosdyn_msgs_target_one_of_target_to_proto(ros_msg, proto):
    proto.ClearField("target")
    if ros_msg.target_choice == ros_msg.TARGET_NAVIGATE_TO_SET:
        convert_bosdyn_msgs_target_navigate_to_to_proto(ros_msg.navigate_to, proto.navigate_to)
    if ros_msg.target_choice == ros_msg.TARGET_NAVIGATE_ROUTE_SET:
        convert_bosdyn_msgs_target_navigate_route_to_proto(ros_msg.navigate_route, proto.navigate_route)

def convert_proto_to_bosdyn_msgs_target(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_target_one_of_target(proto, ros_msg.target)
    convert_proto_to_bosdyn_msgs_target_relocalize(proto.relocalize, ros_msg.relocalize)
    ros_msg.relocalize_is_set = proto.HasField("relocalize")

def convert_bosdyn_msgs_target_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_target_one_of_target_to_proto(ros_msg.target, proto)
    if ros_msg.relocalize_is_set:
        convert_bosdyn_msgs_target_relocalize_to_proto(ros_msg.relocalize, proto.relocalize)

def convert_proto_to_bosdyn_msgs_action_sleep(proto, ros_msg):
    convert_proto_to_builtin_interfaces_duration(proto.duration, ros_msg.duration)
    ros_msg.duration_is_set = proto.HasField("duration")

def convert_bosdyn_msgs_action_sleep_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.duration_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.duration, proto.duration)

def convert_proto_to_bosdyn_msgs_action_data_acquisition(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_acquire_data_request(proto.acquire_data_request, ros_msg.acquire_data_request)
    ros_msg.acquire_data_request_is_set = proto.HasField("acquire_data_request")
    ros_msg.completion_behavior.value = proto.completion_behavior

def convert_bosdyn_msgs_action_data_acquisition_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.acquire_data_request_is_set:
        convert_bosdyn_msgs_acquire_data_request_to_proto(ros_msg.acquire_data_request, proto.acquire_data_request)
    proto.completion_behavior = ros_msg.completion_behavior.value

def convert_proto_to_bosdyn_msgs_action_remote_grpc(proto, ros_msg):
    ros_msg.service_name = proto.service_name
    convert_proto_to_builtin_interfaces_duration(proto.rpc_timeout, ros_msg.rpc_timeout)
    ros_msg.rpc_timeout_is_set = proto.HasField("rpc_timeout")
    ros_msg.lease_resources = []
    for _item in proto.lease_resources:
        ros_msg.lease_resources.append(_item)
    from bosdyn_msgs.msg import KeyValue
    ros_msg.inputs = []
    for _item in proto.inputs:
        ros_msg.inputs.append(KeyValue())
        convert_proto_to_bosdyn_msgs_key_value(_item, ros_msg.inputs[-1])

def convert_bosdyn_msgs_action_remote_grpc_to_proto(ros_msg, proto):
    proto.Clear()
    proto.service_name = ros_msg.service_name
    if ros_msg.rpc_timeout_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.rpc_timeout, proto.rpc_timeout)
    del proto.lease_resources[:]
    for _item in ros_msg.lease_resources:
        proto.lease_resources.add(_item)
    del proto.inputs[:]
    for _item in ros_msg.inputs:
        convert_bosdyn_msgs_key_value_to_proto(_item, proto.inputs.add())

def convert_proto_to_bosdyn_msgs_action_one_of_action(proto, ros_msg):
    if proto.HasField("sleep"):
        ros_msg.action_choice = ros_msg.ACTION_SLEEP_SET
        convert_proto_to_bosdyn_msgs_action_sleep(proto.sleep, ros_msg.sleep)
    if proto.HasField("data_acquisition"):
        ros_msg.action_choice = ros_msg.ACTION_DATA_ACQUISITION_SET
        convert_proto_to_bosdyn_msgs_action_data_acquisition(proto.data_acquisition, ros_msg.data_acquisition)
    if proto.HasField("remote_grpc"):
        ros_msg.action_choice = ros_msg.ACTION_REMOTE_GRPC_SET
        convert_proto_to_bosdyn_msgs_action_remote_grpc(proto.remote_grpc, ros_msg.remote_grpc)
    if proto.HasField("node"):
        ros_msg.action_choice = ros_msg.ACTION_NODE_SET
        convert_proto_to_bosdyn_msgs_node(proto.node, ros_msg.node)

def convert_bosdyn_msgs_action_one_of_action_to_proto(ros_msg, proto):
    proto.ClearField("action")
    if ros_msg.action_choice == ros_msg.ACTION_SLEEP_SET:
        convert_bosdyn_msgs_action_sleep_to_proto(ros_msg.sleep, proto.sleep)
    if ros_msg.action_choice == ros_msg.ACTION_DATA_ACQUISITION_SET:
        convert_bosdyn_msgs_action_data_acquisition_to_proto(ros_msg.data_acquisition, proto.data_acquisition)
    if ros_msg.action_choice == ros_msg.ACTION_REMOTE_GRPC_SET:
        convert_bosdyn_msgs_action_remote_grpc_to_proto(ros_msg.remote_grpc, proto.remote_grpc)
    if ros_msg.action_choice == ros_msg.ACTION_NODE_SET:
        convert_bosdyn_msgs_node_to_proto(ros_msg.node, proto.node)

def convert_proto_to_bosdyn_msgs_action(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_action_one_of_action(proto, ros_msg.action)

def convert_bosdyn_msgs_action_to_proto(ros_msg, proto):
    proto.Clear()
    convert_bosdyn_msgs_action_one_of_action_to_proto(ros_msg.action, proto)

def convert_bosdyn_msgs_action_wrapper_robot_body_sit_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_action_wrapper_robot_body_pose(proto, ros_msg):
    convert_proto_to_geometry_msgs_pose(proto.target_tform_body, ros_msg.target_tform_body)
    ros_msg.target_tform_body_is_set = proto.HasField("target_tform_body")

def convert_bosdyn_msgs_action_wrapper_robot_body_pose_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.target_tform_body_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.target_tform_body, proto.target_tform_body)

def convert_proto_to_bosdyn_msgs_action_wrapper_spot_cam_led(proto, ros_msg):
    from bosdyn_msgs.msg import KeyInt32ValueFloat32
    ros_msg.brightnesses = []
    for _item in proto.brightnesses:
        ros_msg.brightnesses.append(KeyInt32ValueFloat32())
        ros_msg.brightnesses[-1].key = _item
        ros_msg.brightnesses[-1].value = proto.brightnesses[_item]

def convert_bosdyn_msgs_action_wrapper_spot_cam_led_to_proto(ros_msg, proto):
    proto.Clear()
    for _item in ros_msg.brightnesses:
        proto.brightnesses[_item.key] = _item.value

def convert_proto_to_bosdyn_msgs_action_wrapper_spot_cam_ptz(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_ptz_position(proto.ptz_position, ros_msg.ptz_position)
    ros_msg.ptz_position_is_set = proto.HasField("ptz_position")

def convert_bosdyn_msgs_action_wrapper_spot_cam_ptz_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.ptz_position_is_set:
        convert_bosdyn_msgs_ptz_position_to_proto(ros_msg.ptz_position, proto.ptz_position)

def convert_proto_to_bosdyn_msgs_action_wrapper_arm_sensor_pointing(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_arm_joint_trajectory(proto.joint_trajectory, ros_msg.joint_trajectory)
    ros_msg.joint_trajectory_is_set = proto.HasField("joint_trajectory")
    convert_proto_to_geometry_msgs_pose(proto.wrist_tform_tool, ros_msg.wrist_tform_tool)
    ros_msg.wrist_tform_tool_is_set = proto.HasField("wrist_tform_tool")
    convert_proto_to_bosdyn_msgs_se3_trajectory(proto.pose_trajectory_rt_target, ros_msg.pose_trajectory_rt_target)
    ros_msg.pose_trajectory_rt_target_is_set = proto.HasField("pose_trajectory_rt_target")
    convert_proto_to_bosdyn_msgs_se2_pose(proto.target_tform_measured_offset, ros_msg.target_tform_measured_offset)
    ros_msg.target_tform_measured_offset_is_set = proto.HasField("target_tform_measured_offset")
    convert_proto_to_bosdyn_msgs_body_control_params_body_assist_for_manipulation(proto.body_assist_params, ros_msg.body_assist_params)
    ros_msg.body_assist_params_is_set = proto.HasField("body_assist_params")
    ros_msg.force_stow_override = proto.force_stow_override

def convert_bosdyn_msgs_action_wrapper_arm_sensor_pointing_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.joint_trajectory_is_set:
        convert_bosdyn_msgs_arm_joint_trajectory_to_proto(ros_msg.joint_trajectory, proto.joint_trajectory)
    if ros_msg.wrist_tform_tool_is_set:
        convert_geometry_msgs_pose_to_proto(ros_msg.wrist_tform_tool, proto.wrist_tform_tool)
    if ros_msg.pose_trajectory_rt_target_is_set:
        convert_bosdyn_msgs_se3_trajectory_to_proto(ros_msg.pose_trajectory_rt_target, proto.pose_trajectory_rt_target)
    if ros_msg.target_tform_measured_offset_is_set:
        convert_bosdyn_msgs_se2_pose_to_proto(ros_msg.target_tform_measured_offset, proto.target_tform_measured_offset)
    if ros_msg.body_assist_params_is_set:
        convert_bosdyn_msgs_body_control_params_body_assist_for_manipulation_to_proto(ros_msg.body_assist_params, proto.body_assist_params)
    proto.force_stow_override = ros_msg.force_stow_override

def convert_proto_to_bosdyn_msgs_action_wrapper_gripper_camera_params(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_gripper_camera_params(proto.params, ros_msg.params)
    ros_msg.params_is_set = proto.HasField("params")

def convert_bosdyn_msgs_action_wrapper_gripper_camera_params_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.params_is_set:
        convert_bosdyn_msgs_gripper_camera_params_to_proto(ros_msg.params, proto.params)

def convert_proto_to_bosdyn_msgs_action_wrapper_gripper_command(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_gripper_command_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")

def convert_bosdyn_msgs_action_wrapper_gripper_command_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_gripper_command_request_to_proto(ros_msg.request, proto.request)

def convert_proto_to_bosdyn_msgs_action_wrapper(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_action_wrapper_robot_body_sit(proto.robot_body_sit, ros_msg.robot_body_sit)
    ros_msg.robot_body_sit_is_set = proto.HasField("robot_body_sit")
    convert_proto_to_bosdyn_msgs_action_wrapper_robot_body_pose(proto.robot_body_pose, ros_msg.robot_body_pose)
    ros_msg.robot_body_pose_is_set = proto.HasField("robot_body_pose")
    convert_proto_to_bosdyn_msgs_action_wrapper_spot_cam_led(proto.spot_cam_led, ros_msg.spot_cam_led)
    ros_msg.spot_cam_led_is_set = proto.HasField("spot_cam_led")
    convert_proto_to_bosdyn_msgs_action_wrapper_spot_cam_ptz(proto.spot_cam_ptz, ros_msg.spot_cam_ptz)
    ros_msg.spot_cam_ptz_is_set = proto.HasField("spot_cam_ptz")
    convert_proto_to_bosdyn_msgs_action_wrapper_arm_sensor_pointing(proto.arm_sensor_pointing, ros_msg.arm_sensor_pointing)
    ros_msg.arm_sensor_pointing_is_set = proto.HasField("arm_sensor_pointing")
    convert_proto_to_bosdyn_msgs_action_wrapper_gripper_camera_params(proto.gripper_camera_params, ros_msg.gripper_camera_params)
    ros_msg.gripper_camera_params_is_set = proto.HasField("gripper_camera_params")
    convert_proto_to_bosdyn_msgs_action_wrapper_gripper_command(proto.gripper_command, ros_msg.gripper_command)
    ros_msg.gripper_command_is_set = proto.HasField("gripper_command")

def convert_bosdyn_msgs_action_wrapper_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.robot_body_sit_is_set:
        convert_bosdyn_msgs_action_wrapper_robot_body_sit_to_proto(ros_msg.robot_body_sit, proto.robot_body_sit)
    if ros_msg.robot_body_pose_is_set:
        convert_bosdyn_msgs_action_wrapper_robot_body_pose_to_proto(ros_msg.robot_body_pose, proto.robot_body_pose)
    if ros_msg.spot_cam_led_is_set:
        convert_bosdyn_msgs_action_wrapper_spot_cam_led_to_proto(ros_msg.spot_cam_led, proto.spot_cam_led)
    if ros_msg.spot_cam_ptz_is_set:
        convert_bosdyn_msgs_action_wrapper_spot_cam_ptz_to_proto(ros_msg.spot_cam_ptz, proto.spot_cam_ptz)
    if ros_msg.arm_sensor_pointing_is_set:
        convert_bosdyn_msgs_action_wrapper_arm_sensor_pointing_to_proto(ros_msg.arm_sensor_pointing, proto.arm_sensor_pointing)
    if ros_msg.gripper_camera_params_is_set:
        convert_bosdyn_msgs_action_wrapper_gripper_camera_params_to_proto(ros_msg.gripper_camera_params, proto.gripper_camera_params)
    if ros_msg.gripper_command_is_set:
        convert_bosdyn_msgs_action_wrapper_gripper_command_to_proto(ros_msg.gripper_command, proto.gripper_command)

def convert_proto_to_bosdyn_msgs_failure_behavior_safe_power_off(proto, ros_msg):
    convert_proto_to_bosdyn_msgs_safe_power_off_command_request(proto.request, ros_msg.request)
    ros_msg.request_is_set = proto.HasField("request")

def convert_bosdyn_msgs_failure_behavior_safe_power_off_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.request_is_set:
        convert_bosdyn_msgs_safe_power_off_command_request_to_proto(ros_msg.request, proto.request)

def convert_bosdyn_msgs_failure_behavior_proceed_if_able_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_failure_behavior_return_to_start_and_try_again_later(proto, ros_msg):
    convert_proto_to_builtin_interfaces_duration(proto.try_again_delay, ros_msg.try_again_delay)
    ros_msg.try_again_delay_is_set = proto.HasField("try_again_delay")

def convert_bosdyn_msgs_failure_behavior_return_to_start_and_try_again_later_to_proto(ros_msg, proto):
    proto.Clear()
    if ros_msg.try_again_delay_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.try_again_delay, proto.try_again_delay)

def convert_bosdyn_msgs_failure_behavior_return_to_start_and_terminate_to_proto(ros_msg, proto):
    proto.Clear()

def convert_proto_to_bosdyn_msgs_failure_behavior_one_of_default_behavior(proto, ros_msg):
    if proto.HasField("safe_power_off"):
        ros_msg.default_behavior_choice = ros_msg.DEFAULT_BEHAVIOR_SAFE_POWER_OFF_SET
        convert_proto_to_bosdyn_msgs_failure_behavior_safe_power_off(proto.safe_power_off, ros_msg.safe_power_off)
    if proto.HasField("proceed_if_able"):
        ros_msg.default_behavior_choice = ros_msg.DEFAULT_BEHAVIOR_PROCEED_IF_ABLE_SET
        convert_proto_to_bosdyn_msgs_failure_behavior_proceed_if_able(proto.proceed_if_able, ros_msg.proceed_if_able)
    if proto.HasField("return_to_start_and_try_again_later"):
        ros_msg.default_behavior_choice = ros_msg.DEFAULT_BEHAVIOR_RETURN_TO_START_AND_TRY_AGAIN_LATER_SET
        convert_proto_to_bosdyn_msgs_failure_behavior_return_to_start_and_try_again_later(proto.return_to_start_and_try_again_later, ros_msg.return_to_start_and_try_again_later)
    if proto.HasField("return_to_start_and_terminate"):
        ros_msg.default_behavior_choice = ros_msg.DEFAULT_BEHAVIOR_RETURN_TO_START_AND_TERMINATE_SET
        convert_proto_to_bosdyn_msgs_failure_behavior_return_to_start_and_terminate(proto.return_to_start_and_terminate, ros_msg.return_to_start_and_terminate)

def convert_bosdyn_msgs_failure_behavior_one_of_default_behavior_to_proto(ros_msg, proto):
    proto.ClearField("default_behavior")
    if ros_msg.default_behavior_choice == ros_msg.DEFAULT_BEHAVIOR_SAFE_POWER_OFF_SET:
        convert_bosdyn_msgs_failure_behavior_safe_power_off_to_proto(ros_msg.safe_power_off, proto.safe_power_off)
    if ros_msg.default_behavior_choice == ros_msg.DEFAULT_BEHAVIOR_PROCEED_IF_ABLE_SET:
        convert_bosdyn_msgs_failure_behavior_proceed_if_able_to_proto(ros_msg.proceed_if_able, proto.proceed_if_able)
    if ros_msg.default_behavior_choice == ros_msg.DEFAULT_BEHAVIOR_RETURN_TO_START_AND_TRY_AGAIN_LATER_SET:
        convert_bosdyn_msgs_failure_behavior_return_to_start_and_try_again_later_to_proto(ros_msg.return_to_start_and_try_again_later, proto.return_to_start_and_try_again_later)
    if ros_msg.default_behavior_choice == ros_msg.DEFAULT_BEHAVIOR_RETURN_TO_START_AND_TERMINATE_SET:
        convert_bosdyn_msgs_failure_behavior_return_to_start_and_terminate_to_proto(ros_msg.return_to_start_and_terminate, proto.return_to_start_and_terminate)

def convert_proto_to_bosdyn_msgs_failure_behavior(proto, ros_msg):
    ros_msg.retry_count = proto.retry_count
    convert_proto_to_builtin_interfaces_duration(proto.prompt_duration, ros_msg.prompt_duration)
    ros_msg.prompt_duration_is_set = proto.HasField("prompt_duration")
    convert_proto_to_bosdyn_msgs_failure_behavior_one_of_default_behavior(proto, ros_msg.default_behavior)

def convert_bosdyn_msgs_failure_behavior_to_proto(ros_msg, proto):
    proto.Clear()
    proto.retry_count = ros_msg.retry_count
    if ros_msg.prompt_duration_is_set:
        convert_builtin_interfaces_duration_to_proto(ros_msg.prompt_duration, proto.prompt_duration)
    convert_bosdyn_msgs_failure_behavior_one_of_default_behavior_to_proto(ros_msg.default_behavior, proto)

def convert_proto_to_bosdyn_msgs_battery_monitor(proto, ros_msg):
    ros_msg.battery_start_threshold = proto.battery_start_threshold
    ros_msg.battery_stop_threshold = proto.battery_stop_threshold

def convert_bosdyn_msgs_battery_monitor_to_proto(ros_msg, proto):
    proto.Clear()
    proto.battery_start_threshold = ros_msg.battery_start_threshold
    proto.battery_stop_threshold = ros_msg.battery_stop_threshold

######### AUTOMATICALLY CREATED FILES END HERE ########
