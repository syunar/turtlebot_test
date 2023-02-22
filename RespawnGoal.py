import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

def spawn_circle(position):
    node = rclpy.create_node('spawn_circle_node')

    spawn_entity = node.create_client(SpawnEntity, '/spawn_entity')
    print(f'{position[0], position[1]}')
    circle_sdf = """
    <?xml version='1.0'?>
    <sdf version='1.6'>
    <model name='circle'>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <link name='link'>
            <visual name='visual'>
                <pose frame=''>0 0 0.0005 0 -0 0</pose>
                <geometry>
                    <cylinder>
                        <radius>0.25</radius>
                        <length>0.001</length>
                    </cylinder>
                </geometry>
                <material>
                    <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Grey</name>
                    </script>
                    <ambient>1 0 0 1</ambient>
                </material>
            </visual>
            <pose frame=''>0 0 0 0 -0 0</pose>
        </link>
        <static>1</static>
    </model>
    </sdf>
    """
    entity_name = 'circle'

    req = SpawnEntity.Request()
    req.xml = circle_sdf
    req.initial_pose.position = Point(x=float(position[0]), y=float(position[1]), z=0.0)
    req.reference_frame = 'world'

    future = spawn_entity.call_async(req)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Successfully spawned {entity_name} at {position}")
    else:
        node.get_logger().error(f"Failed to spawn {entity_name}: {future.exception()}")
        
    node.destroy_node()
        

def delete_circle():
    node = rclpy.create_node('delete_circle_node')

    delete_entity = node.create_client(DeleteEntity, '/delete_entity')

    entity_name = 'circle'

    req = DeleteEntity.Request()
    req.name = entity_name

    future = delete_entity.call_async(req)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Successfully deleted {entity_name}")
    else:
        node.get_logger().error(f"Failed to delete {entity_name}: {future.exception()}")
        
    node.destroy_node()
