import os
import json
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory 
from kinova_interface.srv import GetObjectCoordinates, GetObjectList


class CoordinateServiceNode(Node):
    def __init__(self):
        super().__init__("coordinate_service_node")
        self.get_logger().info('Coordinate Mapping Node started')

        self.coord_dictionary = self.load_coordinate_dictionary()
        self.srv_coords = self.create_service(GetObjectCoordinates, 'get_coordinates', self.get_coordinates_callback)
        self.srv_list = self.create_service(GetObjectList, 'get_object_list', self.get_object_list_callback)

    def load_coordinate_dictionary(self):
        pkg_share = get_package_share_directory('kinova_interface')
        json_path = os.path.join(pkg_share,'data','coordinate_dictionary.json')
        try:
            with open(json_path, 'r') as file:
                data = json.load(file)
                self.get_logger().info(f'Loaded {len(self.coord_dictionary)} objects from coordinate dictionary')
            return data.get('objects', {})
            
        except FileNotFoundError:
                self.get_logger().fatal(f'Coordinate Dictionary file not found at: {json_path}')
                raise SystemExit(1)
        except json.JSONDecodeError:
                self.get_logger().fatal(f'Failed to decode JSON from the file')
                raise SystemExit(1)
   
    def get_coordinates_callback(self, request, response):
        #request is the obj_id, the respose will be coordinates of obj
        obj_id = request.object_id
    
        if obj_id in self.coord_dictionary:
            coords = self.coord_dictionary[obj_id]

            response.x = coords['x']
            response.y = coords['y']
            response.z = coords['z']

            response.success = True
            response.message = "Object Found"
        else:
            response.success = False
            response.message = f"Object {obj_id} NOT Found"
        return response 

    def get_object_list_callback(self, request, response):
        # Service for the LLM Proxy || send list of objects
        response.object_list = list(self.coord_dictionary.keys())
        return response



def main(args=None):
    rclpy.init(args=args)
    node = CoordinateServiceNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
