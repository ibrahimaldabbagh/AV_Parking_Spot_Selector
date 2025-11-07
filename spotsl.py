import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Int32, Bool
from v2x.msg import ItsEVCSNData
import pymap3d as pm

class SpotFinder(Node):
    """ROS 2 node initialization"""
    def __init__(self):
        super().__init__('spot_finder')
        self.evcsn_message = ItsEVCSNData()

        # Publisher and subscribers setup:
        self.location_publisher = self.create_publisher(PoseStamped, '/selected_spot', 10)
        self.station_id_publisher = self.create_publisher(Int32, '/station_id', 10)
        self.spot_state_publisher = self.create_publisher(Bool, '/spot_state', 10)  # Publisher for the state of the spot
        # Subscriber for EV charging station notifications:
        self.evcsn_subscriber = self.create_subscription(
            ItsEVCSNData, '/evcsn_msg', self.update_evcsn_data, 10
        )

        # Subscriber for start data (user preferences):
        self.start_subscriber = self.create_subscription(
            String, '/vi_start', self.handle_start_data, 10
        )
        self.spot_state_publisher.publish(Bool(data=False))

    def update_evcsn_data(self, msg: ItsEVCSNData):
        """Updates with the latest EVCSN data received"""
        self.evcsn_message = msg
        self.get_logger().info('Updated EVCSN data')

    def handle_start_data(self, msg: String):
        """Extracts preference from the incoming message."""
        _, _, preference, _ = msg.data.split(':')
        self.get_logger().info(f'Received preference: {preference}')
        chosen_station = self.select_station(preference)
        if chosen_station:
            self.publish_location(chosen_station)
            self.publish_station_id(chosen_station)
            self.spot_state_publisher.publish(Bool(data=True))

    def select_station(self, preference):
        """Chooses the parking spot based on user preference for free or paid parking"""
        self.get_logger().info(f'Preference: {preference}')
        for station in self.evcsn_message.charging_stations_data:
            self.get_logger().info(f'Station accessibility: {station.accessibility}')
            if station.accessibility == ("Free" if preference == 'Free' else "Paying"):
                self.get_logger().info(f'Matching station found: {station}')
                return station
        self.get_logger().info("No matching charging stations found.")
        return None

    def publish_location(self, station):
        """Publishes the location of the selected parking spot in a standardized format"""

        location = PoseStamped()
        # Conversion from microdegrees to degrees
        latitude = station.charging_station_location.latitude / 1e7
        longitude = station.charging_station_location.longitude / 1e7

        # Define the reference point
        ref_lat = 50.24132213367954
        ref_lon = 11.321265180951718
        ref_alt = 0

        # Convert geodetic to ENU coordinates
        x, y, z = pm.geodetic2enu(latitude, longitude, 0, ref_lat, ref_lon, ref_alt)

        #round coordinates to 2 decimals
        x= round(x, 2)
        y= round(y, 2)
        z= round(z, 2)
        # Update PoseStamped message with ENU coordinates
        location.pose.position.x = x
        location.pose.position.y = y
        location.pose.position.z = z

        self.location_publisher.publish(location)
        self.get_logger().info(f'Published spot at ENU coordinates x: {x}, y: {y}, z: {z}')

    def publish_station_id(self, station):
        """Publishes the ID of the selected charging station"""
        station_id_msg = Int32()
        station_id_msg.data = station.charging_station_id
        self.station_id_publisher.publish(station_id_msg)
        self.get_logger().info(f'Published station ID: {station.charging_station_id}')

def main(args=None):
    """ROS 2 recall"""
    rclpy.init(args=args)
    finder = SpotFinder()
    rclpy.spin(finder)
    finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()