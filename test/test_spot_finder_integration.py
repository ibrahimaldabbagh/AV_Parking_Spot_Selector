import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped
from v2x.msg import ItsEVCSNData, ItsChargingStationData, ReferencePosition, PositionConfidenceEllipse, Altitude
from unittest.mock import MagicMock
from adapt_spotsl.spotsl import SpotFinder 
import time

class IntegrationHelperNode(Node):
    def __init__(self):
        super().__init__('integration_helper_node')
        self.received_pose = None
        self.received_station_id = None
        self.received_spot_state = None

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/selected_spot',
            self.pose_callback,
            10
        )
        self.station_id_subscription = self.create_subscription(
            Int32,
            '/station_id',
            self.station_id_callback,
            10
        )
        self.spot_state_subscription = self.create_subscription(
            Bool,
            '/spot_state',
            self.spot_state_callback,
            10
        )

    def pose_callback(self, msg):
        self.received_pose = msg

    def station_id_callback(self, msg):
        self.received_station_id = msg.data

    def spot_state_callback(self, msg):
        self.received_spot_state = msg.data

@pytest.fixture(scope="module", autouse=True)
def rclpy_setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_spot_finder_integration(rclpy_setup_teardown):
    spot_finder_node = SpotFinder()
    helper_node = IntegrationHelperNode()
    executor = SingleThreadedExecutor()
    executor.add_node(spot_finder_node)
    executor.add_node(helper_node)

    try:
        # Simulate EVCSN message reception
        evcsn_msg = ItsEVCSNData()
        station = ItsChargingStationData()
        station.accessibility = "Free"  # Use string type
        location = ReferencePosition()
        location.latitude = 502413221
        location.longitude = 113212652
        station.charging_station_location = location
        station.charging_station_id = 123  # Mock station ID
        evcsn_msg.charging_stations_data.append(station)

        # Publish the EVCSN message
        spot_finder_node.evcsn_message = evcsn_msg
        # Manually trigger the callback to simulate message reception
        spot_finder_node.update_evcsn_data(evcsn_msg)

        # Simulate start message reception
        start_msg = String()
        start_msg.data = "start:Zone 2:Free:end"
        spot_finder_node.handle_start_data(start_msg)

        # Run the executor to process the messages
        for _ in range(5):
            executor.spin_once(timeout_sec=1)
            if helper_node.received_pose and helper_node.received_station_id is not None and helper_node.received_spot_state is not None:
                break
            time.sleep(0.1)

        # Verify that the location was published
        assert helper_node.received_pose is not None
        assert helper_node.received_pose.pose.position.x != 0
        assert helper_node.received_pose.pose.position.y != 0
        assert helper_node.received_pose.pose.position.z != 0

        # Verify that the station ID was published
        assert helper_node.received_station_id is not None
        assert helper_node.received_station_id == 123

        # Verify that the spot state was published
        assert helper_node.received_spot_state is not None
        assert helper_node.received_spot_state is True  # Assuming the state should be True when a spot is selected

    finally:
        executor.shutdown()
        spot_finder_node.destroy_node()
        helper_node.destroy_node()

if __name__ == '__main__':
    pytest.main()