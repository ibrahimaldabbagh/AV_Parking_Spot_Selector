import unittest
from unittest.mock import MagicMock, patch
from adapt_spotsl.spotsl import SpotFinder 
from v2x.msg import ItsEVCSNData, ItsChargingStationData, ReferencePosition
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped
import pymap3d as pm
import rclpy

class TestSpotFinder(unittest.TestCase):

    @patch('adapt_spotsl.spotsl.pm.geodetic2enu', return_value=(0.001358832633856962, -0.0037463000580704392, -5.650605170402778e-10))  # Mocking geodetic2enu function
    def setUp(self, mock_geodetic2enu):
        # Initialize rclpy
        rclpy.init()
        
        # Create SpotFinder node
        self.node = SpotFinder()

        # Mock publishers
        self.node.location_publisher = MagicMock()
        self.node.station_id_publisher = MagicMock()
        self.node.spot_state_publisher = MagicMock()

    def tearDown(self):
        # Shutdown the node and rclpy
        self.node.destroy_node()
        rclpy.shutdown()

    def test_update_evcsn_data(self):
        msg = ItsEVCSNData()
        self.node.update_evcsn_data(msg)
        self.assertEqual(self.node.evcsn_message, msg)

    def test_handle_start_data(self):
        msg = String()
        msg.data = "start:Zone 2:Free:end"
        
        # Mocking a charging station
        station = ItsChargingStationData()
        station.accessibility = "Free"
        station.charging_station_id = 123
        location = ReferencePosition()
        location.latitude = 502413221
        location.longitude = 113212652
        station.charging_station_location = location

        self.node.evcsn_message.charging_stations_data.append(station)

        self.node.handle_start_data(msg)

        # Check if location was published
        self.node.location_publisher.publish.assert_called_once()
        
        # Check if station id was published
        self.node.station_id_publisher.publish.assert_called_once()

        # Check if spot state was published
        self.node.spot_state_publisher.publish.assert_called_once_with(Bool(data=True))

    def test_select_station(self):
        station_free = ItsChargingStationData()
        station_free.accessibility = "Free"

        station_paying = ItsChargingStationData()
        station_paying.accessibility = "Paying"

        self.node.evcsn_message.charging_stations_data = [station_free, station_paying]

        selected_station = self.node.select_station('Free')
        self.assertEqual(selected_station, station_free)

        selected_station = self.node.select_station('Paying')
        self.assertEqual(selected_station, station_paying)

    def test_publish_location(self):
        station = ItsChargingStationData()
        station.charging_station_location.latitude = 502413221
        station.charging_station_location.longitude = 113212652

        self.node.publish_location(station)
        self.node.location_publisher.publish.assert_called_once()
        published_msg = self.node.location_publisher.publish.call_args[0][0]
        # Match the values as returned by the mocked `geodetic2enu` function
        self.assertAlmostEqual(published_msg.pose.position.x, 0.001358832633856962, places=7)
        self.assertAlmostEqual(published_msg.pose.position.y, -0.0037463000580704392, places=7)
        self.assertAlmostEqual(published_msg.pose.position.z, -5.650605170402778e-10, places=7)

    def test_publish_station_id(self):
        station = ItsChargingStationData()
        station.charging_station_id = 123

        self.node.publish_station_id(station)
        self.node.station_id_publisher.publish.assert_called_once_with(Int32(data=123))

if __name__ == '__main__':
    unittest.main()