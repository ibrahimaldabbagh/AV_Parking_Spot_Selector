import pytest
import rclpy
from unittest.mock import MagicMock
from adapt_spotsl.spotsl import SpotFinder
from std_msgs.msg import String
from v2x.msg import ItsEVCSNData, ItsChargingStationData, ReferencePosition

@pytest.fixture(scope="module", autouse=True)
def rclpy_setup_teardown():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_update_evcsn_data(rclpy_setup_teardown):
    node = SpotFinder()
    msg = ItsEVCSNData()
    node.update_evcsn_data(msg)
    assert node.evcsn_message == msg

def test_handle_start_data(rclpy_setup_teardown):
    node = SpotFinder()
    node.publish_location = MagicMock()  # Mock the publish_location method

    # Prepare mock data
    msg = String()
    msg.data = "start:Zone 2:end"
    station = ItsChargingStationData()
    station.accessibility = 0  # Integer value for 'free'
    node.evcsn_message = ItsEVCSNData()
    node.evcsn_message.charging_stations_data = [station]

    # Call handle_start_data
    node.handle_start_data(msg)

    # Verify that publish_location was called with the correct station
    node.publish_location.assert_called_once_with(station)

def test_select_station(rclpy_setup_teardown):
    node = SpotFinder()
    station1 = ItsChargingStationData()
    station1.accessibility = 0  # Integer value for 'free'
    station2 = ItsChargingStationData()
    station2.accessibility = 1  # Integer value for 'paying'
    node.evcsn_message = ItsEVCSNData()
    node.evcsn_message.charging_stations_data = [station1, station2]
    assert node.select_station('Zone 2') == station1
    assert node.select_station('Zone 3') == station2

def test_publish_location(rclpy_setup_teardown):
    node = SpotFinder()
    node.location_publisher = MagicMock()

    station = ItsChargingStationData()
    location = ReferencePosition()
    location.latitude = 502413221
    location.longitude = 113212652
    station.charging_station_location = location

    node.publish_location(station)

    # Check if publish was called once
    assert node.location_publisher.publish.call_count == 1

    # Retrieve the published message
    published_msg = node.location_publisher.publish.call_args[0][0]

    # Verify the published message coordinates
    assert published_msg.pose.position.x != 0
    assert published_msg.pose.position.y != 0
