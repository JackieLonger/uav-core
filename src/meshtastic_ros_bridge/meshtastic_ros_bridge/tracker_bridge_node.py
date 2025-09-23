#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import meshtastic
import meshtastic.serial_interface
from pubsub import pub

import threading
import time
import math
import glob
import os

from meshtastic_ros_bridge_msgs.msg import MeshtasticNode, MeshtasticNodeArray, MeshtasticLinkQuality, MeshtasticLinkQualityArray
from meshtastic_ros_bridge_msgs.srv import GetMeshtasticNode
from builtin_interfaces.msg import Time as TimeMsg
from std_msgs.msg import Header


class TrackerBridgeNode(Node):
    """
    A ROS 2 node to bridge Meshtastic devices to the ROS ecosystem.
    It listens to Meshtastic packet events, caches node information,
    and publishes it to various topics.
    """

    def __init__(self):
        super().__init__('tracker_bridge_node')

        # Declare parameters according to the spec
        self._declare_parameters()

        # Internal state
        self.nodes_cache = {}
        self.data_lock = threading.Lock()
        self.interfaces = []
        self.my_node_id = None

        # ROS Publishers
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.nodes_pub = self.create_publisher(MeshtasticNodeArray, '/meshtastic/nodes', qos_profile)
        self.node_updates_pub = self.create_publisher(MeshtasticNode, '/meshtastic/node_updates', qos_profile)
        self.link_quality_pub = self.create_publisher(MeshtasticLinkQualityArray, '/meshtastic/link_quality', qos_profile)

        # ROS Service
        self.get_node_srv = self.create_service(GetMeshtasticNode, '/meshtastic/get_node', self._get_node_service_callback)

        # Connect to devices and start processing
        self._connect_to_devices()
        
        # Subscribe to Meshtastic events
        pub.subscribe(self._on_receive_callback, "meshtastic.receive")

        # ROS Timer for periodic snapshot publishing
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.publish_timer = self.create_timer(1.0 / publish_rate, self._publish_snapshots_callback)

        self.get_logger().info("Tracker Bridge Node has started successfully.")

    def _declare_parameters(self):
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('ports', ['auto'])
        self.declare_parameter('frame_id', "meshtastic_mesh")
        self.declare_parameter('service_include_unseen', False)
        # Link Quality Parameters
        self.declare_parameter('rssi_floor', -120.0)
        self.declare_parameter('rssi_ceil', -60.0)
        self.declare_parameter('snr_floor', -20.0)
        self.declare_parameter('snr_ceil', 10.0)
        self.declare_parameter('rssi_gate', -110.0)
        self.declare_parameter('snr_gate', -10.0)
        self.declare_parameter('w_rssi', 0.4)
        self.declare_parameter('w_snr', 0.6)
        self.declare_parameter('fresh_tau', 5.0)
        self.get_logger().info("Parameters declared.")

    def _connect_to_devices(self):
        ports_param = self.get_parameter('ports').value

        # Handle if a single string is passed instead of a list
        if isinstance(ports_param, str):
            ports_param = [ports_param]

        ports_to_try = []
        if 'auto' in ports_param:
            ports_to_try.extend(self._autoscan_ports())
        else:
            ports_to_try.extend(ports_param)

        if not ports_to_try:
            self.get_logger().warning("No serial ports specified or found in auto-scan.")
        
        connected_ports = []
        for port in set(ports_to_try): # Use set to avoid duplicates
            try:
                self.get_logger().info(f"Attempting to connect to Meshtastic device on {port}...")
                # noProto=False is critical to get structured data
                iface = meshtastic.serial_interface.SerialInterface(port, noProto=False)
                # Store local node ID to filter it out later
                self.my_node_id = iface.myInfo.my_node_num 
                self.interfaces.append(iface)
                connected_ports.append(port)
                self.get_logger().info(f"Successfully connected to {port}. Local Node ID: !{self.my_node_id:x}")
            except Exception as e:
                self.get_logger().warning(f"Failed to connect to {port}: {e}. If it's a permission error, try 'sudo usermod -a -G dialout $USER'.")

        if not self.interfaces:
            self.get_logger().error(f"No Meshtastic serial device found. Tried: {ports_to_try}")
            raise RuntimeError('No Meshtastic serial device found')
        
        self.get_logger().info(f"Connected to ports: {connected_ports}")


    def _autoscan_ports(self):
        """Scans for likely serial ports."""
        self.get_logger().info("Auto-scanning for serial ports...")
        # Prefer by-id for stability
        by_id_paths = glob.glob('/dev/serial/by-id/*')
        if by_id_paths:
            self.get_logger().info(f"Found ports by-id: {by_id_paths}")
            return by_id_paths
        
        # Fallback to ttyUSB
        tty_usb_paths = glob.glob('/dev/ttyUSB*')
        if tty_usb_paths:
            self.get_logger().info(f"Found ports by ttyUSB: {tty_usb_paths}")
            return tty_usb_paths

        self.get_logger().info("Auto-scan found no potential ports.")
        return []

    def _on_receive_callback(self, packet, interface):
        """Callback for meshtastic.receive events."""
        try:
            if 'fromId' not in packet:
                return # Ignore packets without a source ID

            node_id = packet['fromId']

            # This is a crucial filter: we only care about packets from OTHER nodes
            if packet.get('channel') == 0 and packet.get('decoded', {}).get('portnum') == 'ROUTING_APP' :
                return # Filter out mesh-internal routing packets
            
            # The official python API getNode() is thread-safe and provides a convenient way
            # to get the latest info, which might include data not in this specific packet (like name)
            node_info = interface.nodes.get(node_id)
            if not node_info:
                return # Should not happen if fromId is present, but good practice

            now_ros_time = self.get_clock().now()
            now_msg_time = now_ros_time.to_msg()
            
            with self.data_lock:
                # Initialize cache for new node
                if node_id not in self.nodes_cache:
                    self.nodes_cache[node_id] = { 'seen': False }

                cached_node = self.nodes_cache[node_id]
                
                # Create a MeshtasticNode message for comparison and potential update publication
                update_msg = MeshtasticNode()
                update_msg.id = node_id

                # Update values and check for changes
                changed = False
                
                # Names
                long_name = node_info.get('user', {}).get('longName', '')
                short_name = node_info.get('user', {}).get('shortName', '')
                if cached_node.get('longname') != long_name: changed = True
                if cached_node.get('shortname') != short_name: changed = True
                cached_node['longname'] = update_msg.longname = long_name
                cached_node['shortname'] = update_msg.shortname = short_name

                # Position
                lat, lon = None, None
                if 'position' in packet.get('decoded', {}):
                    lat = packet['decoded']['position'].get('latitude')
                    lon = packet['decoded']['position'].get('longitude')
                elif 'position' in packet: # Sometimes it is at the top level
                    lat = packet['position'].get('latitude')
                    lon = packet['position'].get('longitude')

                if lat is not None and lon is not None:
                    if cached_node.get('lat') != lat: changed = True
                    if cached_node.get('lon') != lon: changed = True
                    cached_node['lat'] = update_msg.lat = float(lat)
                    cached_node['lon'] = update_msg.lon = float(lon)
                
                # RF stats
                rssi = packet.get('rxRssi')
                snr = packet.get('rxSnr')

                if rssi is not None and cached_node.get('rssi') != rssi:
                    changed = True
                cached_node['rssi'] = update_msg.rssi = float(rssi if rssi is not None else -200.0)

                if snr is not None and cached_node.get('snr') != snr:
                    changed = True
                cached_node['snr'] = update_msg.snr = float(snr if snr is not None else -99.0)

                # Timestamps
                cached_node['last_seen_ros'] = now_ros_time
                cached_node['last_seen'] = update_msg.last_seen = now_msg_time

                # This is the first time we've truly "seen" this node via packet
                if not cached_node.get('seen', False):
                    changed = True
                    cached_node['seen'] = True

                # Publish single update if anything changed
                if changed:
                    self.get_logger().debug(f"Node update for {node_id}: RSSI={rssi}, SNR={snr}")
                    self.node_updates_pub.publish(update_msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to parse received packet: {e}\nPacket: {packet}")

    def _publish_snapshots_callback(self):
        """Periodically publishes a snapshot of all seen nodes and their link quality."""
        with self.data_lock:
            if not self.nodes_cache:
                return

            now = self.get_clock().now()
            header = Header(stamp=now.to_msg(), frame_id=self.get_parameter('frame_id').value)
            
            nodes_array_msg = MeshtasticNodeArray(header=header)
            link_quality_array_msg = MeshtasticLinkQualityArray(header=header)

            for node_id, node_data in self.nodes_cache.items():
                if not node_data.get('seen', False):
                    continue

                # 1. Populate Node Array
                node_msg = MeshtasticNode(
                    id=node_id,
                    shortname=node_data.get('shortname', ''),
                    longname=node_data.get('longname', ''),
                    rssi=float(node_data.get('rssi', -200.0)),
                    snr=float(node_data.get('snr', -99.0)),
                    lat=float(node_data.get('lat', 0.0)),
                    lon=float(node_data.get('lon', 0.0)),
                    last_seen=node_data.get('last_seen', TimeMsg())
                )
                nodes_array_msg.nodes.append(node_msg)

                # 2. Populate Link Quality Array
                lq = self._calculate_link_quality(node_data)
                link_quality_msg = MeshtasticLinkQuality(
                    id=node_id,
                    rssi=lq['rssi'],
                    snr=lq['snr'],
                    q_rssi=lq['q_rssi'],
                    q_snr=lq['q_snr'],
                    q=lq['q'],
                    last_seen=node_data.get('last_seen', TimeMsg())
                )
                link_quality_array_msg.links.append(link_quality_msg)
            
            if nodes_array_msg.nodes:
                self.nodes_pub.publish(nodes_array_msg)
            if link_quality_array_msg.links:
                self.link_quality_pub.publish(link_quality_array_msg)

    def _calculate_link_quality(self, node_data):
        """Implements the Link Quality algorithm from the spec."""
        rssi = float(node_data.get('rssi', -200.0))
        snr = float(node_data.get('snr', -99.0))
        
        # Get params
        rssi_floor = self.get_parameter('rssi_floor').value
        rssi_ceil = self.get_parameter('rssi_ceil').value
        snr_floor = self.get_parameter('snr_floor').value
        snr_ceil = self.get_parameter('snr_ceil').value
        rssi_gate_val = self.get_parameter('rssi_gate').value
        snr_gate_val = self.get_parameter('snr_gate').value
        w_rssi = self.get_parameter('w_rssi').value
        w_snr = self.get_parameter('w_snr').value
        fresh_tau = self.get_parameter('fresh_tau').value

        # Normalize RSSI and SNR to 0..1 range
        q_rssi = max(0.0, min(1.0, (rssi - rssi_floor) / (rssi_ceil - rssi_floor)))
        q_snr = max(0.0, min(1.0, (snr - snr_floor) / (snr_ceil - snr_floor)))
        
        # Gate factor
        gate = 1.0 if (rssi >= rssi_gate_val and snr >= snr_gate_val) else 0.25
        
        # Freshness factor
        age_seconds = (self.get_clock().now() - node_data.get('last_seen_ros', self.get_clock().now())).nanoseconds / 1e9
        fresh = math.exp(-age_seconds / fresh_tau)
        
        # Final LQ
        quality = gate * fresh * (w_rssi * q_rssi + w_snr * q_snr)

        return {
            'rssi': rssi,
            'snr': snr,
            'q_rssi': q_rssi,
            'q_snr': q_snr,
            'q': quality
        }

    def _get_node_service_callback(self, request, response):
        """Handles service calls to retrieve data for a specific node."""
        with self.data_lock:
            node_id = request.id
            node_data = self.nodes_cache.get(node_id)
            include_unseen = self.get_parameter('service_include_unseen').value

            if node_data and (node_data.get('seen', False) or include_unseen):
                response.found = True
                response.node = MeshtasticNode(
                    id=node_id,
                    shortname=node_data.get('shortname', ''),
                    longname=node_data.get('longname', ''),
                    rssi=float(node_data.get('rssi', -200.0)),
                    snr=float(node_data.get('snr', -99.0)),
                    lat=float(node_data.get('lat', 0.0)),
                    lon=float(node_data.get('lon', 0.0)),
                    last_seen=node_data.get('last_seen', TimeMsg())
                )
            else:
                response.found = False
        return response

    def destroy_node(self):
        """Cleanly closes resources."""
        self.get_logger().info("Shutting down Tracker Bridge Node.")
        for iface in self.interfaces:
            iface.close()
        super().destroy_node()


def main(args=None):
    #print("Start")
    rclpy.init(args=args)
    try:
        tracker_bridge_node = TrackerBridgeNode()
        rclpy.spin(tracker_bridge_node)
    except (RuntimeError, KeyboardInterrupt) as e:
        if isinstance(e, RuntimeError):
             # The error is already logged in the constructor
             pass
    finally:
        if 'tracker_bridge_node' in locals() and tracker_bridge_node.destroy_node:
            tracker_bridge_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()