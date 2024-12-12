import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import adafruit_gps
import time

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')
        
        # Create publisher for GPS data
        self.publisher_ = self.create_publisher(NavSatFix, 'gps/fix', 10)
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('publish_rate', 1.0)  # Hz
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Set up GPS
        uart = serial.Serial(serial_port, baudrate=baud_rate, timeout=10)
        self.gps = adafruit_gps.GPS(uart, debug=False)
        
        # Initialize the GPS
        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK220,1000")
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0/publish_rate, self.publish_gps_data)
        
        self.get_logger().info('GPS Publisher Node initialized')

    def publish_gps_data(self):
        self.gps.update()
        
        if self.gps.has_fix:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            
            msg.latitude = self.gps.latitude
            msg.longitude = self.gps.longitude
            msg.altitude = self.gps.altitude_m if self.gps.altitude_m is not None else float('nan')
            
            # Set covariance to unknown
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published GPS fix: Lat={msg.latitude}, Lon={msg.longitude}, Alt={msg.altitude}')
        else:
            self.get_logger().warn('No GPS fix available')

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()
    
    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        gps_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
