#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import csv
import os
from datetime import datetime
import struct

class LidarCsvLogger(Node):
    def __init__(self):
        super().__init__('lidar_csv_logger')
        
        # Parameters
        self.declare_parameter('output_file', 'lidar_pointcloud.csv')
        self.declare_parameter('lidar_topic', '/lidar')
        self.declare_parameter('log_frequency', 10)  # Hz
        self.declare_parameter('max_points_per_scan', 10000)  # Limit points to avoid huge files
        
        # Get parameters
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.log_frequency = self.get_parameter('log_frequency').get_parameter_value().integer_value
        self.max_points = self.get_parameter('max_points_per_scan').get_parameter_value().integer_value
        
        # Initialize CSV file
        self.init_csv_file()
        
        # Create subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.pointcloud_callback,
            10
        )
        
        # Timer for logging at specified frequency
        self.last_log_time = self.get_clock().now()
        self.log_interval = 1.0 / self.log_frequency
        self.scan_count = 0
        
        self.get_logger().info(f'3D LIDAR PointCloud CSV Logger initialized')
        self.get_logger().info(f'Subscribing to: {self.lidar_topic}')
        self.get_logger().info(f'Output file: {self.output_file}')
        self.get_logger().info(f'Log frequency: {self.log_frequency} Hz')
        self.get_logger().info(f'Max points per scan: {self.max_points}')
        
    def init_csv_file(self):
        """Initialize the CSV file with headers"""
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(self.output_file) if os.path.dirname(self.output_file) else '.', exist_ok=True)
        
        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write headers for point cloud data
            headers = [
                'timestamp',
                'scan_id',
                'frame_id',
                'point_id',
                'x',
                'y', 
                'z',
                'intensity',
                'ring',  # For multi-ring LIDAR
                'time'   # Time offset within scan
            ]
            writer.writerow(headers)
    
    def pointcloud_callback(self, msg):
        """Callback function for PointCloud2 data"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_log_time).nanoseconds / 1e9
        
        # Check if enough time has passed based on log frequency
        if time_diff >= self.log_interval:
            self.log_pointcloud_data(msg, current_time)
            self.last_log_time = current_time
    
    def log_pointcloud_data(self, msg, timestamp):
        """Log PointCloud2 data to CSV file"""
        try:
            # Convert timestamp to readable format
            timestamp_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            
            # Read point cloud data
            points_list = []
            point_count = 0
            
            # Get field names from the point cloud
            field_names = [field.name for field in msg.fields]
            self.get_logger().info(f'Available fields: {field_names}')
            
            # Read points using sensor_msgs_py
            for point_data in pc2.read_points(msg, skip_nans=True):
                if point_count >= self.max_points:
                    break
                    
                # Extract basic XYZ coordinates
                x, y, z = point_data[0], point_data[1], point_data[2]
                
                # Extract additional fields if available
                intensity = 0.0
                ring = 0
                time_offset = 0.0
                
                # Try to extract intensity
                if len(point_data) > 3:
                    if 'intensity' in field_names:
                        intensity_idx = field_names.index('intensity')
                        if intensity_idx < len(point_data):
                            intensity = point_data[intensity_idx]
                    elif len(point_data) > 3:
                        intensity = point_data[3]  # Assume 4th field is intensity
                
                # Try to extract ring information (for multi-ring LIDAR like Velodyne)
                if 'ring' in field_names:
                    ring_idx = field_names.index('ring')
                    if ring_idx < len(point_data):
                        ring = point_data[ring_idx]
                
                # Try to extract time information
                if 'time' in field_names:
                    time_idx = field_names.index('time')
                    if time_idx < len(point_data):
                        time_offset = point_data[time_idx]
                elif 't' in field_names:
                    t_idx = field_names.index('t')
                    if t_idx < len(point_data):
                        time_offset = point_data[t_idx]
                
                points_list.append([
                    timestamp_str,
                    self.scan_count,
                    msg.header.frame_id,
                    point_count,
                    x, y, z,
                    intensity,
                    ring,
                    time_offset
                ])
                
                point_count += 1
            
            # Write all points to CSV
            if points_list:
                with open(self.output_file, 'a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerows(points_list)
                
                self.get_logger().info(
                    f'Logged scan #{self.scan_count}: {len(points_list)} points, '
                    f'Frame: {msg.header.frame_id}'
                )
                
                # Log some statistics
                if points_list:
                    x_coords = [p[4] for p in points_list]
                    y_coords = [p[5] for p in points_list]
                    z_coords = [p[6] for p in points_list]
                    
                    self.get_logger().info(
                        f'Point cloud bounds - X: [{min(x_coords):.2f}, {max(x_coords):.2f}], '
                        f'Y: [{min(y_coords):.2f}, {max(y_coords):.2f}], '
                        f'Z: [{min(z_coords):.2f}, {max(z_coords):.2f}]'
                    )
            
            self.scan_count += 1
                    
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def create_summary_csv(self, input_file=None, output_file=None):
        """Create a summary CSV with scan-level statistics"""
        if input_file is None:
            input_file = self.output_file
        if output_file is None:
            output_file = self.output_file.replace('.csv', '_summary.csv')
            
        try:
            scan_stats = {}
            
            with open(input_file, 'r') as infile:
                reader = csv.DictReader(infile)
                
                for row in reader:
                    scan_id = int(row['scan_id'])
                    
                    if scan_id not in scan_stats:
                        scan_stats[scan_id] = {
                            'timestamp': row['timestamp'],
                            'frame_id': row['frame_id'],
                            'point_count': 0,
                            'x_min': float('inf'), 'x_max': float('-inf'),
                            'y_min': float('inf'), 'y_max': float('-inf'),
                            'z_min': float('inf'), 'z_max': float('-inf'),
                            'avg_intensity': 0,
                            'total_intensity': 0
                        }
                    
                    stats = scan_stats[scan_id]
                    stats['point_count'] += 1
                    
                    x, y, z = float(row['x']), float(row['y']), float(row['z'])
                    intensity = float(row['intensity'])
                    
                    stats['x_min'] = min(stats['x_min'], x)
                    stats['x_max'] = max(stats['x_max'], x)
                    stats['y_min'] = min(stats['y_min'], y)
                    stats['y_max'] = max(stats['y_max'], y)
                    stats['z_min'] = min(stats['z_min'], z)
                    stats['z_max'] = max(stats['z_max'], z)
                    stats['total_intensity'] += intensity
            
            # Calculate averages and write summary
            with open(output_file, 'w', newline='') as outfile:
                writer = csv.writer(outfile)
                writer.writerow([
                    'scan_id', 'timestamp', 'frame_id', 'point_count',
                    'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max',
                    'avg_intensity', 'x_range', 'y_range', 'z_range'
                ])
                
                for scan_id, stats in sorted(scan_stats.items()):
                    avg_intensity = stats['total_intensity'] / stats['point_count'] if stats['point_count'] > 0 else 0
                    x_range = stats['x_max'] - stats['x_min']
                    y_range = stats['y_max'] - stats['y_min'] 
                    z_range = stats['z_max'] - stats['z_min']
                    
                    writer.writerow([
                        scan_id, stats['timestamp'], stats['frame_id'], stats['point_count'],
                        stats['x_min'], stats['x_max'], stats['y_min'], stats['y_max'],
                        stats['z_min'], stats['z_max'], avg_intensity,
                        x_range, y_range, z_range
                    ])
            
            self.get_logger().info(f'Summary CSV created: {output_file}')
            
        except Exception as e:
            self.get_logger().error(f'Error creating summary CSV: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    lidar_logger = LidarCsvLogger()
    
    try:
        rclpy.spin(lidar_logger)
    except KeyboardInterrupt:
        lidar_logger.get_logger().info('Shutting down 3D LIDAR CSV Logger...')
        
        # Create summary file on shutdown
        lidar_logger.create_summary_csv()
        
    finally:
        lidar_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
