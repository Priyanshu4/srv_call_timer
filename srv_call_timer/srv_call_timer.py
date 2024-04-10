import rclpy
from rclpy.node import Node
import time
import argparse
import importlib
from typing import Tuple

class ServiceTimer(Node):
    """ Measures the time it takes to call a service and receive a response.
        The service name, package, and type are provided as parameters.
        The number of calls to the service can be specified.
        Currently only services with no arguments are supported.
    """

    def __init__(self, service_name: str = "", service_package: str = "", service_type: str = "", num_calls: int = 1):
        super().__init__('service_timer')

        self.initialized = False

        # Get service name parameter
        self.declare_parameter('service_name', service_name)
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        if not self.service_name:
            self.get_logger().error('Service name not provided. Exiting...')
            return
        
        # Get service package parameter
        self.declare_parameter('service_package', service_package)
        self.service_package = self.get_parameter('service_package').get_parameter_value().string_value
        if not self.service_package:
            self.get_logger().error('Service package name not provided. Exiting...')
            return
                     
        # Get service type parameter
        self.declare_parameter('type', service_type)
        self.service_type = self.get_parameter('type').get_parameter_value().string_value
        if not self.service_type:
            self.get_logger().error('Service type not provided. Exiting...')
            return

        # Get number of calls parameter (optional)
        self.declare_parameter('num_calls', num_calls)
        self.num_calls = self.get_parameter('num_calls').get_parameter_value().integer_value

        # Dynamically import the service type
        service_import_path = f"{self.service_package}.srv"
        try:
            module = importlib.import_module(service_import_path)
            service_class = getattr(module, self.service_type)
        except Exception as e:
            self.get_logger().error(str(e))
            self.get_logger().error(f'Failed to import service from {service_import_path}. Exiting...')
            return
        
        # Create a client and measure the time spent waiting for the service to be available
        self.client = self.create_client(service_class, self.service_name)
        initial_time = time.perf_counter()
        self.get_logger().info(f'Waiting for service to be available...')
        while not self.client.wait_for_service(timeout_sec=1.0) and rclpy.ok():
            pass
        time_spent_waiting = time.perf_counter() - initial_time

        if self.client.service_is_ready():
            self.get_logger().info(f'Service is available after {time_spent_waiting} seconds.')
            self.req = service_class.Request()
        else:
            self.get_logger().error(f'Service is not available after waiting for {time_spent_waiting} seconds.')
            return

        self.initialized = True

    def time_service(self) -> list[float]:
        """ Measures the time it takes to call a service and receive a response.
            Sends self.num_calls requests to the service.
            Returns the list of sorted response times and logs relevant statistics.
        """

        response_times = []
        calls_failed = 0
        calls_succeeded = 0

        for i in range(self.num_calls):
            self.call_success = False
            success, time = self.send_request()
            if success:
                response_times.append(time)
                calls_succeeded += 1
            else:
                calls_failed += 1
            
        self.get_logger().info(f'Number of calls: {self.num_calls}')
        self.get_logger().info(f'Calls failed   : {calls_failed}')
        self.get_logger().info(f'Calls succeeded: {calls_succeeded}')

        if calls_succeeded == 0:
            self.get_logger().error('No calls succeeded. Exiting...')
            return response_times
        
        if calls_succeeded == 1:
            self.get_logger().info(f'Response Time: {response_times[0]}')
            return response_times

        response_times.sort()
        min_response_time = response_times[0]
        max_response_time = response_times[-1]
        median_response_time = median(response_times)
        mean_response_time = sum(response_times) / len(response_times)
        variance = (sum([(x - mean_response_time) ** 2 for x in response_times]) / len(response_times)) 
        std_dev = variance ** 0.5

        self.get_logger().info(f'Minimum Response Time: {min_response_time}')
        self.get_logger().info(f'Maximum Response Time: {max_response_time}')
        self.get_logger().info(f'Median Response Time : {median_response_time}')
        self.get_logger().info(f'Mean Response Time   : {mean_response_time}')
        self.get_logger().info(f'Standard Deviation   : {std_dev}')

    def send_request(self) -> Tuple[bool, float]:
        """ Sends a request to the service.
            Returns a tuple with a boolean indicating success and a float measuring response time in seconds.
        """
        request_sent_time = time.perf_counter()
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
    
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))
            return False, 0
        
        response_received_time = time.perf_counter()
        response_time = response_received_time - request_sent_time
        return True, response_time


def median(sorted_list: list[float]) -> float:
    """ Returns the median of a sorted list of floats. """
    n = len(sorted_list)
    if n % 2 == 0:
        return (sorted_list[n // 2 - 1] + sorted_list[n // 2]) / 2
    return sorted_list[n // 2]

def main(args=None) -> None:
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Service Timer Node')
    parser.add_argument('service_name', type=str, help='Name of the service')
    parser.add_argument('service_package', type=str, help='Package of the service')
    parser.add_argument('type', type=str, help='Type of the service')
    parser.add_argument('-n', '--num_calls', type=int, default=1, help='Number of calls to the service')
    args = vars(parser.parse_args(args))

    service_client = ServiceTimer(
        service_name=args.get("service_name", ""),
        service_package=args.get("service_package", ""),
        service_type=args.get("type", ""),
        num_calls=args.get("num_calls")
    )

    if service_client.initialized:
        service_client.time_service()

    rclpy.shutdown()

if __name__ == '__main__':
    main()