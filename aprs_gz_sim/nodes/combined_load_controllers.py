from typing import cast

import rclpy
import asyncio

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from controller_manager_msgs.srv import (
    LoadController,
    ConfigureController,
    SwitchController
)

from aprs_gz_sim.utils import ROSAsyncAdapter

   
class ControllerStarter(Node):
    def __init__(self):
        super().__init__("controller_starter_node")

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])

        base_controllers = ['joint_state_broadcaster', 'joint_trajectory_controller']

        self.robot_controllers = {
            'franka': base_controllers,
            'fanuc': base_controllers,
            'motoman': base_controllers,
            'ur': base_controllers,
        }

    async def load_controllers(self):
        for name, controllers in self.robot_controllers.items():
            client = self.create_client(LoadController, f"/simulation/{name}/controller_manager/load_controller")
            self.get_logger().info(f"Waiting for load service to be ready for {name}")
            await ROSAsyncAdapter.await_service_ready(client)
            for controller in controllers:
                req = LoadController.Request()
                req.name = controller

                self.get_logger().info(f"Loading {controller} for {name}")
                result = await ROSAsyncAdapter.await_service_response(client, req)

                response = cast(LoadController.Response, result)

                if not response.ok:
                    raise Exception(f"{name} {controller} failed to load")

    async def configure_controllers(self):
        for name, controllers in self.robot_controllers.items():
            client = self.create_client(ConfigureController, f"/{name}/controller_manager/configure_controller")
            self.get_logger().info(f"Waiting for configure service to be ready for {name}")
            await ROSAsyncAdapter.await_service_ready(client)
            for controller in controllers:
                req = ConfigureController.Request()
                req.name = controller

                self.get_logger().info(f"Configuring {controller} for {name}")
                result = await ROSAsyncAdapter.await_service_response(client, req)

                response = cast(ConfigureController.Response, result)

                if not response.ok:
                    raise Exception(f"{name} {controller} failed to configure")
    
    async def switch_controllers(self):
        for name, controllers in self.robot_controllers.items():
            client = self.create_client(SwitchController, f"/{name}/controller_manager/switch_controller")
            self.get_logger().info(f"Waiting for activivate service for {name}")
            await ROSAsyncAdapter.await_service_ready(client)
            req = SwitchController.Request()
            req.activate_controllers = controllers
            req.strictness = 1
            req.timeout = Duration(seconds=10).to_msg()
            
            self.get_logger().info(f"Activating {", ".join(controllers)} for {name}")
            result = await ROSAsyncAdapter.await_service_response(client, req)

            response = cast(SwitchController.Response, result)

            if not response.ok:
                raise Exception(f"{name} controllers failed to activate")

async def main():
    rclpy.init()
    controller_starter = ControllerStarter()

    executor = MultiThreadedExecutor()
    executor.add_node(controller_starter)

    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(ROSAsyncAdapter.spin_executor(executor, shutdown_event))

    controller_starter = ControllerStarter()
    try:
        await controller_starter.load_controllers()
        await controller_starter.configure_controllers()
        await controller_starter.switch_controllers()
    except Exception as e:
        print(e)


if __name__ == "__main__":
    asyncio.run(main())