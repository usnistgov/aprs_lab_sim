#!/usr/bin/env python3

from typing import cast
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.duration import Duration

from controller_manager_msgs.srv import (
    LoadController,
    ConfigureController,
    SwitchController
)

from std_srvs.srv import Trigger

from aprs_gz_sim.utils import ROSAsyncAdapter

   
class ControllerStarter(Node):
    def __init__(self):
        super().__init__("controller_starter_node")

        sim_time_param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([sim_time_param])

        # Declare mirror_env so it can be read when passed in from the launch file
        self.declare_parameter('mirror_env', True)
        raw_mirror = self.get_parameter("mirror_env").value

        # mirror_env may be provided as a bool or as a string from launch substitutions
        if isinstance(raw_mirror, bool):
            self.mirror_env = raw_mirror
        else:
            self.mirror_env = str(raw_mirror).lower() == 'true'

        base_controllers = ['joint_state_broadcaster', 'joint_trajectory_controller']
        mimic_controller = 'passthrough_controller'

        self.robot_controllers = {
            'fanuc': base_controllers + ([mimic_controller] if self.mirror_env else []),
            'motoman': base_controllers + ([mimic_controller] if self.mirror_env else []),
            'franka': base_controllers,
            'ur': base_controllers
        }

        self.get_logger().info(f"{self.robot_controllers}")

        if self.mirror_env:
            self.active_controllers = {
                "fanuc": "passthrough_controller",
                "motoman": "passthrough_controller"
            }

            self.create_service(Trigger, 'switch_motoman_controller', self.handle_switch_motoman)
            self.create_service(Trigger, 'switch_fanuc_controller', self.handle_switch_fanuc)


    def handle_switch_motoman(self, request: Trigger.Request, response: Trigger.Response):
        # Schedule the coroutine on the running asyncio loop from the ROS callback thread
        try:
            loop = getattr(self, '_asyncio_loop')
            asyncio.run_coroutine_threadsafe(self._async_switch_controllers("motoman"), loop)
        except Exception:
            # Fallback if loop not set yet (should not happen normally)
            asyncio.create_task(self._async_switch_controllers("motoman"))
        response.success = True
        response.message = 'Switch request queued'
        return response

    def handle_switch_fanuc(self, request: Trigger.Request, response: Trigger.Response):
        try:
            loop = getattr(self, '_asyncio_loop')
            asyncio.run_coroutine_threadsafe(self._async_switch_controllers("fanuc"), loop)
        except Exception:
            asyncio.create_task(self._async_switch_controllers("fanuc"))
        response.success = True
        response.message = 'Switch request queued'
        return response

    async def _async_switch_controllers(self, robot_name: str):
        try:
            self.get_logger().info(f"Switching from controller {self.active_controllers[robot_name]} for robot {robot_name}")
            client = self.create_client(SwitchController, f"/simulation/{robot_name}/controller_manager/switch_controller")
            await ROSAsyncAdapter.await_service_ready(client)

            req = SwitchController.Request()
            # Activate the trajectory controller for motoman
            controller_to_activate = 'joint_trajectory_controller' if self.active_controllers[robot_name] == "passthrough_controller" else "passthrough_controller"
            req.activate_controllers = [controller_to_activate]
            # deactivate_controllers must be a list
            req.deactivate_controllers = [self.active_controllers[robot_name]] if isinstance(self.active_controllers[robot_name], str) else list(self.active_controllers[robot_name])
            # Optionally, you can set deactivate controllers if supported by your controller manager
            req.strictness = 1
            req.timeout = Duration(seconds=3).to_msg()

            result = await ROSAsyncAdapter.await_service_response(client, req)
            response = cast(SwitchController.Response, result)

            if response.ok:
                self.active_controllers[robot_name] = controller_to_activate
                self.get_logger().info(f'{robot_name.capitalize()} switched to {self.active_controllers[robot_name]}')
            else:
                self.get_logger().error(f'{robot_name.capitalize()} failed to switch controllers')

        except Exception as e:
            self.get_logger().error(f'Error switching {robot_name} controllers: {e}')

    async def load_controllers(self):
        for name, controllers in self.robot_controllers.items():
            self.get_logger().info(f"Loading {controllers} for {name}")
            client = self.create_client(LoadController, f"/simulation/{name}/controller_manager/load_controller")
            await ROSAsyncAdapter.await_service_ready(client)
            for controller in controllers:
                req = LoadController.Request()
                req.name = controller

                result = await ROSAsyncAdapter.await_service_response(client, req)

                response = cast(LoadController.Response, result)

                if not response.ok:
                    raise Exception(f"{name} {controller} failed to load")

    async def configure_controllers(self):
        for name, controllers in self.robot_controllers.items():
            client = self.create_client(ConfigureController, f"/simulation/{name}/controller_manager/configure_controller")
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
            client = self.create_client(SwitchController, f"/simulation/{name}/controller_manager/switch_controller")
            await ROSAsyncAdapter.await_service_ready(client)
            req = SwitchController.Request()

            if self.mirror_env and name in ["fanuc", "motoman"]:
                req.activate_controllers = ["joint_state_broadcaster", "passthrough_controller"]
            else:
                req.activate_controllers = controllers
            req.strictness = 1
            req.timeout = Duration(seconds=10).to_msg()
            
            result = await ROSAsyncAdapter.await_service_response(client, req)

            response = cast(SwitchController.Response, result)

            if not response.ok:
                raise Exception(f"{name} controllers failed to activate")
    
async def run():
    controller_starter = ControllerStarter()

    executor = MultiThreadedExecutor()
    executor.add_node(controller_starter)
    shutdown_event = asyncio.Event()

    # store running asyncio loop on the controller starter so ROS callbacks can schedule coroutines
    controller_starter._asyncio_loop = asyncio.get_running_loop()

    # Start executor spinner in background so the node stays responsive
    spinner = asyncio.create_task(ROSAsyncAdapter.spin_executor(executor, shutdown_event))

    try:
        await controller_starter.load_controllers()
        await controller_starter.configure_controllers()
        await controller_starter.switch_controllers()

        controller_starter.get_logger().info('Controller starter configured and running; ready to accept switch requests')

        # Wait until interrupted (spinner runs until shutdown_event is set)
        await spinner

    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    except Exception as e:
        print(e)
    finally:
        # Signal spinner to stop and cleanup
        shutdown_event.set()
        await spinner
        try:
            executor.remove_node(controller_starter)
        except Exception:
            pass
        try:
            controller_starter.destroy_node()
        except Exception:
            pass

if __name__ == "__main__":
    rclpy.init()
    asyncio.run(run())
