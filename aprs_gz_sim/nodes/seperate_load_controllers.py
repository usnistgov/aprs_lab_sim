#!/usr/bin/env python3

from typing import cast
import asyncio

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

        self.mimic_env = self.get_parameter("mimc_env").value

        base_controllers = ['joint_state_broadcaster', 'joint_trajectory_controller']
        mimic_controller = 'passthrough_controller'

        self.robot_controllers = {
            'fanuc': base_controllers + [mimic_controller] if self.mimic_env else [],
            'motoman': base_controllers + [mimic_controller] if self.mimic_env else [],
            'franka': base_controllers,
            'ur': base_controllers
        }

        if self.mimic_env:
            self.active_controllers = {
                "fanuc": "passthrough_controller",
                "motoman": "passthrough_controller"
            }

            self.create_service(Trigger, 'switch_motoman_controller', self.handle_switch_motoman)
            self.create_service(Trigger, 'switch_fanuc_controller', self.handle_switch_fanuc)


    def handle_switch_motoman(self, request: Trigger.Request, response: Trigger.Response):
        asyncio.create_task(self._async_switch_controllers("motoman"))
        response.success = True
        response.message = 'Switch request queued'
        return response

    def handle_switch_fanuc(self, request: Trigger.Request, response: Trigger.Response):
        asyncio.create_task(self._async_switch_controllers("fanuc"))
        response.success = True
        response.message = 'Switch request queued'
        return response

    async def _async_switch_controllers(self, robot_name: str):
        try:
            client = self.create_client(SwitchController, f"/simulation/{robot_name}/controller_manager/switch_controller")
            await ROSAsyncAdapter.await_service_ready(client)

            req = SwitchController.Request()
            # Activate the trajectory controller for motoman
            controller_to_activate = 'joint_trajectory_controller' if self.active_controllers[robot_name] == "passthrough_controller" else "passthrough_controller"
            req.activate_controllers = [controller_to_activate]
            req.deactivate_controllers = self.active_controllers[robot_name]
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

            if self.mimic_env and name in ["fanuc", "motoman"]:
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
    



    