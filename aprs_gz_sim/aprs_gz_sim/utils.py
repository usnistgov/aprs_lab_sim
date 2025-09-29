#!/usr/bin/env python3

import math

import PyKDL
from geometry_msgs.msg import Pose, Quaternion

import re
import asyncio

from typing import Tuple, Callable, TypeVar, Any

from rclpy.task import Future
from rclpy.client import Client
from rclpy.executors import Executor

def convert_pi_string_to_float(s: str) -> float:
    """Parse simple expressions involving the token 'pi' and return a float.

    Supported patterns are like 'pi', '2pi', 'pi/2', '-3*pi/4'. If the input
    cannot be parsed the function returns 0.0.

    Args:
        s: Input string containing an expression with 'pi'.

    Returns:
        The evaluated float value or 0.0 on parse failure.
    """
    try:
        return float(eval(s, {"__builtins__": None}, {"pi": math.pi}))
    except Exception:
        return 0.0

def quaternion_from_euler(roll, pitch, yaw) -> list[float]:
    """Convert Euler angles (roll, pitch, yaw) into a quaternion list [w,x,y,z].

    Args:
        roll: rotation about X axis in radians.
        pitch: rotation about Y axis in radians.
        yaw: rotation about Z axis in radians.

    Returns:
        A list [w, x, y, z] representing the quaternion.
    """

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0.0] * 4
    q[0] = cy * cp * cr + sy * sp * sr # type: ignore
    q[1] = cy * cp * sr - sy * sp * cr # type: ignore
    q[2] = sy * cp * sr + cy * sp * cr # type: ignore
    q[3] = sy * cp * cr - cy * sp * sr # type: ignore

    return q

def pose_info(xyz: list, rpy: list) -> Pose:
    """Build a ROS Pose message from string/float xyz and rpy lists.

    This function accepts strings containing numeric values or simple pi
    expressions (e.g. 'pi/2') and converts them to floats.

    Args:
        xyz: list of x,y,z values (str or float).
        rpy: list of roll,pitch,yaw values (str or float).

    Returns:
        geometry_msgs.msg.Pose populated with the provided values.
    """

    xyz_floats = []
    rpy_floats = []
    for s in xyz:
        try:
            xyz_floats.append(float(s))
        except ValueError:
            xyz_floats.append(convert_pi_string_to_float(s))
    for s in rpy:
        try:
            rpy_floats.append(float(s))
        except ValueError:
            rpy_floats.append(convert_pi_string_to_float(s))

    pose = Pose()
    pose.position.x = xyz_floats[0]
    pose.position.y = xyz_floats[1]
    pose.position.z = xyz_floats[2]
    q = quaternion_from_euler(*rpy_floats)
    pose.orientation.w = q[0]
    pose.orientation.x = q[1]
    pose.orientation.y = q[2]
    pose.orientation.z = q[3]

    return pose

def multiply_pose(p1: Pose, p2: Pose) -> Pose:
    """Left-multiply two geometry_msgs Pose messages using PyKDL.

    The returned Pose corresponds to the transform p1 * p2.

    Args:
        p1: First pose (applied first).
        p2: Second pose (applied second).

    Returns:
        Pose: Resulting composed pose.
    """

    o1 = p1.orientation
    frame1 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o1.x, o1.y, o1.z, o1.w),
        PyKDL.Vector(p1.position.x, p1.position.y, p1.position.z))

    o2 = p2.orientation
    frame2 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o2.x, o2.y, o2.z, o2.w),
        PyKDL.Vector(p2.position.x, p2.position.y, p2.position.z))

    frame3 = frame1 * frame2

    # return the resulting pose from frame3
    pose = Pose()
    pose.position.x = frame3.p.x()
    pose.position.y = frame3.p.y()
    pose.position.z = frame3.p.z()

    q = frame3.M.GetQuaternion()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


def rpy_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
    """Convert a geometry_msgs Quaternion to roll, pitch, yaw angles.

    Args:
        q: geometry_msgs.msg.Quaternion

    Returns:
        (roll, pitch, yaw) in radians.
    """

    R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
    return R.GetRPY()

def build_pose(x,y,z,q : Quaternion)->Pose:
    """Create a geometry_msgs Pose from position and quaternion values.

    Args:
        x,y,z: position coordinates.
        q: geometry_msgs.msg.Quaternion instance for orientation.

    Returns:
        A populated Pose message.
    """
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = q
    return p

def quaternion_to_msg(q: list[float]):
    """Convert a quaternion list [w,x,y,z] into a geometry_msgs Quaternion.

    Args:
        q: sequence or list with four floats [w,x,y,z].

    Returns:
        geometry_msgs.msg.Quaternion
    """
    q_ret = Quaternion()
    q_ret.w = q[0]
    q_ret.x = q[1]
    q_ret.y = q[2]
    q_ret.z = q[3]

    return q_ret

def rad_to_deg(radians: float) -> float:
    """Convert radians to degrees (float).

    Args:
        radians: Angle in radians.

    Returns:
        Angle in degrees.
    """
    
    return radians * 180/math.pi

T = TypeVar('T')

class ROSAsyncAdapter:
    @staticmethod
    def ros_future_to_asyncio_future(ros_future: Future) -> asyncio.Future:
    
        loop = asyncio.get_running_loop()

        asyncio_future = loop.create_future()

        def _on_ros_future_complete(fut: Future):
            if fut.cancelled():
                asyncio_future.cancel()
            elif fut.done():
                exc = fut.exception()
                if exc is not None:
                    loop.call_soon_threadsafe(asyncio_future.set_exception, exc)
                else:
                    loop.call_soon_threadsafe(asyncio_future.set_result, fut.result())

        ros_future.add_done_callback(_on_ros_future_complete)
        
        return asyncio_future

    @staticmethod
    async def await_service_ready(client: Client, timeout: float = 20.0, poll_interval: float = 0.1):
        start_time = asyncio.get_running_loop().time()
        while not client.service_is_ready():
            if (asyncio.get_running_loop().time() - start_time) > timeout:
                raise TimeoutError(f"Timed out waiting for service {client.srv_name} to become available")
            await asyncio.sleep(poll_interval)

    @staticmethod
    async def await_service_response(client: Client, request, timeout: float = 20.0):
        future = ROSAsyncAdapter.ros_future_to_asyncio_future(client.call_async(request))

        try:
            return await asyncio.wait_for(future, timeout=timeout)
        except asyncio.TimeoutError:
            raise TimeoutError(f'Timed out waiting for response from {client.srv_name} service.')
    
    @staticmethod
    async def spin_executor(executor: Executor, shutdown_event: asyncio.Event, spin_timeout_sec: float = 0.01):
        while not shutdown_event.is_set():
            executor.spin_once(timeout_sec=spin_timeout_sec)
            await asyncio.sleep(0.001)

    @staticmethod
    async def await_condition(condition_fn: Callable[[], bool], timeout: float = 20.0, poll_interval: float = 0.1):
        start_time = asyncio.get_running_loop().time()
        while not condition_fn():
            if (asyncio.get_running_loop().time() - start_time) > timeout:
                raise TimeoutError("Condition not met within timeout.")
            await asyncio.sleep(poll_interval)

    @staticmethod
    async def call_blocking_with_timeout(func: Callable[..., T], *args: Any, timeout: float = 20.0) -> T:
        loop = asyncio.get_running_loop()
        future = loop.run_in_executor(None, func, *args)

        try:
            return await asyncio.wait_for(future, timeout=timeout)
        except asyncio.TimeoutError:
            raise TimeoutError("Blocking function call timed out")