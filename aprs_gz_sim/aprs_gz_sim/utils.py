#!/usr/bin/env python3

import math

import PyKDL
from geometry_msgs.msg import (
    Pose,
    Quaternion
)

import re
import asyncio

from typing import Tuple, Callable, TypeVar, Any

from rclpy.task import Future
from rclpy.client import Client
from rclpy.executors import Executor


def convert_pi_string_to_float(s: str) -> float:
    """Takes a string that contains pi and evaluates the expression. Returns a float
    Returns 0.0 if the expression cannot be evaluated"""
    s=str(s)
    value = 0.0
    negative = False

    if s.isdigit():
        return float(s)

    if s.find('pi') == -1:
        # Return 0 if string does not contain pi
        return value

    if not s.find('-') == -1:
        negative = True
        s = s.replace('-', '')

    split = s.split('pi')
    if not len(split) == 2:
        # Can't evaluate strings with multiple pi's, return 0
        return value

    before, after = split
    if before and after:
        before = before.replace('*', '')
        if before.isdigit():
            value = float(before) * math.pi
        after = after.replace('/', '')
        if after.isdigit():
            value /= float(after)
    elif before:
        before = before.replace('*', '')
        if before.isdigit():
            value = float(before) * math.pi
    elif after:
        after = after.replace('/', '')
        if after.isdigit():
            value = math.pi / float(after)
    else:
        value = math.pi

    if negative:
        return -value
    else:
        return value

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr # type: ignore
    q[1] = cy * cp * sr - sy * sp * cr # type: ignore
    q[2] = sy * cp * sr + cy * sp * cr # type: ignore
    q[3] = sy * cp * cr - cy * sp * sr # type: ignore

    return q

def pose_info(xyz: list, rpy: list) -> Pose:
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
    '''
    Use KDL to multiply two poses together.
    Args:
        p1 (Pose): Pose of the first frame
        p2 (Pose): Pose of the second frame
    Returns:
        Pose: Pose of the resulting frame
    '''

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
    ''' 
    Use KDL to convert a quaternion to euler angles roll, pitch, yaw.
    Args:
        q (Quaternion): quaternion to convert
    Returns:
        Tuple[float, float, float]: roll, pitch, yaw
    '''
    
    R = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w)
    return R.GetRPY()

def build_pose(x,y,z,q : Quaternion)->Pose:
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation = q
    return p


def rad_to_deg_str(radians: float) -> str:
    '''
    Converts radians to degrees in the domain [-PI, PI]
    Args:
        radians (float): value in radians
    Returns:
        str: String representing the value in degrees
    '''
    
    degrees = math.degrees(radians)
    if degrees > 180:
        degrees = degrees - 360
    elif degrees < -180:
        degrees = degrees + 360

    if -1 < degrees < 1:
        degrees = 0 
    
    return f'{degrees:.0f}' + chr(176)

def rad_to_deg(radians: float) -> float:
    '''
    Converts radians to degrees
    Args:
        radians (float): Value in radians
    Returns:
        float: Value in degrees
    '''
    
    return radians * math.pi/180

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


def evaluate_pi_expression(expr) -> float:
    if isinstance(expr, (int, float)):
        return float(expr)

    if isinstance(expr, str):
        try:
            # Remove spaces and prepare expression
            expr = expr.replace(' ', '')
            expr = re.sub(r'(\d)(pi)', r'\1*pi', expr)
            
            # Allow only valid characters/operators
            if not re.fullmatch(r'[\d\.\+\-\*/\(\)pi]+', expr):
                raise ValueError("Invalid characters in expression.")
            
            # Replace 'pi' with its float value
            expr = expr.replace('pi', str(math.pi))

            # Evaluate using eval with empty builtins
            result = eval(expr, {"__builtins__": {}})
            return float(result)

        except Exception as e:
            print(e)
    
    return 0.0