#!/usr/bin/env python3

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import random
import rclpy
from rclpy.node import Node
from etsi_its_cpm_ts_msgs.msg import *
import utils

class Publisher(Node):

    def __init__(self):

        super().__init__("cpm_publisher")
        topic = "/vehicle_7303/CPM"
        self.publisher = self.create_publisher(CollectivePerceptionMessage, topic, 1)
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish)
        self.get_logger().info(f"CPM Publisher node has been started. The message will be published at {1/self.timer_period:.0f} Hz on topic: {topic}")

        # initialize three tracked objects
        # object 0 will be static; objects 1 and 2 will be moving
        # positions/velocities are in meters, conversion to message uses *1e2 (centimeters)
        self.tracked = [
            { "id": 0, "static": True,  "x_m": 10.0, "y_m": 2.0,  "vx_m_s": 0.0,  "vy_m_s": 0.0 },
            { "id": 1, "static": False, "x_m": 12.0, "y_m": 4.0,  "vx_m_s": 1.2,  "vy_m_s": 0.3 },
            { "id": 2, "static": False, "x_m": 8.0,  "y_m": -1.0, "vx_m_s": -0.5, "vy_m_s": 0.8 },
        ]

        # keep last time to compute accurate movement deltas
        self.last_time_ns = self.get_clock().now().nanoseconds
        self.start_time_ns = self.last_time_ns  # start time for deterministic inclusion cycle

        # optionally vary how many objects are published each tick (1..3)
        random.seed()

    def publish(self):

        now = self.get_clock().now()
        now_ns = now.nanoseconds
        dt_s = (now_ns - self.last_time_ns) / 1e9
        # clamp dt to reasonable values to avoid huge jumps after pauses
        if dt_s <= 0 or dt_s > 1.0:
            dt_s = self.timer_period
        self.last_time_ns = now_ns

        # update moving objects positions
        for obj in self.tracked:
            if not obj["static"]:
                obj["x_m"] += obj["vx_m_s"] * dt_s
                obj["y_m"] += obj["vy_m_s"] * dt_s

        # deterministic inclusion cycle:
        # phase length = 5s, sequence (repeat):
        # 0..5s   -> [0]
        # 5..10s  -> [0,1]
        # 10..15s -> [0,1,2]
        # 15..20s -> [0,2]
        # 20..25s -> [] (empty phase)
        elapsed_s = (now_ns - self.start_time_ns) / 1e9
        phase = int(elapsed_s // 5) % 5 # 5 seconds each phase with 5 phases total

        if phase == 0:
            ids = [0]
        elif phase == 1:
            ids = [0, 1]
        elif phase == 2:
            ids = [0, 1, 2]
        elif phase == 3:
            ids = [0, 2]
        else:  # phase == 4 -> [] (empty phase)
            ids = []

        msg = CollectivePerceptionMessage()

        msg.header.protocol_version.value = 2
        msg.header.message_id.value = msg.header.message_id.CPM
        msg.header.station_id.value = 7303

        msg.payload.management_container.reference_time.value = utils.get_t_its(now_ns)
        msg.payload.management_container.reference_position.latitude.value = int(40.628507 * 1e7)
        msg.payload.management_container.reference_position.longitude.value = int(-8.733892 * 1e7)
        # TODO: remaining fields of management container

        # add OriginatingVehicleContainer
        cpm_container = WrappedCpmContainer()
        cpm_container.container_id.value = cpm_container.CHOICE_CONTAINER_DATA_ORIGINATING_VEHICLE_CONTAINER

        originating_vehicle_container = OriginatingVehicleContainer()
        originating_vehicle_container.orientation_angle.value.value = originating_vehicle_container.orientation_angle.value.UNAVAILABLE
        originating_vehicle_container.orientation_angle.confidence.value = originating_vehicle_container.orientation_angle.confidence.UNAVAILABLE

        cpm_container.container_data_originating_vehicle_container = originating_vehicle_container
        msg.payload.cpm_containers.value.array.append(cpm_container)

        # add PerceivedObjectContainer
        cpm_container = WrappedCpmContainer()
        cpm_container.container_id.value = cpm_container.CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER

        perceived_object_container = PerceivedObjectContainer()
        perceived_object_container.number_of_perceived_objects.value = len(ids)

        for i, obj_id in enumerate(ids):
            # find tracked object
            to = next(t for t in self.tracked if t["id"] == obj_id)

            perceived_object = PerceivedObject()
            perceived_object.object_id_is_present = True
            perceived_object.object_id.value = int(to["id"])
            perceived_object.measurement_delta_time.value = 10

            # convert meters to message units (centimeters)
            x_val = int(to["x_m"] * 1e2)
            y_val = int(to["y_m"] * 1e2)

            perceived_object.position.x_coordinate.value.value = x_val
            perceived_object.position.x_coordinate.confidence.value = perceived_object.position.x_coordinate.confidence.UNAVAILABLE
            perceived_object.position.y_coordinate.value.value = y_val
            perceived_object.position.y_coordinate.confidence.value = perceived_object.position.y_coordinate.confidence.UNAVAILABLE

            perceived_object.object_dimension_x_is_present = True
            perceived_object.object_dimension_x.value.value = int(3.5 * 1e1)
            perceived_object.object_dimension_x.confidence.value = perceived_object.object_dimension_x.confidence.UNAVAILABLE
            perceived_object.object_dimension_y_is_present = True
            perceived_object.object_dimension_y.value.value = int(1.8 * 1e1)
            perceived_object.object_dimension_y.confidence.value = perceived_object.object_dimension_y.confidence.UNAVAILABLE
            perceived_object.object_dimension_z_is_present = True
            perceived_object.object_dimension_z.value.value = int(1.6 * 1e1)
            perceived_object.object_dimension_z.confidence.value = perceived_object.object_dimension_z.confidence.UNAVAILABLE

            perceived_object_container.perceived_objects.array.append(perceived_object)

        cpm_container.container_data_perceived_object_container = perceived_object_container
        msg.payload.cpm_containers.value.array.append(cpm_container)

        self.get_logger().info(f"Publishing CPM with {len(ids)} perceived objects: ids={ids} (phase={phase}, elapsed_s={elapsed_s:.1f})")
        self.publisher.publish(msg)

if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
