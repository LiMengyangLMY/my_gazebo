#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import DeleteModel, SpawnModel


BALL_SDF_TEMPLATE = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Yellow</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def as_pose(ball_spec):
    pose = Pose()
    pose.position.x = float(ball_spec.get("x", 0.0))
    pose.position.y = float(ball_spec.get("y", 0.0))
    pose.position.z = float(ball_spec.get("z", 0.033))
    pose.orientation.w = 1.0
    return pose


def main():
    rospy.init_node("ball_scene_spawner")

    ball_radius = rospy.get_param("~ball_radius", 0.033)
    balls = rospy.get_param("~balls", [])
    scene_name = rospy.get_param("~scene_name", "unnamed_scene")

    if not balls:
        rospy.logwarn("ball_scene_spawner: no balls configured for scene '%s'.", scene_name)
        return

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/delete_model")

    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    for ball in balls:
        ball_name = str(ball["name"])
        try:
            response = delete_model(ball_name)
            if not response.success:
                status = getattr(response, "status_message", "")
                if "does not exist" not in status.lower():
                    rospy.logwarn("DeleteModel skipped for %s: %s", ball_name, status)
        except rospy.ServiceException:
            pass

        model_xml = BALL_SDF_TEMPLATE.format(name=ball_name, radius=ball_radius)
        pose = as_pose(ball)
        try:
            spawn_model(ball_name, model_xml, "", pose, "world")
            rospy.loginfo("Spawned %s at (%.3f, %.3f, %.3f)",
                          ball_name, pose.position.x, pose.position.y, pose.position.z)
        except rospy.ServiceException as exc:
            rospy.logerr("Failed to spawn %s: %s", ball_name, exc)

    rospy.loginfo("ball_scene_spawner finished scene '%s' with %d balls.", scene_name, len(balls))


if __name__ == "__main__":
    main()
