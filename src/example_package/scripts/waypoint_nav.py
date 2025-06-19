#!/usr/bin/env python
import rospy, yaml, actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, GoalStatus
from tf.transformations import quaternion_from_euler

if __name__=='__main__':
    rospy.init_node('waypoint_nav')
    fn = rospy.get_param('~waypoints_file')
    with open(fn) as f:
        pts = yaml.safe_load(f)['waypoints']
    rospy.loginfo("Loaded %d waypoints", len(pts))

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    summary = []
    start_time = rospy.Time.now()
    rospy.loginfo("Starting waypoint navigation at %.2f", start_time.to_sec())

    for idx, p in enumerate(pts):
        # ### Hinzugefügter Code-Snippet START ###
        rospy.loginfo("Next waypoint %d: x=%.2f, y=%.2f, yaw=%.2f",
                      idx, p['x'], p['y'], p['yaw'])
        # ### Hinzugefügter Code-Snippet ENDE ###

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p['x']
        goal.target_pose.pose.position.y = p['y']
        q = quaternion_from_euler(0, 0, p['yaw'])
        (goal.target_pose.pose.orientation.x,
         goal.target_pose.pose.orientation.y,
         goal.target_pose.pose.orientation.z,
         goal.target_pose.pose.orientation.w) = q

        client.send_goal(goal)
        rospy.loginfo("Sent goal %d", idx)

        finished = client.wait_for_result(rospy.Duration(60.0))
        t = rospy.Time.now()
        state = client.get_state()

        if not finished:
            rospy.logwarn("Waypoint %d: no result within timeout", idx)
            summary.append((idx, False, t.to_sec()))
        elif state != GoalStatus.SUCCEEDED:
            rospy.logwarn("Waypoint %d: failed with state %d", idx, state)
            summary.append((idx, False, t.to_sec()))
        else:
            rospy.loginfo("Waypoint %d reached at %.2f", idx, t.to_sec())
            summary.append((idx, True, t.to_sec()))

    end_time = rospy.Time.now()
    rospy.loginfo("Finished all waypoints at %.2f (total %.2fs)",
                  end_time.to_sec(), (end_time - start_time).to_sec())

    rospy.loginfo("=== Waypoint Summary ===")
    for idx, ok, ts in summary:
        status = "OK" if ok else "FAILED"
        rospy.loginfo("  %d: %s at %.2f", idx, status, ts)
