
------------------------------------------ set values ------------------------------------
set_joint_positions=function(joint_hds, positions)
    for i=1,#joint_hds,1 do
        simSetJointPosition(joint_hds[i], positions[i])
    end
end

set_robot_pose=function(robot_hd, pos, ori)
    simSetObjectPosition(robot_hd,-1,pos)
    simSetObjectQuaternion(robot_hd,-1,ori)
end