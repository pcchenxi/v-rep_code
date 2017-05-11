package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/?.lua"
require("get_values")
require("set_values")
require("get_handles")
require("robot_pose_generator")

local torch = require 'torch'

_callback_task_hd=nil
_callback_foot_hds={}
_callback_joint_hds={}
_callback_robot_hd=nil

_callback_init_config={}

_callback_collision_hd_1 = nil
_callback_collision_hd_2 = nil

_callback_state_dim = 3

_callback_path = nil
_callback_path_index = 1
_callback_start = nil
_callback_goal = nil
_pose_generator = nil

test = 0

_init = false
_check_path={}
_is_add_to_tree = true
_pre_index = 0
_pose_index = 1

type = 1
g_index = 0
sampled_states={}

sample_from_collection = function()
    -- forbidThreadSwitches(true)
    -- if not _is_add_to_tree then
    --     _pose_index = _pose_index -1
    -- end

    local state = {}
    local path_state = {}
    -- if type == 1 then 
    local dim = 3
    local path_length = #_callback_path/dim
    local pose_index = _pose_index%path_length
    -- local pose_index = math.random(0, path_length-1)

    local index =  pose_index * dim

    path_state[1] = _callback_path[index + 1]
    path_state[2] = _callback_path[index + 2]
    path_state[3] = _callback_path[index + 3]

    local pose_collection_size = #_pose_generator.pose_list
    local candidate_pose = get_candidate_states(path_state[3], _pose_generator.pose_list)
    -- local pose_collection_index = math.random(#candidate_pose)
    
    for i=1, #candidate_pose, 1 do
        local distance = 0.15
        state = get_state (i, path_state, candidate_pose, distance)    
        -- forbidThreadSwitches(true)
        local r = simExtOMPL_writeState(_callback_task_hd, state)
        local res=simCheckCollision(_callback_collision_hd_1,_callback_collision_hd_2)

        if res == 0 then
            print('found sample: '..i..' '..pose_index..' '..state[1], state[2], path_state[2])
            _pose_index = _pose_index+1
            -- local r = simExtOMPL_writeState(_callback_task_hd, _callback_start)
            -- forbidThreadSwitches(false)
            return state
        end
    end
    -- print('sample: '..state[1], state[2])
    local distance = 0.15
    local pose_collection_index = math.random(#candidate_pose)
    state = get_state (pose_collection_index, path_state, candidate_pose, distance)    

    _pose_index = _pose_index+1
    -- forbidThreadSwitches(false)
    return state
end

get_state = function(index, path_state, pose_list, distance)
    -- local distance = 0.05
    local state = pose_list[index]    

    state[1] = torch.normal(path_state[1], distance)
    state[2] = torch.normal(path_state[2], distance)   
        -- state[3] = torch.normal(state[3]+0.0, 0.06)    

    return state
end

get_candidate_states = function(z, pose_list)
    local candidate_pose={}
    for i=1, #pose_list, 1 do
        local diff_z = math.abs(pose_list[i][3] - z)
        if diff_z < 0.05 then
            candidate_pose[#candidate_pose+1] = pose_list[i]
        end
    end
    return candidate_pose
end

sample_callback = function()
    --forbidThreadSwitches(true)
    local state = {}
    -- if type == 1 then 
    local dim = 3
    local path_length = #_callback_path/dim
    -- local pose_index = math.random(0,path_length-1)
    local pose_index = _pose_index%path_length
    local dice = math.random()

    local index =  pose_index * dim
    -- displayInfo('in callback 1 '..pose_index)
    local distance = 0.05
    -- if dice > 0.5 and #sampled_states > 2 then 
    --     -- displayInfo('pose_index2 '..#sampled_states)

    --     local pose_index2 = math.random(#sampled_states)
    --     state = sampled_states[pose_index2]
    -- else
    state[1] = _callback_path[index + 1]
    state[2] = _callback_path[index + 2]
    state[3] = _callback_path[index + 3]
    state[4] = 0
    state[5] = 0
    state[6] = 0
    state[7] = 1
    for i = 1, #_callback_init_config, 1 do 
        state[7+i] = _callback_init_config[i]
    end

    -- distance = 0.3
    -- end

    local found_pose, sampled_state = sample_state(_callback_robot_hd, _callback_joint_hds, state, distance)
    -- if sampled_states == nil then
    --     sampled_states[1] = sampled_state
    --     sampled_states[2] = sampled_state
    --     sampled_states[3] = sampled_state
    -- end
    -- if found_pose== 1 then 
    --     sampled_states[#sampled_states + 1] = sampled_state
    -- end
    -- displayInfo('sampled_state '..#sampled_states)

    --forbidThreadSwitches(false)

    -- sleep(3)
    --simSwitchThread()

    _pose_index = _pose_index+1
    return sampled_state
end

sampleNear_callback = function(state, distance)
    -- test = 1
    displayInfo('in sample near!!!!!!!!!!!!!!!! ')

    local found_pose, sampled_state = sample_state(_callback_robot_hd, _callback_joint_hds, state, distance)
        
    return state
end

get_nearest_index = function(list, index)
    for i = index, #list-1, 1 do
        if list[i] == 0 then
            print(i..' '..index)
            return i
        end
    end

    for i = index, 1, -1 do
        if list[i] == 0 then
            print(i..' '..index)
            return i
        end
    end
    return index
end

sample_state=function(robot_hd, joint_hds, start_state, distance)
    local pan_hds = get_leg_pan_hds()
    -- local ikgroup_hds = get_ik_handles()

    local sample_pose = {}
    sample_pose[1] = torch.normal(start_state[1], distance)
    sample_pose[2] = torch.normal(start_state[2], distance)    
    sample_pose[3] = start_state[3] --torch.normal(start_state[3]+0.0, 0.06)    

    local sample_ori = {}
    sample_ori[1] = start_state[4] --torch.normal(start_state[4], 0.04)
    sample_ori[2] = start_state[5] --torch.normal(start_state[5], 0.04)    
    sample_ori[3] = start_state[6] --torch.normal(start_state[6], 0.05)
    sample_ori[4] = start_state[7]

    set_robot_body(robot_hd, sample_pose, sample_ori)

    marker_knee = simGetObjectHandle('temp_knee')
    marker_foot = simGetObjectHandle('temp_foot')

    local ang1s={}
    local ang2s={}
    local leg_hds={}
    local found_pose = 1
    local res = 0
    for i=1,4,1 do
        leg_hds[i]=get_leg_hds(i)
        res, ang1s[i], ang2s[i] = sample_leg_pos(i, pan_hds[i], leg_hds[i])
        found_pose = found_pose*res
    end
    
    if found_pose == 0 then
        -- sampled_state = start_state
        -- set_robot_body(robot_hd, startpos, startorient)
        -- set_joint_positions(joint_hds, startconfigs)
        local r = simExtOMPL_writeState(_callback_task_hd, start_state)
    end

    local res, sampled_state = simExtOMPL_readState(_callback_task_hd)
    --displayInfo('in sample_state '..#sampled_state)

    -- sleep(2)
    -- simSwitchThread()
    return found_pose, sampled_state
end

sample_leg_pos=function(index, pan_hd, leg_hds)
    local pan_pos = simGetJointPosition(pan_hd, -1)
    -- local sample_pan = torch.normal(pan_pos, 1.575)
    -- local sample_r = torch.normal(0.0, 0.05)

    local sample_r = math.random(-0.08, 0.08)
    local sample_pan = math.random(pan_pos-1.575, pan_pos+1.575)

    simSetJointPosition(pan_hd, sample_pan)

    local pos = simGetObjectPosition(leg_hds[1], -1)
    local knee_pos = simGetObjectPosition(leg_hds[2], leg_hds[1])

    local r0 = 0.07238 --math.sqrt(knee_pos[1]^2 + knee_pos[2]^2)
    local r1 = 0.10545 --math.sqrt(tip_pos[1]^2 + tip_pos[2]^2)

    local tip_pos={}
    tip_pos[1] = sample_r
    tip_pos[2] = pos[3] - 0.0475
    tip_pos[3] = 0
    simSetObjectPosition(marker_foot, leg_hds[1], tip_pos)

    local knee_x, knee_y = get_intersection_point(0, 0, tip_pos[1], tip_pos[2], r0, r1)
    if knee_x == -1 then
        return 0, 0, 0
    end
    local knee_pos={}
    knee_pos[1] = knee_x
    knee_pos[2] = knee_y
    knee_pos[3] = 0
    simSetObjectPosition(marker_knee, leg_hds[1], knee_pos)

    local tip_x_fromknee = tip_pos[1]-knee_x
    local tip_y_fromknee = tip_pos[2]-knee_y

    --displayInfo('knee pos: '..tip_x_fromknee..' '..tip_y_fromknee)


    local angle_thigh = math.atan(knee_y/knee_x)
    local angle_knee = math.atan(tip_y_fromknee/tip_x_fromknee)
    if angle_knee<0 then 
        angle_knee = angle_knee + math.pi 
    end

    simSetJointPosition(leg_hds[1], angle_thigh)
    simSetJointPosition(leg_hds[2], angle_knee-angle_thigh)

    local pose_real = simGetObjectPosition(leg_hds[3], leg_hds[1])

    local error_x = math.abs(tip_pos[1]-pose_real[1])
    local error_y = math.abs(tip_pos[2]-pose_real[2])
    local error_z = math.abs(tip_pos[3]-pose_real[3])

    local good_pos = 1
    if error_x > 0.01 or error_y > 0.01 or error_z > 0.01 then
        good_pos = 0
    end
    --displayInfo('good pos: '..good_pos)

    -- simSwitchThread()


    -- local target_matrix=simGetObjectMatrix(marker_foot,-1)
    -- local jointPositions = check_ik(target_matrix, tip_hd, ik_group_hd, leg_hds)

    -- if jointPositions == nil then
    --     --displayInfo('not found: ')
    -- else
    --     set_joint_positions(leg_hds, jointPositions)
    -- end
    return good_pos, angle_thigh, angle_knee-angle_thigh
end
 
stateValidation=function(state)
    -- displayInfo('in stateValidation ')

    -- Read the current state:
    --local res, current_state = simExtOMPL_readState(_task_hd)
    --_sample_num = _sample_num+1
    local r = simExtOMPL_writeState(_callback_task_hd, state)
    local pass=false
    
    -- check if the foot is on the ground
    local isOnGround = true
    -- foot_pos = get_foottip_positions(_callback_foot_hds)
    -- for i=1,#foot_pos,1 do
    --     local pos = foot_pos[i]
    --     if pos[3] > 0.03 then
    --         isOnGround = false
    --         break
    --     end
    -- end

    if isOnGround then
        local res=simCheckCollision(_callback_collision_hd_1,_callback_collision_hd_2)
        --local res, dist = simCheckDistance(simGetCollectionHandle('robot_body'),simGetCollectionHandle('obstacles'),0.02)
        if res == 0 then
            pass=true
            --_valid_num = _valid_num+1
        end
    end
    --res = simExtOMPL_writeState(_task_hd, current_state)
    -- sleep(1)
    -- simSwitchThread()
    --displayInfo('callback: '..test)

    -- Return whether the tested state is valid or not:
    -- print('stateValidation: '..state[1], state[2], tostring(pass))
    -- _is_add_to_tree = pass
    return pass
end


goalSatisfied = function(state)
    local satisfied=0
    local dist=0
    local diff={}
    for i=1, #_callback_goal, 1 do
        diff[i]=math.abs(state[i]-_callback_goal[i])
    end

    local min_dist = 0.1
    if diff[1] < min_dist and diff[2] < min_dist then
    -- if state[1]-_callback_goal[1] < 0.05 and state[2]-_callback_goal[2] < 0.1 then
        satisfied=1
    end

    dist=diff[3]+diff[4]+diff[5]+diff[6]+diff[7]
    return satisfied, dist
end


check_ik=function(target_matrix, tip_hd, ik_group_hd, jh)
    simSetObjectMatrix(tip_hd,-1,target_matrix)
    local jointPositions = simGetConfigForTipPose(ik_group_hd,jh,0.05,10)
    return jointPositions
end


get_intersection_point=function(x0, y0, x1, y1, r0, r1)
    local d=math.sqrt((x1-x0)^2 + (y1-y0)^2)
    if d>(r0+r1) then
        return -1, -1
    end
    
    local a=(r0^2-r1^2+d^2)/(2*d)
    local h=math.sqrt(r0^2-a^2)
    local x2=x0+a*(x1-x0)/d   
    local y2=y0+a*(y1-y0)/d   
    local x3=x2+h*(y1-y0)/d       -- also x3=x2-h*(y1-y0)/d
    local y3=y2-h*(x1-x0)/d       -- also y3=y2+h*(x1-x0)/d

    return x3, y3
end