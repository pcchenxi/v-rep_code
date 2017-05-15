package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/common/?.lua"
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

_matching_mode = false
_matching_pair = {}
_matching_index = 1
_init = false
_check_path={}
_applied_path={}
_is_add_to_tree = true

_pose_index = 1

_failed_time = 0

_min_dist = 0.6

type = 1
g_index = 0
sampled_states={}

_dummy_list={}

render_pose = function(state, foot_hds)
    local foot_pos = simSetObjectPosition(foot_hds[1], _callback_robot_hd)

    local robot_pos = {}
    robot_pos[1] = state[1]
    robot_pos[2] = state[2]
    robot_pos[3] = state[3]

    foot_pos[1] = state[4]
    simSetObjectPosition(foot_hds[1], _callback_robot_hd)
    simSetObjectPosition(foot_hds[3], _callback_robot_hd)

    foot_pos[1] = -state[4]
    simSetObjectPosition(foot_hds[2], _callback_robot_hd)
    simSetObjectPosition(foot_hds[4], _callback_robot_hd)
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
