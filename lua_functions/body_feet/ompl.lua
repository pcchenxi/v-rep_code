package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/?.lua"
package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/body_feet/?.lua"
require("OMPL_Processor")


function OMPL_Processor:sample_callback()
    print('in call back')
    --forbidThreadSwitches(true)
    local state = {}
    -- if type == 1 then 
    local dim = 3
    local path_length = #self.path/dim
    local pose_index = math.random(0,path_length-1)

    local dice = math.random()

    local index =  pose_index * dim
    -- displayInfo('in callback 1 '..pose_index)
    local distance = 0.25
    -- if dice > 0.5 and #sampled_states > 2 then 
    --     -- displayInfo('pose_index2 '..#sampled_states)

    --     local pose_index2 = math.random(#sampled_states)
    --     state = sampled_states[pose_index2]
    -- else
    state[1] = self.path[index + 1]
    state[2] = self.path[index + 2]
    state[3] = self.path[index + 3]
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
    return sampled_state
end