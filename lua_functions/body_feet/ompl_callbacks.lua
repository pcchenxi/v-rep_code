package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/common/?.lua"
require("get_values")
require("set_values")
require("get_handles")

OMPL_Callback_Feet = {}
OMPL_Callback_Feet.__index = OMPL_Callback_Feet

function OMPL_Callback_Feet:new(task_hd, robot_hd, body_hd, foot_hds, robot_dim, joint_dim, collision_hd_1, collision_hd_2)
    local this =
    {
        name = 'OMPL_Callback_Feet',
        task_hd = task_hd,
        robot_hd = robot_hd,
        body_hd = body_hd,

        collision_hd_1 = collision_hd_1,
        collision_hd_2 = collision_hd_2,

        robot_dim = robot_dim,
        joint_dim = joint_dim,

        foot_hds = foot_hds
    }
    setmetatable(this, OMPL_Callback_Feet)
    return this
end

function OMPL_Callback_Feet:render_pose(state)
    -- print('in render pose')
    local foot_pos = simGetObjectPosition(self.foot_hds[1], self.robot_hd)

    -- print(foot_pos[1], foot_pos[2], foot_pos[3])

    local robot_pos = {}
    robot_pos[1] = state[1]
    robot_pos[2] = state[2]
    robot_pos[3] = 0
    simSetObjectPosition(self.robot_hd, -1, robot_pos)  -- robot center

    if self.joint_dim > 0 then 
        foot_pos[1] = state[#state]                                         -- feet
        simSetObjectPosition(self.foot_hds[1], self.robot_hd, foot_pos)
        foot_pos[1] = -state[#state]
        simSetObjectPosition(self.foot_hds[2], self.robot_hd, foot_pos)

        foot_pos[2] = foot_pos[2]-0.3
        simSetObjectPosition(self.foot_hds[3], self.robot_hd, foot_pos)
        foot_pos[1] = state[#state]
        simSetObjectPosition(self.foot_hds[4], self.robot_hd, foot_pos)
    end
    local body_pos = {}
    body_pos[1] = 0
    body_pos[2] = 0
    if self.joint_dim > 1 then
        body_pos[3] = state[3]  
    else
        body_pos[3] = 0.26
    end

    simSetObjectPosition(self.body_hd, self.robot_hd, body_pos)  -- body 

end


function sample_state()
    
    return state
end



quick_motionValidation = function(state_tree, state, valid)
    local check_motion = true

    local diff_x = math.abs(state_tree[1] - state[1])
    local diff_y = math.abs(state_tree[2] - state[2])

    -- if diff_x > 0.3 or diff_y > 0.3 then 
    --     check_motion = false
    -- end

    -- print('qmc '..state_tree[1]..'  '..state_tree[2]..'  '..state[1]..'  '..state[2]..'  '..tostring(check_motion))
    -- _is_add_to_tree = tru
    return check_motion
end

function motionValidation(state_tree, state, valid)
    
    local diff_x = math.abs(state_tree[1] - state[1])
    local diff_y = math.abs(state_tree[2] - state[2])

    local dist = math.sqrt(diff_x*diff_x + diff_y*diff_y)

    if valid then 
        if _matching_index ~= 1 then 
            _matching_mode = false
            _matching_pair = {}
        end
        _is_add_to_tree = true

        if not _matching_mode then
            _failed_time = 0
        end
        -- local hd1 = simGetObjectHandle('state_tree')
        -- local hd2 = simGetObjectHandle('state')

        local pos1={}
        local pos2={}
        local ori={}

        for i=1, 3, 1 do
            pos1[i] = state_tree[i]
            pos2[i] = state[i]
        end
        for i=4, 7, 1 do
            ori[i-3] = state[i]
        end        

        -- simSetObjectPosition(hd1, -1, pos2)
        -- simSetObjectPosition(hd2, -1, pos2)
        -- sleep(1)
        -- simSwitchThread()
        create_dummy(pos2, ori)
        print ('motion validation: '.._matching_index..' '..tostring(valid))


    else        
        if dist < _min_dist then 
            _matching_mode = true
        end  
    end
    -- print ('motion validation: '.._matching_index..' '..tostring(valid))

    return true
end

function create_dummy(pos, ori)
    local hd = simCreateDummy(0.1)
    pos[3] = pos[3]+0.4
    simSetObjectPosition(hd, -1, pos)
    simSetObjectQuaternion(hd, -1, ori)
    _dummy_list[#_dummy_list+1] = hd
end

function OMPL_Callback_Feet:state_validation(state)
    -- print('validation!!!!!!!!!! in call back '.. self.name)
    -- print('state:',state[1], state[2], state[3])

    self:render_pose(state)
    local pass=false
    
    -- check if the foot is on the ground
    local isOnGround = true
    -- foot_pos = get_foottip_positions(self.foot_hds)
    -- for i=1,#foot_pos,1 do
    --     local pos = foot_pos[i]
    --     if pos[3] > 0.03 then
    --         isOnGround = false
    --         break
    --     end
    -- end

    if isOnGround then
        local res=simCheckCollision(self.collision_hd_1, self.collision_hd_2)
        --local res, dist = simCheckDistance(simGetCollectionHandle('robot_body'),simGetCollectionHandle('obstacles'),0.02)
        if res == 0 then
            pass=true
            --_valid_num = _valid_num+1
        end
        print('collision result: ', res)

    end

    return pass
end
