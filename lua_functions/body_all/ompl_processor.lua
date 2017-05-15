package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/common/?.lua"
package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/body_all/?.lua"

require("common_functions")
require("ompl_callbacks")


OMPL_Processor = {}
OMPL_Processor.__index = OMPL_Processor
function OMPL_Processor:new(robot_dim, joint_dim)
    local this =
    {
        name = 'OMPL_Processor',
        task_hd = nil,

        robot_dim = robot_dim,
        joint_dim = joint_dim,

        collision_name_1 = 'robot_body',
        collision_name_2 = 'obstacles',

        use_validation_callback = false,
        use_sampler_callback = false,
        use_collision_check = true,

        validate_callback_nume = 'stateValidation',
        sampler_callback_name = 'state_sampler',
        sampler_near_callback_name = 'state_sampler_near',

        ompl_algorithm = sim_ompl_algorithm_RRTConnect,

        path = {},

        state_dim = 0,

        sample_num     = 0,
        valid_num      = 0
    }
    setmetatable(this, OMPL_Processor)
    return this
end



function OMPL_Processor:stateValidation(state)
    -- displayInfo('in stateValidation ')

    -- Read the current state:
    --local res, current_state = simExtOMPL_readState(_task_hd)
    --_sample_num = _sample_num+1

    local r = simExtOMPL_writeState(_callback_task_hd, state)
    local pass=false
    
    -- check if the foot is on the ground
    local isOnGround = true
    foot_pos = get_foottip_positions(_callback_foot_hds)
    for i=1,#foot_pos,1 do
        local pos = foot_pos[i]
        if pos[3] > 0.03 then
            isOnGround = false
            break
        end
    end

    if isOnGround then
        local res=simCheckCollision(_callback_collision_hd_1,_callback_collision_hd_2)
        --local res, dist = simCheckDistance(simGetCollectionHandle('robot_body'),simGetCollectionHandle('obstacles'),0.02)
        if res == 0 then
            pass=true
            --_valid_num = _valid_num+1
        end
    end
    --res = simExtOMPL_writeState(_task_hd, current_state)
    -- sleep(2)
    -- simSwitchThread()
    --displayInfo('callback: '..test)

    -- Return whether the tested state is valid or not:
    return pass
end


function OMPL_Processor:init_task(start_name, target_name, task_id)
    if start_name ~= nil then
        print(start_name)
    end
    local robot_hd       =simGetObjectHandle(start_name)
    local target_hd      =simGetObjectHandle(target_name)
    local joint_hds      =get_joint_hds()

    self.task_hd = simExtOMPL_createTask(task_id)
    -- simExtOMPL_setVerboseLevel(task_hd, 3)
    simExtOMPL_setAlgorithm(self.task_hd,self.ompl_algorithm)

    ------ callbacks ---------------
    --simExtOMPL_setGoalCallback(t, 'goalSatisfied')
    print('valication: '..tostring(self.use_validation_callback)..' sampler: '..tostring(self.use_validation_callback))
    if self.use_validation_callback then
        _callback_collision_hd_1 = simGetCollectionHandle(self.collision_name_1)
        _callback_collision_hd_2 = simGetCollectionHandle(self.collision_name_2)
        _callback_foot_hds = get_foot_tip_hds()
        _callback_task_hd = self.task_hd
        local res = simExtOMPL_setStateValidationCallback(self.task_hd, 'stateValidation')
    end

    if self.use_sampler_callback then 
        _callback_joint_hds = joint_hds
        _callback_robot_hd = robot_hd
        _callback_path = self.path
        _callback_state_dim = self.state_dim
        _callback_init_config = get_joint_positions(joint_hds, self.joint_dim)

        local r = simExtOMPL_setValidStateSamplerCallback(self.task_hd, 'sample_callback', 'sampleNear_callback')        
        -- displayInfo('use callback '..#_path)

    end

    -- start pose --
    local startpose=get_robot_pose(robot_hd, joint_hds, self.robot_dim, self.joint_dim)
    -- print_table(startpose)
    -- target pose --
    local goalpose=get_robot_pose(target_hd, joint_hds, self.robot_dim, self.joint_dim)
    self.state_dim = #startpose
    local satat_spaces = self:init_statespace(robot_hd, joint_hds, startpose, goalpose) -- for sample state
    simExtOMPL_setStateSpace(self.task_hd, satat_spaces)

    if self.use_collision_check then
        simExtOMPL_setCollisionPairs(self.task_hd, {simGetCollectionHandle(self.collision_name_1), simGetCollectionHandle(self.collision_name_2)}) -- collision 
    end

    simExtOMPL_setStartState(self.task_hd, startpose)    
    simExtOMPL_setGoalState(self.task_hd, goalpose)

    local txt='state_space dim ' ..#startpose
    -- displayInfo(txt)
    -- simExtOMPL_printTaskInfo(task_hd)
    return self.task_hd, self.state_dim
end

function OMPL_Processor:compute_path(max_time)
    --forbidThreadSwitches(true)
    local r = nil
    r, self.path=simExtOMPL_compute(self.task_hd, max_time, -1, 0)
    --forbidThreadSwitches(false)

    local path_step = #self.path/self.state_dim
    print(self.name .. 'found path with step ' ..path_step )
    --local txt='finish compute' ..path_step
    --displayInfo(txt)
    
    return self.path
end

function OMPL_Processor:init_statespace(robot_hd, joint_hds, start_pose, goal_pose)
    local state_spaces={}
    local min_v = 0
    local max_v = 0
    local weight = 0
    
    local min_y = start_pose[2]-0.5
    local max_y = goal_pose[2]+0.5

    local min_x = -0.5
    local max_x = 0.5

    local min_z = start_pose[3]
    local max_z = start_pose[3] + 0.1

    if self.robot_dim == 1 then
        min_range={min_x,min_y}
        max_range={max_x,max_y}
        local weight_move = 3
        state_spaces[1]=simExtOMPL_createStateSpace('base_space',sim_ompl_statespacetype_position2d,robot_hd,min_range,max_range,weight_move)               -- base space
    elseif self.robot_dim == 2 then
        min_range={min_x,min_y}
        max_range={max_x,max_y}
        local weight_move = 3
        state_spaces[1]=simExtOMPL_createStateSpace('base_space',sim_ompl_statespacetype_pose2d,robot_hd,min_range,max_range,weight_move)               -- base space
    elseif self.robot_dim == 3 then
        min_range={min_x,min_y, min_z}
        max_range={max_x,max_y, max_z}
        local weight_move = 3
        state_spaces[1]=simExtOMPL_createStateSpace('base_space',sim_ompl_statespacetype_position3d,robot_hd,min_range,max_range,weight_move)               -- base space
    elseif self.robot_dim == 6 then
        min_range={min_x,min_y, min_z}
        max_range={max_x,max_y, max_z}
        local weight_move = 3
        state_spaces[1]=simExtOMPL_createStateSpace('base_space',sim_ompl_statespacetype_pose3d,robot_hd,min_range,max_range,weight_move)               -- base space        
    end

    for i=1,self.joint_dim,1 do
        if i < 5 then
            min_v = {-90*math.pi/180}    --{-170*math.pi/180}
            max_v = {90*math.pi/180}    --{170*math.pi/180}
            weight = 0
        elseif i < 9 then
            min_v = {-90*math.pi/180}    --{-170*math.pi/180}
            max_v = {90*math.pi/180}    --{170*math.pi/180}
            weight = 0
        else 
            min_v = {-90*math.pi/180}    --{-170*math.pi/180}
            max_v = {90*math.pi/180}    --{170*math.pi/180}
            weight = 0
        end
        state_spaces[i+1]=simExtOMPL_createStateSpace('joint'..i,sim_ompl_statespacetype_joint_position,joint_hds[i],min_v,max_v,weight,robot_hd)
    end

    -- print("init state space "..#state_spaces)
    return state_spaces
end

function OMPL_Processor:apply_path()
    local state = {}
    for i=1, #self.path-self.state_dim, self.state_dim do
        for j=1,self.state_dim,1 do
            state[j]=self.path[i+j-1]
        end
        res = simExtOMPL_writeState(self.task_hd, state) -- 12 joints, yaw,x,y

        sleep(0.02)
        simSwitchThread()
    end
end

function OMPL_Processor:set_param(robot_dim, joint_dim, use_collision_check, use_validation_callback, use_sampler_callback)
    self.robot_dim = robot_dim
    self.joint_dim = joint_dim
    self.use_collision_check = use_collision_check or false
    self.use_validation_callback = use_validation_callback or false 
    self.use_sampler_callback = use_sampler_callback or false

end

function OMPL_Processor:set_collision_pire(collision_hd1, collision_hd2)
    self.collision_hd_1 = collision_hd1
    self.collision_hd_2 = collision_hd2

end

function OMPL_Processor:set_callback_functions(function_validation, function_sampler, function_sampler_near)
    self.validate_callback_nume = function_validation
    self.use_sampler_callback = function_sampler
    self.sampler_near_callback_name = function_sampler_near
end

function OMPL_Processor:set_algorithm(algorithm_name)
    self.ompl_algorithm = algorithm_name
end