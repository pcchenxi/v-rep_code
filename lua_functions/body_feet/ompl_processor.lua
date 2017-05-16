package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/common/?.lua"
package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/body_feet/?.lua"

require("common_functions")
require("ompl_callbacks")


feet_callback_hd = nil

OMPL_Processor = {}
OMPL_Processor.__index = OMPL_Processor
function OMPL_Processor:new(robot_dim, joint_dim)
    local this =
    {
        name = 'OMPL_Processor',
        task_hd = nil,

        robot_dim = robot_dim,
        joint_dim = joint_dim,

        collision_name_1 = 'robot_feet',
        collision_name_2 = 'obstacles',

        use_validation_callback = true,
        use_sampler_callback = false,
        use_collision_check = true,

        validate_callback_nume = 'stateValidation',
        sampler_callback_name = 'state_sampler',
        sampler_near_callback_name = 'state_sampler_near',

        ompl_algorithm = sim_ompl_algorithm_RRTConnect,

        foot_hds = {},
        body_hd = nil,
        robot_hd = nil,

        path = {},

        state_dim = 0,

        sample_num     = 0,
        valid_num      = 0
    }
    setmetatable(this, OMPL_Processor)
    return this
end


function stateValidation(state)

    local pass = feet_callback_hd:state_validation(state)
    return pass
end

function OMPL_Processor:init_task(start_name, target_name, task_id)
    self.foot_hds[1] = simGetObjectHandle('foot')
    self.foot_hds[2] = simGetObjectHandle('foot#0')
    self.foot_hds[3] = simGetObjectHandle('foot#1')
    self.foot_hds[4] = simGetObjectHandle('foot#2')
    self.body_hd     = simGetObjectHandle('robot_body')

    local robot_hd       =simGetObjectHandle(start_name)
    local target_hd      =simGetObjectHandle(target_name)
    local joint_hds      =get_joint_hds()
    self.robot_hd        = robot_hd

    local collision_hd_1 =simGetCollectionHandle(self.collision_name_1)
    local collision_hd_2 =simGetCollectionHandle(self.collision_name_2)

    self.task_hd = simExtOMPL_createTask(task_id)
    feet_callback_hd = OMPL_Callback_Feet:new(self.task_hd, robot_hd, self.body_hd, self.foot_hds, self.robot_dim, self.joint_dim, collision_hd_1, collision_hd_2)
    
    -- a = self:stateValidation()
    -- simExtOMPL_setVerboseLevel(self.task_hd, 3)

    simExtOMPL_setAlgorithm(self.task_hd,self.ompl_algorithm)

    ------ callbacks ---------------
    --simExtOMPL_setGoalCallback(t, 'goalSatisfied')
    -- print('validation: '..tostring(self.use_validation_callback)..' sampler: '..tostring(self.use_sampler_callback))
    if self.use_validation_callback then
        local res = simExtOMPL_setStateValidationCallback(self.task_hd, 'stateValidation')
    end

    -- simExtOMPL_printTaskInfo(self.task_hd)

    -- if self.use_sampler_callback then 
    --     _callback_joint_hds = joint_hds
    --     _callback_robot_hd = robot_hd
    --     _callback_path = self.path
    --     _callback_state_dim = self.state_dim
    --     _callback_init_config = get_joint_positions(joint_hds, self.joint_dim)

    --     local r = simExtOMPL_setValidStateSamplerCallback(self.task_hd, 'sample_callback', 'sampleNear_callback')        
    --     -- displayInfo('use callback '..#_path)

    -- end

    -- start pose --
    local startpose= self:get_pose(robot_hd)
    -- print_table(startpose)
    -- target pose --
    local goalpose= self:get_pose(target_hd) --get_robot_pose(target_hd, joint_hds, self.robot_dim, self.joint_dim)
    -- goalpose[3] = startpose[3]
    -- goalpose[4] = startpose[4]

    print(startpose[2], goalpose[2])

    self.state_dim = #startpose

    local satat_spaces = self:init_statespace(robot_hd, joint_hds, startpose, goalpose) -- for sample state
    simExtOMPL_setStateSpace(self.task_hd, satat_spaces)

    if self.use_collision_check then
        simExtOMPL_setCollisionPairs(self.task_hd, {collision_hd_1, collision_hd_2}) -- collision 
    end

    simExtOMPL_setStartState(self.task_hd, startpose)    
    simExtOMPL_setGoalState(self.task_hd, goalpose)

    local txt='state_space dim ' ..#startpose
    -- displayInfo(txt)
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

function check_range(p1, p2)
    local temp
    if p1 > p2 then
        temp = p1
        p1 = p2
        p2 = temp
    end
    return p1, p2 
end

function OMPL_Processor:init_statespace(robot_hd, joint_hds, start_pose, goal_pose)
    local state_spaces={}
    local min_v = 0
    local max_v = 0
    local weight = 0
    
    local dis_x = math.abs(start_pose[1] - goal_pose[1]) + 0.5
    local dis_y = math.abs(start_pose[2] - goal_pose[2]) + 0.5

    local mid_x = (start_pose[1] + goal_pose[1])/2
    local mid_y = (start_pose[2] + goal_pose[2])/2

    local min_y = mid_y - dis_y/2
    local max_y = mid_y + dis_y/2

    local min_x = mid_x - dis_x/2
    local max_x = mid_x + dis_x/2

    local min_z = start_pose[3] 
    local max_z = goal_pose[3]

    print(min_x,max_x, min_y, max_y, mid_x, mid_y)

    if self.robot_dim == 1 then
        min_range={min_x,min_y}
        max_range={max_x,max_y}
        local weight_move = 1
        state_spaces[1]=simExtOMPL_createStateSpace('base_space',sim_ompl_statespacetype_position2d,robot_hd,min_range,max_range,weight_move)               -- base space
    elseif self.robot_dim == 2 then
        min_range={min_x,min_y}
        max_range={max_x,max_y}
        local weight_move = 1
        state_spaces[1]=simExtOMPL_createStateSpace('base_space',sim_ompl_statespacetype_pose2d,robot_hd,min_range,max_range,weight_move)               -- base space
    elseif self.robot_dim == 3 then
        min_range={min_x,min_y, min_z}
        max_range={max_x,max_y, max_z}
        local weight_move = 1
        state_spaces[1]=simExtOMPL_createStateSpace('base_space',sim_ompl_statespacetype_position3d,robot_hd,min_range,max_range,weight_move)               -- base space
    elseif self.robot_dim == 6 then
        min_range={min_x,min_y, min_z}
        max_range={max_x,max_y, max_z}
        local weight_move = 1
        state_spaces[1]=simExtOMPL_createStateSpace('base_space',sim_ompl_statespacetype_pose3d,robot_hd,min_range,max_range,weight_move)               -- base space        
    end

    if self.joint_dim > 0 then
        state_spaces[#state_spaces+1]=simExtOMPL_createStateSpace('feet_space',sim_ompl_statespacetype_joint_position, self.foot_hds[1], {0.02}, {0.38}, 5)               -- base space        
    end
    if self.joint_dim > 1 then
        state_spaces[#state_spaces+1]=simExtOMPL_createStateSpace('body_space',sim_ompl_statespacetype_joint_position, self.body_hd, {0.05}, {0.28}, 4)               -- base space        
    end

    print("init state space "..#state_spaces, self.robot_dim)
    return state_spaces
end

function OMPL_Processor:apply_path()
    local state = {}
    for i=1, #self.path-self.state_dim, self.state_dim do
        for j=1,self.state_dim,1 do
            state[j]=self.path[i+j-1]
        end
        feet_callback_hd:render_pose(state, self.foot_hds)
        -- res = simExtOMPL_writeState(self.task_hd, state) -- 12 joints, yaw,x,y

        sleep(0.1)
        simSwitchThread()
    end
end

function OMPL_Processor:get_pose(hd)
    local pos = simGetObjectPosition(hd, -1)
    local ori   = simGetObjectQuaternion(hd,-1)
    local foot_pos = simGetObjectPosition(self.foot_hds[1], self.robot_hd)
    local body_pos = simGetObjectPosition(self.body_hd, self.robot_hd)
    print('ori 4',ori[4])

    local state={}
    state[1] = pos[1]
    state[2] = pos[2]

    if self.robot_dim == 2 then
        state[3] = ori[3]
    end

    if self.joint_dim > 0 then 
        state[#state+1] = math.abs(foot_pos[1])
    end
    if self.joint_dim > 1 then
        state[#state+1] = body_pos[3]
    end

    return state
end

function OMPL_Processor:set_param(robot_dim, joint_dim, use_collision_check, use_validation_callback, use_sampler_callback)
    self.robot_dim = robot_dim
    self.joint_dim = joint_dim
    self.use_collision_check = use_collision_check or false
    self.use_validation_callback = use_validation_callback or false 
    self.use_sampler_callback = use_sampler_callback or false

end

function OMPL_Processor:set_collision_pire(collision_name1, collision_name2)
    self.collision_name_1 = collision_name1
    self.collision_name_2 = collision_name2

end

function OMPL_Processor:set_callback_functions(function_validation, function_sampler, function_sampler_near)
    self.validate_callback_nume = function_validation
    self.use_sampler_callback = function_sampler
    self.sampler_near_callback_name = function_sampler_near
end

function OMPL_Processor:set_algorithm(algorithm_name)
    self.ompl_algorithm = algorithm_name
end