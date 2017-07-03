package.path=package.path .. ";/home/xi/workspace/v-rep_code/rl_planner/lua_functions/?.lua"
require("common_functions")
require("ompl_functions")
require("robot_control")

simSetThreadSwitchTiming(2) 
simExtRemoteApiStart(19999)

-------- remote functions ---------------------
function step(inInts,inFloats,inStrings,inBuffer)
    print (#inFloats)
    robot_state, res = do_action(robot_hd, joint_hds, inFloats)
    return {}, robot_state, {}, res
end


function get_global_path(inInts,inFloats,inStrings,inBuffer)
    -- forbidThreadSwitches(true)
    -- local isvalid = stateValidation(inFloats)
    -- forbidThreadSwitches(false)

    -- return {},{},{},'f'
    path_in_robot_frame = transform_path_to_robotf(path_dummy_list, robot_hd)

    print(#path_in_robot_frame)
    return {}, path_in_robot_frame, {}, ''

end

function generate_path()
    init_params(1, 0, 'robot_base', 'obstacles_tall', false, false)
    task_hd, state_dim = init_task('base_yaw','task_1')
    path = compute_path(task_hd, 10)
    displayInfo('finish 1 '..#path)
    -- applyPath(task_hd, path, 0.1)
    return path
end

function applyPath(task_hd, path, speed)
    local state = {}
    for i=1,#path-state_dim,state_dim do
        for j=1,state_dim,1 do
            state[j]=path[i+j-1]
        end
        res = simExtOMPL_writeState(task_hd, state) -- 12 joints, yaw,x,y

        sleep(speed)
        simSwitchThread()
    end
end

function create_dummy(pos)
    local hd = simCreateDummy(0.1)
    pos[3] = pos[3]+0.2
    simSetObjectPosition(hd, -1, pos)
    -- simSetObjectQuaternion(hd, -1, ori)
    -- path_dummy_list[#path_dummy_list+1] = hd
    return hd
end

function create_path_dummy(path)
    local dummy_list = {}
    local pos={}
    for i=1, #path, 2 do  
        pos[1] = path[i]
        pos[2] = path[i+1]
        pos[3] = 0
        local hd = create_dummy(pos)
        dummy_list[#dummy_list+1] = hd
    end
    return dummy_list
end

function transform_path_to_robotf(path_d_list, robot_hd)
    path_in_robotf = {}
    for i=1, #path_d_list, 1 do
        local d_hd = path_d_list[i]
        local d_pos = simGetObjectPosition(d_hd, robot_hd)

        local dist = math.sqrt(d_pos[1]*d_pos[1] + d_pos[2]*d_pos[2])
        local angle_thigh = math.atan(d_pos[2]/d_pos[1])

        path_in_robotf[#path_in_robotf + 1] = dist
        path_in_robotf[#path_in_robotf + 1] = angle_thigh
    end

    return path_in_robotf
end

function reset()
    local robot_pos = {}
    robot_pos[1] = math.random() * x_range + x_shift
    robot_pos[2] = math.random() * y_range + y_shift
    robot_pos[3] = 0
    print ('robot location: ', robot_pos[1], robot_pos[2])

    local target_pos = {}
    target_pos[1] = math.random() * x_range + x_shift
    target_pos[2] = math.random() * y_range + y_shift
    target_pos[3] = 0
    print ('target location: ', target_pos[1], target_pos[2])

    -- set robot --
    simSetObjectPosition(robot_hd,-1,robot_pos)
    simSetObjectQuaternion(robot_hd,-1,start_ori)
    set_joint_positions(joint_hds, start_joints)

    -- set target --
    simSetObjectPosition(target_hd,-1,target_pos)

    -- check collision --
    local res=simCheckCollision(collision_hd_1, collision_hd_2)
    
    -- g_path = generate_path()
    -- path_dummy_list = create_path_dummy(g_path)
end



g_path = {}
path_in_robot_frame = {}
path_dummy_list = {}

start_joints = {}

x_range = 2
x_shift = -1

y_range = 3
y_shift = -3

target_hd = simGetObjectHandle('target')
robot_hd = simGetObjectHandle('rwRobot')
joint_hds = get_joint_hds()

start_joints = get_joint_positions(joint_hds)
start_ori = simGetObjectQuaternion(robot_hd,-1)

reset()

-- g_path = generate_path()
-- path_dummy_list = create_path_dummy(g_path)

-- action = {1, 1, 0, -1, -1}
-- act = do_action(robot_hd, joint_hds, action)
-- print (act[1], act[2])

while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
    -- do something in here
    simSwitchThread()
end



--simExtOMPL_destroyTask(task_hd)

--init_params(6, 12, 'robot_body', 'obstacles', false, true)
--task_hd2, state_dim = init_task('start','task_2')
--path_2 = compute_path(task_hd2, 50)
--applyPath(task_hd2, path_2, 0.2)

--displayInfo('finish 2 '..#path..' '..#path_2)

-- while true do
--     sleep(0.01)
--     simSwitchThread()
-- end



