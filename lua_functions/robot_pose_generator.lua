-- generate robot poses --
package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/?.lua"
require("get_values")
require("set_values")
require("get_handles")

Pose_Generator = {}
Pose_Generator.__index = Pose_Generator
function Pose_Generator:new()
    local this =
    {
        name = "Pose_Generator",

        pose_list = {},
        angle_list = {-80, -45, 0, 45, 80},
        knee_list = {-80, -45, 0, 45, 80},

        robot_hd = simGetObjectHandle('rwRobot'),
        joint_hds = get_joint_hds(),
        leg_hds = {},

        robot_dim = 6,
        joint_dim = 12,
    }
    setmetatable(this, Pose_Generator)
    return this
end

function Pose_Generator:print_name()
    print(self.name .. ": POSE!!!")
end

function Pose_Generator:get_leg_collection_hds()
    local hd1 = simGetCollectionHandle('leg')
    local hd2 = simGetCollectionHandle('leg#0')
    local hd3 = simGetCollectionHandle('leg#1')
    local hd4 = simGetCollectionHandle('leg#2')

    self.leg_hds = {hd1, hd2, hd3, hd4}
    print('body down '..hd3)

end

function Pose_Generator:init_pose_list()
    self:get_leg_collection_hds()

    for i=1, #self.angle_list, 1 do
        for j=1, #self.angle_list, 1 do 
            for k=1, #self.knee_list, 1 do 
                local state = self:get_one_pose(i, j, k)
                -- sleep(1)
                -- simSwitchThread()
                local is_valid = self:check_pose(state)
                if is_valid then self.pose_list[#self.pose_list+1] = state end
            end
        end
    end

    print('valid pose: '..#self.pose_list)
    self:view_pose_list()
end

function Pose_Generator:view_pose_list()
    for i=1, #self.pose_list, 1 do
        local state = self.pose_list[i]
        self:render_pose(state)
        sleep(1)
        simSwitchThread()
    end
end

function Pose_Generator:check_pose(state)
    local pos = simGetObjectPosition(self.robot_hd, -1)
    if pos[3] < 0.05 then 
        return false
    end

    local is_valid = true
    for i=2, #self.leg_hds, 1 do
        local res=simCheckCollision(self.leg_hds[1], self.leg_hds[i])
        if res == 1 then 
            is_valid = false
        end
    end

    if is_valid then 
        print('good pose')
    else
        print('Collision pose')
    end   
    return is_valid

end

function Pose_Generator:get_one_pose(index_ori, index_tilt, index_knee)
    -- local index_ori = math.random(5)
    -- local index_tilt = math.random(5)
    -- local index_knee = math.random(3)

    local state = self:generate_one_pose(index_ori, index_tilt, index_knee)
    self:render_pose(state)
    state = self:adjust_body(state)
    self:render_pose(state)

    return state
end

function Pose_Generator:generate_one_pose(index_ori, index_tilt, index_knee)
    local current_state = self:read_current_pose()
    local current_state = self:set_leg_ori(current_state, self.angle_list[index_ori])
    local current_state = self:set_leg_tilt(current_state, self.angle_list[index_tilt])
    local current_state = self:set_leg_knee(current_state, self.knee_list[index_knee])

    return current_state
end

function Pose_Generator:read_current_pose()
    state = get_robot_pose(self.robot_hd, self.joint_hds, self.robot_dim, self.joint_dim)
    return state
end

function Pose_Generator:render_pose(state)
    set_robot_pose(self.robot_hd, self.joint_hds, state)
end

function Pose_Generator:set_leg_ori(state, ori_value)
    local value = (ori_value)*math.pi/180
    local value2 = -(ori_value)*math.pi/180

    state[self.robot_dim+2] = value
    state[self.robot_dim+3] = value2
    state[self.robot_dim+4] = -state[self.robot_dim+2]
    state[self.robot_dim+5] = -state[self.robot_dim+3]

    return state
end

function Pose_Generator:set_leg_tilt(state, tilt_value)
    local value = -(tilt_value)*math.pi/180
    local index = self.robot_dim+5
    for i=1, 4, 1 do
        state[index + i] = value
    end
    return state
end

function Pose_Generator:set_leg_knee(state, tilt_value)
    local value = (-tilt_value+90)*math.pi/180
    local index = self.robot_dim+9
    for i=1, 4, 1 do
        state[index + i] = value
    end
    return state
end

function Pose_Generator:adjust_body(state)
    local posi = simGetObjectPosition(self.joint_hds[20], -1)
    state[3] = state[3] - posi[3] + 0.02
    return state
end
