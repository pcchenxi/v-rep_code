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
        angle_list = {-75, -45, 0, 45, 75},

        robot_hd = simGetObjectHandle('rwRobot'),
        joint_hds = get_joint_hds(),

        robot_dim = 6,
        joint_dim = 12
    }
    setmetatable(this, Pose_Generator)
    return this
end

function Pose_Generator:print_name()
    print(self.name .. ": POSE!!!")
end

function Pose_Generator:get_one_pose()
    local state = self:generate_one_pose(1, 3, 1)
    self:render_pose(state)
    state = self:adjust_body(state)
    self:render_pose(state)
    local posi = simGetObjectPosition(self.joint_hds[16], -1)
    print(posi[3]..' '..state[3])

end

function Pose_Generator:generate_one_pose(index_ori, index_tilt, index_knee)
    local current_state = self:read_current_pose()
    local current_state = self:set_leg_ori(current_state, self.angle_list[index_ori])
    local current_state = self:set_leg_tilt(current_state, self.angle_list[index_tilt])
    local current_state = self:set_leg_knee(current_state, self.angle_list[index_knee])

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
    print(posi[3]..' '..state[3])
    state[3] = state[3] - posi[3] + 0.02
    return state
end
