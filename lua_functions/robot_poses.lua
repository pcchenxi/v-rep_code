-- generate robot poses --

Pose_Generator = {}
Pose_Generator.__index = Pose_Generator
function Pose_Generator:new()
    local this =
    {
        name = "Pose_Generator",
        pose_list = {},
        angle_list = {-90, -45, 0, 45, 90}
    }
    setmetatable(this, Pose_Generator)
    return this
end

function Pose_Generator:print_name()
    print(self.name .. ": POSE!!!")
end

-- function Pose_Generator:generate_pose(index)


-- end

function Pose_Generator:read_current_pose(ompl_hd)

    local res, current_state = simExtOMPL_readState(ompl_hd)
    return res, state
end

function Pose_Generator:render_pose(ompl_hd, state)
    local res = simExtOMPL_writeState(ompl_hd, state)
    return res
end

