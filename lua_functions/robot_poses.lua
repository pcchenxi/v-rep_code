

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

function Pose_Generator:generate_pose(index)


end

function Pose_Generator:render_pose(state, ompl_hd)


end

