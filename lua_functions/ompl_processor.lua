package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/?.lua"
require("ompl_callbacks")

OMPL_Processor = {}
OMPL_Processor.__index = OMPL_Processor
function OMPL_Processor:new()
    local this =
    {
        robot_dim = 3,
        joint_dim = 12,
        collision_hd_1 = nil,
        collision_hd_2 = nil,
        use_validation_callback = false,
        use_sampler_callback = false,
        use_collision_check = false,

        validate_callback_nume = 'stateValidation',

        path = {},

        state_dim = 0,

        sample_num     = 0,
        valid_num      = 0
    }
    setmetatable(this, OMPL_Processor)
    return this
end


function OMPL_Processor:init_param(robot_dim, joint_dim, use_collision_check, use_validation_callback, use_sampler_callback)
    self.robot_dim = robot_dim
    self.joint_dim = joint_dim
    self.use_collision_check = use_collision_check or false
    self.use_validation_callback = use_validation_callback or false 
    self.use_sampler_callback = use_sampler_callback or false

end

function OMPL_Processor:set_collision_pire(collision_hd1, collision_hd2)
    self.collision_hd_1 = collision_name1
    self.joint_dim = joint_dim
    self.use_collision_check = use_collision_check or false
    self.use_validation_callback = use_validation_callback or false 
    self.use_sampler_callback = use_sampler_callback or false

end