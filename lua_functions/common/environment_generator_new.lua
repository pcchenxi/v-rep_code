-- generate environment --
package.path=package.path .. ";/home/xi/workspace/v-rep_code/lua_functions/common/?.lua"
require("get_values")
require("set_values")
require("get_handles")

Environment_Generator = {}
Environment_Generator.__index = Environment_Generator
function Environment_Generator:new()
    local this =
    {
        name = "Environment_Generator",
        obs_prob = 0.98,

        max_obs_h = 0.5,
        
        min_x = -0.4,
        max_x = 0.4,

        resolution = 0.05,

        max_y = 0.4,
        min_y = -0.4,

        collection_hd = simGetCollectionHandle('obstacles'),

        object_list = {}
    }
    setmetatable(this, Environment_Generator)
    return this
end

function Environment_Generator:print_name()
    print(self.name .. "")
end

function Environment_Generator:run()
    self:move_all_object()
    local row = self:generate_new_row()
    self.object_list[#self.object_list] = row
end

function Environment_Generator:init_world()
    for y=self.min_y, self.max_y, self.resolution do
        local row = self:create_new_row(y, 0.01)
        self.object_list[#self.object_list+1] = row
    end
end

function Environment_Generator:move_all_object()
    local new_list = {}
    for i=1, #self.object_list-1, 1 do
        for j=1, #self.object_list[i], 1 do
            local pos_nex = simGetObjectPosition(self.object_list[i+1][j], -1)
            local pos_cur = simGetObjectPosition(self.object_list[i][j], -1)
            pos_cur[3] = pos_nex[3]
            simSetObjectPosition(self.object_list[i][j], -1, pos_cur)
        end
    end 
end

function Environment_Generator:create_new_row(y, h)
    local row={}

    for x=self.min_x, self.max_x, self.resolution do
        row[#row+1] = self:create_cell(x, y, h)

    end    
    return row
end


function Environment_Generator:generate_new_row()
    for x=self.min_x, self.max_x, self.resolution do
        local rand_obs = math.random()
        if rand_obs > self.obs_prob then
            local object_hd = self.object_list[#self.object_list]
            self:generate_cell_height(x, self.max_y, h)
        else 
            row[#row+1] = self:set_cell_value(x, self.max_y, h)
        end 
    end    
    return row
end


function Environment_Generator:create_cell(x, y, h)

    local rand_h = math.random()
    local rand_z = math.random()

    if h == nil then h = self.max_obs_h * rand_h end
    -- local z = self.max_obs_h * rand_z + h/2

    local shape = {self.resolution, self.resolution, h}
    local pos = {x, y, h/2-0.005}

    return self:create_one_cube(pos, shape) -- return object handle

end

function Environment_Generator:set_cell_height(x, y, h)

    local rand_h = math.random()
    local rand_z = math.random()

    if h == nil then h = self.max_obs_h * rand_h end
    -- local z = self.max_obs_h * rand_z + h/2

    local shape = {self.resolution, self.resolution, h}
    local pos = {x, y, h/2-0.005}

    
end

function Environment_Generator:create_one_cube(pos, shape)
    local handle = simCreatePureShape(0,16,shape,1)
    simSetObjectPosition(handle, -1, pos)

    local p=simGetObjectSpecialProperty(handle)
    p=simBoolOr16(p,sim_objectspecialproperty_collidable)
    p=simBoolOr16(p,sim_objectspecialproperty_measurable)
    p=simBoolOr16(p,sim_objectspecialproperty_renderable)
    simSetObjectSpecialProperty(handle,p)

    self:add_to_collection(handle)

    return handle
end

function Environment_Generator:remove_object(index)
    local object_hd = self.object_list[index]
    local res = simAddObjectToCollection(self.collection_hd, object_hd, sim_handle_single, 1+0) -- remove
    res = simRemoveObject(object_hd)

    -- table.remove(self.object_list, index)

    return res
end

function Environment_Generator:add_to_collection(object_hd)
    local res = simAddObjectToCollection(self.collection_hd, object_hd, sim_handle_single, 0+0)
    return res
end
