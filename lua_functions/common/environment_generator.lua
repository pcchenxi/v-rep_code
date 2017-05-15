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
        obs_prob = 0.9,

        max_obs_h = 0.5,
        
        min_x = -0.4,
        max_x = 0.4,

        resolution = 0.1,

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
    self:set_new_row()
end

function Environment_Generator:move_all_object()
    local new_list = {}
    for i=1, #self.object_list, 1 do
        local pos = simGetObjectPosition(self.object_list[i], -1)
        pos[2] = pos[2] - self.resolution

        if pos[2] < self.min_y then 
            self:remove_object(i)
        else
            simSetObjectPosition(self.object_list[i], -1, pos)
            new_list[#new_list+1] = self.object_list[i]
        end
    end 

    self.object_list = new_list
end

function Environment_Generator:set_new_row()

    for x=self.min_x, self.max_x, self.resolution do
        if #self.object_list < 5 then
            local rand_obs = math.random()
            if rand_obs > self.obs_prob then
                self:set_cell_value(x, self.max_y)
            end 
        end
    end

    print("number of object: " ..#self.object_list)
end


function Environment_Generator:set_cell_value(x, y)

    local rand_h = math.random()
    local rand_z = math.random()

    local h = self.max_obs_h * rand_h
    -- local z = self.max_obs_h * rand_z + h/2

    local shape = {self.resolution, self.resolution, h}
    local pos = {x, y, h/2}

    self.object_list[#self.object_list+1] = self:create_one_cube(pos, shape)

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
