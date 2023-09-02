--[[
Author: Siqi Wang sq_wang@sjtu.edu.cn
Date: 2023-09-02 20:49:51
LastEditors: Siqi Wang sq_wang@sjtu.edu.cn
LastEditTime: 2023-09-02 20:50:01
FilePath: \script\scripts\Floor\draw_grid.lua
Description: This is a script for drawing grids on the floor

Copyright (c) 2023 by Siqi Wang, All Rights Reserved. 
--]]


-- This function is called once at the start of the simulation
function sysCall_init()
    -- Get the floor object
    local floor = sim.getObjectHandle('Floor') -- 'Floor' is the name of the floor object; change it according to your scene

    -- Set the parameters for the grid lines
    local gridSize = 10 -- Grid size
    local gridSpacing = 1.0 -- Grid spacing
    local gridColor = {0.5, 0.5, 0.5} -- Grid line color (gray)
    
    -- Set the line width (you can adjust this value)
    local lineWidth = 0.05 -- Line width
    
    -- Set the start Z coordinate (you can adjust this value)
    local startZ = 0.003 -- Start Z coordinate

    -- Create the grid lines
    for i = -gridSize / 2, gridSize / 2, gridSpacing do
        local startX = -gridSize / 2
        local startY = i
        local endX = gridSize / 2
        local endY = i

        -- Create a line object with the specified line width
        local line = sim.addDrawingObject(sim.drawing_lines, lineWidth, 0, floor, 1, gridColor)

        -- Set the coordinates of the line with the specified start Z coordinate
        sim.addDrawingObjectItem(line, {startX, startY, startZ, endX, endY, startZ})
    end

    for i = -gridSize / 2, gridSize / 2, gridSpacing do
        local startX = i
        local startY = -gridSize / 2
        local endX = i
        local endY = gridSize / 2

        -- Create a line object with the specified line width
        local line = sim.addDrawingObject(sim.drawing_lines, lineWidth, 0, floor, 1, gridColor)

        -- Set the coordinates of the line with the specified start Z coordinate
        sim.addDrawingObjectItem(line, {startX, startY, startZ, endX, endY, startZ})
    end
end

-- This function is called at the end of the simulation
function sysCall_cleanup()
    -- Clean up any resources if needed
end
