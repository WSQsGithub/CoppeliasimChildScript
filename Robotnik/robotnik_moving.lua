--[[
Author: Siqi Wang sq_wang@sjtu.edu.cn
Date: 2023-08-03 19:33:37
LastEditors: Siqi Wang sq_wang@sjtu.edu.cn
LastEditTime: 2023-09-02 19:30:28
FilePath: \script\ready\robotnik_moving.lua
Description: 

Copyright (c) 2023 by Siqi Wang, All Rights Reserved. 
--]]
--[[
Author: Siqi Wang sq_wang@sjtu.edu.cn
Date: 2023-08-03 19:33:37
LastEditors: Siqi Wang sq_wang@sjtu.edu.cn
LastEditTime: 2023-09-02 19:17:59
FilePath: \script\ready\robotnik_moving.lua
Description: keyboard moving child script for robotnik

Copyright (c) 2023 by Siqi Wang, All Rights Reserved. 
--]]

function sysCall_init()
    baseHandle = sim.getObject('.')
    targetTransitionVelocity = 0.05
    targetTransitionVelocity_z = 0.01
    targetRotationVelocity = math.rad(5)
end

function sysCall_actuation() 
    message,auxiliaryData=sim.getSimulatorMessage()
    while message~=-1 do
        if (message==sim.message_keypress) then
            print(auxiliaryData)
            local position = {0.0, 0.0,  0.0}
            local orientation = {0.0, 0.0, 0.0}
            if (auxiliaryData[1]==119) then
                -- w key: rise
                position[3] = targetTransitionVelocity_z
                -- sim.setObjectPosition(targetObj, targetObj, position)
            end
            if (auxiliaryData[1]==115) then
                -- s key: drop
                position[3] = -targetTransitionVelocity_z
                -- 
            end
            if (auxiliaryData[1]==2007) then
                -- up key: forward
                position[1] = targetTransitionVelocity
            end
            if (auxiliaryData[1]==2008) then
                -- down key: backward
                position[1] = -targetTransitionVelocity
            end
            if (auxiliaryData[1]==2009) then
                -- left key: turn left
                orientation[3] = targetRotationVelocity
            end
            if (auxiliaryData[1]==2010) then
                -- right key: turn right
                orientation[3] = -targetRotationVelocity
            end
            sim.setObjectPosition(baseHandle, baseHandle, position)
            sim.setObjectOrientation(baseHandle, baseHandle, orientation)
        end
        message,auxiliaryData=sim.getSimulatorMessage()
    end
end


