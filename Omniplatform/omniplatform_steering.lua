--[[
Author: Siqi Wang sq_wang@sjtu.edu.cn
Date: 2023-09-02 18:48:02
LastEditors: Siqi Wang sq_wang@sjtu.edu.cn
LastEditTime: 2023-09-02 19:27:23
FilePath: \script\ready\omniPlatform_steering.lua
Description:  A non-threading keyboard steering implementation of the OmniPlation

Copyright (c) 2023 by Siqi Wang, All Rights Reserved. 
--]]


function sysCall_init()
    -- Retrieve handles and prepare initial parameters:
    pathVisible = true
    platform=sim.getObject('.')
    omniPads={}
    for i=1,4,1 do
        omniPads[i]=sim.getObject('./link['..(i-1)..']/regularRotation')
        print(omniPads)
    end
    v=80*2.398795*math.pi/180-- 2.398795 is a factor needed to obtain the right pad rotation velocity

    if (pathVisible) then
        -- visualization cold: 
        graph=sim.getObject('/Graph')
        -- Define the X/Y graph curved:
        objectPosX=sim.addGraphStream(graph,'object pos x','m',1)
        objectPosY=sim.addGraphStream(graph,'object pos y','m',1)
        sim.addGraphCurve(graph,'object pos x/y',2,{objectPosX,objectPosY},{0,0},'m by m')
        -- Define a drawing object:
        drawing=sim.addDrawingObject(sim.drawing_linestrip,2,0,-1,0,{1,0,1})
        -- Define a structure that holds the recoded data:
        recording={}
    end
    
end

function sysCall_actuation() 
    pos = sim.getObjectPosition(platform,sim.handle_world)
    if (pathVisible) then
        -- visualization code
        sim.setGraphStreamValue(graph,objectPosX,pos[1])
        sim.setGraphStreamValue(graph,objectPosY,pos[2])
        -- Add another point to the drawing object:
        sim.addDrawingObjectItem(drawing,pos)
        -- Record that point:
        recording[#recording+1]=pos
    end
    
    message,auxiliaryData=sim.getSimulatorMessage()
    while message~=-1 do
        if (message==sim.message_keypress) then
            print(auxiliaryData)
            if (auxiliaryData[1]==49) then
               goSouthWest(v)
            end
            if (auxiliaryData[1]==50) then
               goSouth(v)
            end
            if (auxiliaryData[1]==51) then
               goSouthEast(v)
            end
            if (auxiliaryData[1]==52) then
               goWest(v)
            end
            if (auxiliaryData[1]==53) then
               stayStill()
            end
            if (auxiliaryData[1]==54) then
               goEast(v)
            end
            if (auxiliaryData[1]==55) then
               goNorthWest(v)
            end
            if (auxiliaryData[1]==56) then
               goNorth(v)
            end
            if (auxiliaryData[1]==57) then
               goNorthEast(v)
            end
        end
        message,auxiliaryData=sim.getSimulatorMessage()
    end

end

function goNorth(v)
    sim.setJointTargetVelocity(omniPads[1],v)
    sim.setJointTargetVelocity(omniPads[2],-v)
    sim.setJointTargetVelocity(omniPads[3],-v)
    sim.setJointTargetVelocity(omniPads[4],v)
end

function goSouth(v)
    sim.setJointTargetVelocity(omniPads[1],-v)
    sim.setJointTargetVelocity(omniPads[2],v)
    sim.setJointTargetVelocity(omniPads[3],v)
    sim.setJointTargetVelocity(omniPads[4],-v)
end

function goWest(v)
    sim.setJointTargetVelocity(omniPads[1],v)
    sim.setJointTargetVelocity(omniPads[2],v)
    sim.setJointTargetVelocity(omniPads[3],-v)
    sim.setJointTargetVelocity(omniPads[4],-v)
end

function goEast(v)
    sim.setJointTargetVelocity(omniPads[1],-v)
    sim.setJointTargetVelocity(omniPads[2],-v)
    sim.setJointTargetVelocity(omniPads[3],v)
    sim.setJointTargetVelocity(omniPads[4],v)
end

function goNorthWest(v)
    sim.setJointTargetVelocity(omniPads[1],2*v)
    sim.setJointTargetVelocity(omniPads[2],0)
    sim.setJointTargetVelocity(omniPads[3],-2*v)
    sim.setJointTargetVelocity(omniPads[4],0)
end

function goSouthEast(v)
    sim.setJointTargetVelocity(omniPads[1],-2*v)
    sim.setJointTargetVelocity(omniPads[2],0)
    sim.setJointTargetVelocity(omniPads[3],2*v)
    sim.setJointTargetVelocity(omniPads[4],0)
end

function goNorthEast(v)
    sim.setJointTargetVelocity(omniPads[1],0)
    sim.setJointTargetVelocity(omniPads[2],-2*v)
    sim.setJointTargetVelocity(omniPads[3],0)
    sim.setJointTargetVelocity(omniPads[4],2*v)
end

function goSouthWest(v)
    sim.setJointTargetVelocity(omniPads[1],0)
    sim.setJointTargetVelocity(omniPads[2],2*v)
    sim.setJointTargetVelocity(omniPads[3],0)
    sim.setJointTargetVelocity(omniPads[4],-2*v)
end




function stayStill()
    sim.setJointTargetVelocity(omniPads[1],0)
    sim.setJointTargetVelocity(omniPads[2],0)
    sim.setJointTargetVelocity(omniPads[3],0)
    sim.setJointTargetVelocity(omniPads[4],0)   
end


function stay()
    sim.setJointTargetVelocity(omniPads[1],0)
    sim.setJointTargetVelocity(omniPads[2],0)
    sim.setJointTargetVelocity(omniPads[3],0)
    sim.setJointTargetVelocity(omniPads[4],0)    
end