--[[
Author: Siqi Wang sq_wang@sjtu.edu.cn
Date: 2023-08-10 10:56:28
LastEditors: Siqi Wang sq_wang@sjtu.edu.cn
LastEditTime: 2023-09-02 19:13:29
FilePath: \script\ready\pathVisualization.lua
Description: A TEMPLATE for non-threading code with path visualization， add graph to the scene first


Copyright (c) 2023 by Siqi Wang, All Rights Reserved. 
--]]

function sysCall_init() 
    pathVisible = true
    -- Other initiation code below:




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

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    for i=1,#particleObjects,1 do
        sim.removeParticleObject(particleObjects[i])
    end
end 

function sysCall_actuation() 

    if (pathVisible) then
        -- visualization code
        sim.setGraphStreamValue(graph,objectPosX,pos[1])
        sim.setGraphStreamValue(graph,objectPosY,pos[2])
        -- Add another point to the drawing object:
        sim.addDrawingObjectItem(drawing,pos)
        -- Record that point:
        recording[#recording+1]=pos
    end


    -- other code below: 

end 


function handlePropeller(index,particleVelocity)
    propellerRespondable=propellerHandles[index]
    propellerJoint=jointHandles[index]
    propeller=sim.getObjectParent(propellerRespondable)
    particleObject=particleObjects[index]
    maxParticleDeviation=math.tan(particleScatteringAngle*0.5*math.pi/180)*particleVelocity
    notFullParticles=0

    local t=sim.getSimulationTime()
    sim.setJointPosition(propellerJoint,t*10)
    ts=sim.getSimulationTimeStep()
    
    m=sim.getObjectMatrix(propeller,sim.handle_world)
    particleCnt=0
    pos={0,0,0}
    dir={0,0,1}
    
    requiredParticleCnt=particleCountPerSecond*ts+notFullParticles
    notFullParticles=requiredParticleCnt % 1
    requiredParticleCnt=math.floor(requiredParticleCnt)
    while (particleCnt<requiredParticleCnt) do
        -- we want a uniform distribution:
        x=(math.random()-0.5)*2
        y=(math.random()-0.5)*2
        if (x*x+y*y<=1) then
            if (simulateParticles) then
                pos[1]=x*0.08
                pos[2]=y*0.08
                pos[3]=-particleSize*0.6
                dir[1]=pos[1]+(math.random()-0.5)*maxParticleDeviation*2
                dir[2]=pos[2]+(math.random()-0.5)*maxParticleDeviation*2
                dir[3]=pos[3]-particleVelocity*(1+0.2*(math.random()-0.5))
                pos=sim.multiplyVector(m,pos)
                dir=sim.multiplyVector(m,dir)
                itemData={pos[1],pos[2],pos[3],dir[1],dir[2],dir[3]}
                sim.addParticleObjectItem(particleObject,itemData)
            end
            particleCnt=particleCnt+1
        end
    end
    -- Apply a reactive force onto the body:
    totalExertedForce=particleCnt*particleDensity*particleVelocity*math.pi*particleSize*particleSize*particleSize/(6*ts)
    force={0,0,totalExertedForce}
    m[4]=0
    m[8]=0
    m[12]=0
    force=sim.multiplyVector(m,force)
    local rotDir=1-math.mod(index,2)*2
    torque={0,0,rotDir*0.002*particleVelocity}
    torque=sim.multiplyVector(m,torque)
    sim.addForceAndTorque(propellerRespondable,force,torque)
end


