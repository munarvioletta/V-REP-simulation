enableIk=function(enable)
    if enable then
        simSetObjectMatrix(ikTarget,-1,simGetObjectMatrix(ikTip,-1))
        for i=1,#jointHandles,1 do
            simSetJointMode(jointHandles[i],sim_jointmode_ik,1)
        end

        simSetExplicitHandling(ikGroupHandle,0)
    else
        simSetExplicitHandling(ikGroupHandle,1)
        for i=1,#jointHandles,1 do
            simSetJointMode(jointHandles[i],sim_jointmode_force,0)
        end
    end
end

setGripperData=function(open,velocity,force)
    if not velocity then
        velocity=0.11
    end
    if not force then
        force=20
    end
    if not open then
        velocity=-velocity
    end
    local data=simPackFloatTable({velocity,force})
    simSetStringSignal(modelName..'_rg2GripperData',data)
end

jointHandles={-1,-1,-1,-1,-1,-1}
for i=1,6,1 do
    jointHandles[i]=simGetObjectHandle('UR10_joint'..i)
end

ikGroupHandle=simGetIkGroupHandle('UR10_IK')
ikTip=simGetObjectHandle('UR10_ikTip')
ikTarget=simGetObjectHandle('UR10_ik_target')
targetBase = simGetObjectHandle('UR10_Base')
p5 = simGetObjectHandle('p5')
p6 = simGetObjectHandle('p6')
p7 = simGetObjectHandle('p7')
p8 = simGetObjectHandle('p8')
p9 = simGetObjectHandle('p9')
p10 = simGetObjectHandle('p10')
sensor2=simGetObjectHandle('conveyorBelt_sensor2')
initPos = simGetObjectPosition(ikTarget,targetBase)
initOr = simGetObjectQuaternion(ikTarget,targetBase)
modelBase=simGetObjectAssociatedWithScript(sim_handle_self)
modelName=simGetObjectName(modelBase)
-- Set-up some of the RML vectors:
enableIk(false)   

vel=180
accel=40
jerk=80
currentVel={0,0,0,0,0,0,0}
currentAccel={0,0,0,0,0,0,0}
maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
targetVel={0,0,0,0,0,0}

ikMaxVel={0.8,0.8,0.8,0.8}
ikMaxAccel={0.8,0.8,0.8,0.8}
ikMaxJerk={0.8,0.8,0.8,0.8}

--targetPos1={0,10*math.pi/180,-45*math.pi/180,0,0,90*math.pi/180}
--targetPos1={180*math.pi/180,0,0,0,0,90*math.pi/180}
--targetPos2={90*math.pi/180,10*math.pi/180,50*math.pi/180,0,0,90*math.pi/180}
--targetPos3={180*math.pi/180,0,50*math.pi/180,0,0,90*math.pi/180}
y=0

while y<5 do

        

    if simReadProximitySensor(sensor2)==0 then
        simWait(0.5)
   
    else   
        if y==0 then     

            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then 
        enableIk(true)
        setGripperData(true)
        pos1=simGetObjectPosition(p5,targetBase)
        quat1=simGetObjectQuaternion(p5,targetBase)
        simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos1,quat1,nil)

        end
            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then 
        --simSetInt32Parameter(sim_intparam_current_page,1)        
      
        pos1=simGetObjectPosition(p6,targetBase)
        quat1=simGetObjectQuaternion(p6,targetBase)
        simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos1,quat1,nil)

        pos2=simGetObjectPosition(p7,targetBase)
        quat2=simGetObjectQuaternion(p7,targetBase)
        pos3=simGetObjectPosition(p8,targetBase)
        quat3=simGetObjectQuaternion(p8,targetBase)
        simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos2,quat3,nil)    
        simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos3,quat3,nil)
        end

            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
            simWait(0.5)
            setGripperData(false)
            simWait(2)
            end

            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
    
                simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos2,quat2,nil)

                pos4=simGetObjectPosition(p9,targetBase)
                quat4=simGetObjectQuaternion(p9,targetBase)
                simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos4[1],pos4[2],pos4[3]+0.105*y},quat4,nil)

                pos5=simGetObjectPosition(p10,targetBase)
                quat5=simGetObjectQuaternion(p10,targetBase)
                simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos5[1],pos5[2],pos5[3]+0.105*y},quat4,nil)
            end

            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
                    setGripperData(true)
                    simWait(0.5)
            end

            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then

            simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos4[1],pos4[2],pos4[3]+0.105*y},quat4,nil)

            end

       else

            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
   
                setGripperData(true)
                pos2=simGetObjectPosition(p7,targetBase)
                quat2=simGetObjectQuaternion(p7,targetBase)
                pos3=simGetObjectPosition(p8,targetBase)
                quat3=simGetObjectQuaternion(p8,targetBase)
                simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos2,quat3,nil)    
                simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos3,quat3,nil)
            end

            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then           
                    simWait(0.5)
                    setGripperData(false)
                    simWait(2)
             end

            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
    
                simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos2,quat2,nil)

                pos4=simGetObjectPosition(p9,targetBase)
                quat4=simGetObjectQuaternion(p9,targetBase)
                simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos4[1],pos4[2],pos4[3]+0.105*y},quat4,nil)

                pos5=simGetObjectPosition(p10,targetBase)
                quat5=simGetObjectQuaternion(p10,targetBase)
                simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos5[1],pos5[2],pos5[3]+0.105*y},quat4,nil)
            end
            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
                    setGripperData(true)
                    simWait(0.5)
            end
            if simGetSimulationState()~=sim_simulation_advancing_abouttostop then

            simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos4[1],pos4[2],pos4[3]+0.105*y},quat4,nil)
           
            end
        end

        y=y+1
       
    end

end
