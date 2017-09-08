# V-REP-simulation
Symulacja procesu paletyzacji w opaciu o silnik fizyczny Bulet 2.78 z ykorzystaniem robotów UR5 oraz UR10 Universal Robots.

![Preview image 1](https://raw.githubusercontent.com/munarvioletta/V-REP-simulation/master/IK_test/Vrep_picture.png)

UR5

####LUA implementation 
####LUA code 

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
        force=40
    end
    if not open then
        velocity=-velocity
    end
    local data=simPackFloatTable({velocity,force})
    simSetStringSignal(modelName..'_rg2GripperData',data)
end

jointHandles={-1,-1,-1,-1,-1,-1}
for i=1,6,1 do
    jointHandles[i]=simGetObjectHandle('UR5_joint'..i)
end

ikGroupHandle=simGetIkGroupHandle('UR5_IK')
ikTip=simGetObjectHandle('UR5_Tip')
ikTarget=simGetObjectHandle('UR5_ik_target')
targetBase = simGetObjectHandle('RobotBase')
p1 = simGetObjectHandle('p1')
p2 = simGetObjectHandle('p2')
p2_up = simGetObjectHandle('p2_up')
p3 = simGetObjectHandle('p3')
p3_up = simGetObjectHandle('p3_up')
p4 = simGetObjectHandle('p4')
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

ikMaxVel={0.5,0.5,0.5,0.5}
ikMaxAccel={0.2,0.2,0.2,0.2}
ikMaxJerk={0.2,0.2,0.2,0.2}

--targetPos1={0,10*math.pi/180,-45*math.pi/180,0,0,90*math.pi/180}
--targetPos1={180*math.pi/180,0,0,0,0,90*math.pi/180}
--targetPos2={90*math.pi/180,10*math.pi/180,50*math.pi/180,0,0,90*math.pi/180}
--targetPos3={180*math.pi/180,0,50*math.pi/180,0,0,90*math.pi/180}
x=0
while x<5 do
if simGetSimulationState()~=sim_simulation_advancing_abouttostop then

--simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)
--simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos2,targetVel)
--setGripperData(false)
--simWait(0.5)
--setGripperData(true)
--simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos3,targetVel)
end

if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
   
    --simSetInt32Parameter(sim_intparam_current_page,1)
    enableIk(true)
    pos1=simGetObjectPosition(p4,targetBase)
    quat1=simGetObjectQuaternion(p4,targetBase)
    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos1,quat1,nil)
    --pos=simGetObjectPosition(ikTip,-1)
    --quat=simGetObjectQuaternion(ikTip,-1)
   -- simRMLMoveToPosition(ikTip,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2]+0.02,pos[3]},quat,nil)
   
end

if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
   
    
    --enableIk(true)
    --pos=simGetObjectPosition(ikTip,targetBase)
    --quat=simGetObjectQuaternion(ikTip,targetBase)
   -- simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2]-0.1,pos[3]-0.02},quat,nil)
    --simRMLMoveToPosition(ikTip,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1],pos[2]+0.02,pos[3]},quat,nil)
    --enableIk(true)   
    setGripperData(true)
    pos1=simGetObjectPosition(p1,targetBase)
    quat1=simGetObjectQuaternion(p1,targetBase)
    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos1,quat1,nil)

    pos2=simGetObjectPosition(p2_up,targetBase)
    quat2=simGetObjectQuaternion(p2_up,targetBase)
    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos2,quat2,nil)

    pos3=simGetObjectPosition(p2,targetBase)
    quat3=simGetObjectQuaternion(p2,targetBase)
    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos3,quat3,nil)
end
 if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
        setGripperData(false)
        simWait(1)
 end
 if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
    
    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos2,quat2,nil)

    pos4=simGetObjectPosition(p3_up,targetBase)
    quat4=simGetObjectQuaternion(p3_up,targetBase)
    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos4,quat4,nil)

    pos5=simGetObjectPosition(p3,targetBase)
    quat5=simGetObjectQuaternion(p3,targetBase)
    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos5,quat5,nil)
end
if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
        setGripperData(true)
        simWait(0.5)
end
 if simGetSimulationState()~=sim_simulation_advancing_abouttostop then

    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos4,quat4,nil)
    simRMLMoveToPosition(ikTarget,targetBase,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,pos1,quat1,nil)

end
--enableIk(true)
--pos=simGetObjectPosition(ikTip,-1)
--quat=simGetObjectQuaternion(ikTip,-1)
--simRMLMoveToPosition(ikTarget,-1,-1,nil,nil,ikMaxVel,ikMaxAccel,ikMaxJerk,{pos[1]+0.8,pos[2],pos[3]},quat,nil)
--simRMLMoveToPosition(ikTarget,-1,-1,nil,nil,MaxVel,MaxAccel,MaxJerk,{pos[1]+0.8,pos[2],pos[3]},quat,nil)



x=x+1

end

--if simGetSimulationState()~=sim_simulation_advancing_abouttostop then
   --     setGripperData(false)
    --    simWait(0.5)
--end
