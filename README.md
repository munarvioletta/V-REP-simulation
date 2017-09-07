# V-REP-simulation
Symulacja procesu paletyzacji w opaciu o silnik fizyczny Bulet 2.78 z ykorzystaniem robotów UR5 oraz UR10 Universal Robots.

![Preview image 1](https://raw.githubusercontent.com/munarvioletta/V-REP-simulation/master/IK_test/Vrep_picture.png)

UR5
####LUA code 

--if (sim_call_type==sim_childscriptcall_actuation) then
 --   local v=-motorVelocity
  --  local data=simGetIntegerSignal('RG2_open')
   -- if data and data~=0 then
   --     v=motorVelocity
   -- end
   -- simSetJointForce(motorHandle,motorForce)
   -- simSetJointTargetVelocity(motorHandle,v)
--end


if (sim_call_type==sim_childscriptcall_initialization) then
    modelBase=simGetObjectAssociatedWithScript(sim_handle_self)
    robotBase=modelBase
    while true do
        robotBase=simGetObjectParent(robotBase)
        if robotBase==-1 then
            robotName='UR5'
            break
        end
        robotName=simGetObjectName(robotBase)
        local suffix,suffixlessName=simGetNameSuffix(robotName)
        if suffixlessName=='UR5' then
            break
        end
    end
    robotName=simGetObjectName(robotBase)
    motorHandle=simGetObjectHandle('RG2_openCloseJoint')
    motorVelocity=0.05 -- m/s
    motorForce=20 -- N
end


if (sim_call_type==sim_childscriptcall_actuation) then
    local data=simGetStringSignal(robotName..'_rg2GripperData')
    if data then
        velocityAndForce=simUnpackFloatTable(data)
        simSetJointTargetVelocity(motorHandle,velocityAndForce[1])
        simSetJointForce(motorHandle,velocityAndForce[2])
    end
end


    -- You have basically 2 alternatives to grasp an object:
    --
    -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
    --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
    --    coefficients, object masses and inertias)
    --
    -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
    --    much easier and offers very stable results.
    --
    -- Alternative 2 is explained hereafter:
    --
    --
    -- a) In the initialization phase, retrieve some handles:
    -- 
    -- connector=simGetObjectHandle('RG2_attachPoint')
    -- objectSensor=simGetObjectHandle('RG2_attachProxSensor')
    
    -- b) Before closing the gripper, check which dynamically non-static and respondable object is
    --    in-between the fingers. Then attach the object to the gripper:
    --
    -- index=0
    -- while true do
    --     shape=simGetObjects(index,sim_object_shape_type)
    --     if (shape==-1) then
    --         break
    --     end
    --     if (simGetObjectInt32Parameter(shape,sim_shapeintparam_static)==0) and (simGetObjectInt32Parameter(shape,sim_shapeintparam_respondable)~=0) and (simCheckProximitySensor(objectSensor,shape)==1) then
    --         -- Ok, we found a non-static respondable shape that was detected
    --         attachedShape=shape
    --         -- Do the connection:
    --         simSetObjectParent(attachedShape,connector,true)
    --         break
    --     end
    --     index=index+1
    -- end
    
    -- c) And just before opening the gripper again, detach the previously attached shape:
    --
    -- simSetObjectParent(attachedShape,-1,true)
