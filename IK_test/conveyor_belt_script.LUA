if (sim_call_type==sim_childscriptcall_initialization) then 
    pathHandle=simGetObjectHandle("ConveyorBeltPath")
    sensor=simGetObjectHandle('conveyorBelt_sensor')
    simSetPathTargetNominalVelocity(pathHandle,0) -- for backward compatibility
end 

if (sim_call_type==sim_childscriptcall_actuation) then 
    local beltVelocity=simGetScriptSimulationParameter(sim_handle_self,"conveyorBeltVelocity")

    beltVelocity=-0.08

    simHandleProximitySensor(sim_handle_all_except_explicit,0,1,1)
    if simReadProximitySensor(sensor)>0 then
        beltVelocity=0
    end

    local dt=simGetSimulationTimeStep()
    local pos=simGetPathPosition(pathHandle)
    pos=pos+beltVelocity*dt
    simSetPathPosition(pathHandle,pos) -- update the path's intrinsic position
    relativeLinearVelocity={beltVelocity,0,0}
   
end


