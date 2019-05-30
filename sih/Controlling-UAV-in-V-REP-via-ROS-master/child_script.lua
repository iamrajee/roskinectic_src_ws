function sysCall_init() 
    ---------------ROS Interface Checking -----------------

    local moduleName=0
    local moduleVersion=0
    local index=0
    local pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    if (pluginNotFound) then
        -- Display an error message if the plugin was not found:
        sim.displayDialog('Error','RosInterface plugin not found. Run roscore before launching V-REP',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    else
        -- Enable topic subscription:
        droneTopicSub = simROS.subscribe('/drone_command','plutodrone/PlutoMsg','drone_callback')
        droneYawPub=simROS.advertise('/drone_yaw', 'std_msgs/Float64')
    end


    baseHandle = sim.getObjectHandle('eDroneBase')
    droneHandle = sim.getObjectHandle('eDrone')

 
    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('eDronePropellerRespondable_'..i)
    end
 
    particlesTargetVelocities={0,0,0,0}
 
    old_velx = 0
    old_vely = 0
    old_velz = 0

    ref_velx = 0
    ref_vely = 0
    ref_velz = 0

    ref_angz = 0
    oldAngVelZ = 0

    drone_orient = 0

 
    pAlphaE=0
    pBetaE=0
    prevEuler=0
 
    armFlag = false

end


function map(x,old_low,old_high,new_low,new_high)
    return ((x-old_low)*((new_high-new_low)/(old_high-old_low))) + new_low
end

function drone_callback(msg)

-------------Condition Checking for arming and disarming and accordingly setting simulateParticles-------------
    if (msg.rcThrottle==1000)and(msg.rcAUX4 >= 1300)and(armFlag==false) then
        sim.setScriptSimulationParameter(sim.handle_tree, 'propRotate', 'true')
        init_orient = sim.getObjectOrientation(droneHandle,-1)
        armFlag = true
        sim.setScriptSimulationParameter(sim.handle_tree, 'simulateParticles', 'false')
        if not(dialogHandle == null) then
            sim.endDialog(dialogHandle)
            dialogHandle = null
        end
        dialogHandle = sim.displayDialog('eDrone','ARMED',sim.dlgstyle_dont_center,false,nil,{0,0.8,0,0,0,0},{0,0.5,0,1,1,1}) --displaying dialog box for arming
        
    elseif (msg.rcAUX4<=1200)and(armFlag==true) then
        sim.setScriptSimulationParameter(sim.handle_tree, 'propRotate', 'false')
        sim.setScriptSimulationParameter(sim.handle_tree, 'simulateParticles', 'false')

        if not(dialogHandle == null) then
            sim.endDialog(dialogHandle)
            dialogHandle = null
        end

        dialogHandle= sim.displayDialog('eDrone','DISARMED',sim.dlgstyle_dont_center,false,nil,{0.5,0.3,0,0,0,0},{0.3,0.1,0,1,1,1}) --displaying dialog box for disarming
        armFlag=false

    elseif (armFlag == true)and(msg.rcThrottle > 1000) then
        sim.setScriptSimulationParameter(sim.handle_tree, 'simulateParticles', 'true')
        if not(dialogHandle == null) then
            sim.endDialog(dialogHandle)
            dialogHandle = null
        end


        -------- Mapping /drone_command [1200,1800] in the corresponding range------------
        -------- [1200,1800] and not [1000,2000] eventhough the real pluto drone takes command because
        -------- due to that range of velocity, the drone immediately gets away from the camera frame ---------------
        
        ref_velx = map(msg.rcPitch,1200,1800,-3,3)
        ref_vely = map(msg.rcRoll,1200,1800,-3,3)
        ref_velz = map(msg.rcThrottle,1200,1800,-15,15)
        ref_angz = map(msg.rcYaw,1200,1800,-3,3)
    end
end

function sysCall_actuation() 

    posDrone = sim.getObjectPosition(baseHandle,-1)

    velDroneGlobal, angVelDrone = sim.getObjectVelocity(baseHandle) ---- Getting velocity of eDrone in every simulation step
 
    -- Transformation to obtain velocities related to the quadrotor heading
    --------------------------------------------------------------------------
    -- Orientation of the quadcopter related to the World
    eulerDrone = sim.getObjectOrientation(baseHandle,-1)
 
    -- We create a Transformation Matrix centered at the quadrotor and rotated only in Z
    eulerHeading = eulerDrone
    eulerHeading[1] = 0
    eulerHeading[2] = 0
    m_heading = sim.buildMatrix(posDrone,eulerHeading)
 
    -- We create a Transformation Matrix for converting the global referenced velocities
    -- to a reference frame based on the heading of the quadrotor
    global_pos = {0,0,0}
    m_global = sim.buildMatrix(global_pos,eulerHeading)
    m = simGetInvertedMatrix(m_global)
    velDroneLocal=sim.multiplyVector(m,velDroneGlobal)
 
    -- We calculate the altitude of the Quadrotor in the frame based on the heading
    droneMat=sim.getObjectMatrix(baseHandle,-1)
    invHeadingMat = simGetInvertedMatrix(m_heading)
    m = sim.multiplyMatrices(invHeadingMat,droneMat)
    euler = sim.getEulerAnglesFromMatrix(m)
 
  
    -- Vertical velocity control 
    vel_errorz = ref_velz - velDroneLocal[3]
    thrust = 5.335 + vel_errorz*0.05 + (velDroneLocal[3]-old_velz)*0.05
    old_velz = velDroneLocal[3]

 
    -- Horizontal control
    vel_errorx = ref_velx - velDroneLocal[1]
    vel_errory = ref_vely - velDroneLocal[2]

 
    -- Error in Alpha angle (simple PD controler)
    alphaE = euler[1]
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
 
    -- Error in Beta angle (simple PD controler)
    betaE = -euler[2]
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
 
    -- We add the Velocity reference controller to the previous attitude control
    -- also a simple PD controler
    alphaCorr=alphaCorr+vel_errory*0.05 + (velDroneLocal[2]-old_vely)*0.05
    betaCorr=betaCorr-vel_errorx*0.05 - (velDroneLocal[1]-old_velx)*0.05
    old_velx = velDroneLocal[1]
    old_vely = velDroneLocal[2]
 

    -- Rotational control
    angVelZErr = ref_angz - angVelDrone[3]
    rotCorr= -angVelZErr - (angVelDrone[3]-oldAngVelZ)
    oldAngVelZ = angVelDrone[3]

 
    -- Decide the motor velocities:
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)
 
 
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end


    yaw_msg={}
    yaw_msg['data']=math.deg(sim.getObjectOrientation(droneHandle,-1)[3])
    
    simROS.publish(droneYawPub,yaw_msg)    
end 


function sysCall_cleanup() 

    simROS.shutdownSubscriber(droneTopicSub)
    
end 