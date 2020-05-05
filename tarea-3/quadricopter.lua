 local quadcopter = require("quadcopter")
 
 function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)
    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from CoppeliaSim version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('20cmHighPillar10cm')
    --sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d=sim.getObjectHandle('Quadricopter_base')


	
    particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
    simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(simulateParticles))

    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    heli=sim.getObjectAssociatedWithScript(sim.handle_self)

    particlesTargetVelocities={0,0,0,0}

    pParam= 2
    iParam=0.1
    dParam=0.02
    vParam= -2 

    ecum=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0

    prevEuler=0


    fakeShadow=sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end

    -- Prepare 2 floating views with the camera views:
    floorCam=sim.getObjectHandle('Quadricopter_floorCamera')
    frontCam=sim.getObjectHandle('Quadricopter_frontCamera')
    floorView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(floorView,floorCam,64)
    sim.adjustView(frontView,frontCam,64)
    
    
     camera=sim.getObjectHandle('Vision_sensor')
     
     
     targetAlt = 0.5
     imgPos = {0.5,0.5}
         
     qdrPos = {0,0,0}
     numclose = 0
     euler  = sim.getObjectOrientation( d, -1) 
     rotVal = euler[3]
     newrot = rotVal
     
     qcopter = quadcopter:new( d )
end


function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(floorView)
    sim.floatingViewRemove(frontView)
end 





function showShadow( qdcpt)
    s = sim.getObjectSizeFactor( qdcpt )
    qpos = sim.getObjectPosition( qdcpt, -1 )
    if fakeShadow then
        itemData={ qpos[1], qpos[2], 0.002, 0, 0, 1, 0.2*s }
        sim.addDrawingObjectItem( shadowCont, itemData )
    end
end






function sysCall_actuation() 
    
	showShadow( d )
	
	res = qcopter:cameraDetectKhepera( camera )
	liderDetection = res[1]
    imgPos = ( liderDetection == true ) and res[2] or imgPos
    liderDetection = res[1]
    
    

    
    pos = sim.getObjectPosition(d,-1)
    
    
    -- Vertical position control:
    targetAlt = qcopter:calculateAltitude( liderDetection )
    l = sim.getVelocity(heli)
    e = targetAlt - pos[3]
    ecum = ecum + e
    thrust = 5.335 + pParam*e + iParam*ecum + dParam*(e-lastE) + vParam*l[3]
    lastE = e
    

    -- Horizontal position control: 
    m  = sim.getObjectMatrix( d, -1)
    
        --Corregir proyeccion cámara
        --[[
    if liderDetection then
		corrCam[1] = -1*math.sin(orit[2])
		corrCam[2] =  1*math.sin(orit[1])
    end
    sp = {0.5 - imgPos[1]  + corrCam[1], 0.5 - imgPos[2]  + corrCam[2]}
    ]]
    
    sp = qcopter:calculatMovement( liderDetection, imgPos )
    
    vy = sim.multiplyVector( m, {0,1,0} )
    alphaE = vy[3] - m[12]
    alphaCorr = 0.15*alphaE + 1.8*(alphaE - pAlphaE) + sp[2]*0.005 + 0.5*(sp[2] - psp2)
    pAlphaE = alphaE
    psp2 = sp[2]
    
    vx = sim.multiplyVector( m, {1,0,0} )
    betaE = vx[3] - m[12]
    betaCorr  = -0.15*betaE - 1.8*(betaE - pBetaE) - sp[1]*0.005 - 0.5*(sp[1] - psp1)
    pBetaE = betaE
    psp1 = sp[1]
    
    
    -- Rotational position control:
    rTocenter = math.sqrt( math.pow(sp[1],2) +  math.pow(sp[2],2) )
	targetRot = qcopter:calculateRotation( liderDetection, rTocenter )
    rotCorr   = targetRot*0.001 + 0.002*( targetRot - prevEuler )
    prevEuler = targetRot
    
    
    -- Decide of the motor velocities:
    particlesTargetVelocities[1] = thrust*(1 - alphaCorr + betaCorr + rotCorr)
    particlesTargetVelocities[2] = thrust*(1 - alphaCorr - betaCorr - rotCorr)
    particlesTargetVelocities[3] = thrust*(1 + alphaCorr - betaCorr + rotCorr)
    particlesTargetVelocities[4] = thrust*(1 + alphaCorr + betaCorr - rotCorr)
    
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end
    
    
    

    

    

    
    
    
end 

