 local quadcopter = require("quadcopter")
 
 function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)
    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from CoppeliaSim version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:

    d=sim.getObjectHandle('Quadricopter_base')

    particlesAreVisible = sim.getScriptSimulationParameter( sim.handle_self, 'particlesAreVisible' )
    sim.setScriptSimulationParameter( sim.handle_tree, 'particlesAreVisible', tostring(particlesAreVisible) )
    simulateParticles = sim.getScriptSimulationParameter( sim.handle_self,'simulateParticles' )
    sim.setScriptSimulationParameter( sim.handle_tree, 'simulateParticles', tostring(simulateParticles) )

    propellerScripts = { -1, -1, -1, -1 }
    for i=1,4,1 do
        propellerScripts[i] = sim.getScriptHandle( 'Quadricopter_propeller_respondable'..i )
    end
    heli = sim.getObjectAssociatedWithScript( sim.handle_self )
    particlesTargetVelocities={0,0,0,0}


    fakeShadow = sim.getScriptSimulationParameter( sim.handle_self, 'fakeShadow' )
    if (fakeShadow) then
        shadowCont = sim.addDrawingObject( sim.drawing_discpoints
							+ sim.drawing_cyclic 
							+ sim.drawing_25percenttransparency 
							+ sim.drawing_50percenttransparency 
							+ sim.drawing_itemsizes, 0.2, 0, -1, 1 )
    end
    
    camera = sim.getObjectHandle('Vision_sensor')
    view=sim.floatingViewAdd(0.9,0.8,0.5,0.5,0)
    sim.adjustView( view, camera, 64)
    
    imgPos = {0.5,0.5}
    qcopter = quadcopter:new( d )
end


function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(view)
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
    
    -- Vertical position control:
    pos    = sim.getObjectPosition(d,-1)
    trgAlt = qcopter:calculateAltitude( liderDetection )
    vel    = sim.getVelocity(heli)
    thrust = qcopter:updateThrust( pos[3], trgAlt, vel )

    -- Horizontal position control: 
    m    = sim.getObjectMatrix( d, -1)
    sp 	 = qcopter:calculatMovement( liderDetection, imgPos )
    corr = qcopter:updateHorizontalCorrection( m, sp )

    -- Rotational position control:
    rTocenter = math.sqrt( math.pow(sp[1],2) +  math.pow(sp[2],2) )
	targetRot = qcopter:calculateRotation( liderDetection, rTocenter )
    rotCorr   = qcopter:updateRotation( targetRot )
    
    
    -- Decide of the motor velocities:
    particlesTargetVelocities[1] = thrust*(1 - corr[1] + corr[2] + rotCorr)
    particlesTargetVelocities[2] = thrust*(1 - corr[1] - corr[2] - rotCorr)
    particlesTargetVelocities[3] = thrust*(1 + corr[1] - corr[2] + rotCorr)
    particlesTargetVelocities[4] = thrust*(1 + corr[1] + corr[2] - rotCorr)
    
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end

end 

