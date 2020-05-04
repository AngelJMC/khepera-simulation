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

    cumul=0
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
     res=sim.getVisionSensorResolution(camera)
     
     
     
     lider=sim.getObjectHandle('Khepera_IV')
     
     targetHigh = 1
     imgPos = {0.5,0.5}
         corrCam = {0,0}
     qdrPos = {0,0,0}
     numclose = 0
     euler  = sim.getObjectOrientation( d, -1) 
     rotVal = euler[3]
     newrot = rotVal
end


function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(floorView)
    sim.floatingViewRemove(frontView)
end 


function sysCall_actuation() 
    
    
	
	liderDetection = false	
	result,t0,t1=sim.readVisionSensor(camera) -- Here we read the image processing camera!
    if t1 then -- in t1 we should have the blob information if the camera was set-up correctly
            
        blobCount=t1[1]
        dataSizePerBlob=t1[2]
        lowestYofDetection=100
            
        -- Now we go through all blobs:
        for i=1,blobCount,1 do
			idxb = 2+(i-1)*dataSizePerBlob
			blobSize		  = t1[idxb + 1]
            blobOrientation   = t1[idxb + 2]
            blobPos			  = { t1[idxb + 3], t1[idxb + 4] }
            blobBoxDimensions = { t1[idxb + 5], t1[idxb + 6] }
    
			if blobSize>0.0001 then
				blobCol = sim.getVisionSensorImage(camera,res[1]*blobPos[1],res[2]*blobPos[2],1,1)
				if blobCol[1]>blobCol[2] and blobCol[1]>blobCol[3] then
					--print('X: ', blobPos[1], 'Y: ', blobPos[2], 'Size:', blobSize, 'Orient:', blobOrientation )
					imgPos = blobPos
					liderDetection = true
				end
			end
		end
	end
    

    s = sim.getObjectSizeFactor( d )
    pos = sim.getObjectPosition( d, -1 )
    if fakeShadow then
        itemData={ pos[1], pos[2], 0.002, 0, 0, 1, 0.2*s }
        sim.addDrawingObjectItem( shadowCont, itemData )
    end
    
    
    targetPos = sim.getObjectPosition(targetObj,-1)
    pos = sim.getObjectPosition(d,-1)
    orit = sim.getObjectOrientation(d,-1)
    

    -- Rising quadcopter if drone is no detected
    high = liderDetection and 0.5 or 4
    alpha = 0.99
    targetHigh = high*(1 - alpha) + alpha*targetHigh
    targetPos[3] = targetHigh
    
    -- Vertical control:
    l = sim.getVelocity(heli)
    e = targetPos[3] - pos[3]
    cumul = cumul + e
    thrust = 5.335 + pParam*e + iParam*cumul + dParam*(e-lastE) + vParam*l[3]
    lastE = e
    
    --Corregir proyeccion cámara
    if liderDetection then
		corrCam[1] = -1*math.sin(orit[2])
		corrCam[2] =  1*math.sin(orit[1])
    end

    -- Horizontal control: 
    m  = sim.getObjectMatrix( d, -1)
    
    sp = {}
    sp[1] = 0.5 - imgPos[1]  + corrCam[1]
    sp[2] = 0.5 - imgPos[2]  + corrCam[2]
    
    
    rTocenter = math.sqrt( math.pow(sp[1],2) +  math.pow(sp[2],2) )
    
    if liderDetection and rTocenter < 0.2 then
		if numclose == 0 then
			qdrPos = pos
		elseif numclose == 10 then
			delta = {pos[1] - qdrPos[1], pos[2] - qdrPos[2]}
			qdrPos = pos
			newrot = math.atan2( delta[2], delta[1] ) 
			
			numclose = 0
		end
		numclose = numclose + 1
    else
		numclose = 0
	end
	
	
	--print('Dir:', rotVal , 'X', pos[1], 'Y', pos[2] )
	
    
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
    
    -- Rotational control:
    alpha = 0
    rotVal = newrot*(1 - alpha) + alpha*rotVal
    rotCorr   = rotVal*0.001+0.002*(rotVal-prevEuler)
    prevEuler = rotVal
    
    
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

