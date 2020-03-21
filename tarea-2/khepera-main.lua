
	---------------- User Options  -----------------------
	--p = {-1,-1.5,0.1} 	--P1
	--p = {1.5, 1,0.1} 		--P2
	--p = {1,0,0.1} 		--P3
	p = {-1,1.5,0.1} 		--P4
	
	--controlRule = "basic" 
	controlRule = "advance"
	------------------------------------------------------
	
	
	------------Llamando a los elementos------------------    
        
    bodyElements=sim.getObjectHandle('Khepera_IV')
    rightMotor=sim.getObjectHandle('K4_Right_Motor')
    leftMotor=sim.getObjectHandle('K4_Left_Motor')
	
	floorElement = sim.getObjectHandle('Floor')
	targeElement = sim.getObjectHandle('Targetpoint')
	
    ------------------------------------------------------

    -------Separando a Objetivo del Robot Khepera IV------

    ------------------------------------------------------ 

    ----------Calling camera------------------------------

    frontCam=sim.getObjectHandle('K4_Camera')
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(frontView,frontCam,64)

    ------------------------------------------------------
    
    ---------- Geometry properties -----------------------
	
	wheelsdist = 0.1
	rwheel = 0.02
	------------------------------------------------------
	----------- Global variables -------------------------
	
	p_robotOrig = simGetObjectPosition( bodyElements, -1 )
	d_toOrig = 0
	
	ise_d  = 0
	itse_d = 0
	iae_d  = 0
	itae_d = 0
	
    ise_a  = 0
	itse_a = 0
	iae_a  = 0
	itae_a = 0
	
	t = 0
	
	
	----------------- Set target position -----------------------------
	targeElement = sim.getObjectHandle('Targetpoint')
	sim.setObjectPosition( targeElement, -1, p )
	
	
function getDistanceToTarget ( bodyElements, targeElement )

	p_robot  = sim.getObjectPosition( bodyElements, -1 )
    p_target = sim.getObjectPosition( targeElement, -1 )     
    return math.sqrt( ( p_target[1] - p_robot[1] )^2 + ( p_target[2] - p_robot[2])^2 )
end
	

function getOrientationErrorToTarget ( bodyElements, targeElement )

	p_robot  = sim.getObjectPosition( bodyElements, -1 )
	o_robot  = sim.getObjectOrientation( bodyElements, -1 )
    p_target = sim.getObjectPosition( targeElement, -1 )
    alpha = math.atan2( p_target[2] - p_robot[2], p_target[1] - p_robot[1] )   
    return o_robot[3] - alpha
end

	
function runBasicControlRule ( d , ori_err )
	
	Kp = 0.22
    wmax = 1.5
    
    v={}
    v[1] = Kp * d 
	v[2] = wmax * math.sin( ori_err )
    
	return v
end	


function runAdvanceControlRule ( d_toTarget , ori_err )
	
	wmax = 1.5
    Vmax = 0.8
    Vmin = 0.1
    Kr = 0.6
    Ki = 0.4
    
    v={}
    if d_toOrig < Ki then
		v[1] = math.max ( d_toOrig * (Vmax / Ki), Vmin )
    elseif d_toTarget < Kr  then
		v[1] = d_toTarget * (Vmax / Kr)
	else
		v[1] = Vmax
	end
	
	v[2] = wmax * math.sin( ori_err )
    
	return v
end	


function calculateStatics ( d_toTarget , ori_err )
	t = t +1
	
	ise_d  = ise_d  + d_toTarget^2
	itse_d = itse_d + t*d_toTarget^2
	iae_d  = iae_d  + math.abs(d_toTarget)
	itae_d = itae_d + t*math.abs(d_toTarget)
	
    ise_a  = ise_a  + ori_err^2
	itse_a = itse_a + t*ori_err^2
	iae_a  = iae_a  + math.abs(ori_err)
	itae_a = itae_a + t*math.abs(ori_err)
	
end


	
while (true) do 

         
        d = getDistanceToTarget ( bodyElements, targeElement )
        d_toOrig = math.sqrt( ( p_robot[1] - p_robotOrig[1] )^2 + ( p_robot[2] - p_robotOrig[2] )^2 )
        ori_err = getOrientationErrorToTarget ( bodyElements, targeElement )
        

        if  controlRule == "basic" then
			v = runBasicControlRule ( d , ori_err )
        elseif controlRule == "advance" then
			v = runAdvanceControlRule ( d , ori_err )
		else
			print('Error control rule' )
        end

        -- Cinematic model, get the angular velocity wheels from linear 
        -- and angular robot velocity.
        wr = (2*v[1] - v[2]*wheelsdist) / (2*rwheel)
        wl = (2*v[1] + v[2]*wheelsdist) / (2*rwheel)
        
        -- Set angular velocity for each wheel.
        sim.setJointTargetVelocity(leftMotor,wl) --Cm/s a m/s
        sim.setJointTargetVelocity(rightMotor,wr)
		
     
		calculateStatics ( d , ori_err )
		if 0.005 > d then
			--print('ise_d', ise_d, 'itse_d', itse_d, 'iae_d', iae_d, 'itae_d', itae_d, 'ise_a', ise_a, 'itse_a', itse_a, 'iae_a', iae_a, 'itae_a', itae_a )
			print( string.format("%d & %d & %d & %d", math.floor(iae_d), math.floor(ise_d), math.floor(itae_d),math.floor(itse_d)) )
			--print( string.format("%d & %d & %d & %d", math.floor(iae_a), math.floor(ise_a), math.floor(itae_a),math.floor(itse_a)) )
			sim.pauseSimulation()
		end 

end 
