

 	
	
	------------Llamando a los elementos------------------    
    
    ir_sensorHdl = {}
    for i=1,5 do 
		ir_sensorHdl[i] = sim.getObjectHandle( 'K4_Infrared_' .. i + 1 )
    end
    SdistMax = 0.25 --m     
        
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

	

function runAdvanceControlRule ( d_toTarget , ori_err )
	
	wmax = 0.5
    Vmax = 0.15
    Vmin = 0.08
    Kr = 0.3
    Ki = 0.1
    
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


function MatMul( m1, m2 )
    if #m1[1] ~= #m2 then       -- inner matrix-dimensions must agree
        return nil      
    end 
 
    local res = {}
 
    for i = 1, #m1 do
        res[i] = {}
        for j = 1, #m2[1] do
            res[i][j] = 0
            for k = 1, #m2 do
                res[i][j] = res[i][j] + m1[i][k] * m2[k][j]
            end
        end
    end
 
    return res
end


	

function getDistanceVector( )
	distance = {}
	for i=1,5 do
		irDetect,irDist=sim.readProximitySensor( ir_sensorHdl[i], sim_handle_all )
		distance[i] = SdistMax
		
		if irDetect == 1 then
			distance[i] = irDist
		end		
	end
	
	return distance
end


function isAnyDetection( )
	detection = false
	for i=1,5 do
		irDetect,irDist=sim.readProximitySensor( ir_sensorHdl[i], sim_handle_all )
		if irDetect == 1 then
			detection = true
		end
	end
	return detection
end



function calculateObs( ) 


	mat1 = {{ 0.8,  1.5, 0.1, -0.9,   0 }, 
			{   0, -0.9, 0.1,  1.5, 0.8 } }
	
	Sdist = getDistanceVector( )
	--print(Sdist)
	
	dist = {}          -- create the matrix
    for i=1,5 do
      dist[i] = {}     -- create a new row
      for j=1,1 do
        dist[i][j] = ( Sdist[i] / SdistMax ) 
      end
    end
	
	
	res = MatMul( mat1, dist )
	--print( res )
	--print(dist[1],dist[2],dist[3],dist[4],dist[5])
	return res
end


function sma(period)
	local t = {}
	function sum(a, ...)
		if a then return a+sum(...) else return 0 end
	end
	function average(n)
		if #t == period then table.remove(t, 1) end
		t[#t + 1] = n
		return sum(unpack(t)) / #t
	end
	return average
end
 
wl_sma5 = sma(15)
wr_sma5 = sma(15)

	
while (true) do 

         
        d = getDistanceToTarget ( bodyElements, targeElement )
        d_toOrig = math.sqrt( ( p_robot[1] - p_robotOrig[1] )^2 + ( p_robot[2] - p_robotOrig[2] )^2 )
        ori_err = getOrientationErrorToTarget ( bodyElements, targeElement )
        
		v = runAdvanceControlRule ( d , ori_err )
        res = calculateObs( ) 
        
                
        if isAnyDetection( ) then
			wr = res[1][1]*2.2 
			wl = res[2][1]*2.2
        else
			-- Cinematic model, get the angular velocity wheels from linear 
			-- and angular robot velocity.
			wr = (2*v[1] - v[2]*wheelsdist) / (2*rwheel)
			wl = (2*v[1] + v[2]*wheelsdist) / (2*rwheel)
        end
        
        wl_f = wl_sma5(wl)
        wr_f = wr_sma5(wr)
        print( wl_f, wl)
        --print("v1", v[1]," v2", v[2] )
                
        -- Set angular velocity for each wheel.
        sim.setJointTargetVelocity(leftMotor,wl_f) --Cm/s a m/s
        sim.setJointTargetVelocity(rightMotor,wr_f)
		
		
     
		calculateStatics ( d , ori_err )
		if 0.005 > d then
			--print('ise_d', ise_d, 'itse_d', itse_d, 'iae_d', iae_d, 'itae_d', itae_d, 'ise_a', ise_a, 'itse_a', itse_a, 'iae_a', iae_a, 'itae_a', itae_a )
			print( string.format("%d & %d & %d & %d", math.floor(iae_d), math.floor(ise_d), math.floor(itae_d),math.floor(itse_d)) )
			--print( string.format("%d & %d & %d & %d", math.floor(iae_a), math.floor(ise_a), math.floor(itae_a),math.floor(itse_a)) )
			sim.pauseSimulation()
		end 

end 
