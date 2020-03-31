

 	
	
	------------Llamando a los elementos------------------    
    
    ir_sensorHdl = {}
    for i=1,5 do 
		ir_sensorHdl[i] = sim.getObjectHandle( 'K4_Infrared_' .. i + 1 ..'#0' )
    end
      
        
    bodyElements=sim.getObjectHandle('Khepera_IV#0')
    rightMotor=sim.getObjectHandle('K4_Right_Motor#0')
    leftMotor=sim.getObjectHandle('K4_Left_Motor#0')
	
	--floorElement = sim.getObjectHandle('Floor')
	--targeElement = sim.getObjectHandle('Targetpoint')
	------------------------------------------------------

    
    ---------- Geometry properties -----------------------
	
	wheelsdist = 0.1
	rwheel = 0.02
	------------------------------------------------------
	
	----------- Global variables -------------------------
	
	p_robotOrig = simGetObjectPosition( bodyElements, -1 )
	d_toOrig = 0
	
	-- Braintenberg Algorithm param
	SdistMax = 0.25 --m  
	Coef = 2
	Ws = {{ 0,  -1.5, 0.9, 1.3,   1 }, 
			{   1, 1.3, 0.9,  -1.5, 0 } }
			
	-- Control law aram
	wmax = 0.7
    Vmax = 0.2
    Vmin = 0.08
    Kr = 0.3
    Ki = 0.1
			
	
	------------------------------------------------------

	
	
	
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

	Sdist = getDistanceVector( )
	
	dist = {}          -- create the matrix
    for i=1,5 do
      dist[i] = {}     -- create a new row
      for j=1,1 do
        dist[i][j] = ( 1 - (  1 - SdistMax /  Sdist[i] ) ) 
      end
    end
	
	res = MatMul( Ws, dist )
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
 

wl_sma = sma(15)
wr_sma = sma(15)
while (true) do 
		
		local myData=sim.getStringSignal("masterPos")
		if myData then
			p_robotMaster = sim.unpackTable(myData)
		end
        
		print(p_robotMaster)
		
		
end 
