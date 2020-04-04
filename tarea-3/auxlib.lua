local auxlib = {} -- The main table
	
	
	---------- Global Geometry properties -----------------------
	
	auxlib.wheelsdist = 0.1
	auxlib.rwheel = 0.02
	auxlib.Coef = 2
	------------------------------------------------------
	
	----------- Algorithm variables -------------------------
	

	
	-- Braintenberg Algorithm param
	local SdistMax = 0.25 --m  
	
	local Ws = {{ 0,  -1.5, 0.9, 1.3,   1 }, 
			{   1, 1.3, 0.9,  -1.5, 0 } }
			
	-- Control law aram
	local wmax = 0.7
    local Vmax = 0.2
    local Vmin = 0.08
    local Kr = 0.3
    local Ki = 0.1
	
	
	function auxlib.test()
		print("Library!!")
	end
	
	function auxlib.getDistanceToTarget ( bodyElements, targeElement )

		local p_robot  = sim.getObjectPosition( bodyElements, -1 )
		local p_target = sim.getObjectPosition( targeElement, -1 )     
		return math.sqrt( ( p_target[1] - p_robot[1] )^2 + ( p_target[2] - p_robot[2])^2 )
	end
		

	function auxlib.getOrientationErrorToTarget ( bodyElements, targeElement )

		local p_robot  = sim.getObjectPosition( bodyElements, -1 )
		local o_robot  = sim.getObjectOrientation( bodyElements, -1 )
		local p_target = sim.getObjectPosition( targeElement, -1 )
		local alpha = math.atan2( p_target[2] - p_robot[2], p_target[1] - p_robot[1] )   
		return o_robot[3] - alpha
	end

		
	function auxlib.runAdvanceControlRule ( d_toTarget, d_toOrig, ori_err )
			
		local v={}
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



	function auxlib.MatMul( m1, m2 )
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


	function auxlib.sma(period)
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


	function auxlib.calculateObs( Sdist ) 

		local dist = {}          -- create the matrix
		for i=1,5 do
		  dist[i] = {}     -- create a new row
		  for j=1,1 do
			dist[i][j] = ( 1 - (  1 - SdistMax /  Sdist[i] ) ) 
		  end
		end
 
		return auxlib.MatMul( Ws, dist )
	end

	
	function auxlib.isAnyDetection( ir_sensorHdl )
		local detection = false
		for i=1,5 do
			irDetect,irDist=sim.readProximitySensor( ir_sensorHdl[i], sim_handle_all )
			if irDetect == 1 then
				detection = true
			end
		end
		return detection
	end
	
	
	function auxlib.getDistanceVector( ir_sensorHdl )
		local distance = {}
		for i=1,5 do
			local irDetect,irDist=sim.readProximitySensor( ir_sensorHdl[i], sim_handle_all )
			distance[i] = SdistMax
			
			if irDetect == 1 then
				distance[i] = irDist
			end		
		end
		return distance
	end


return auxlib



