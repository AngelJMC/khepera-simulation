module(..., package.seeall )

	local lib = require("auxlib")
	local self = { }
	
	
	wheelsdist = 0.1
	rwheel = 0.02
	Coef = 2
	
	
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
	
	
	function robot:new( class, id )
	
		o = o or {}
		setmetatable(o, self)
		self.__index = self
		
		self.class = class == 'master' and class or 'slave'
		self.body 		= sim.getObjectHandle( 'Khepera_IV' .. id )
		self.rightMotor = sim.getObjectHandle( 'K4_Right_Motor' .. id )
		self.leftMotor 	= sim.getObjectHandle( 'K4_Left_Motor' .. id )
		self.sensor 	= {}
		for i=1,5 do 
			self.sensor[i] = sim.getObjectHandle( 'K4_Infrared_' .. i + 1 .. id )
		end
		
		self.wl_sma = lib.sma(15)
		self.wr_sma = lib.sma(15)
		self.p_robotOrig = simGetObjectPosition( self.body , -1 )
		
		return o
	end
	
	
	function robot:test()
		print("Library OOP!!")
	end
	
	
	function robot:getposition( )
		return sim.getObjectPosition( self.body, -1 )
	end
	
	function robot:getDistanceToTarget( p_target )

		local p_robot  = sim.getObjectPosition( self.body, -1 )    
		return math.sqrt( ( p_target[1] - p_robot[1] )^2 + ( p_target[2] - p_robot[2])^2 )
	end
		

	function robot:getOrientationErrorToTarget ( p_target )

		local p_robot  = sim.getObjectPosition( self.body, -1 )
		local o_robot  = sim.getObjectOrientation( self.body, -1 )
		local alpha = math.atan2( p_target[2] - p_robot[2], p_target[1] - p_robot[1] )   
		return o_robot[3] - alpha
	end

		
	function robot:runAdvanceControlRule ( d_toTarget, d_toOrig, ori_err )
			
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


	function robot:calculateObs( Sdist ) 

		local dist = {}          -- create the matrix
		for i=1,5 do
		  dist[i] = {}     -- create a new row
		  for j=1,1 do
			dist[i][j] = ( 1 - (  1 - SdistMax /  Sdist[i] ) ) 
		  end
		end
 
		return lib.MatMul( Ws, dist )
	end

	
	function robot:isAnyDetection( )
		local detection = false
		for i=1,5 do
			irDetect,irDist=sim.readProximitySensor( self.sensor[i], sim_handle_all )
			if irDetect == 1 then
				detection = true
			end
		end
		return detection
	end
	
	
	function robot:getDistanceVector( )
		local distance = {}
		for i=1,5 do
			local irDetect,irDist=sim.readProximitySensor( self.sensor[i], sim_handle_all )
			distance[i] = SdistMax
			
			if irDetect == 1 then
				distance[i] = irDist
			end		
		end
		return distance
	end
	
	
	
	function robot:getAngularSpeed( p_target )
			
			
			p_robot  = sim.getObjectPosition( self.body, -1 )
			
			if self.class == 'master' then
				d_toOrig = math.sqrt( ( p_robot[1] - self.p_robotOrig[1] )^2 + ( p_robot[2] - self.p_robotOrig[2] )^2 )
			else 
				d_toOrig = 0
			end
	
			d_toTarg = robot:getDistanceToTarget ( p_target )
			ori_err = robot:getOrientationErrorToTarget ( p_target )
			v = robot:runAdvanceControlRule ( d_toTarg, d_toOrig, ori_err )
			
			Sdist = robot:getDistanceVector( )
			res = robot:calculateObs( Sdist ) 
			
					
			if robot:isAnyDetection( ) then
				wr = res[1][1]*Coef
				wl = res[2][1]*Coef
			else
				-- Cinematic model, get the angular velocity wheels from linear 
				-- and angular robot velocity.
				wr = (2*v[1] - v[2]*wheelsdist) / (2*rwheel)
				wl = (2*v[1] + v[2]*wheelsdist) / (2*rwheel)
			end
		
			
		return wr, wl
	end
	
	function robot:setTargetVelocity( wr, wl )
	
			--wl_f = 
			--self.wr_f = 
			--print( wl_f, wl)
					
			-- Set angular velocity for each wheel.
			sim.setJointTargetVelocity(leftMotor,  self.wl_sma(wl) ) --Cm/s a m/s
			sim.setJointTargetVelocity(rightMotor, self.wr_sma(wr) )
	
	end
	
	function robot:isTargetReach( p_target )
	
		d_toTarg = robot:getDistanceToTarget ( p_target )
		return 0.005 > d_toTarg and true or false 
			
	end

