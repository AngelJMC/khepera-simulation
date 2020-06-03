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
			
	
	function robot:new( class, id, theta, dist )
	
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
		self.theta = theta
		self.dist  = dist
		self.wl_sma = lib.sma(15)
		self.wr_sma = lib.sma(15)
		self.p_robotOrig = simGetObjectPosition( self.body , -1 )
		self.d_toTarg = 0
		self.slaveErr = 0
		self.Kf = 0
		
		if class == 'master' then
			self.Vmax   = 0.19
			self.Vmin	= 0.08
			self.Kr     = 0.3
			self.Ki		= 0.1
			self.wmax   = 1.5
		else
			self.Vmax   = 0.20
			self.Vmin	= 0.12
			self.Kr     = 0.2
			self.Ki		= 0.1
			self.wmax   = 3
		end
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
		
	function robot:getOrientation( )
		local o_robot =  sim.getObjectOrientation( self.body, -1)
		return o_robot[3]
	end

	function robot:getSensorIDs( )
		return self.sensor
	end


	function robot:getSensorsData( )
		local sensData = {}
		for i=1,5 do
			--irDetect,irDist = sim.readProximitySensor( self.sensor[i], sim_handle_all )
			--sPos = sim.getObjectPosition( self.sensor[i], -1 )
			--sOri = sim.getObjectOrientation( self.sensor[i], -1 )
			--sensData[i] = {sPos, sOri, irDetect, irDist }
			sensData[i] = self.sensor[i]
		end
		return sensData
	end

	function robot:getOrientationErrorToTarget ( p_target )

		local p_robot  = sim.getObjectPosition( self.body, -1 )
		local o_robot  = sim.getObjectOrientation( self.body, -1 )
		local alpha = math.atan2( p_target[2] - p_robot[2], p_target[1] - p_robot[1] )   
		return o_robot[3] - alpha
	end

		
	function robot:runAdvanceControlRule ( d_toTarget, d_toOrig, ori_err )
		
		local v={}
		if d_toOrig < self.Ki then
			v[1] = math.max ( d_toOrig * (self.Vmax / self.Ki), self.Vmin )
		elseif d_toTarget < Kr  then
			v[1] = d_toTarget * (self.Vmax / self.Kr)
		else
				v[1] = self.Vmax
		end
			
		v[2] = self.wmax * math.sin( ori_err )
		
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
				d_toOrig = 0.2
			end
	
			self.d_toTarg = robot:getDistanceToTarget ( p_target )
			ori_err = robot:getOrientationErrorToTarget ( p_target )
			v = robot:runAdvanceControlRule ( self.d_toTarg, d_toOrig, ori_err )
			
			-- Cooperative rule
			if self.class == 'master' and self.Kf > 0.0 then
				v[1] = v[1]  -  self.Kf*self.slaveErr
				v[1] = v[1] < 0.05 and 0.05 or v[1]
			end
			
					
			if robot:isAnyDetection( ) then
				Sdist = robot:getDistanceVector( )
				res = robot:calculateObs( Sdist ) 
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
	
	function robot:moveRandom( vlineal, vangular)
		local v={}
		v[1] = vlineal
		v[2] = vangular
			
		if robot:isAnyDetection( ) then
			Sdist = robot:getDistanceVector( )
			res = robot:calculateObs( Sdist ) 
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
			-- Set angular velocity for each wheel.
			sim.setJointTargetVelocity(leftMotor,  self.wl_sma(wl) ) --Cm/s a m/s
			sim.setJointTargetVelocity(rightMotor, self.wr_sma(wr) )
	
	end
	
	
	function robot:isTargetReach( p_target )
	
		return 0.005 > self.d_toTarg and true or false 
			
	end
	
	function robot:getSlaveTargetPos( )
	
		local data=sim.getStringSignal("masterPos")
		p_robotMaster = {}
		if data then
			p_robotMaster = sim.unpackTable(data)
		end
				
		p_master = p_robotMaster[1] == nil and robot:getposition( ) or p_robotMaster[1]
		o_master  = p_robotMaster[2] == nil and robot:getOrientation( ) or p_robotMaster[2]
		
		offset = math.pi/2	
		p_target = p_master
		p_target[1] = p_master[1] + self.dist*math.cos( o_master + self.theta - offset)
		p_target[2] = p_master[2] + self.dist*math.sin( o_master + self.theta - offset)
		
		
		return p_target
	end

	function robot:getErrorToTarget( )
		return self.d_toTarg
	end

	
	function robot:getSlavesError( nslaves )
		acumerr = 0
		for i=1,nslaves do
			data=sim.getStringSignal("errSl#" .. i -1)
			acumerr = acumerr +  (data and sim.unpackTable(data) or {0})[1]
		end
		self.slaveErr = acumerr
	end
	
	function robot:setCooperativeFactor( value ) 
		self.Kf = value
	end
