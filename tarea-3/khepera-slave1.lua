
 	local lib = require "auxlib"
	lib.test()
 	
	
	------------Llamando a los elementos------------------    
    
    local ir_sensorHdl = {}
    for i=1,5 do 
		ir_sensorHdl[i] = sim.getObjectHandle( 'K4_Infrared_' .. i + 1 ..'#1' )
    end
      
        
    local bodyElements=sim.getObjectHandle('Khepera_IV#1')
    local rightMotor=sim.getObjectHandle('K4_Right_Motor#1')
    local leftMotor=sim.getObjectHandle('K4_Left_Motor#1')
	
	--floorElement = sim.getObjectHandle('Floor')
	--targeElement = sim.getObjectHandle('Targetpoint')
	------------------------------------------------------

    
    ---------- Geometry properties -----------------------
	
	wheelsdist = 0.1
	rwheel = 0.02
	------------------------------------------------------
	
	----------- Global variables -------------------------
	
	local p_robotOrig = simGetObjectPosition( bodyElements, -1 )
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
local wl_sma = lib.sma(15)
local wr_sma = lib.sma(15)

while (true) do 
		
		local myData=sim.getStringSignal("masterPos")
		if myData then
			p_robotMaster = sim.unpackTable(myData)
		end
        
        p_robot  = sim.getObjectPosition( bodyElements, -1 )
        
        --d = lib.getDistanceToTarget ( bodyElements, targeElement )
		if( p_robotMaster == nil ) then
			p_target = p_robot
		else
			p_target = p_robotMaster
		end
		 --sim.getObjectPosition( targeElement, -1 )     
		d = math.sqrt( ( p_target[1] - p_robot[1] )^2 + ( p_target[2] - p_robot[2])^2 )
        --ori_err = lib.getOrientationErrorToTarget ( bodyElements, targeElement )
        o_robot  = sim.getObjectOrientation( bodyElements, -1 )
		alpha = math.atan2( p_target[2] - p_robot[2], p_target[1] - p_robot[1] )   
		ori_err = o_robot[3] - alpha
		
		v = lib.runAdvanceControlRule ( d , ori_err )
		
		Sdist = lib.getDistanceVector( ir_sensorHdl )
        res = lib.calculateObs( Sdist ) 
        
                
        if lib.isAnyDetection( ir_sensorHdl ) then
			wr = res[1][1]*Coef
			wl = res[2][1]*Coef
        else
			-- Cinematic model, get the angular velocity wheels from linear 
			-- and angular robot velocity.
			wr = (2*v[1] - v[2]*wheelsdist) / (2*rwheel)
			wl = (2*v[1] + v[2]*wheelsdist) / (2*rwheel)
        end
        
        wl_f = wl_sma(wl)
        wr_f = wr_sma(wr)
        --print( wl_f, wl)
                
        -- Set angular velocity for each wheel.
        sim.setJointTargetVelocity(leftMotor,wl_f) --Cm/s a m/s
        sim.setJointTargetVelocity(rightMotor,wr_f)
        
		--print(p_robotMaster)
		
		
end 
