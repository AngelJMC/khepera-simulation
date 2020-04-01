

 	local lib = require "auxlib"
	lib.test()
	
	------------Llamando a los elementos------------------    
    
    local ir_sensorHdl = {}
    for i=1,5 do 
		ir_sensorHdl[i] = sim.getObjectHandle( 'K4_Infrared_' .. i + 1 )
    end
      
        
    local bodyElements=sim.getObjectHandle('Khepera_IV')
    print(bodyElements)
    local rightMotor=sim.getObjectHandle('K4_Right_Motor')
    local leftMotor=sim.getObjectHandle('K4_Left_Motor')
	
	local floorElement = sim.getObjectHandle('Floor')
	local targeElement = sim.getObjectHandle('Targetpoint')
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


local wl_sma = lib.sma(15)
local wr_sma = lib.sma(15)
	
while (true) do 

        p_robot  = sim.getObjectPosition( bodyElements, -1 )
        local myData= p_robot 
		sim.setStringSignal("masterPos",sim.packTable(myData))
		
        d = lib.getDistanceToTarget ( bodyElements, targeElement )
        ori_err = lib.getOrientationErrorToTarget ( bodyElements, targeElement )
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
        
        --Stop simulation
		if 0.005 > d then
			sim.pauseSimulation()
		end 
		
		
end 
