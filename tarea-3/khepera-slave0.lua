

 	local lib = require "auxlib"
	
	
function sysCall_threadmain()
	------------Llamando a los elementos------------------    
    
    local ir_sensorHdl = {}
    for i=1,5 do 
		ir_sensorHdl[i] = sim.getObjectHandle( 'K4_Infrared_' .. i + 1 ..'#0' )
    end
      
        
    local bodyElements=sim.getObjectHandle('Khepera_IV#0')
    local rightMotor=sim.getObjectHandle('K4_Right_Motor#0')
    local leftMotor=sim.getObjectHandle('K4_Left_Motor#0')
	
	
	----------- Global variables -------------------------
	
	local p_robotOrig = simGetObjectPosition( bodyElements, -1 )
	local d_toOrig = 0

	local wl_sma = lib.sma(15)
	local wr_sma = lib.sma(15)
	
	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches
	lib.test()

	while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
			
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
			
			v = lib.runAdvanceControlRule ( d, d_toOrig, ori_err )
			
			Sdist = lib.getDistanceVector( ir_sensorHdl )
			res = lib.calculateObs( Sdist ) 
			
					
			if lib.isAnyDetection( ir_sensorHdl ) then
				wr = res[1][1]*lib.Coef
				wl = res[2][1]*lib.Coef
			else
				-- Cinematic model, get the angular velocity wheels from linear 
				-- and angular robot velocity.
				wr = (2*v[1] - v[2]*lib.wheelsdist) / (2*lib.rwheel)
				wl = (2*v[1] + v[2]*lib.wheelsdist) / (2*lib.rwheel)
			end
			
			wl_f = wl_sma(wl)
			wr_f = wr_sma(wr)
			--print( wl_f, wl)
					
			-- Set angular velocity for each wheel.
			sim.setJointTargetVelocity(leftMotor,wl_f) --Cm/s a m/s
			sim.setJointTargetVelocity(rightMotor,wr_f)
			
			--print(p_robotMaster)
			
			sim.switchThread() -- Explicitely switch to another thread now!
        -- from now on, above loop is executed once in each simulation step.
        -- this way you do not waste precious computation time and run synchronously.
			
			
			
			
	end 
	
end


function sysCall_cleanup()
	-- Put some clean-up code here:
end
