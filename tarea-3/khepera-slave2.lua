
local rob = require("robot")

function sysCall_threadmain()
	------------Llamando a los elementos------------------    
            
	
	local robotID = '#2'
	local robotClass = 'slave'
	
	local rb = robot:new( robotClass, robotID, 3*math.pi/2, 0.5 )
	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches
	
	local testpoint = sim.getObjectHandle('testpoint#2')

	while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do

			p_target = rb:getSlaveTargetPos( )
			
			sim.setObjectPosition( testpoint, -1 , p_target)
			wr,wl = rb:getAngularSpeed( p_target )
			rb:setTargetVelocity( wr, wl ) 
			 
			sim.switchThread() -- Explicitely switch to another thread now!
	end 
	
end


function sysCall_cleanup()
	-- Put some clean-up code here:
end
