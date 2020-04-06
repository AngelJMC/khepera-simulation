
local rob = require("robot")
	
function sysCall_threadmain()
	------------Llamando a los elementos------------------    
	
	local robotID = '#1'
	local rb = robot:new( 'slave', robotID, math.pi, 0.5 )
	local testpoint = sim.getObjectHandle('testpoint#1')

	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches
	sim.setStringSignal("errSl" .. robotID,sim.packTable( {0} ) )
	
	while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do

			p_target = rb:getSlaveTargetPos( )
			
			sim.setObjectPosition( testpoint, -1 , p_target)
			wr,wl = rb:getAngularSpeed( p_target )
			rb:setTargetVelocity( wr, wl ) 
			sim.setStringSignal("errSl" .. robotID,sim.packTable( {rb:getErrorToTarget( )} ) )

			sim.switchThread() -- Explicitely switch to another thread now!
	end 
	
end


function sysCall_cleanup()
	-- Put some clean-up code here:
end
