

local rob = require("robot")
	
function sysCall_threadmain()
	
	------------Llamando a los elementos------------------    

	local robotID = ''
	local robotClass = 'master'
    local rb = robot:new( robotClass, robotID )
    
    local targeElement = sim.getObjectHandle('Targetpoint')
    local p_target = sim.getObjectPosition( targeElement, -1 )    

	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches
	
	-- Here we execute the regular thread code:
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do

			local p_robot  = rb:getposition( )
			sim.setStringSignal("masterPos",sim.packTable( p_robot ))
						
			wr,wl = rb:getAngularSpeed( p_target )
			rb:setTargetVelocity( wr, wl )
			
			if rb:isTargetReach( p_target ) then
				sim.pauseSimulation()
			end
			
			sim.switchThread() -- Explicitely switch to another thread now!
			
	end 

end


function sysCall_cleanup()
	-- Put some clean-up code here:
end
