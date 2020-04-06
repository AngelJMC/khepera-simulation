

local rob = require("robot")
	
function sysCall_threadmain()
	
	------------Llamando a los elementos------------------    

	local robotID = ''
	local Kf = 0.1 --Cooperative factor
    local rb = robot:new( 'master', robotID )
    
    local targeElement = sim.getObjectHandle('Targetpoint')
    local p_target = sim.getObjectPosition( targeElement, -1 )    
	
	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches
	rb:setCooperativeFactor( Kf ) 
	
	-- Here we execute the regular thread code:
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do

			sim.setStringSignal("masterPos",sim.packTable( {rb:getposition( ), rb:getOrientation( ) } ))
			rb:getSlavesError( 3 )
			
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
