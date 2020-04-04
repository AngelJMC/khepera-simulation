
local rob = require("robot")
	
function sysCall_threadmain()
	------------Llamando a los elementos------------------    
            
	
	local robotID = '#0'
	local robotClass = 'slave'
	
	local rb = robot:new( robotClass, robotID )
	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches

	while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
			
			local myData=sim.getStringSignal("masterPos")
			if myData then
				p_robotMaster = sim.unpackTable(myData)
			end
			
			p_target = p_robotMaster == nil and rb:getposition( ) or p_robotMaster
			wr,wl = rb:getAngularSpeed( p_target )
			rb:setTargetVelocity( wr, wl ) 
			 
			sim.switchThread() -- Explicitely switch to another thread now!
	end 
	
end


function sysCall_cleanup()
	-- Put some clean-up code here:
end
