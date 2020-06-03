
local rob = require("robot")
	
function sysCall_threadmain()
	------------Llamando a los elementos------------------    
            
	local robotID = '#0'	
	local rb = robot:new( 'slave', robotID )
	local testpoint = sim.getObjectHandle('testpoint#0')

	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches

	while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
			
			-- Send the sensor data to sensormap
			sim.setStringSignal("slave#0",sim.packTable( robot:getSensorsData( ) ))

			wr,wl = rb:moveRandom( 0.1 , -0.05 )
			rb:setTargetVelocity( wr, wl ) 
			 
			sim.switchThread() -- Explicitely switch to another thread now!
	end 
	
end


function sysCall_cleanup()
	-- Put some clean-up code here:
end
