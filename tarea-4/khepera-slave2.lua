
local rob = require("robot")

function sysCall_threadmain()
	------------Llamando a los elementos------------------    
            
	local robotID = '#2'	
	local rb = robot:new( 'slave', robotID )
	local testpoint = sim.getObjectHandle('testpoint#2')

	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches

	while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do

			-- Send the sensor data to sensormap
			sim.setStringSignal("slave#2",sim.packTable( robot:getSensorsData( ) ))
			
			wr,wl = rb:moveRandom( 0.2 , 0.0 )
			rb:setTargetVelocity( wr, wl )
			 
			sim.switchThread() -- Explicitely switch to another thread now!
	end 
	
end


function sysCall_cleanup()
	-- Put some clean-up code here:
end
