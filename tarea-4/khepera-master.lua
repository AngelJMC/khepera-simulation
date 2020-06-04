
local rob = require("robot")

function sysCall_threadmain()
	------------Llamando a los elementos------------------    

	local robotID = ''
	local rb = robot:new( 'master', robotID )
	
   
	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches
	
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
		
			-- Send the sensor data to sensormap
			sim.setStringSignal("master",sim.packTable( robot:getSensorsData( ) ))

			wr,wl = rb:moveRandom( 0.2 , 0.0 )
			rb:setTargetVelocity( wr, wl )
			
			sim.switchThread() -- Explicitely switch to another thread now!
	end 

end


function sysCall_cleanup()
	-- Put some clean-up code here:
end

