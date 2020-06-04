


local rob = require("robot")
	
function sysCall_threadmain()
	
	------------Llamando a los elementos------------------    

    ----------Calling camera------------------------------

    frontCam=sim.getObjectHandle('K4_Camera')
    frontView=sim.floatingViewAdd(0.7,0.9,0.2,0.2,0)
    sim.adjustView(frontView,frontCam,64)

	local robotID = ''
	local Kf = 0.0 --Cooperative factor
    local rb = robot:new( 'master', robotID )
    
    local targeElement = sim.getObjectHandle('Targetpoint')
    local p_target = sim.getObjectPosition( targeElement, -1 )    
	
	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches
	rb:setCooperativeFactor( Kf ) 
	
	-- Here we execute the regular thread code:
	while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
	--while true do
			
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