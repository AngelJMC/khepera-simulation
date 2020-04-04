

local rob = require("robot")
	
function sysCall_threadmain()
	
	------------Llamando a los elementos------------------    

	local robotID = ''
	local robotClass = 'master'
    local rb = robot:new( robotClass, robotID )
    
    local targeElement = sim.getObjectHandle('Targetpoint')
    local p_target = sim.getObjectPosition( targeElement, -1 )    
	
	--local testpoint = sim.getObjectHandle('testpoint1')

	
	sim.setThreadAutomaticSwitch(false) -- disable automatic thread switches
	
	-- Here we execute the regular thread code:
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do

			local p_robot  = rb:getposition( )
			o_robot = rb:getOrientation( )
			sim.setStringSignal("masterPos",sim.packTable( {rb:getposition( ), rb:getOrientation( ) } ))
			
			--[[
			p_test = p_robot
			offset = math.pi/2
			
			dist = 0.5
			theta = 0
			p_test[1] = p_robot[1] + dist*math.cos( o_robot + theta - offset)
			p_test[2] = p_robot[2] + dist*math.sin( o_robot + theta - offset)
			sim.setObjectPosition( testpoint, -1 , p_test)
			]]
			
						
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
