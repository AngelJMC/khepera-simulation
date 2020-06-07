local slam = require("slam")

function sysCall_init() 
    
    mapSensor=sim.getObjectHandle('mapSensor')
    floatingView=sim.floatingViewAdd(0.8,0.3,0.5,0.8,0)
    sim.adjustView(floatingView,mapSensor,64)

    local mapDimensions = {{-2600,2600}, {-2600,2600}} --mm  { {Xmin,Xmax}, {Ymin,Ymax}}
    res = sim.getVisionSensorResolution(mapSensor)
    slam = slam:new( mapDimensions, res )

    --Init image    
    imgData = { }
    for i=1, res[1]*res[2]*3 do
        imgData[i] = 0.5
    end
    
end

function sysCall_sensing() 
    
    -- Get all sensor info 
    rname = {"master", "master#0", "master#1", "master#2"}
    local sensorData = {}
    for i=1,4 do
        local pack = sim.getStringSignal( rname[i] )
        if pack then
            sensorData[i] = sim.unpackTable( pack )
        end
    end

    slam:updateGridMap( sensorData )

    -- Update image from gridmap data.
    gridData = slam:getGrid ( )
    for i=1,res[1] do
        for j=1,res[2] do   
            idx = (i-1)*(3*res[2]) + (j-1)*3+1
            imgData[idx] = gridData[i][j]
        end
    end
    sim.setVisionSensorImage(mapSensor,imgData)
    sim.handleVisionSensor(mapSensor)


end 

function sysCall_cleanup() 

end 
