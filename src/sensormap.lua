local slam = require("slam")

function sysCall_init() 
    mapSensor=sim.getObjectHandle('mapSensor')
    
    --mapSensor = sim.handleVisionSensor('mapSensor')
    floatingView=sim.floatingViewAdd(0.8,0.3,0.5,0.8,0)
    sim.adjustView(floatingView,mapSensor,64)
    cnt=0
    senseEveryXPass=1

    local mapDimensions = {{-2600,2600}, {-2600,2600}} --mm  { {Xmin,Xmax}, {Ymin,Ymax}}
    res = sim.getVisionSensorResolution(mapSensor)
    
    local numCells = res --mm
    print(res )
    local slam = slam:new( mapDimensions, numCells )
        
    imgData = { }
    for i=1, res[1]*res[2]*3 do
        imgData[i] = 0.5
    end
    
end

function sysCall_sensing() 
    

    --Number/string signalValue=sim.waitForSignal("masterSensor")
    rname = {"master", "master#0", "master#1", "master#2"}
    local sensorData = {}
    for i=1,4 do
        local pack = sim.getStringSignal( rname[i] )
        if pack then
            sensorData[i] = sim.unpackTable( pack )
            --print(rname[i], sensorData[i])
        end
    end


    cnt=cnt+1
    if (cnt>=senseEveryXPass) then
        cnt=0
            
        
        --Iterate over each Khepera
        for i=1,4 do
            if sensorData[i] then
                slam:updateNeighborhoodCells( sensorData[i] )
            end
            --print( sensorData[i] ) 
        end

        gridData = slam:getGrid ( )
        for i=1,res[1] do
            for j=1,res[2] do
            
            idx = (i-1)*(3*res[2]) + (j-1)*3+1
            imgData[idx] = gridData[i][j]
            --print(gridData[i][j])
            end
        end
        
        --print(imgData)
        sim.setVisionSensorImage(mapSensor,imgData)
        sim.handleVisionSensor(mapSensor)

        --
    end
end 

function sysCall_cleanup() 

end 
