module(..., package.seeall )
	local lib = require("auxlib")
	local self = { }
	
	
	function slam:new( mapDimensions, numCells )
	
		o = o or {}
		setmetatable(o, self)
		self.__index = self

		self.mapSize = mapDimensions
		--self.cellSize = cellSize

		self.numCells = numCells
		--self.numCells = {}
		--self.numCells[1] = math.abs( self.mapSize[1][1] - self.mapSize[1][2]) / self.cellSize; --Num cell X
		--self.numCells[2] = math.abs( self.mapSize[2][1] - self.mapSize[2][2]) / self.cellSize; --Num cell Y

		self.testpoint = sim.getObjectHandle('Dummy')
		
        self.grid = {}          -- create the matrix
        for i=1,self.numCells[1] do
          self.grid[i] = {}     -- create a new row
          for j=1,self.numCells[2] do
            self.grid[i][j] = 0.5
          end
        end
        print("Created matrix ith size:", self.numCells[1], "x", self.numCells[2], "MapSize", self.mapSize )

		return o
	end
	
    
    

	function slam:test()
		print("Library SLAM OOP!!")
	end
	
	
	function slam:getposition( )
		return sim.getObjectPosition( self.body, -1 )
	end 

	function slam:transformCoordinatesToGrid( self, pos )
		
		px = ( self.numCells[1] - 0 )/( self.mapSize[1][2] - self.mapSize[1][1]) 
		i = pos[1]*px + self.numCells[1]/2
		
		py = ( self.numCells[2] - 0 )/( self.mapSize[2][2] - self.mapSize[2][1]) 
		j = pos[2]*px + self.numCells[2]/2

		print( "pos:", pos, "Gridx", i, "Gridy", j)
	end


	function slam:getGrid ( )
		return self.grid
	end

	function slam:updateNeighborhoodCells( sensorIDs )

			local distance = {}
			--Iterate over each sensor
			for i=1,5 do
				
				local irDetect,irDist = sim.readProximitySensor( sensorIDs[i], sim_handle_all )
				
				sPos = sim.getObjectPosition( sensorIDs[i], -1 )
				sOri = sim.getObjectOrientation( sensorIDs[i], -1 )

				dist = 0.25
				if irDetect == 1 then
					dist = irDist
					--print(irDist)
				end

				sim.setObjectPosition( self.testpoint, sensorIDs[i] , {0,0,dist} )
				
				dPos = sim.getObjectPosition( self.testpoint, -1 )
				coord = dPos
				
				--for i=1,4 do
				i = 4
					coord[1] = sPos[1] + (i/4)*(dPos[1] - sPos[1])
					coord[2] = sPos[2] + (i/4)*(dPos[2] - sPos[2])
					sim.setObjectPosition( self.testpoint,-1 , coord )

					--Transform coordenates to matrix grid
					px = ( self.numCells[1] - 0 )/( self.mapSize[1][2] - self.mapSize[1][1]) 
					ix = lib.round( coord[1]*px*1000 + self.numCells[1]/2 )
					
					py = ( self.numCells[2] - 0 )/( self.mapSize[2][2] - self.mapSize[2][1]) 
					iy = lib.round( coord[2]*py*1000 + self.numCells[2]/2 )
					
					--print( "pos:", coord, "Gridx", lib.round(ix), "Gridy", lib.round(iy))

					sensVal = 0
					
					if( irDetect == 1 ) then
						sensVal = 0.8
					else
						sensVal = 0.1
					end
					
					--Aplicar bayes
					self.grid[iy][ix] = ( sensVal * self.grid[iy][ix] ) / ( sensVal * self.grid[iy][ix] + (1-sensVal)*(1-self.grid[iy][ix])) 
					
				--end
			end


	end