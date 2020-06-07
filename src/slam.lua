module(..., package.seeall )
	local lib = require("auxlib")
	local self = { }
	
	Kd = 2

	function slam:new( mapDimensions, numCells )
	
		o = o or {}
		setmetatable(o, self)
		self.__index = self

		self.mapSize = mapDimensions
		self.numCells = numCells
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
	
	
	function slam:transformCoordinatesToGrid( coord )
		
		
		px = ( self.numCells[1] - 0 )/( self.mapSize[1][2] - self.mapSize[1][1]) 
		ix = lib.round( coord[1]*px*1000 + self.numCells[1]/2 )
					
		py = ( self.numCells[2] - 0 )/( self.mapSize[2][2] - self.mapSize[2][1]) 
		iy = lib.round( coord[2]*py*1000 + self.numCells[2]/2 )

		--print( "pos:", coord, "Gridx", ix, "Gridy", iy)
		return ix, iy
	end


	function slam:getGrid ( )
		return self.grid
	end


	function slam:updateGridMap( sensorData )

		local distance = {}
		
		--Iterate over each Khepera
		for i=1,4 do
			if sensorData[i] then
				--Iterate over each sensor
				sensorIDs = sensorData[i]
				for i=1,5 do		
					local irDetect,irDist = sim.readProximitySensor( sensorIDs[i], sim_handle_all )

					dist = ( irDetect == 1 ) and irDist or 0.20
					sim.setObjectPosition( self.testpoint, sensorIDs[i] , {0,0,dist} )
					coord = sim.getObjectPosition( self.testpoint, -1 )
					--Transform coordenates to matrix grid
					ix,iy = slam:transformCoordinatesToGrid( coord )
					sensVal = ( irDetect == 1 ) and (0.5 + 0.5/Kd ) or 0.1

					--Aplicar bayes
					self.grid[iy][ix] = ( sensVal*self.grid[iy][ix] ) / ( sensVal*self.grid[iy][ix] + (1-sensVal)*(1-self.grid[iy][ix])) 	
				end
			end
		end


	end