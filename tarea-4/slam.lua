module(..., package.seeall )

	local self = { }
	
	
	function slam:new( sizeCell )
	
		o = o or {}
		setmetatable(o, self)
		self.__index = self
        
        self.sizeCell = sizeCell
        self.grid = {}          -- create the matrix
        for i=1,sizeCell[1] do
          self.grid[i] = {}     -- create a new row
          for j=1,sizeCell[2] do
            self.grid[i][j] = 0
          end
        end
        print("Created matrix ith size:", sizeCell[1], "x",sizeCell[2],self.grid )

		return o
	end
	
    
    

	function slam:test()
		print("Library SLAM OOP!!")
	end
	
	
	function slam:getposition( )
		return sim.getObjectPosition( self.body, -1 )
	end 