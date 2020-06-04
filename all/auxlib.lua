local auxlib = {} -- The main table
	
	
	function auxlib.MatMul( m1, m2 )
		if #m1[1] ~= #m2 then       -- inner matrix-dimensions must agree
			return nil      
		end 
	 
		local res = {}
	 
		for i = 1, #m1 do
			res[i] = {}
			for j = 1, #m2[1] do
				res[i][j] = 0
				for k = 1, #m2 do
					res[i][j] = res[i][j] + m1[i][k] * m2[k][j]
				end
			end
		end
	 
		return res
	end


	function auxlib.sma(period)
		local t = {}
		function sum(a, ...)
			if a then return a+sum(...) else return 0 end
		end
		function average(n)
			if #t == period then table.remove(t, 1) end
			t[#t + 1] = n
			return sum(unpack(t)) / #t
		end
		return average
	end


	function auxlib.round(num) 
        if num >= 0 then return math.floor(num+.5) 
        else return math.ceil(num-.5) end
    end




return auxlib



