module(..., package.seeall )

	local self = { }
	
	
	function quadcopter:new(  d )
	
		o = o or {}
		setmetatable(o, self)
		self.__index = self
		
		
		self.d = d
		self.altitude = 0.5
		self.numclose = 0
		self.qdrPos   = {0,0,0}
		self.corrCam  = {0,0}
		
		self.thrust_e    = 0
		self.thrust_ecum = 0
		
		self.pAlphaE = 0
		self.psp2	 = 0
		
		self.pBetaE  = 0
		self.psp1    = 0
		
		self.prevRot = 0
		
		euler  = sim.getObjectOrientation( d, -1) 
		rotVal = euler[3]
		self.rotation = rotVal
		
		return o
	end
	
	
	function quadcopter:test()
		print("Library OOP quadcopter!!")
	end
	

	function quadcopter:cameraDetectKhepera( cam )

		liderDetection = false	
		result,t0,t1=sim.readVisionSensor(cam) -- Here we read the image processing camera!
		pos = {0,0}
		
		if t1 then -- in t1 we should have the blob information if the camera was set-up correctly		
			blobCount          = t1[1]
			dataSizePerBlob    = t1[2]
				
			-- Now we go through all blobs:
			for i=1,blobCount,1 do
				idxb = 2+(i-1)*dataSizePerBlob
				blobSize		  = t1[idxb + 1]
				blobPos			  = { t1[idxb + 3], t1[idxb + 4] }
		
				if blobSize>0.0001 then
					res = sim.getVisionSensorResolution(cam)
					blobCol = sim.getVisionSensorImage( cam, res[1]*blobPos[1], res[2]*blobPos[2], 1, 1)
					if blobCol[1] > blobCol[2] and blobCol[1] > blobCol[3] then
						pos = blobPos
						liderDetection = true
					end
				end
			end
		end
		return{ liderDetection, pos }
	end

	
	function quadcopter:calculateAltitude( detection )
		alpha = 0.985
		targetAltitude = detection and 0.6 or 4
		self.altitude = targetAltitude*(1 - alpha) + alpha*self.altitude
		return self.altitude
	end


	function quadcopter:calculateRotation( detection, rTocenter )
	
		pos = sim.getObjectPosition( self.d,-1 )
		
		if detection and rTocenter < 0.2 then
			if self.numclose == 0 then
				self.qdrPos = pos
			elseif self.numclose == 10 then			
				self.rotation = math.atan2( pos[2] - self.qdrPos[2], pos[1] - self.qdrPos[1] ) 
				self.qdrPos = pos
				self.numclose = 0
			end
			self.numclose = self.numclose + 1
		else
			self.numclose = 0
		end
		
		return self.rotation
	end


	function quadcopter:calculatMovement( detection, imgPos )
        --Corregir proyeccion cámara
        orit = sim.getObjectOrientation( self.d, -1 )
		if detection then
			self.corrCam[1] = -1*math.sin(orit[2])
			self.corrCam[2] =  1*math.sin(orit[1])
		end
		sp = {0.5 - imgPos[1]  + self.corrCam[1], 0.5 - imgPos[2]  + self.corrCam[2]}
		return sp
	end
	
	
	function quadcopter:updateThrust( altitude, targetAlt, l )
		
		pParam = 2
		iParam = 0.1
		dParam = 0.02
		vParam = -2 
		
		e = targetAlt - altitude
		self.thrust_ecum = self.thrust_ecum + e
		thrust = 5.335 + pParam*e + iParam*self.thrust_ecum + dParam*(e-self.thrust_e) + vParam*l[3]
		self.thrust_e    = e
		
		return thrust
	end
	
	
	function quadcopter:updateHorizontalCorrection( m, sp )
	    	    
	    vy = sim.multiplyVector( m, {0,1,0} )
		alphaE = vy[3] - m[12]
		alphaCorr = 0.15*alphaE + 1.8*(alphaE - self.pAlphaE) + sp[2]*0.005 + 0.5*(sp[2] - self.psp2)
		self.pAlphaE = alphaE
		self.psp2 = sp[2]
    
		vx = sim.multiplyVector( m, {1,0,0} )
		betaE = vx[3] - m[12]
		betaCorr  = -0.15*betaE - 1.8*(betaE - self.pBetaE) - sp[1]*0.005 - 0.5*(sp[1] - self.psp1)
		self.pBetaE = betaE
		self.psp1 = sp[1]
		
		return { alphaCorr, betaCorr}
	end
	
	
	function quadcopter:updateRotation( targetRot )
		rotCorr   = targetRot*0.001 + 0.002*( targetRot - self.prevRot )
		self.prevRot = targetRot
		return rotCorr
	end
	
	

	
	
	
	
	
	
	
	

