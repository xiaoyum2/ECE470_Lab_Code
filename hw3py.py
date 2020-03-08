class particleFilter:
    
    def __init__(self, dimension = 2, model = default_2D_Model(), numParticles = numberOfParticles, \
                 timeSpan = timsSpan, resamplingNoise = 0.01, positionStd = 5):
        self.model = model
        self.numParticles = numParticles
        self.dimension = dimension
        self.timeSpan = timeSpan
        self.std = positionStd                                     
        self.curMax = self.model.readMax()                       
        self.curMin = self.model.readMin()                         
        self.resNoise = [x*resamplingNoise for x in self.curMax]   
        
        ############# The initial particles are uniformely distributed #############
        ## TODO: self.particles = ? self.weights = ?
        
        # Generate uniformly distributed variables in x and y direction within [0, 1]
        # Hint: np.random.uniform(0, 1, ...) 
        
        # Spread these generated particles on the maze
        # Hint: use self.curMax, remember X direction: self.curMax[0], Y direction: self.curMax[1]
        # particles should be something like [[x1,y1], [x2,y2], ...]
        
        # Generate weight, initially all the weights for particle should be equal, namely 1/num_of_particles
        # weights should be something like [1/num_of_particles, 1/num_of_particles, 1/num_of_particles, ...]
        
        ## Your Code start from here
        
        test = np.random.uniform(0,1,(numParticles,2))
        a = np.zeros((2,2))
        a[0][0] = self.curMax[0]
        a[1][1] = self.curMax[1]
        self.particles = np.dot(test, a)
        self.weights = np.ones(numParticles)*1/numParticles
        
        
        
        
        
        
        ################################### End ###################################
    
        
    def Sample_Motion_Model(self, u_t=0):
        
        ########## Sample the Motion Model to Propagate the Particles ###########
        ## TODO: self.particles = ?
        
        # For each particle in self.particles [[x1,y1], [x2,y2], ...], get the nextEstimate
        # Hint: use self.model.simulateNextPosition(?, u_t)
        # Update self.particles
        
        ##########################################################################

        ## Your Code start from here
        old_particles = self.particles
        store_particles = []
        for i in range(self.numParticles):
            temp = self.model.simulateNextPosition(old_particles[i],u_t)
            store_particles.append(temp)
        self.particles = store_particles
        
        
        
        
        ################################### End ###################################
    
    
    def Measurement_Model(self):
        
        ##################### Measurement Motion Model #####################
        ## TODO: update self.weights, normalized
        
        # Get the sensor measurements for robot's position
        # Hint: use self.model.readingSensor()
        
        # For each particle in self.particles [[x1,y1], [x2,y2], ...], get the its position
        # Hint: use self.model.readingMap()
        
        # Calculate distance between robot's postion and each particle's position
        # Calculate weight for each particle, w_t = exp(-distance**2/(2*self.std))
        
        # Collect all the particles' weights in a list
        # For all the weights of particles, normalized them
        # Hint: pay attention to the case that sum(weights)=0, avoid round-off to zero
        
        # Update self.weights
        
        ## Your Code start from here
        loc_reading = self.model.readingSensor()
        #par_readings = []
        #par_distances = []
        set_wt = []
        old_particles = self.particles
        for i in range(self.numParticles):
            tempreading = self.model.readingMap(old_particles[i])
            arr_loc_read = np.asarray(loc_reading)
            arr_temp_read = np.asarray(tempreading)
            #par_readings.append(tempreading)
            tempdis = np.linalg.norm(arr_loc_read-arr_temp_read)
            #par_distances.append(tempdis)
            tempwt = np.exp(-tempdis**2/(2*self.std))
            set_wt.append(tempwt)
        
        norm_wt = [float(i)/sum(set_wt) for i in set_wt]
        self.weights = norm_wt
            
        
        
        
        
        
        
        

        
        
        
        
        ################################### End ###################################
    
    
    def calcPosition(self):
        
        ############# Calculate the position update estimate ###############
        ## TODO: return a list with two elements [x,y],  estimatePosition
        
        # For all the particles in direction x and y, get one estimated x, and one estimated y
        # Hint: use the normalized weights, self.weights, estimated x, y can not be out of the
        # boundary, use self.curMin, self.curMax to check
        
        ## Your Code start from here
        estimatePosition = [1.,1.]
        x = 0
        y = 0
        for i in range(self.numParticles):
            x = x+self.particles[i][0]*self.weights[i]
            y = y+self.particles[i][1]*self.weights[i]
        if x<self.curMin[0]:
            estimatePosition[0] = self.curMin[0]
        elif x>self.curMax[0]:
            estimatePosition[0] = self.curMax[0]
        else:
            estimatePosition[0] = x
        
        
        if y<self.curMin[1]:
            estimatePosition[1] = self.curMin[1]
        elif y>self.curMax[1]:
            estimatePosition[1] = self.curMax[1]
        else:
            estimatePosition[1] = y
            
        return estimatePosition
        
        
        
        
        
        
        
        

        ################################### End ###################################
    
    ## Method 1
    def resampling(self):
        
        ## TODO: Comment the code below:
        """
        
        
        """
        newParticles = []
        N = len(self.particles)
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1
        for i in range(N):
            randomProb = np.random.uniform()
            index = np.searchsorted(cumulative_sum, randomProb)
            newParticles.append(self.particles[index])
        self.particles = newParticles
        
        
#     ## Method 2: Roulette Wheel
#     def resampling(self):
        
#         ## TODO: Comment the code below:
#         """
        
        
#         """
#         newParticles = []
#         N = len(self.particles)
#         index = int(np.random.random() * N)
#         beta = 0
#         mw = np.max(self.weights)
#         for i in range(N):
#             beta += np.random.random() * 2.0 * mw
#             while beta > self.weights[index]:
#                 beta -= self.weights[index]
#                 index = (index + 1) % N
#             newParticles.append(self.particles[index])
#         self.particles = newParticles    
        
    
    def runParticleFilter(self):

        # Control command: 0 halt, 1 down, 2 right, 3 up, 4 left        
        # Hard code your control input in this array
        controls = [2,3,2,2,3,2,1,2,2,2,2,2,2,3,3,2,2,3,3,3,2,3,3,2,2,3,0]
        control = 0
        
        time.sleep(1)

        for t in range(self.timeSpan - 1):

            if controls:
                control = controls.pop()
            else: 
                control = 0

            if (t > 0):
                self.Sample_Motion_Model(control)

            self.Measurement_Model()
    
            estimatePosition = self.calcPosition()

            self.resampling()

            self.model.plotParticles(self.particles)

            self.model.plotEstimation(estimatePosition)

            self.model.map.clear_objects()

            self.model.run(control)     
