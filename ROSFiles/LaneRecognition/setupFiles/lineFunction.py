class LineFunction():
    
    slope = 1
    constant = 0
    x_minValues = 0
    x_maxValues = 0
    y_minValues = 0
    y_maxValues = 0
    slopeValues = 0
    values = 1
    x_constant = False
    x_min = 0
    x_max = 0
    y_min = 0
    y_max = 0
    
    def __init__(self, x1, y1, x2, y2, ymin, ymax):
        self.x_min = x1
        self.x_max = x2
        if y1 < y2:
            self.y_min = y1
            self.y_max = y2
        else :
            self.y_min = y2
            self.y_max = y1
        if x2 == x1:
            self.x_constant = True
            self.constant = x1
        else :
            self.slope = round((y2-y1)/(x2-x1),3)
            self.constant = round(y1 - x1*(self.slope))
        #self.x_minValues = self.calculateXValue(ymin)
        #self.x_maxValues = self.calculateXValue(ymax) 
    
    def calculateYValue(self, x):
        return round(self.slope*x + self.constant)

    def calculateXValue(self, y):
        if self.slope != 0:
            return round((y-self.constant)/self.slope)
        return 1500000  

    def checkMinAndMax(self, lineToCompare, ymin, ymax):
        if lineToCompare.x_min < self.x_min:
            self.x_min = lineToCompare.x_min
        if lineToCompare.x_max > self.x_max:
            self.x_max = lineToCompare.x_max
        if lineToCompare.y_min < self.y_min:
            self.y_min = lineToCompare.y_min
        if lineToCompare.y_max > self.y_max:
            self.y_max = lineToCompare.y_max

        self.x_minValues = self.x_minValues + lineToCompare.x_min
        self.x_maxValues = self.x_maxValues + lineToCompare.x_max
        self.y_minValues = self.y_minValues + lineToCompare.y_min
        self.y_maxValues = self.y_maxValues + lineToCompare.y_max
        self.slopeValues = self.slopeValues + lineToCompare.slope

        self.values = self.values + 1 

    def average(self, ymin, ymax):
        """
        result = LineFunction(self.x_minValues/self.values, ymin, self.x_maxValues/self.values, ymax, ymin, ymax)
        print(self.x_minValues/self.values, self.x_maxValues/self.values,self.y_min, self.y_max, result.slope, result.constant)
        return LineFunction(result.calculateXValue(self.y_min),self.y_min,result.calculateXValue(self.y_max),self.y_max, ymin, ymax)
        """
        #result = LineFunction(self.x_min, self.y_min, self.x_max, self.y_max, ymin, ymax)
        #print(round(self.slopeValues/self.values, 3))
        return LineFunction(round(self.x_minValues/self.values), round(self.y_minValues/self.values), round(self.x_maxValues/self.values), round(self.y_maxValues/self.values), ymin, ymax)

    def reshape():
        if self.slope > 0:
            return [self.x_min, self.y_min, self.x_max, self.y_max]
        else :
            return [self.x_min, self.y_max, self.x_max, self.y_min]

    def printValues(self):#, y_begin, y_final):
        print("x_min = ",self.x_min,"y_min = ",self.y_min,"x_max = ",self.x_max,"y_max = ",self.y_max, "slope = ", self.slope)#, "x at min = ",self.calculateXValue(y_begin),"x at max = ", self.calculateXValue(y_final))
