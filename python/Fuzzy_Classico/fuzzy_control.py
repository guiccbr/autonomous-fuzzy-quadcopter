from math import exp, pi


class fuzzyController:

    def __init__(self, ranges, defuzRanges, membershipNumber=5.0, variance=0.2):
        self._ranges=ranges
        self._variance = variance
        self._membershipNumber = membershipNumber
        self._defuzRanges = defuzRanges

    # Inputs = [alt; dalt]
    def output(self, inputs):
        
        # Fuzzification
        fuzzyfied = self.fuzzification(self._membershipNumber, inputs, self._ranges)


        alphas = self.generateAlphas(fuzzyfied)

        # Defuzzification
        output = self.defuzzification(alphas, self._defuzRanges, self._membershipNumber)

        return output

    def generateAlphas(self, fuzzyfied):
        alphas = []
        f = fuzzyfied
        alphas.append(max([min([f[0][4],f[1][4]]),min([f[0][4],f[1][3]]),min([f[0][4],f[1][2]]),min([f[0][3],f[1][4]])]))
        alphas.append(max([min([f[0][3],f[1][0]]),min([f[0][4],f[1][0]]),min([f[0][4],f[1][1]]),min([f[0][3],f[1][2]]),min([f[0][3],f[1][3]]),min([f[0][2],f[1][3]]),min([f[0][2],f[1][4]])]))
        alphas.append(max([min([f[0][3],f[1][1]]),min([f[0][2],f[1][2]])]))
        alphas.append(max([min([f[0][2],f[1][0]]),min([f[0][2],f[1][1]]),min([f[0][1],f[1][1]]),min([f[0][1],f[1][2]]),min([f[0][1],f[1][3]]),min([f[0][0],f[1][3]]),min([f[0][0],f[1][4]])]))
        alphas.append(max([min([f[0][0],f[1][0]]),min([f[0][1],f[1][0]]),min([f[0][0],f[1][1]]),min([f[0][0],f[1][2]])]))

        return alphas


    def fuzzification(self, membershipFunctionsNumber, inputs, ranges):
        
        fuzzyfied = []
        i=0
        for input_ in inputs:
            subAlpha = []
            rangeInput = ranges[i]
            membershipsRange = (rangeInput[1]-rangeInput[0])/(float(membershipFunctionsNumber)-1)
         
            membershipsMean = []
            for membership in range(int(membershipFunctionsNumber)):
                membershipsMean.append(rangeInput[0]+membershipsRange*membership)
 
            for mean in membershipsMean:
                if mean==rangeInput[0] and input_ < mean:
                    subAlpha.append(1.0)
                    continue
                if mean==rangeInput[1] and input_ > mean:
                    subAlpha.append(1.0)
                    continue
                subAlpha.append(exp((-(input_-mean)**2)/(2*self._variance**2)))
        

            fuzzyfied.append(subAlpha)
            i+=1

        return fuzzyfied

    def defuzzification(self, alphas, ranges, membershipFunctionsNumber):

        subRange = (ranges[1]-ranges[0])/float(membershipFunctionsNumber-1)
        

        output = ranges[0]*alphas[0]+(ranges[0]+subRange)*alphas[1]+(ranges[0]+subRange*2)*alphas[2]+(ranges[0]+subRange*3)*alphas[3]+(ranges[1])*alphas[4]
        output = float(output)/(alphas[0]+alphas[1]+alphas[2]+alphas[3]+alphas[4])

        return output
