# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 19:08:07 2015
# GA for Evolving behavior parameters for ARGoS
#Last revision: 10/31/2015
# GA.py
@author: Muninn
"""
import random
import math
import subprocess
import os
import matplotlib.pyplot as plt
import sys

GENESIZE = 7
POPSIZE = 200
GENERATIONS = 30

class Gene(object):
    def __init__(self):
        self.dna = initialize()
        self.fitness = 0
        self.argos_xml = None
        
    def getDna(self):
        return self.dna
        
    def getFitness(self):
        return self.fitness
        
    def setFitness(self,fVal):
        self.fitness = fVal

#==============================================================================
#   Initialize population.
#==============================================================================
def initialize():
    dna = []
    for i in range(0,7):
        if i == 0:                  #return to nest
            dna.append(str(random.uniform(0.0, 0.0005)))
        elif i > 0 and i <= 3:       #switch to search and informed search decay
            dna.append(str(random.uniform(0.0, 1.0)))
        elif i > 3 and i <= 5:      #rate of laying pheromone and rate of using site fidelity
            dna.append(str(random.uniform(0, 20.0))) 
        else:                       #uninformed search var
            dna.append(str(random.uniform(0.0, 4 * math.pi)))
    return dna

#==============================================================================
#   Generate random value for gene.
#==============================================================================
def randVal():  
    return random.uniform( 0.0, 1.0 )
#==============================================================================
#   Create xml file with individual's parameters.
#==============================================================================
def xmlGen(gene):
    returnToNest = gene[0] 
    switchToSearch = gene[1]
    informedSearchDecay = gene[2]
    pherDecay = gene[3]
    layPher = gene[4]
    siteFidelity = gene[5]
    uninfSearchVar = gene[6]

    r = {"8.1630147178657353e-05" :returnToNest,"0.51964199542999268": switchToSearch, 
    "0.37007546424865723": informedSearchDecay,  
    "0.15931577980518341": pherDecay,"10.791043281555176": layPher,
    "7.79852294921875" : siteFidelity, "11.033935": uninfSearchVar}

    f = open("/Users/linhtran/Desktop/iAnt-ARGoS-master/experiments/iAnt.xml",'r')
    filedata = f.read()
    f.close()
    for (i,j) in r.iteritems():
        filedata = filedata.replace(i,j)

    f = open("/Users/linhtran/Desktop/iAnt-ARGoS-master/experiments/iAntNew.xml",'w')
    f.write(filedata)
    f.close()
    return 
#==============================================================================
#   Number of food items collected by the robots.
#   Parse string returned by ARGoS.
#==============================================================================
def fitness():
    count = 0
    with open("/Users/linhtran/Desktop/iAnt-ARGoS-master/iAntTagData.txt") as f:
        for line in f:
            if count == 0:
                fit = int(line.rstrip())
    f.close()
    os.remove("/Users/linhtran/Desktop/iAnt-ARGoS-master/iAntTagData.txt")
    return fit
    
#==============================================================================
#    Crossover at a random point.
#==============================================================================
def onePointCrossover(one, two):
    split = random.randint(0, GENESIZE -2)
    one.dna = (one.dna[:split] + two.dna[split:])
    two.dna = (two.dna[:split] + one.dna[split:])
    return(one, two)
    
#==============================================================================
#    Uniform crossover.
#==============================================================================
def uniformCrossover(one, two):
    for i in range(GENESIZE):
        if coinFlip() == 1:
            one.dna[i] == two.dna[i]
            two.dna[i] == one.dna[i]
    return(one, two)

#==============================================================================
#    Mutate. 
#==============================================================================
def mutate(g):
    if random.randint(0,100) <= 5:
        for i in range(1, GENESIZE):
            if random.randint(0,100) <= 50:
                floatDna = float(g.dna[i])            
                if i >= 0 or i <= 3:
                    maxVal = 1.0
                if i == 4 or i == 5:
                    maxVal = 20.0
                if i == 6:
                    maxVal = 4 * math.pi
                mutRange = float(maxVal/3.0)
                mutation = random.uniform(0.0, mutRange)
                g.dna[i] = calcMut(floatDna, mutation, maxVal)
    return g
    
#==============================================================================
#   Helper calculation for mutate.
#==============================================================================
def calcMut(fDna, mutation, maxVal):
    if coinFlip() == 0 and fDna - mutation >= 0:
        fDna -= mutation
    elif fDna + mutation <= maxVal:
        fDna += mutation
    else:
        return str(fDna)
    return str(fDna)

#==============================================================================
#   Tournament selection with size 2.
#==============================================================================
def tournament(pop, ind): 
    while(len(pop) < POPSIZE):
        winners = []
        for i in range(0,2):
            one = ind[random.randint(0, POPSIZE-1)]
            two = ind[random.randint(0, POPSIZE-1)]
            if one.getFitness() > two.getFitness():
                winners.append(one)
            else:
                winners.append(two)
        crossChance = random.uniform(0,10)
        if crossChance <= 4.0:
            children = onePointCrossover(winners[0], winners[1])
            pop.append(mutate(children[0]))
            pop.append(mutate(children[1]))
        if crossChance > 4.0 and crossChance <= 8.0:
            children = uniformCrossover(winners[0], winners[1])
            pop.append(mutate(children[0]))
            pop.append(mutate(children[1]))
        else:
            pop.append(mutate(one))
    
#==============================================================================
#   Plot the data.     
#==============================================================================
def fitnessPlot(gen, avr, bst, wst):
    a, = plt.plot(gen, avr, 'y', linewidth = 2.0)
    b, = plt.plot(gen, bst, 'c', linewidth = 2.0)
    w, = plt.plot(gen, wst, 'm', linewidth = 2.0)
    plt.grid(True)
    plt.axis([0, max(gen), 0, max(bst)+5])
    plt.xlabel('Generations')
    plt.ylabel('Fitness Value')
    plt.title('Random Distribution Parameter Evolution Performance\nStandard Arena Size')
    legend = plt.legend([b, a, w], ['best','average', 'worst'],loc=0, shadow=True, fontsize='medium')
    plt.savefig('randofi')  
    exit(0)

#==============================================================================
#    Record evolution results.
#==============================================================================
def writeBestParams(results):    
    with open("result.txt",'w') as x:
        x.write(results) 
    x.close()

#==============================================================================
#    Flip a coin.
#==============================================================================
def coinFlip():
    return random.randint(0,1)    
    
#==============================================================================
#     MAIN
#==============================================================================
if __name__ ==  "__main__":   
    pop = []
    gen = []
    avr = []
    bst = []
    wst = []

    for i in range(0, POPSIZE):
        new = Gene()
        pop.append(new)  

    for generation in range(GENERATIONS):
        print("Generation = %f" % generation)
        gen.append(generation)
        average = 0        
        best = -1
        worst = 1000
        
        for individual in pop:
            individual.argos_xml = xmlGen(individual.dna)
            subprocess.call(['argos3 -c experiments/iAnt.xml'], shell=True, stderr=subprocess.STDOUT)

            fVal = fitness()
            individual.setFitness(fVal)
            
            average += fVal
            if fVal > best:
                best = fVal
                bestGene = individual.getDna()
            if fVal < worst:
                worst = fVal 
                worstGene = individual.getDna()
            # print(fVal)
            # print(individual.dna)
            # print(best)
            # print(bestGene)
        
        average /= POPSIZE
        avr.append(average)
        bst.append(best)
        wst.append(worst)
        print("average = %f" % average)
        print("best = %f" % best)
        print("worst = %f" % worst)
        print(bestGene)
        print(worstGene)
        
        ind = pop
        pop = []
        tournament(pop, ind)
    
    results = "Best:" + str(bestGene) + "Worst:" + str(worstGene)
    print(results)
    writeBestParams(results)
    fitnessPlot(gen, avr, bst, wst)
   
   
   
   
   
   
    
    
    
    
    
    