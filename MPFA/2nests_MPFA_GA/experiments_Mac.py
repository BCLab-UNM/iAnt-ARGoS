from xml.dom.minidom import parse, parseString
import random
import subprocess
#import sys

class Random_Argos:

    def __init__(self, argos_xml = None):
        self.argos_xml = argos_xml

        # Updates the ARGoS XML file's random seed.
    def update_random_seed(self):

        ## looks through the xml file

        xml = parse(self.argos_xml)

        #looks at the variables under the experiment tag in the argos XML file.
        framework = xml.getElementsByTagName("experiment")

        attrs = framework[0]


        attrs.setAttribute('random_seed', str(random.randint(1, 1000000)))
        xml.writexml(open(self.argos_xml, 'w'))
    
    def set_random_seed(self, value):
        xml = parse(self.argos_xml)
        framework = xml.getElementsByTagName("experiment")
        attrs = framework[0]
        attrs.setAttribute('random_seed', str(value))
        xml.writexml(open(self.argos_xml, 'w'))
    
    def update_robot_quantity(self, value):
        xml = parse(self.argos_xml)
        
        #looks at the variables under the experiment tag in the argos XML file.
        framework = xml.getElementsByTagName("entity")
        
        attrs = framework[0]
        attrs.setAttribute('quantity', str(value))
        
        
#        attrs = framework[1]
#        attrs.setAttribute('quantity', str(value))
#        
#        
#        attrs = framework[2]
#        attrs.setAttribute('quantity', str(value))
#        
#        
#        attrs = framework[3]
#        attrs.setAttribute('quantity', str(value))
#        
#        
#        attrs = framework[4]
#        attrs.setAttribute('quantity', str(value))

        
        xml.writexml(open(self.argos_xml, 'w'))

    def update_arena_size(self, width):
        xml = parse(self.argos_xml)
        
        #looks at the variables under the experiment tag in the argos XML file.
        framework = xml.getElementsByTagName("arena")
        attrs = framework[0]
        attrs.setAttribute('size', str(width)+", "+str(width) +", 0.0")
        xml.writexml(open(self.argos_xml, 'w'))

    def update_rank(self, rank):
        xml = parse(self.argos_xml)
        #looks at the variables under the experiment tag in the argos XML file.
        framework = xml.getElementsByTagName("_2_FoodDistribution_PowerLaw")
        attrs = framework[0]
        attrs.setAttribute('PowerRank', str(rank))
        xml.writexml(open(self.argos_xml, 'w'))

    def update_nest_position(self, arena_width):
        xml = parse(self.argos_xml)
        #looks at the variables under the experiment tag in the argos XML file.
        framework = xml.getElementsByTagName("simulation")
        attrs = framework[0]
        attrs.setAttribute('nestPosition_0', str(arena_width/4.0)+" , "+str(arena_width/4.0))
        attrs.setAttribute('nestPosition_1', str(arena_width/4.0)+" , -"+str(arena_width/4.0))
        attrs.setAttribute('nestPosition_2', "-"+str(arena_width/4.0)+" , "+str(arena_width/4.0))
        attrs.setAttribute('nestPosition_3', "-"+str(arena_width/4.0)+" , -"+str(arena_width/4.0))
        xml.writexml(open(self.argos_xml, 'w'))
    
    
    def update_robot_position(self, arena_width):
        xml = parse(self.argos_xml)
        
        #looks at the variables under the experiment tag in the argos XML file.
        framework = xml.getElementsByTagName("position")
        
        attrs = framework[0]
        attrs.setAttribute('min', str(-(arena_width/5.0))+" , "+str(-(arena_width/5.0))+" , 0.0")
        attrs.setAttribute('max', str(arena_width/5.0)+" , "+str(arena_width/5.0)+" , 0.0")
        
#        attrs = framework[1]
#        attrs.setAttribute('min', str(arena_width/5.0)+" , "+str(arena_width/5.0)+" , 0.0")
#        attrs.setAttribute('max', str(arena_width*2/5.0)+" , "+str(arena_width*2/5.0)+" , 0.0")
#        
#        attrs = framework[2]
#        attrs.setAttribute('min', str(arena_width/5.0)+" , "+str(-(arena_width*2/5.0))+" , 0.0")
#        attrs.setAttribute('max', str(arena_width*2/5.0)+" , "+str(-(arena_width/5.0))+" , 0.0")
#                                                                 
#        attrs = framework[3]
#        attrs.setAttribute('min', str(-(arena_width*2/5.0))+" , "+str(arena_width/5.0)+" , 0.0")
#        attrs.setAttribute('max', str(-(arena_width/5.0))+" , "+str(arena_width*2/5.0)+" , 0.0")
#        
#        attrs = framework[4]
#        attrs.setAttribute('min', str(-(arena_width*2/5.0))+" , "+str(-(arena_width*2/5.0))+" , 0.0")
#        attrs.setAttribute('max', str(-(arena_width/5.0))+" , "+str(-(arena_width/5.0))+" , 0.0")

        xml.writexml(open(self.argos_xml, 'w'))

    def update_maxSearchDistance(self, distance):
        xml = parse(self.argos_xml)
        framework = xml.getElementsByTagName("iAnt_params")
    
        attrs = framework[0]
        attrs.setAttribute('maxSearchDistance', str(distance))
        xml.writexml(open(self.argos_xml, 'w'))

    def update_numCluster(self, numCluster):
        xml = parse(self.argos_xml)
        framework = xml.getElementsByTagName("_1_FoodDistribution_Cluster")
        
        attrs = framework[0]
        attrs.setAttribute('NumberOfClusters', str(numCluster))
        xml.writexml(open(self.argos_xml, 'w'))

if __name__ == "__main__":

    this_run = Random_Argos("./experiments/iAnt_mac.argos")
    count =0
    for _ in range(10):
        print "Run "+str(count)
        #this_run.update_random_seed()
        this_run.set_random_seed(4189110232+count*735)
        count = count+1
        output = subprocess.check_output(['argos3 -n -c ' + this_run.argos_xml], shell=True, stderr=subprocess.STDOUT)
        print output